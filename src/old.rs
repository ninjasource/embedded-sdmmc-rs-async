#![no_std]
#![feature(type_alias_impl_trait, async_fn_in_trait)]

use defmt::{debug, info, trace, warn};

use embassy_nrf::{
    gpio::{Output, Pin},
    spim::{Instance, Spim},
};
use embedded_hal_async::spi::SpiBus;

#[macro_use]
mod structure;

mod blockdevice;
//mod sdmmc;
mod sdmmc_proto;

pub async fn init_sd<P: Pin>(
    ncs: &mut Output<'_, P>,
    spi: &mut impl SpiBus<u8>,
    options: AcquireOpts,
) -> Result<(), Error> {
    ncs.set_high();

    for _ in 0..10 {
        send(spi, 0xFF).await?;
    }

    ncs.set_low();

    let mut delay = Delay::new();
    let mut attempts = 32;
    while attempts > 0 {
        trace!("Enter SPI mode, attempt: {}..", 32i32 - attempts);

        match card_command(CMD0, 0, spi).await {
            Err(Error::TimeoutCommand(0)) => {
                // Try again?
                warn!("Timed out, trying again..");
                attempts -= 1;
            }
            Err(e) => {
                return Err(e);
            }
            Ok(R1_IDLE_STATE) => {
                info!("connected to sd card!");
                break;
            }
            Ok(r) => {
                // Try again
                warn!("Got response: {:x}, trying again..", r);
            }
        }

        delay.delay(Error::TimeoutCommand(CMD0))?;
    }

    if attempts == 0 {
        panic!("card not found")
    }

    Ok(())
}

/// Options for acquiring the card.
#[cfg_attr(feature = "defmt-log", derive(defmt::Format))]
#[derive(Debug)]
pub struct AcquireOpts {
    /// Some cards don't support CRC mode. At least a 512MiB Transcend one.
    pub require_crc: bool,
}

impl Default for AcquireOpts {
    fn default() -> Self {
        AcquireOpts { require_crc: true }
    }
}

/// Receive a byte from the SD card by clocking in an 0xFF byte.
async fn receive(spi: &mut impl SpiBus<u8>) -> Result<u8, Error> {
    transfer(spi, 0xFF).await
}

/// Send a byte from the SD card.
async fn send(spi: &mut impl SpiBus<u8>, out: u8) -> Result<(), Error> {
    transfer(spi, out).await?;
    Ok(())
}

/// Send one byte and receive one byte.
async fn transfer(spi: &mut impl SpiBus<u8>, out: u8) -> Result<u8, Error> {
    let mut data = [out];
    spi.transfer_in_place(&mut data).await.unwrap();
    Ok(data[0])
}

/// Receive a byte from the SD card by clocking in an 0xFF byte.
fn _blocking_receive<T: Instance>(spim: &mut Spim<'_, T>) -> Result<u8, Error> {
    _blocking_transfer(spim, 0xFF)
}

/// Send a byte from the SD card.
fn _blocking_send<T: Instance>(spim: &mut Spim<'_, T>, out: u8) -> Result<(), Error> {
    _blocking_transfer(spim, out)?;
    Ok(())
}

/// Send one byte and receive one byte.
fn _blocking_transfer<T: Instance>(spim: &mut Spim<'_, T>, out: u8) -> Result<u8, Error> {
    let mut data = [out];
    spim.blocking_transfer_in_place(&mut data).unwrap();
    Ok(data[0])
}

/// Perform the 7-bit CRC used on the SD card
pub fn crc7(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut d in data.iter().cloned() {
        for _bit in 0..8 {
            crc <<= 1;
            if ((d & 0x80) ^ (crc & 0x80)) != 0 {
                crc ^= 0x09;
            }
            d <<= 1;
        }
    }
    (crc << 1) | 1
}

/// Perform a command.
async fn card_command(command: u8, arg: u32, spi: &mut impl SpiBus<u8>) -> Result<u8, Error> {
    wait_not_busy(spi).await?;

    let mut write = [
        0x40 | command,
        (arg >> 24) as u8,
        (arg >> 16) as u8,
        (arg >> 8) as u8,
        arg as u8,
        0,
    ];
    write[5] = crc7(&write[0..5]);

    let mut read = [0u8];
    spi.transfer(&mut read, &write).await.unwrap();
    let result = read[0];
    if (result & 0x80) == ERROR_OK {
        return Ok(result);
    }

    Err(Error::TimeoutCommand(command))
}

/// Card indicates last operation was a success
pub const ERROR_OK: u8 = 0x00;

/// The possible errors `SdMmcSpi` can generate.
#[derive(Debug, Copy, Clone)]
pub enum Error {
    /// We got an error from the SPI peripheral
    Transport,
    /// We failed to enable CRC checking on the SD card
    CantEnableCRC,
    /// We didn't get a response when reading data from the card
    TimeoutReadBuffer,
    /// We didn't get a response when waiting for the card to not be busy
    TimeoutWaitNotBusy,
    /// We didn't get a response when executing this command
    TimeoutCommand(u8),
    /// We didn't get a response when executing this application-specific command
    TimeoutACommand(u8),
    /// We got a bad response from Command 58
    Cmd58Error,
    /// We failed to read the Card Specific Data register
    RegisterReadError,
    /// We got a CRC mismatch (card gave us, we calculated)
    CrcError(u16, u16),
    /// Error reading from the card
    ReadError,
    /// Error writing to the card
    WriteError,
    /// Can't perform this operation with the card in this state
    BadState,
    /// Couldn't find the card
    CardNotFound,
    /// Couldn't set a GPIO pin
    GpioError,
}

/// status for card in the idle state
pub const R1_IDLE_STATE: u8 = 0x01;

//==============================================================================

// SD Card Commands

/// GO_IDLE_STATE - init card in spi mode if CS low
pub const CMD0: u8 = 0x00;
/// SEND_IF_COND - verify SD Memory Card interface operating condition.
pub const CMD8: u8 = 0x08;
/// SEND_CSD - read the Card Specific Data (CSD register)
pub const CMD9: u8 = 0x09;
/// STOP_TRANSMISSION - end multiple block read sequence
pub const CMD12: u8 = 0x0C;
/// SEND_STATUS - read the card status register
pub const CMD13: u8 = 0x0D;
/// READ_SINGLE_BLOCK - read a single data block from the card
pub const CMD17: u8 = 0x11;
/// READ_MULTIPLE_BLOCK - read a multiple data blocks from the card
pub const CMD18: u8 = 0x12;
/// WRITE_BLOCK - write a single data block to the card
pub const CMD24: u8 = 0x18;
/// WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION
pub const CMD25: u8 = 0x19;
/// APP_CMD - escape for application specific command
pub const CMD55: u8 = 0x37;
/// READ_OCR - read the OCR register of a card
pub const CMD58: u8 = 0x3A;
/// CRC_ON_OFF - enable or disable CRC checking
pub const CMD59: u8 = 0x3B;
/// SD_SEND_OP_COMD - Sends host capacity support information and activates
/// the card's initialization process
pub const ACMD41: u8 = 0x29;

//==============================================================================

/// status for card in the ready state
pub const R1_READY_STATE: u8 = 0x00;

/// Spin until the card returns 0xFF, or we spin too many times and
/// timeout.
async fn wait_not_busy(spi: &mut impl SpiBus<u8>) -> Result<(), Error> {
    let mut delay = Delay::new();
    loop {
        let s = receive(spi).await?;
        if s == 0xFF {
            break;
        }
        delay.delay(Error::TimeoutWaitNotBusy)?;
    }
    Ok(())
}

/// The different types of card we support.
#[cfg_attr(feature = "defmt-log", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq)]
enum CardType {
    SD1,
    SD2,
    SDHC,
}

/// A terrible hack for busy-waiting the CPU while we wait for the card to
/// sort itself out.
///
/// @TODO replace this!
struct Delay(u32);

impl Delay {
    fn new() -> Delay {
        Delay(DEFAULT_DELAY_COUNT)
    }

    fn delay(&mut self, err: Error) -> Result<(), Error> {
        if self.0 == 0 {
            Err(err)
        } else {
            let dummy_var: u32 = 0;
            for _ in 0..100 {
                unsafe { core::ptr::read_volatile(&dummy_var) };
            }
            self.0 -= 1;
            Ok(())
        }
    }
}

const DEFAULT_DELAY_COUNT: u32 = 32_000;
