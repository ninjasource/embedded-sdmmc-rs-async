//! embedded-sdmmc-rs - SDMMC Protocol
//!
//! Implements the SD/MMC protocol on some generic SPI interface.
//!
//! This is currently optimised for readability and debugability, not
//! performance.

use super::sdmmc_proto::*;
use super::{Block, BlockCount, BlockDevice, BlockIdx};
use core::cell::RefCell;
use core::ops::Deref;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

//#[cfg(feature = "std-log")]
//use log::{debug, info, trace, warn};

#[cfg(feature = "defmt-log")]
use defmt::{debug, info, trace, warn};

const DEFAULT_DELAY_COUNT: u32 = 32_000;

/// Represents an inactive SD Card interface.
/// Built from an SPI peripheral and a Chip
/// Select pin. We need Chip Select to be separate so we can clock out some
/// bytes without Chip Select asserted (which puts the card into SPI mode).
pub struct SdMmcSpi<SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
{
    spi: RefCell<SPI>,
    cs: RefCell<CS>,
    card_type: CardType,
    state: State,
}

/// An initialized block device used to access the SD card.
/// **Caution**: any data must be flushed manually before dropping `BlockSpi`, see `deinit`.
/// Uses SPI mode.
pub struct BlockSpi<'a, SPI, CS>(&'a mut SdMmcSpi<SPI, CS>)
where
    SPI: SpiBus<u8>,
    CS: OutputPin;

/// The possible errors `SdMmcSpi` can generate.
#[cfg_attr(feature = "defmt-log", derive(defmt::Format))]
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
    CommandRetryExhausted(u8),
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

/// The possible states `SdMmcSpi` can be in.
#[cfg_attr(feature = "defmt-log", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum State {
    /// Card is not initialised
    NoInit,
    /// Card is in an error state
    Error,
    /// Card is initialised and idle
    Idle,
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

impl<SPI, CS> SdMmcSpi<SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
{
    /// Create a new SD/MMC controller using a raw SPI interface.
    pub fn new(spi: SPI, cs: CS) -> SdMmcSpi<SPI, CS> {
        SdMmcSpi {
            spi: RefCell::new(spi),
            cs: RefCell::new(cs),
            card_type: CardType::SD1,
            state: State::NoInit,
        }
    }

    fn cs_high(&self) -> Result<(), Error> {
        self.cs
            .borrow_mut()
            .set_high()
            .map_err(|_| Error::GpioError)
    }

    fn cs_low(&self) -> Result<(), Error> {
        self.cs.borrow_mut().set_low().map_err(|_| Error::GpioError)
    }

    /// Initializes the card into a known state
    pub async fn acquire(&mut self) -> Result<BlockSpi<SPI, CS>, Error> {
        self.acquire_with_opts(Default::default()).await
    }

    /// Initializes the card into a known state
    pub async fn acquire_with_opts(
        &mut self,
        options: AcquireOpts,
    ) -> Result<BlockSpi<SPI, CS>, Error> {
        debug!("acquiring card with opts: {:?}", options);
        let result = self.acquire_with_opts_inner(options).await;
        self.cs_high()?;
        let _ = self.receive().await;
        result.map(move |()| BlockSpi(self))
    }

    async fn acquire_with_opts_inner(&mut self, options: AcquireOpts) -> Result<(), Error> {
        // Assume it hasn't worked
        self.state = State::Error;
        trace!("Reset card..");
        // Supply minimum of 74 clock cycles without CS asserted.
        self.cs_high()?;
        for _ in 0..10 {
            self.send(0xFF).await?;
        }
        // Assert CS
        self.cs_low()?;
        // Enter SPI mode
        let mut delay = Delay::new();
        let mut attempts = 32;
        while attempts > 0 {
            trace!("Enter SPI mode, attempt: {}..", 32i32 - attempts);

            match self.card_command(CMD0, 0).await {
                Err(Error::TimeoutCommand(0)) => {
                    // Try again?
                    warn!("Timed out, trying again..");
                    attempts -= 1;
                }
                Err(e) => {
                    return Err(e);
                }
                Ok(R1_IDLE_STATE) => {
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
            return Err(Error::CardNotFound);
        }

        debug!("Card found");

        // Enable CRC
        debug!("Enable CRC: {}", options.require_crc);
        if self.card_command(CMD59, 1).await? != R1_IDLE_STATE && options.require_crc {
            return Err(Error::CantEnableCRC);
        }

        // Check card version
        let r = self.card_command(CMD8, 0x1AA).await?;
        if r == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
            self.card_type = CardType::SD1;
        } else {
            // read next 4 bytes and check the last one
            // we should get back what we sent
            self.receive().await?;
            self.receive().await?;
            self.receive().await?;
            let status = self.receive().await?;
            if status == 0xAA {
                self.card_type = CardType::SD2;
            } else {
                return Err(Error::BadState);
            }
        }

        let arg = match self.card_type {
            CardType::SD1 => 0,
            CardType::SD2 | CardType::SDHC => 0x4000_0000,
        };

        let mut delay = Delay::new();
        while self.card_acmd(ACMD41, arg).await? != R1_READY_STATE {
            delay.delay(Error::TimeoutACommand(ACMD41))?;
        }

        if self.card_type == CardType::SD2 {
            if self.card_command(CMD58, 0).await? != 0 {
                return Err(Error::Cmd58Error);
            }
            if (self.receive().await? & 0xC0) == 0xC0 {
                self.card_type = CardType::SDHC;
            }
            // Discard other three bytes
            self.receive().await?;
            self.receive().await?;
            self.receive().await?;
        }

        debug!("Card type: {:?}", self.card_type);

        self.state = State::Idle;
        Ok(())
    }

    async fn command_with_retry(&self, command: u8, arg: u32) -> Result<u8, Error> {
        let mut delay = Delay::new();
        let mut attempts = 32;
        while attempts > 0 {
            trace!("Card command {}, attempt: {}..", command, 32i32 - attempts);

            match self.card_command(command, arg).await {
                Err(Error::TimeoutCommand(command)) => {
                    // Try again?
                    warn!("Timed out, trying again..");
                    attempts -= 1;
                }
                Err(e) => {
                    return Err(e);
                }
                Ok(r) => return Ok(r),
            }

            delay.delay(Error::TimeoutCommand(command))?;
        }

        Err(Error::CommandRetryExhausted(command))
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

    /// Perform an application-specific command.
    async fn card_acmd(&self, command: u8, arg: u32) -> Result<u8, Error> {
        self.card_command(CMD55, 0).await?;
        self.card_command(command, arg).await
    }

    /// Perform a command.
    async fn card_command(&self, command: u8, arg: u32) -> Result<u8, Error> {
        self.wait_not_busy().await?;

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
        self.spi
            .borrow_mut()
            .transfer(&mut read, &write)
            .await
            .map_err(|_| Error::Transport)?;
        //  let result = read[0];
        //   if (result & 0x80) == ERROR_OK {
        //       return Ok(result);
        //   }

        // skip stuff byte for stop read
        if command == CMD12 {
            let _result = self.receive().await?;
        }

        for _ in 0..512 {
            let result = self.receive().await?;
            if result == 0xFF {
                continue;
            }

            if (result & 0x80) == ERROR_OK {
                return Ok(result);
            } else {
                warn!("Command {} result {}", command, result);
                break;
            }
        }

        Err(Error::TimeoutCommand(command))
    }

    /// Receive a byte from the SD card by clocking in an 0xFF byte.
    async fn receive(&self) -> Result<u8, Error> {
        self.transfer(0xFF).await
    }

    /// Send a byte from the SD card.
    async fn send(&self, out: u8) -> Result<(), Error> {
        self.transfer(out).await?;
        Ok(())
    }

    /// Send one byte and receive one byte.
    async fn transfer(&self, out: u8) -> Result<u8, Error> {
        let mut read = [0];
        let write = [out];
        self.spi
            .borrow_mut()
            .transfer(&mut read, &write)
            .await
            .unwrap();
        Ok(read[0])
    }

    /// Spin until the card returns 0xFF, or we spin too many times and
    /// timeout.
    async fn wait_not_busy(&self) -> Result<(), Error> {
        let mut delay = Delay::new();
        loop {
            let s = self.receive().await?;
            if s == 0xFF {
                break;
            }
            delay.delay(Error::TimeoutWaitNotBusy)?;
        }
        Ok(())
    }
}
