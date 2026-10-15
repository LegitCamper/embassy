//! Startup installers for experimental native ISO TX and fixed-channel RX.
//!
//! Enable `bluetooth-iso`. Call after
//! `new_with_bluetooth`, before spawning the runner or any host/Wi-Fi task.
//! Every error or cancelled future requires a chip restart and firmware reload.
//! HCI Reset alone does not reclaim the allocation or reliably remove RAM hooks. Resident memory
//! remains allocated until reset. This is not standard CIS/BIG negotiation.
//! Use raw `BtDriver` transport and explicit ISO credits after starting. A later
//! HCI Reset invalidates this session; normal TrouBLE startup sends that reset
//! and is unsupported. Concurrent Wi-Fi operation has not been validated.

use crate::{Runner, SpiBus, SpiBusCyw43};
use embedded_hal_1::digital::OutputPin;

use cyw43_controller_blob as blob;

const LOOKUP: u32 = 0x203a34;
const STOCK_LOOKUP: u32 = 0xd035d;
const TABLE: [u32; 11] = [
    0x712f3, 0x7131f, 0x71321, 0x71345, 0x71547, 0x713e3, 0x7143f, 0x714cf, 0, 0x713fd, 0x7154f,
];

/// Reset is mandatory after any error; rollback is best effort.
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    /// Driver timeout, malformed ring/event, or failed write readback.
    Transport,
    /// Controller version, stock pointer, idle state, or scratch differs.
    StockMismatch,
    /// Guarded allocation failed, overlapped config, or changed unexpectedly.
    Allocation,
    /// Controller rejected a command or returned unexpected capabilities.
    Controller,
    /// At least one published hook could not be restored and verified.
    Rollback,
}

/// Installed fixed mono 8-kHz/10-ms, 26-byte SDU native transport and radio.
/// Start through [`Self::start`] before spawning the normal CYW43 runner.
pub struct Installed {
    /// Controller-owned 4096-byte allocation; never free or rewind its arena.
    pub allocation: u32,
    /// Actual LE Read Buffer Size V2 return parameters, including status.
    pub buffer_size_v2: [u8; 7],
    transport: u32,
    started: bool,
}

impl Installed {
    /// Arm the experimental discovery/BIS radio before the normal runner starts.
    /// Afterwards feed handle `0x0042` with complete 26-byte SDUs and route credits.
    /// Errors or cancellation require reset; transmit may already have started.
    pub async fn start<P: OutputPin, S: SpiBusCyw43>(
        &mut self,
        runner: &mut Runner<'_, SpiBus<P, S>>,
    ) -> Result<(), Error> {
        if self.started {
            return Err(Error::Controller);
        }
        self.started = true;
        let mut event = [0; 260];
        let length = runner
            .bluetooth_runtime_command(&[0x1e, 0x20, 3, 1, 26, 0], &mut event)
            .await
            .map_err(|_| Error::Transport)?;
        if length != 6 || event[5] != 0 {
            return Err(Error::Controller);
        }
        Ok(())
    }

    /// Restore hooks while still owning the unstarted runner; allocation remains.
    /// After `Runner::run` takes ownership, reset is the recovery boundary.
    pub async fn restore<P: OutputPin, S: SpiBusCyw43>(
        self,
        runner: &mut Runner<'_, SpiBus<P, S>>,
    ) -> Result<(), Error> {
        let mut session = Session {
            runner,
            allocation: self.allocation,
            transport: self.transport,
            lookup: true,
            table: true,
            callback: true,
            started: self.started,
            mode: Mode::Tx,
        };
        session.rollback().await
    }
}

/// Validate stock, allocate in the controller task, load/read back native image,
/// and publish verified hooks. No radio starts until [`Installed::start`].
/// Do not concurrently use `BtDriver`, Wi-Fi control, or the runner.
pub async fn install<P: OutputPin, S: SpiBusCyw43>(runner: &mut Runner<'_, SpiBus<P, S>>) -> Result<Installed, Error> {
    let mut session = Session {
        runner,
        allocation: 0,
        transport: 0,
        lookup: false,
        table: false,
        callback: false,
        started: false,
        mode: Mode::Tx,
    };
    match session.install().await {
        Ok(buffers) => Ok(Installed {
            allocation: session.allocation,
            transport: session.transport,
            buffer_size_v2: buffers,
            started: false,
        }),
        Err(error) => {
            session.rollback().await?;
            Err(error)
        }
    }
}

/// Fixed-channel RX diagnostic delivering complete 26-byte ISO SDUs.
/// No BIG synchronization, hopping, or codec. The one-shot installer requires
/// a Read Local Version Information task wake every 250 ms; the stream installer
/// wakes from radio callbacks. Drain `BtDriver` promptly. Reset ends the session.
pub struct InstalledRxDiagnostic {
    /// Controller-owned 1024-byte allocation, retained until chip restart.
    pub allocation: u32,
    transport: u32,
    started: bool,
    mode: Mode,
}

impl InstalledRxDiagnostic {
    /// Start fixed BIS logical channel 0 (2404 MHz), AA 0x718ebed6,
    /// CRC init 0x123401, whitening enabled, using the stock test-radio scheduler.
    /// Requires exclusive ownership of the unstarted runner. Any error or
    /// cancellation requires a chip restart, even if the command was rejected.
    pub async fn start<P: OutputPin, S: SpiBusCyw43>(
        &mut self,
        runner: &mut Runner<'_, SpiBus<P, S>>,
    ) -> Result<(), Error> {
        if self.started {
            return Err(Error::Controller);
        }
        self.started = true;
        let mut event = [0; 260];
        let length = runner
            .bluetooth_runtime_command(&[0x1d, 0x20, 1, 1], &mut event)
            .await
            .map_err(|_| Error::Transport)?;
        if length != 6 || event[5] != 0 {
            return Err(Error::Controller);
        }
        let mut radio = [0; 48];
        let mut parameters = [0; 20];
        let mut phase = [0; 20];
        embassy_time::Timer::after_millis(20).await;
        runner
            .bluetooth_runtime_read(0x209788, &mut radio)
            .await
            .map_err(|_| Error::Transport)?;
        runner
            .bluetooth_runtime_read(0x2097b8, &mut parameters)
            .await
            .map_err(|_| Error::Transport)?;
        runner
            .bluetooth_runtime_read(self.allocation + 768, &mut phase)
            .await
            .map_err(|_| Error::Transport)?;
        if radio[..4] != (self.allocation + 256).to_le_bytes()
            || radio[15] != 1
            || radio[28..32] != 0x2097b8u32.to_le_bytes()
            || parameters[6..8] != [1, 2]
            || phase[..4] == [0; 4]
            || u32::from_le_bytes(phase[4..8].try_into().unwrap()) & 0x00300000 != 0x00300000
            || phase[8..12] != 0x718ebed6u32.to_le_bytes()
            || phase[12..16] != 0x123401u32.to_le_bytes()
            || phase[16..20] != [0; 4]
        {
            return Err(Error::Controller);
        }
        Ok(())
    }

    /// Best-effort restoration before starting the runner; allocation remains.
    /// A pending ISO packet can prevent startup command exchange: reset then.
    pub async fn restore<P: OutputPin, S: SpiBusCyw43>(
        self,
        runner: &mut Runner<'_, SpiBus<P, S>>,
    ) -> Result<(), Error> {
        Session {
            runner,
            allocation: self.allocation,
            transport: self.transport,
            lookup: false,
            table: true,
            callback: true,
            started: self.started,
            mode: self.mode,
        }
        .rollback()
        .await
    }
}

/// Install the checked one-shot RX diagnostic; TX [`install`] stays independent.
/// Same stock, scratch, exclusive startup, cancellation and reset requirements.
pub async fn install_rx_diagnostic<P: OutputPin, S: SpiBusCyw43>(
    runner: &mut Runner<'_, SpiBus<P, S>>,
) -> Result<InstalledRxDiagnostic, Error> {
    install_rx(runner, Mode::Rx).await
}

/// Install continuous fixed-channel diagnostic RX with an interrupt-woken mailbox.
/// One pending 26-byte SDU is retained on ring-full; later radio callbacks retry.
/// This remains a bounded diagnostic without BIG synchronization or hopping.
/// All startup/reset restrictions of [`install_rx_diagnostic`] still apply.
pub async fn install_rx_stream_diagnostic<P: OutputPin, S: SpiBusCyw43>(
    runner: &mut Runner<'_, SpiBus<P, S>>,
) -> Result<InstalledRxDiagnostic, Error> {
    install_rx(runner, Mode::RxStream).await
}

async fn install_rx<P: OutputPin, S: SpiBusCyw43>(
    runner: &mut Runner<'_, SpiBus<P, S>>,
    mode: Mode,
) -> Result<InstalledRxDiagnostic, Error> {
    let mut session = Session {
        runner,
        allocation: 0,
        transport: 0,
        lookup: false,
        table: false,
        callback: false,
        started: false,
        mode,
    };
    match session.install().await {
        Ok(_) => Ok(InstalledRxDiagnostic {
            allocation: session.allocation,
            transport: session.transport,
            started: false,
            mode,
        }),
        Err(error) => {
            session.rollback().await?;
            Err(error)
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Tx,
    Rx,
    RxStream,
}

impl Mode {
    /// Resident image, the bootstrap built for its reservation, and the size
    /// that bootstrap reserves. The three travel together because the
    /// reservation size is baked into the bootstrap's instruction stream: a
    /// bootstrap paired with the wrong image would let the controller hand out
    /// memory the image occupies.
    fn image(self) -> (&'static [u8], &'static [u8; 112], u32) {
        match self {
            Self::Tx => (
                blob::continuous::IMAGE,
                blob::continuous::ALLOCATOR,
                blob::continuous::ALLOCATION_BYTES,
            ),
            Self::Rx => (blob::rx::IMAGE, blob::rx::ALLOCATOR, blob::rx::ALLOCATION_BYTES),
            Self::RxStream => (
                blob::rx_stream::IMAGE,
                blob::rx_stream::ALLOCATOR,
                blob::rx_stream::ALLOCATION_BYTES,
            ),
        }
    }
}

struct Session<'r, 'd, P, S> {
    runner: &'r mut Runner<'d, SpiBus<P, S>>,
    allocation: u32,
    transport: u32,
    lookup: bool,
    table: bool,
    callback: bool,
    started: bool,
    mode: Mode,
}

impl<P: OutputPin, S: SpiBusCyw43> Session<'_, '_, P, S> {
    async fn read<const N: usize>(&mut self, address: u32) -> Result<[u8; N], Error> {
        let mut bytes = [0; N];
        self.runner
            .bluetooth_runtime_read(address, &mut bytes)
            .await
            .map_err(|_| Error::Transport)?;
        Ok(bytes)
    }

    async fn word(&mut self, address: u32) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.read(address).await?))
    }

    async fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), Error> {
        self.runner
            .bluetooth_runtime_write(address, bytes)
            .await
            .map_err(|_| Error::Transport)
    }

    async fn put(&mut self, address: u32, word: u32) -> Result<(), Error> {
        self.write(address, &word.to_le_bytes()).await
    }

    async fn command<const N: usize>(&mut self, bytes: &[u8]) -> Result<[u8; N], Error> {
        let mut event = [0; 260];
        let length = self
            .runner
            .bluetooth_runtime_command(bytes, &mut event)
            .await
            .map_err(|_| Error::Transport)?;
        if length != N || event[5] != 0 {
            return Err(Error::Controller);
        }
        event[..N].try_into().map_err(|_| Error::Controller)
    }

    async fn version(&mut self) -> Result<(), Error> {
        let response = self.command::<14>(&[1, 0x10, 0]).await?;
        // Raw Infineon ID; BTstack rewrites this field to Broadcom for its clients.
        if response[5..] != [0, 0x0b, 0, 0, 0x0b, 0x31, 1, 0x10, 0x23] {
            return Err(Error::StockMismatch);
        }
        Ok(())
    }

    async fn install(&mut self) -> Result<[u8; 7], Error> {
        self.command::<6>(&[3, 0x0c, 0]).await?;
        self.version().await?;
        if self.word(LOOKUP).await? != STOCK_LOOKUP
            || self.word(0x203a80).await? != 0x8b634
            || self.read::<16>(0xd035c).await?
                != [
                    0x30, 0xb4, 0, 0x24, 0x0c, 0x60, 5, 0x2b, 0x1e, 0xdd, 6, 0x2b, 0x1c, 0xd0, 8, 0x2b,
                ]
            || self.read::<48>(0x209788).await? != [0; 48]
            || self.read::<112>(0xd0f90).await? != [0; 112]
        {
            return Err(Error::StockMismatch);
        }
        let mut table = self.read::<44>(0x8b634).await?;
        for (bytes, expected) in table.as_chunks::<4>().0.iter().zip(TABLE) {
            if *bytes != expected.to_le_bytes() {
                return Err(Error::StockMismatch);
            }
        }
        let context = self.word(0x201f1c).await?;
        if context & 3 != 0 || !(0x200000..=0x21dd00).contains(&context) {
            return Err(Error::StockMismatch);
        }
        self.transport = context + 0x100;
        if self.word(self.transport).await? != 0x44105 {
            return Err(Error::StockMismatch);
        }
        let bounds = self.read::<12>(0x200560).await?;
        let cursor = u32::from_le_bytes(bounds[4..8].try_into().unwrap());
        let end = u32::from_le_bytes(bounds[8..12].try_into().unwrap());
        let (image, bootstrap, size) = self.mode.image();
        debug_assert_eq!(image.len() as u32, size, "image does not fill its reservation");
        if !valid_bounds(bounds[0], cursor, end, size) {
            return Err(Error::Allocation);
        }
        embassy_time::Timer::after_millis(20).await;
        if self.read::<112>(0xd0f90).await? != [0; 112] {
            return Err(Error::StockMismatch);
        }
        let mut allocator = *bootstrap;
        allocator[100..108].copy_from_slice(&bounds[4..12]);
        self.write(0xd0f90, &allocator).await?;
        if self.word(LOOKUP).await? != STOCK_LOOKUP {
            return Err(Error::StockMismatch);
        }
        self.lookup = true;
        self.put(LOOKUP, 0xd0f91).await?;
        self.version().await?;
        self.version().await?;
        self.allocation = self.word(0xd0ff0).await?;
        let after = self.read::<12>(0x200560).await?;
        self.put(LOOKUP, STOCK_LOOKUP).await?;
        self.lookup = false;
        self.version().await?;
        if self.allocation != cursor
            || after[0] != 1
            || after[4..8] != (cursor + size).to_le_bytes()
            || after[8..12] != bounds[8..12]
        {
            return Err(Error::Allocation);
        }
        for offset in (0..size).step_by(64) {
            if self.read::<64>(cursor + offset).await? != [0; 64] {
                return Err(Error::Allocation);
            }
        }
        for (index, chunk) in image.as_chunks::<64>().0.iter().enumerate() {
            self.write(cursor + index as u32 * 64, chunk).await?;
        }
        if self.mode != Mode::Tx {
            table[16..20].copy_from_slice(&(cursor + 417).to_le_bytes());
            table[24..28].copy_from_slice(&(cursor | 1).to_le_bytes());
            self.write(cursor + 256, &table).await?;
            if self.word(self.transport).await? != 0x44105
                || self.word(LOOKUP).await? != STOCK_LOOKUP
                || self.word(0x203a80).await? != 0x8b634
            {
                return Err(Error::StockMismatch);
            }
            self.callback = true;
            self.put(self.transport, cursor + 577).await?;
            self.version().await?;
            self.table = true;
            self.put(0x203a80, cursor + 256).await?;
            return Ok([0; 7]);
        }
        table[16..20].copy_from_slice(&(cursor | 1).to_le_bytes());
        table[28..32].copy_from_slice(&(cursor + 129).to_le_bytes());
        self.write(cursor + 224, &table).await?;
        self.put(cursor + 2056, cursor).await?;
        self.put(cursor + 2088, cursor + 3161).await?;
        if self.word(self.transport).await? != 0x44105
            || self.word(LOOKUP).await? != STOCK_LOOKUP
            || self.word(0x203a80).await? != 0x8b634
        {
            return Err(Error::StockMismatch);
        }
        self.callback = true;
        self.put(self.transport, cursor + 2049).await?;
        self.version().await?;
        let acl = self.command::<9>(&[2, 0x20, 0]).await?;
        self.lookup = true;
        self.put(LOOKUP, cursor + 3073).await?;
        let response = self.command::<12>(&[0x60, 0x20, 0]).await?;
        if response[6..9] != acl[6..9] || response[9..12] != [30, 0, 4] {
            return Err(Error::Controller);
        }
        self.table = true;
        self.put(0x203a80, cursor + 224).await?;
        Ok(response[5..12].try_into().unwrap())
    }

    async fn restore_pointer(&mut self, address: u32, expected: u32, stock: u32) -> Result<(), Error> {
        let current = self.word(address).await?;
        if current != expected && current != stock {
            return Err(Error::Rollback);
        }
        self.put(address, stock).await
    }

    async fn rollback(&mut self) -> Result<(), Error> {
        let mut failed = false;
        if self.mode != Mode::Tx && self.started {
            self.command::<8>(&[0x1f, 0x20, 0]).await.map_err(|_| Error::Rollback)?;
            if self.read::<48>(0x209788).await != Ok([0; 48]) {
                return Err(Error::Rollback);
            }
        }
        if self.callback {
            failed |= self
                .restore_pointer(
                    self.transport,
                    self.allocation + if self.mode != Mode::Tx { 577 } else { 2049 },
                    0x44105,
                )
                .await
                .is_err();
        }
        if self.lookup {
            let expected = if self.allocation == 0 {
                0xd0f91
            } else {
                self.allocation + 3073
            };
            // Allocation result may already have been read before restoring bootstrap.
            let current = self.word(LOOKUP).await;
            failed |= match current {
                Ok(0xd0f91) => self.put(LOOKUP, STOCK_LOOKUP).await.is_err(),
                _ => self.restore_pointer(LOOKUP, expected, STOCK_LOOKUP).await.is_err(),
            };
        }
        if self.started && self.mode == Mode::Tx {
            failed |= self.command::<8>(&[0x1f, 0x20, 0]).await.is_err();
            failed |= self.read::<48>(0x209788).await != Ok([0; 48]);
        }
        if self.table {
            failed |= self
                .restore_pointer(
                    0x203a80,
                    self.allocation + if self.mode != Mode::Tx { 256 } else { 224 },
                    0x8b634,
                )
                .await
                .is_err();
        }
        if failed { Err(Error::Rollback) } else { Ok(()) }
    }
}

fn valid_bounds(initialized: u8, cursor: u32, end: u32, size: u32) -> bool {
    initialized == 1
        && (cursor | end) & 3 == 0
        && (0x209bac..=0x21e000 - size).contains(&cursor)
        && end <= 0x220000
        && cursor + size < end
}

#[cfg(test)]
mod tests {
    use super::valid_bounds;
    #[test]
    fn allocation_stays_below_signed_configuration() {
        assert!(valid_bounds(1, 0x217e78, 0x220000, 4096));
        assert!(valid_bounds(1, 0x21dc00, 0x220000, 1024));
        assert!(!valid_bounds(1, 0x21dc04, 0x220000, 1024));
        for (initialized, cursor, end) in [
            (0, 0x217e78, 0x220000),
            (1, 0x21d004, 0x220000),
            (1, 0x217e79, 0x220000),
            (1, 0x217e78, 0x218e78),
            (1, 0xfffff000, 0x220000),
        ] {
            assert!(!valid_bounds(initialized, cursor, end, 4096));
        }
    }
}
