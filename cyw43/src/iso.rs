//! CYW43439 resident ISO patch integration.
//!
//! Patch bytes and the installer state machine live in `cyw43-controller`.
//! This module only supplies that crate exclusive startup access through the
//! Embassy CYW43 runner.

use core::future::Future;

use crate::{Runner, SpiBus, SpiBusCyw43};
use embedded_hal_1::digital::OutputPin;

pub use cyw43_controller::{Error, Installed, InstalledRxDiagnostic};

impl<BUS: crate::runner::Bus> cyw43_controller::Runtime for Runner<'_, BUS> {
    fn read(&mut self, address: u32, data: &mut [u8]) -> impl Future<Output = Result<(), ()>> {
        async move {
            self.bluetooth_runtime_read(address, data)
                .await
                .map_err(|_| ())
        }
    }

    fn write(&mut self, address: u32, data: &[u8]) -> impl Future<Output = Result<(), ()>> {
        async move {
            self.bluetooth_runtime_write(address, data)
                .await
                .map_err(|_| ())
        }
    }

    fn command(
        &mut self,
        command: &[u8],
        response: &mut [u8; 260],
    ) -> impl Future<Output = Result<usize, ()>> {
        async move {
            self.bluetooth_runtime_command(command, response)
                .await
                .map_err(|_| ())
        }
    }

    fn delay_millis(&mut self, millis: u32) -> impl Future<Output = ()> {
        embassy_time::Timer::after_millis(u64::from(millis))
    }
}

/// Validate stock firmware, allocate controller SRAM, install TX image and
/// publish its hooks. No radio starts until [`Installed::start`].
pub async fn install<P: OutputPin, S: SpiBusCyw43>(
    runner: &mut Runner<'_, SpiBus<P, S>>,
) -> Result<Installed, Error> {
    cyw43_controller::install(runner).await
}

/// Install checked one-shot fixed-channel RX diagnostic.
pub async fn install_rx_diagnostic<P: OutputPin, S: SpiBusCyw43>(
    runner: &mut Runner<'_, SpiBus<P, S>>,
) -> Result<InstalledRxDiagnostic, Error> {
    cyw43_controller::install_rx_diagnostic(runner).await
}

/// Install interrupt-woken continuous fixed-channel RX diagnostic.
pub async fn install_rx_stream_diagnostic<P: OutputPin, S: SpiBusCyw43>(
    runner: &mut Runner<'_, SpiBus<P, S>>,
) -> Result<InstalledRxDiagnostic, Error> {
    cyw43_controller::install_rx_stream_diagnostic(runner).await
}

/// Install open-loop CSA#2 hopping continuous RX diagnostic.
///
/// Installation success is not broadcast synchrony: the event counter starts
/// at zero with no BIGInfo anchor.
pub async fn install_rx_hop_diagnostic<P: OutputPin, S: SpiBusCyw43>(
    runner: &mut Runner<'_, SpiBus<P, S>>,
) -> Result<InstalledRxDiagnostic, Error> {
    cyw43_controller::install_rx_hop_diagnostic(runner).await
}
