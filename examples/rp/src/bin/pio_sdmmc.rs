//! This example shows simple sdmmc usage with sdio via pio.

#![no_std]
#![no_main]
use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{Config, InterruptHandler, Pio, ShiftConfig, ShiftDirection};
use embassy_rp::pio_programs::sdmmc::{PioSDMMC, PioSDMMCCmdClkProgram, PioSDMMCDataProgram};
use embassy_rp::{bind_interrupts, Peripheral};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // define pins
    let clk_pin = p.PIN_28;
    let cmd_pin = p.PIN_27;
    let d0_pin = p.PIN_26;
    let d1_pin = p.PIN_22;
    let d2_pin = p.PIN_21;
    let d3_pin = p.PIN_20;

    let pio = p.PIO0;
    let Pio {
        common: mut common,
        sm0: mut sm0,
        sm1: mut sm1,
        ..
    } = Pio::new(pio, Irqs);

    let pio_cmd_clk_prog = PioSDMMCCmdClkProgram::new(&mut common);
    let pio_data_prog = PioSDMMCDataProgram::new(&mut common);

    let high_speed = true;
    let sdmmc = PioSDMMC::new(
        &mut common,
        sm0,
        sm1,
        clk_pin,
        cmd_pin,
        d0_pin,
        d1_pin,
        d2_pin,
        d3_pin,
        p.DMA_CH0,
        p.DMA_CH1,
        &pio_cmd_clk_prog,
        &pio_data_prog,
        true,
    );

    // sdmmc.read(buff);
}
