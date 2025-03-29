//! SDMMC pio driver
//
// implementation created from https://github.com/carlk3/no-OS-FatFS-SD-SDMMC-SPI-RPi-Pico/blob/main/src/sd_driver/SDMMC/rp2040_sdio.pio

use crate::dma::{AnyChannel, Channel, Transfer};
use crate::pio::{
    Common, Config, Direction, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use crate::{into_ref, Peripheral, PeripheralRef};

/// This struct represents an sdiodriver program
pub struct PioSDMMCCmdClkProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioSDMMCCmdClkProgram<'a, PIO> {
    /// Load the program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio::pio_asm!(
            r#"
                .define D0 1
                .define D1 1
                .side_set 1

                mov OSR, NULL       side 1 [D1]    ; Make sure OSR is full of zeros to prevent autopull
                
                wait_cmd:
                    mov Y, !STATUS      side 0 [D0]    ; Check if TX FIFO has data
                    jmp !Y wait_cmd     side 1 [D1]
                
                load_cmd:
                    out NULL, 32        side 0 [D0]    ; Load first word (trigger autopull)
                    out X, 8            side 1 [D1]    ; Number of bits to send
                    set pins, 1         side 0 [D0]    ; Initial state of CMD is high
                    set pindirs, 1      side 1 [D1]    ; Set SDMMC_CMD as output
                
                send_cmd:
                    out pins, 1         side 0 [D0]    ; Write output on falling edge of CLK
                    jmp X-- send_cmd    side 1 [D1]
                
                prep_resp:
                    set pindirs, 0      side 0 [D0]    ; Set SDMMC_CMD as input
                    out X, 8            side 1 [D1]    ; Get number of bits in response
                    nop                 side 0 [D0]    ; For clock alignment
                    jmp !X resp_done    side 1 [D1]    ; Check if we expect a response
                
                wait_resp:
                    nop                  side 0 [D0]
                    jmp PIN wait_resp    side 1 [D1]    ; Loop until SDMMC_CMD = 0
                
                    ; Note: input bits are read at the same time as we write CLK=0.
                    ; Because the host controls the clock, the read happens before
                    ; the card sees the falling clock edge. This gives maximum time
                    ; for the data bit to settle.
                read_resp:
                    in PINS, 1          side 0 [D0]    ; Read input data bit
                    jmp X-- read_resp   side 1 [D1]    ; Loop to receive all data bits
                
                resp_done:
                    push                side 0 [D0]    ; Push the remaining part of response
            "#,
        );
        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// This struct represents an sdiodriver program
pub struct PioSDMMCDataProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioSDMMCDataProgram<'a, PIO> {
    /// Load the program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio::pio_asm!(
            r#"
                .define D0 1
                .define D1 1
                .define CLKDIV 4

                ; This is relative to D0 GPIO number.
                ; The pin is selected by adding Index to the
                ;   PINCTRL_IN_BASE configuration, modulo 32.
                ; This is used as a WAIT index, and must be between 4 and 31.
                ; (Offsets 0-3 are D0, D1, D2, and D3.)
                .define PUBLIC SDMMC_CLK_PIN_D0_OFFSET 30    ; (-2 in mod32 arithmetic)

                .wrap_target

                ; Wait for initial start of block token for reception
                wait_start:
                    mov X, Y                        ; Reinitialize number of nibbles to receive
                    wait 0 pin 0                    ; Wait for zero state on D0
                    wait 1 pin SDMMC_CLK_PIN_D0_OFFSET  [CLKDIV-1]  ; Wait for rising edge and then whole clock cycle

                ; Receive data logic
                rx_data:
                    in PINS, 4                      [CLKDIV-2]  ; Read nibble
                    jmp X--, rx_data                ; Continue receiving until all nibbles are received

                ; Now check whether to transmit or continue receiving
                jmp x--, tx_wait                   ; After receiving, jump to tx_wait for transmission if needed

                ; Data transmission program
                tx_wait:
                    wait 0 pin SDMMC_CLK_PIN_D0_OFFSET
                    wait 1 pin SDMMC_CLK_PIN_D0_OFFSET  [CLKDIV + D1 - 1]  ; Synchronize to falling edge

                tx_loop:
                    out PINS, 4                     [D0]    ; Write nibble and wait for clock cycle
                    jmp X--, tx_loop                [D1]    ; Continue transmitting until all nibbles are sent

                ; Set data bus as input after transmission is complete
                set pindirs, 0x00                 [D0]    ; Set data bus as input


                ; Response loop after transmission
                response_loop:
                    in PINS, 1                     [D1]    ; Read D0 on rising edge for response
                    jmp Y--, response_loop         [D0]    ; Continue reading response

                ; Wait for card idle condition after receiving response
                wait_idle:
                    wait 1 pin 0                   [D1]    ; Wait for card to indicate idle
                    push                           [D0]    ; Push the response token

                .wrap
            "#,
        );
        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// Pio backed SDMMC driver
pub struct PioSDMMC<'d, PIO: Instance, const CCSM: usize, const DSM: usize> {
    tx_dma: PeripheralRef<'d, AnyChannel>,
    rx_dma: PeripheralRef<'d, AnyChannel>,
    cmd_clk_sm: StateMachine<'d, PIO, CCSM>,
    data_sm: StateMachine<'d, PIO, DSM>,
}

impl<'d, PIO: Instance, const CCSM: usize, const DSM: usize> PioSDMMC<'d, PIO, CCSM, DSM> {
    /// Create a new instance the driver
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut cmd_clk_sm: StateMachine<'d, PIO, CCSM>,
        mut data_sm: StateMachine<'d, PIO, DSM>,
        clk_pin: impl PioPin,
        cmd_pin: impl PioPin,
        d0_pin: impl PioPin,
        d1_pin: impl PioPin,
        d2_pin: impl PioPin,
        d3_pin: impl PioPin,
        // det_pin: Option<impl PioPin>,
        tx_dma: impl Peripheral<P = impl Channel> + 'd,
        rx_dma: impl Peripheral<P = impl Channel> + 'd,
        cmd_clk_prog: &PioSDMMCCmdClkProgram<'d, PIO>,
        data_prog: &PioSDMMCDataProgram<'d, PIO>,
        high_speed: bool,
    ) -> Self {
        into_ref!(rx_dma);
        into_ref!(tx_dma);

        let clk = common.make_pio_pin(clk_pin);
        let cmd = common.make_pio_pin(cmd_pin);
        let d0 = common.make_pio_pin(d0_pin);
        let d1 = common.make_pio_pin(d1_pin);
        let d2 = common.make_pio_pin(d2_pin);
        let d3 = common.make_pio_pin(d3_pin);
        // let det = common.make_pio_pin(det_pin);

        // configure clk program
        cmd_clk_sm.set_pin_dirs(Direction::Out, &[&clk, &cmd]);
        let mut cfg = Config::default();
        cfg.use_program(&cmd_clk_prog.prg, &[&clk]);
        cfg.set_in_pins(&[&cmd]);
        cfg.set_out_pins(&[&cmd]);
        cfg.set_set_pins(&[&cmd]);
        cfg.set_jmp_pin(&cmd);
        if high_speed {
            cfg.clock_divider = 5_u8.into();
        } else {
            cfg.clock_divider = 3_u8.into();
        }
        cmd_clk_sm.set_config(&cfg);
        cmd_clk_sm.set_enable(true);

        // configure data state machine
        cmd_clk_sm.set_pin_dirs(Direction::Out, &[&clk, &cmd, &d0, &d1, &d2, &d3]); // , &det
        let mut cfg = Config::default();
        cfg.use_program(&data_prog.prg, &[&clk]);
        cfg.set_in_pins(&[&d0, &d1, &d2, &d3]);
        cfg.set_out_pins(&[&d0, &d1, &d2, &d3]);
        if high_speed {
            cfg.clock_divider = 5_u8.into();
        } else {
            cfg.clock_divider = 3_u8.into();
        }
        data_sm.set_config(&cfg);
        data_sm.set_enable(true);

        Self {
            cmd_clk_sm,
            data_sm,
            tx_dma: tx_dma.map_into(),
            rx_dma: rx_dma.map_into(),
        }
    }

    pub fn write_command<'b>(&'b mut self, buff: &'b [u32]) {
        for b in buff {
            self.cmd_clk_sm.tx().push(*b)
        }
    }

    /// Return an in-prograss dma transfer future. Awaiting it will guarantee a complete transfer.
    pub fn write<'b>(&'b mut self, buff: &'b [u32]) -> Transfer<'b, AnyChannel> {
        self.data_sm.tx().dma_push(self.tx_dma.reborrow(), buff, false)
    }

    /// Return an in-prograss dma transfer future. Awaiting it will guarantee a complete transfer.
    pub fn read<'b>(&'b mut self, buff: &'b mut [u32]) -> Transfer<'b, AnyChannel> {
        self.data_sm.rx().dma_pull(self.rx_dma.reborrow(), buff, false)
    }
}
