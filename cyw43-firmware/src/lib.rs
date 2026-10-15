//! Checked-in CYW43 firmware bytes with the alignment required by `cyw43`.

#![no_std]

use aligned::{A4, Aligned};

/// CYW43439 Wi-Fi firmware used by Raspberry Pi Pico W.
pub static CYW43439_WIFI: &Aligned<A4, [u8]> = &Aligned(*include_bytes!("../43439A0.bin"));

/// CYW43439 country locale matrix used by Raspberry Pi Pico W.
pub static CYW43439_CLM: &Aligned<A4, [u8]> = &Aligned(*include_bytes!("../43439A0_clm.bin"));

/// Stock CYW43439 Bluetooth firmware without resident patch reservation.
pub static CYW43439_BLUETOOTH: &Aligned<A4, [u8]> = &Aligned(*include_bytes!("../43439A0_btfw.bin"));

/// CYW43439 NVRAM configuration for Raspberry Pi Pico W.
pub static CYW43439_NVRAM_RP2040: &Aligned<A4, [u8]> = &Aligned(*include_bytes!("../nvram_rp2040.bin"));
