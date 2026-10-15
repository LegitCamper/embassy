#!/usr/bin/env python3
"""Run actual SPI transfers against a mock, with ISO padding/flags/window reset off and on."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src/spi.rs").read_text()
constants = (ROOT / "src/consts.rs").read_text()


def function(text, name):
    match = re.search(r"(?:pub\(crate\) )?(?:async |const )?fn " + name + r"\(", text)
    assert match, name
    start = text.index("{", match.end())
    depth = 1
    end = start + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[match.start():end]


names = ("READ", "WRITE", "INC_ADDR", "FUNC_BACKPLANE", "BACKPLANE_ADDRESS_MASK",
         "BACKPLANE_ADDRESS_32BIT_FLAG", "BACKPLANE_WINDOW_SIZE",
         "BACKPLANE_MAX_TRANSFER_SIZE", "WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE", "CHIPCOMMON_BASE_ADDRESS")
declarations = []
for name in names:
    match = re.search(r"pub\(crate\) const " + name + r"\b[^;]+;", constants)
    assert match, name
    declarations.append(match[0])
assert re.search(r"self\.write8\(\s*FUNC_BUS,\s*SPI_RESP_DELAY_F1,\s*"
                 r"WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE\s*\)", source)

mock = r'''
#![allow(dead_code)]
use std::{future::Future, pin::pin, task::{Context, Poll, Waker}, mem::size_of_val, slice};
macro_rules! trace { ($($tokens:tt)*) => {}; }
fn ready<F: Future>(future: F) -> F::Output {
    match pin!(future).as_mut().poll(&mut Context::from_waker(Waker::noop())) {
        Poll::Ready(value) => value,
        Poll::Pending => panic!("mock unexpectedly blocked"),
    }
}
fn byte(address: u32) -> u8 { (address ^ (address >> 7) ^ (address >> 15)) as u8 }
#[derive(Default)]
struct MockSpi {
    window: u32, bulk: bool, reads: Vec<(u32, usize, u32)>,
    writes: Vec<(u32, Vec<u8>)>, windows: Vec<u32>,
}
impl MockSpi {
    async fn cmd_read(&mut self, command: u32, words: &mut [u32]) -> u32 {
        let function = (command >> 28) & 3;
        let address = (command >> 11) & 0x1ffff;
        let length = (command & 0x7ff) as usize;
        let padding = if function == 1 {
            if cfg!(feature = "bluetooth-iso") { 4 } else { 1 }
        } else { 0 };
        assert_eq!(words.len(), (length + 3) / 4 + padding);
        let absolute = if function == 1 { self.window + (address & 0x7fff) } else { address };
        if self.bulk {
            assert_eq!(address & 0x8000, if cfg!(feature = "bluetooth-iso") { 0x8000 } else { 0 });
            assert!(length <= BACKPLANE_MAX_TRANSFER_SIZE);
            assert!((address as usize & 0x7fff) + length <= 0x8000);
        }
        words.fill(0xdedec0de);
        let mut bytes = vec![0; (words.len() - padding) * 4];
        for (i, item) in bytes.iter_mut().take(length).enumerate() { *item = byte(absolute + i as u32); }
        for (word, chunk) in words[padding..].iter_mut().zip(bytes.chunks_exact(4)) {
            *word = u32::from_le_bytes(chunk.try_into().unwrap());
        }
        self.reads.push((absolute, length, address));
        0x12345678
    }
    async fn cmd_write(&mut self, words: &[u32]) -> u32 {
        let command = words[0];
        assert_eq!(command >> 28, 0xd);
        let address = (command >> 11) & 0x1ffff;
        let length = (command & 0x7ff) as usize;
        assert_eq!(words.len(), (length + 3) / 4 + 1);
        assert_eq!(address & 0x8000, if cfg!(feature = "bluetooth-iso") { 0x8000 } else { 0 });
        assert!(length <= BACKPLANE_MAX_TRANSFER_SIZE);
        assert!((address as usize & 0x7fff) + length <= 0x8000);
        let bytes: Vec<u8> = words[1..].iter().flat_map(|word| word.to_le_bytes()).take(length).collect();
        self.writes.push((self.window + (address & 0x7fff), bytes));
        0x12345678
    }
}
#[derive(Default)]
struct Reader { spi: MockSpi, status: u32 }
impl Reader {
    async fn backplane_set_window(&mut self, address: u32) {
        self.spi.window = address & !0x7fff;
        self.spi.windows.push(self.spi.window);
    }
    // ACTUAL_METHODS
}
fn check_window(reader: &Reader, address: u32, length: usize) {
    let expected = if cfg!(feature = "bluetooth-iso") { CHIPCOMMON_BASE_ADDRESS }
                   else if length == 0 { 0 }
                   else { (address + length as u32 - 1) & !0x7fff };
    assert_eq!(reader.spi.window, expected);
    if cfg!(feature = "bluetooth-iso") {
        assert_eq!(reader.spi.windows.last(), Some(&CHIPCOMMON_BASE_ADDRESS));
    }
}
fn main() {
    assert_eq!(WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE, if cfg!(feature = "bluetooth-iso") { 16 } else { 4 });
    for function in 0..4 {
        for length in [1, 2, 4] {
            let mut reader = Reader::default();
            let value = ready(reader.readn(function, 0x40, length));
            let expected = (0..length).fold(0, |word, i| word | (byte(0x40 + i) as u32) << (8 * i));
            assert_eq!(value, expected);
            assert_eq!(reader.status, 0x12345678);
        }
    }
    for address in [0x19200000, 0x19207ffc, 0x19208000] {
        for length in [1, 2, 4] {
            let mut reader = Reader::default();
            let value = ready(reader.backplane_readn(address, length));
            let expected = (0..length).fold(0, |word, i| word | (byte(address + i) as u32) << (8 * i));
            assert_eq!(value, expected);
            assert_eq!(reader.spi.reads[0].2 & 0x8000, if length == 4 { 0x8000 } else { 0 });
        }
        for length in 0..=BACKPLANE_MAX_TRANSFER_SIZE * 4 + 3 {
            let mut reader = Reader::default();
            reader.spi.bulk = true;
            let mut data = vec![0xee; length + 2];
            ready(reader.bp_read(address, &mut data[1..1 + length]));
            assert_eq!(data[0], 0xee);
            assert_eq!(data[length + 1], 0xee);
            for i in 0..length { assert_eq!(data[i + 1], byte(address + i as u32)); }
            let mut next = address;
            for (actual, count, _) in &reader.spi.reads {
                assert_eq!(*actual, next);
                next += *count as u32;
            }
            assert_eq!(next, address + length as u32);
            check_window(&reader, address, length);
            let mut writer = Reader::default();
            let input: Vec<u8> = (0..length).map(|i| byte(address + i as u32) ^ 0x5a).collect();
            ready(writer.bp_write(address, &input));
            let mut cursor = 0;
            for (actual, bytes) in &writer.spi.writes {
                assert_eq!(*actual, address + cursor as u32);
                assert_eq!(bytes, &input[cursor..cursor + bytes.len()]);
                cursor += bytes.len();
            }
            assert_eq!(cursor, length);
            check_window(&writer, address, length);
        }
    }
    println!("SPI padding {} bytes: scalar/bulk, flags, partial/window-crossing transfers and window restoration pass",
             WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE);
}
'''
methods = "\n".join(function(source, name) for name in ("readn", "backplane_readn", "bp_read", "bp_write"))
program = mock.replace("// ACTUAL_METHODS", methods) + "\n" + "\n".join(declarations)
program += "\n" + function(source, "cmd_word")
program += "\n" + function((ROOT / "src/util.rs").read_text(), "slice8_mut")
with tempfile.TemporaryDirectory(prefix="cyw43-spi-check-") as directory:
    rust = Path(directory) / "check.rs"
    rust.write_text(program)
    for enabled in (False, True):
        executable = Path(directory) / ("iso" if enabled else "baseline")
        command = ["rustc", "--edition=2024", "-Dwarnings", str(rust), "-o", str(executable)]
        if enabled:
            command += ["--cfg", 'feature="bluetooth-iso"']
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)
