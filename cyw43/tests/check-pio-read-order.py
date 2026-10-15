#!/usr/bin/env python3
"""Execute actual cmd_read against an eager-DMA mock; no physical timing claim."""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def method(path):
    source = path.read_text()
    match = re.search(r"pub async fn cmd_read\(", source)
    assert match
    start = source.index("{", match.end())
    end, depth = start + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


MOCK = r'''
#![allow(dead_code)]
use std::{cell::RefCell, rc::Rc, future::Future, pin::{Pin, pin}, slice,
          task::{Context, Poll, Waker}};
#[derive(Default)]
struct State { armed: bool, command: Option<u32>, log: Vec<&'static str>, x:u32, y:u32 }
struct Rx(Rc<RefCell<State>>);
struct Tx(Rc<RefCell<State>>);
struct Sm { rx: Rx, tx: Tx }
struct Transfer<'a> { state: Rc<RefCell<State>>, data: &'a mut [u32], swap: bool, status: bool }
impl Future for Transfer<'_> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<()> {
        assert!(self.state.borrow().command.is_some());
        let status = self.status;
        let swap = self.swap;
        for (i, value) in self.data.iter_mut().enumerate() {
            let wire = if status { 0x12345679u32 } else { 0xabcdef01u32.wrapping_add(i as u32 * 2) };
            *value = if swap { wire.swap_bytes() } else { wire };
        }
        self.state.borrow_mut().log.push(if status { "status_wait" } else { "data_wait" });
        Poll::Ready(())
    }
}
impl Rx {
    fn dma_pull<'a>(&mut self, _: &mut (), data: &'a mut [u32], swap: bool) -> Transfer<'a> {
        let mut state = self.0.borrow_mut();
        let status = state.command.is_some();
        state.armed = true;
        state.log.push(if status { "status_arm" } else { "data_arm" });
        Transfer { state: self.0.clone(), data, swap, status }
    }
}
impl Tx {
    fn push(&mut self, cmd: u32) {
        let mut state = self.0.borrow_mut();
        assert!(state.armed, "RX DMA not armed before command");
        assert!(state.command.replace(cmd).is_none());
        state.log.push("command");
    }
    async fn dma_push(&mut self, _: &mut (), data: &[u32], swap: bool) {
        assert_eq!(data.len(), 1);
        self.push(if swap { data[0].swap_bytes() } else { data[0] });
    }
}
impl Sm {
    fn set_enable(&mut self, _: bool) {}
    fn clear_fifos(&mut self) {}
    fn restart(&mut self) {}
    fn clkdiv_restart(&mut self) {}
    unsafe fn set_x(&mut self, v:u32) { self.rx.0.borrow_mut().x = v; }
    unsafe fn set_y(&mut self, v:u32) { self.rx.0.borrow_mut().y = v; }
    unsafe fn set_pindir(&mut self, v:u32) { assert_eq!(v, 1); }
    unsafe fn exec_jmp(&mut self, v:u8) { assert_eq!(v, 7); }
    fn rx(&mut self) -> &mut Rx { &mut self.rx }
    fn tx(&mut self) -> &mut Tx { &mut self.tx }
    fn rx_tx(&mut self) -> (&mut Rx, &mut Tx) { (&mut self.rx, &mut self.tx) }
}
struct Driver { sm: Sm, dma: (), wrap_target:u8 }
impl Driver { // ACTUAL_METHOD
}
fn main() {
    let swapped = std::env::args().nth(1).unwrap() == "swap";
    for count in [1, 4, 5, 20, 64] {
        let shared = Rc::new(RefCell::new(State::default()));
        let mut driver = Driver { sm: Sm { rx:Rx(shared.clone()), tx:Tx(shared.clone()) }, dma:(), wrap_target:7 };
        let cmd:u32 = 0x1234abcd;
        let mut data = vec![0; count];
        let status = {
            let mut future = pin!(driver.cmd_read(cmd, &mut data));
            match future.as_mut().poll(&mut Context::from_waker(Waker::noop())) {
                Poll::Ready(status) => status,
                Poll::Pending => panic!("mock should complete"),
            }
        };
        let state = shared.borrow();
        assert_eq!(state.log, ["data_arm", "command", "data_wait", "status_arm", "status_wait"]);
        assert_eq!(state.command, Some(if swapped { cmd.swap_bytes() } else { cmd }));
        assert_eq!((state.x, state.y), (31, count as u32 * 32 + 31));
        assert_eq!(status, if swapped { 0x12345679u32.swap_bytes() } else { 0x12345679 });
        for (i, value) in data.iter().enumerate() {
            let expected = 0xabcdef01u32.wrapping_add(i as u32 * 2);
            assert_eq!(*value, if swapped { expected.swap_bytes() } else { expected });
        }
    }
}
'''


def run(path, swap, baseline=False):
    with tempfile.TemporaryDirectory(prefix="pio-order-") as directory:
        rust = Path(directory) / "check.rs"
        executable = Path(directory) / "check"
        rust.write_text(MOCK.replace("// ACTUAL_METHOD", method(path)))
        subprocess.run(["rustc", "--edition=2024", "-Dwarnings", str(rust), "-o", str(executable)], check=True)
        result = subprocess.run([str(executable), "swap" if swap else "native"], capture_output=True, text=True)
        if baseline:
            assert result.returncode != 0 and "RX DMA not armed before command" in result.stderr, result.stderr
        else:
            assert result.returncode == 0, result.stderr


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source", type=Path, nargs="?",
                    default=Path(__file__).resolve().parents[2] / "cyw43-pio/src/lib.rs")
parser.add_argument("--swap", action="store_true")
parser.add_argument("--baseline", type=Path)
args = parser.parse_args()
run(args.source, args.swap)
if args.baseline:
    run(args.baseline, False, baseline=True)
print("Actual cmd_read arms RX before command; lengths/data/status pass; old ordering rejected when supplied.")
