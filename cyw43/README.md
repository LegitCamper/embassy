# cyw43

Rust driver for the CYW43439 wifi+bluetooth chip. Implementation based on [Infineon/wifi-host-driver](https://github.com/Infineon/wifi-host-driver).

Works on the following boards:

- Raspberry Pi Pico W (RP2040)
- Raspberry Pi Pico 2 W (RP2350A)
- Pimoroni Pico Plus 2 W (RP2350B)
- Any board with Raspberry Pi RM2 radio module.
- Any board with the CYW43439 chip, and possibly others if the protocol is similar enough.

## Features

Working:

- WiFi support
    - Station mode (joining an AP).
    - AP mode (creating an AP)
    - Scanning
    - Sending and receiving Ethernet frames.
    - Using the default MAC address.
    - [`embassy-net`](https://embassy.dev) integration.
    - RP2040 PIO driver for the nonstandard half-duplex SPI used in the Pico W.
    - Using IRQ for device events, no busy polling.
    - GPIO support (for LED on the Pico W).
- Bluetooth support
    - Bluetooth Classic + LE HCI commands.
    - Concurrent operation with WiFi.
    - Implements the [bt-hci](https://crates.io/crates/bt-hci) controller traits.
    - Works with the [TrouBLE](https://github.com/embassy-rs/trouble) bluetooth LE stack. Check its repo for examples using `cyw43`.

## Running the WiFi examples

Experimental fixed-format CYW43439 ISO broadcast support is opt-in through
`bluetooth-iso`. See the experimental ISO section below for its exact firmware,
startup, raw transport, and reset requirements. It does not implement a general
LE Audio controller or normal TrouBLE host startup.

- Install `probe-rs` following the instructions at <https://probe.rs>.
- `cd examples/rp`
### Example 1: Scan the wifi stations
- `cargo run --release --bin wifi_scan`
### Example 2: Create an access point (IP and credentials in the code)
- `cargo run --release --bin wifi_ap_tcp_server`
### Example 3: Connect to an existing network and create a server
- `cargo run --release --bin wifi_tcp_server`

After a few seconds, you should see that DHCP picks up an IP address like this
```
11.944489 DEBUG Acquired IP configuration:
11.944517 DEBUG    IP address:      192.168.0.250/24
11.944620 DEBUG    Default gateway: 192.168.0.33
11.944722 DEBUG    DNS server 0:    192.168.0.33
```
This example implements a TCP echo server on port 1234. You can try connecting to it with:
```
nc 192.168.0.250 1234
```
Send it some data, you should see it echoed back and printed in the firmware's logs.

## Experimental CYW43439 ISO image

`bluetooth-iso` adds a startup-only guarded installer for CYW43439 firmware
`.0065`. It supports one unencrypted broadcast stream: handle `0x0042`, mono
8 kHz LC3, 10 ms frames, 26-byte complete SDUs, four HCI ISO credits.
Continuous discovery permits late joins. No codec, audio application, CIS,
framed ISOAL, standard BIG creation commands, or LE Audio profiles are included.

Initialize `new_with_bluetooth` using the stock signed `.0065` configuration
with the validated 112-byte zero append at controller address `0x000d0f90`.
The installer refuses unknown or nonzero scratch; it never clears arbitrary
controller memory. Required boot image SHA256:
`b3c0c99774135c32beea47af847838077a1f0b7a6be661a68ddcd3641362ad7d`.
The proprietary stock image is not redistributed here.

Before spawning the runner or any host/Wi-Fi task:

```rust,ignore
let mut iso = cyw43::iso::install(&mut runner).await?;
iso.start(&mut runner).await?;
// Spawn runner.run(), then send complete ISO SDUs through BtDriver.
```

Track HCI Number Of Completed Packets for handle `0x0042`; send at most four
outstanding SDUs. Sequence numbers advance modulo 65536. An HCI Reset after
start invalidates the session. Normal TrouBLE initialization sends that reset
and is unsupported. Use raw `BtDriver` transport. Concurrent Wi-Fi use remains
unverified. After the runner starts, controller reset is the recovery boundary.
Before it starts, `iso.restore(&mut runner)` restores owned hooks and stops radio;
resident memory stays allocated until reset. Every installer/start error or
cancelled operation requires chip restart and firmware reload, even if rollback
succeeded. HCI Reset alone does not reclaim memory or reliably remove RAM hooks.

Native assets are referenced directly from sibling `../cyw43_iso/work/iso-blob-continuous`; no blob copies live in this checkout. Generate them first with `cyw43_iso/analysis/check-controller-tx.py
--continuous --output-blob work/iso-blob-continuous` from the associated
`hardware/controller_audio.S`, `hardware/transport_callback.S`,
`hardware/discovery_packet.c`, and `hardware/iso_credits.S` sources.
ABI version 1 uses a controller-owned 4096-byte resident allocation and a
112-byte temporary allocator. Firmware guards include raw Infineon manufacturer
`0x0131`, LMP subversion `0x2310`, stock entry bytes/pointers, idle radio state,
zero scratch, and allocator bounds. BTstack rewrites the manufacturer to Broadcom
`0x000f` in its event buffer; this raw transport does not.

| Asset | SHA256 |
| --- | --- |
| `native.bin` | `be1197f5d05de61a28ba9bebb681882076c749f977931cbbebd35e87cdd92923` |
| `allocator.bin` | `59d2135189219b28fb276ffe74baeca3b68c06b0639fc7a168092f97fc2071e1` |

SPI backplane reads use 16 response-delay bytes with this feature, matching Pico
SDK; stock four-byte delay returns stale controller words. Bulk accesses set the
32-bit access flag and restore the window to CHIPCOMMON after every call. The
sibling `cyw43-pio` driver arms RX DMA before transmitting read commands, so the
response delay cannot fill the four-word FIFO before DMA starts. Installer
writes use verified chunks of at most 64 bytes. SDIO behavior is unchanged.

Live RP2040 validation used the separate `rp2040_audio` application with the
normal `Runner::run`: 1000 SDUs sent and credited, including 80-ms and 250-ms
host pauses. A Nordic SDC receiver discovered the stream over air, synchronized
to its BIG, and reassembled 950 exact fixture-matching SDUs. Capture and validation:
`../cyw43_iso/work/embassy-normal-runner-discovery-sdc-probe`. Repeated fixtures
prove payload validity and cyclic order, not absolute host-frame identity.
This validates SDU transport, not LC3 decoding. Both test boards were restored
to stock firmware after capture. RP2350 support is compile-checked only.

Inbound HCI type 5 already reaches `BtDriver::read` as
`ControllerToHostPacket::Iso`. A complete 26-byte SDU needs 35 receive-buffer
bytes including the packet indicator, or 39 with a timestamp. A short buffer
returns `InvalidInput` without consuming the packet; malformed HCI envelopes
are consumed and rejected. The four-packet RX channel must be drained promptly.
The `bt-hci` 0.7 `iso_sdu_len` field includes raw status bits: inspect `>> 14`,
mask length with `0x0fff`, reject reserved bits/status, and check payload length
before treating an SDU as valid. This transport does not reassemble ISO fragments
or decode LC3. `install` and `Installed::start` retain their transmit behavior.

### One-shot RX diagnostic

`install_rx_diagnostic(&mut runner)` returns `InstalledRxDiagnostic`. Call its
`start(&mut runner)` before running `Runner::run` and reading `BtDriver`. This
uses a separate checked 1024-byte image and allocator from
`../cyw43_iso/work/iso-blob-rx`, referenced directly without copies. Generate the
assets from the sibling project before building with `bluetooth-iso`:

```sh
uv run --with unicorn python analysis/check-controller-rx.py --bis --iso --emulate --output-blob work/iso-blob-rx
```

RX uses fixed logical channel 0 at 2404 MHz, AA `0x718ebed6`, CRC init
`0x123401`, and whitening. One valid 26-byte snapshot becomes one complete HCI
ISO packet on handle `0x0042`, sequence 0. There is no BIG synchronization,
hopping, continuous reception, LC3 decoding, or negotiated receive API.
Send Read Local Version Information every 250 ms through `BtDriver` and drain
incoming packets: transport task delivery currently needs host-assisted wake.
Start the receiver and normal runner before starting the peer transmitter;
unexpected ISO traffic during startup command exchange requires a chip restart.

The sibling `rp2040_audio` app selects this mode with `--features rx-diagnostic`.
Its fixture is bytes `00..19`; it checks every HCI header/payload byte, counts
exactly one packet, stops radio, verifies stock version, and rejects late
packets. The 80 wake cycles use nominal 250-ms waits plus command time, not a
measured 20-second RF window. Silent/wrong-CRC controls report `count=0` and
`pass=false`; their command, stop, version and quiet checks must still pass.
After `Runner::run` starts, hooks remain installed and storage remains allocated
until chip restart; stopping the test radio alone is not restoration.

| RX asset | SHA256 |
| --- | --- |
| `native.bin` | `8dfcf7eee1ad5b54aec1022794f183442f146a58ad56359890ecdcf66b13f610` |
| `allocator.bin` | `4217cca9174a3d829861c58dab5343f9642f41d1b84e1494ebe526f51bfb8a58` |

Live RP2040 normal-runner evidence is in
`../cyw43_iso/work/embassy-normal-rx-series`, using ELF SHA256
`ad919957749c31dd74dcae84bfdfd74854f8d6e8ab233d2a386fa8d937ac7e97`.
The valid marker delivered exactly one SDU; silent and wrong-CRC controls
delivered zero. All three passed command, stop, stock-version, late-packet quiet,
and external stock-controller checks. The hardware owner also verified Nordic
RRAM/UICR restoration. This establishes one-shot delivery, not BIS sync.

### Repeated RX diagnostic

`install_rx_stream_diagnostic(&mut runner)` uses a separate 1024-byte image
from `../cyw43_iso/work/iso-blob-rx-stream`, exported with
`check-controller-rx.py --bis --iso --stream`. Radio callbacks wake the stock
transport task, so the host sends no HCI commands while receiving; just drain
`BtDriver`. Identity, handle and SDU format match the one-shot mode. Sequence
numbers count accepted SDUs and are not synchronized BIG event counters.
One payload mailbox is owned until the HCI copy succeeds: newer packets are
dropped while it is busy, and a full host ring retries on a later radio
callback. All startup, reset and restoration restrictions above still apply.

| Stream RX asset | SHA256 |
| --- | --- |
| `native.bin` | `e8ba26ee19b006d48af04cee2b5183782d508d579fd43d363b05925b03b7451f` |
| `allocator.bin` | `4217cca9174a3d829861c58dab5343f9642f41d1b84e1494ebe526f51bfb8a58` |

Live RP2040 evidence is in
`../cyw43_iso/work/embassy-normal-rx-stream-series`, ELF SHA256
`4e52daa158e94948e6e94fb950cdd2fe7d0dd92278e48196b4cb96f44145cd43`.
The app (`rp2040_audio --features rx-stream`) received 500 of 500 distinct
SDUs with matching payload indices and HCI sequences; silent and wrong-CRC
controls received none. All three passed stop, stock-version, quiet-drain,
external stock-controller and Nordic RRAM/UICR restoration checks. This
establishes repeated fixed-channel reception, not BIG synchronization,
hopping, encryption or LE Audio profiles.

Offline installer check, run from `cyw43/`:

```sh
rustc --edition 2024 --test tests/iso-installer.rs -o /tmp/cyw43-iso-check
/tmp/cyw43-iso-check
python3 tests/check-spi-padding.py
python3 tests/check-pio-read-order.py
cargo test --lib --features bluetooth-iso --target x86_64-unknown-linux-gnu
cargo check --features bluetooth-iso --target thumbv6m-none-eabi
```
