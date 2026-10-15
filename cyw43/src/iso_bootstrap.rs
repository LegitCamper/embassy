// Private startup operations for the guarded ISO installer.
// ponytail: startup-only SRAM installer; add a serialized runner request channel
// when installation must coexist with active Wi-Fi or Bluetooth host tasks.

#[cfg(feature = "bluetooth")]
fn runtime_memory_range(address: u32, length: usize, write: bool) -> bool {
    let Ok(length) = u32::try_from(length) else {
        return false;
    };
    let Some(end) = address.checked_add(length) else {
        return false;
    };
    length != 0
        && length <= 256
        && (address | length) & 3 == 0
        && ((address >= 0x200000 && end <= 0x21e000)
            || (address >= 0xd0f90 && end <= 0xd1000)
            || (!write && end <= 0xd1000))
}

#[cfg(feature = "bluetooth")]
impl<'a, BUS: Bus> Runner<'a, BUS> {
    /// Read aligned controller SRAM/ROM before `run`; no other host task may run.
    /// Errors or cancellation during bootstrap require a controller reset.
    pub(crate) async fn bluetooth_runtime_read(
        &mut self,
        address: u32,
        data: &mut [u8],
    ) -> Result<(), embedded_io_async::ErrorKind> {
        use embedded_io_async::ErrorKind;
        if !runtime_memory_range(address, data.len(), false) {
            return Err(ErrorKind::InvalidInput);
        }
        let bt = self.bt.as_mut().ok_or(ErrorKind::NotConnected)?;
        embassy_time::with_timeout(Duration::from_millis(500), bt.bt_bus_request(&mut self.bus))
            .await
            .map_err(|_| ErrorKind::TimedOut)?;
        self.bus.bp_read(0x19000000 + address, data).await;
        Ok(())
    }

    /// Write and read back aligned controller SRAM/patch scratch before `run`.
    /// Caller must establish allocation and hook ownership. No MMIO is allowed.
    /// Errors or cancellation require reset; bytes may already have changed.
    pub(crate) async fn bluetooth_runtime_write(
        &mut self,
        address: u32,
        data: &[u8],
    ) -> Result<(), embedded_io_async::ErrorKind> {
        use embedded_io_async::ErrorKind;
        if !runtime_memory_range(address, data.len(), true) {
            return Err(ErrorKind::InvalidInput);
        }
        let bt = self.bt.as_mut().ok_or(ErrorKind::NotConnected)?;
        embassy_time::with_timeout(Duration::from_millis(500), bt.bt_bus_request(&mut self.bus))
            .await
            .map_err(|_| ErrorKind::TimedOut)?;
        let mut actual = [0u8; 64];
        for (index, bytes) in data.chunks(64).enumerate() {
            let at = 0x19000000 + address + index as u32 * 64;
            self.bus.bp_write(at, bytes).await;
            self.bus.bp_read(at, &mut actual[..bytes.len()]).await;
            if actual[..bytes.len()] != *bytes {
                return Err(ErrorKind::InvalidData);
            }
        }
        Ok(())
    }

    /// Execute one HCI command before `run`, returning its Command Complete event.
    /// Exclusive startup use only: no queued host traffic, connections, or scan.
    /// Unexpected packets, lost credits, timeout or cancellation require reset.
    pub(crate) async fn bluetooth_runtime_command(
        &mut self,
        command: &[u8],
        response: &mut [u8; 260],
    ) -> Result<usize, embedded_io_async::ErrorKind> {
        use embedded_io_async::ErrorKind;
        if command.len() < 3 || command.len() != usize::from(command[2]) + 3 {
            return Err(ErrorKind::InvalidInput);
        }
        let bt = self.bt.as_mut().ok_or(ErrorKind::NotConnected)?;
        if bt.addr == 0 || bt.addr & 3 != 0 || bt.addr.checked_add(BTSDIO_OFFSET_BT2HOST_OUT + 4).is_none() {
            return Err(ErrorKind::InvalidData);
        }
        if bt.tx_chan.try_receive().is_some() {
            return Err(ErrorKind::InvalidInput);
        }
        embassy_time::with_timeout(Duration::from_millis(500), bt.bt_bus_request(&mut self.bus))
            .await
            .map_err(|_| ErrorKind::TimedOut)?;
        let output = self.bus.bp_read32(bt.addr + BTSDIO_OFFSET_HOST2BT_OUT).await;
        let incoming = self.bus.bp_read32(bt.addr + BTSDIO_OFFSET_BT2HOST_IN).await;
        if (output | incoming | bt.h2b_write_pointer | bt.b2h_read_pointer) & 3 != 0
            || output >= BTSDIO_FWBUF_SIZE
            || incoming >= BTSDIO_FWBUF_SIZE
            || output != bt.h2b_write_pointer
            || incoming != bt.b2h_read_pointer
        {
            return Err(ErrorKind::InvalidData);
        }
        let mut packet = [0; 264];
        let length = command.len();
        packet[..4].copy_from_slice(&[length as u8, (length >> 8) as u8, 0, 1]);
        packet[4..4 + length].copy_from_slice(command);
        let total = 4 + length.div_ceil(4) * 4;
        let first = total.min((BTSDIO_FWBUF_SIZE - bt.h2b_write_pointer) as usize);
        self.bus
            .bp_write(
                bt.addr + BTSDIO_OFFSET_HOST_WRITE_BUF + bt.h2b_write_pointer,
                &packet[..first],
            )
            .await;
        if first != total {
            self.bus
                .bp_write(bt.addr + BTSDIO_OFFSET_HOST_WRITE_BUF, &packet[first..total])
                .await;
        }
        bt.h2b_write_pointer = (bt.h2b_write_pointer + total as u32) % BTSDIO_FWBUF_SIZE;
        self.bus
            .bp_write32(bt.addr + BTSDIO_OFFSET_HOST2BT_IN, bt.h2b_write_pointer)
            .await;
        bt.bt_toggle_intr(&mut self.bus).await;
        for _ in 0..500 {
            let incoming = self.bus.bp_read32(bt.addr + BTSDIO_OFFSET_BT2HOST_IN).await;
            if incoming >= BTSDIO_FWBUF_SIZE || incoming & 3 != 0 {
                return Err(ErrorKind::InvalidData);
            }
            let available = incoming.wrapping_sub(bt.b2h_read_pointer) % BTSDIO_FWBUF_SIZE;
            if available >= 4 {
                let mut header = [0; 4];
                self.bus
                    .bp_read(bt.addr + BTSDIO_OFFSET_HOST_READ_BUF + bt.b2h_read_pointer, &mut header)
                    .await;
                let length = u32::from_le_bytes([header[0], header[1], header[2], 0]) as usize;
                let padded = length.div_ceil(4) * 4;
                if header[3] != 4 || !(6..=257).contains(&length) {
                    return Err(ErrorKind::InvalidData);
                }
                if available as usize >= padded + 4 {
                    let start = (bt.b2h_read_pointer + 4) % BTSDIO_FWBUF_SIZE;
                    let first = padded.min((BTSDIO_FWBUF_SIZE - start) as usize);
                    self.bus
                        .bp_read(bt.addr + BTSDIO_OFFSET_HOST_READ_BUF + start, &mut response[..first])
                        .await;
                    if first != padded {
                        self.bus
                            .bp_read(bt.addr + BTSDIO_OFFSET_HOST_READ_BUF, &mut response[first..padded])
                            .await;
                    }
                    if response[0] != 0x0e
                        || usize::from(response[1]) + 2 != length
                        || response[2] == 0
                        || response[3..5] != command[..2]
                    {
                        return Err(ErrorKind::InvalidData);
                    }
                    bt.b2h_read_pointer = (start + padded as u32) % BTSDIO_FWBUF_SIZE;
                    self.bus
                        .bp_write32(bt.addr + BTSDIO_OFFSET_BT2HOST_OUT, bt.b2h_read_pointer)
                        .await;
                    bt.bt_toggle_intr(&mut self.bus).await;
                    return Ok(length);
                }
            }
            Timer::after_millis(1).await;
        }
        Err(ErrorKind::TimedOut)
    }
}

#[cfg(test)]
mod runtime_tests {
    use super::runtime_memory_range;
    #[test]
    fn refuses_mmio_config_overflow_and_unaligned_access() {
        assert!(runtime_memory_range(0x203a34, 4, true));
        assert!(runtime_memory_range(0xd0f90, 112, true));
        assert!(runtime_memory_range(0x8b634, 44, false));
        for (address, length) in [
            (0x318b70, 4),
            (0x21e000, 4),
            (0xfffffff0, 32),
            (0x203a35, 4),
            (0x203a34, 3),
            (0x203a34, 0),
            (0x203a34, 260),
            (0xd035c, 4),
        ] {
            assert!(!runtime_memory_range(address, length, true));
        }
    }
}
