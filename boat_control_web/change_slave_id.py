#!/usr/bin/env python3
"""
Modbus Slave ID changer for level transmitter.

Usage:
    python change_slave_id.py <port> <current_id> <new_id>
    python change_slave_id.py /dev/ttyAMA2 1 2
"""

import sys
import time

import serial


def modbus_crc16(data: bytes) -> int:
    """Modbus CRC-16 (polynomial 0xA001)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read_request(slave_id: int, start_addr: int, num_regs: int) -> bytes:
    """Build a Modbus RTU Read Holding Registers (0x03) request frame."""
    req = bytearray()
    req.append(slave_id)
    req.append(0x03)  # function code: read holding registers
    req.append((start_addr >> 8) & 0xFF)
    req.append(start_addr & 0xFF)
    req.append((num_regs >> 8) & 0xFF)
    req.append(num_regs & 0xFF)
    crc = modbus_crc16(bytes(req))
    req.append(crc & 0xFF)
    req.append((crc >> 8) & 0xFF)
    return bytes(req)


def build_write_request(slave_id: int, reg_addr: int, value: int) -> bytes:
    """Build a Modbus RTU Write Single Register (0x06) request frame."""
    req = bytearray()
    req.append(slave_id)
    req.append(0x06)  # function code: write single register
    req.append((reg_addr >> 8) & 0xFF)
    req.append(reg_addr & 0xFF)
    req.append((value >> 8) & 0xFF)
    req.append(value & 0xFF)
    crc = modbus_crc16(bytes(req))
    req.append(crc & 0xFF)
    req.append((crc >> 8) & 0xFF)
    return bytes(req)


def hex_str(data: bytes) -> str:
    """Format bytes as hex string."""
    return " ".join(f"{b:02X}" for b in data)


def read_slave_id(ser: serial.Serial, current_id: int) -> int | None:
    """
    Read the current slave address from register 0x0000.
    Returns the address, or None on failure.
    """
    addr_reg = 0x0000
    req = build_read_request(current_id, addr_reg, 1)
    ser.reset_input_buffer()
    ser.write(req)
    print(f"  TX [{len(req)}]: {hex_str(req)}")

    # Wait for response (response = 7 bytes)
    time.sleep(0.05)
    resp = ser.read(7)
    print(f"  RX [{len(resp)}]: {hex_str(resp)}")

    if len(resp) < 7:
        print(f"  ERROR: Short response ({len(resp)} bytes, expected 7)")
        return None

    if resp[0] != current_id:
        print(f"  ERROR: Slave ID mismatch in response (expected {current_id}, got {resp[0]})")
        return None

    if resp[1] & 0x80:
        print(f"  ERROR: Modbus exception (code {resp[2]})")
        return None

    if resp[1] != 0x03:
        print(f"  ERROR: Unexpected function code 0x{resp[1]:02X}")
        return None

    if resp[2] != 2:
        print(f"  ERROR: Unexpected byte count {resp[2]}")
        return None

    # Validate CRC
    crc_received = resp[5] | (resp[6] << 8)
    crc_calc = modbus_crc16(resp[:5])
    if crc_received != crc_calc:
        print(f"  ERROR: CRC mismatch (received 0x{crc_received:04X}, calculated 0x{crc_calc:04X})")
        return None

    slave_id = (resp[3] << 8) | resp[4]
    return slave_id


def write_slave_id(ser: serial.Serial, current_id: int, new_id: int) -> bool:
    """
    Write a new slave address to register 0x0000.
    Returns True on success.
    """
    addr_reg = 0x0000
    req = build_write_request(current_id, addr_reg, new_id)
    ser.reset_input_buffer()
    ser.write(req)
    print(f"  TX [{len(req)}]: {hex_str(req)}")

    # Wait for response (response = 8 bytes, same as request)
    time.sleep(0.05)
    resp = ser.read(8)
    print(f"  RX [{len(resp)}]: {hex_str(resp)}")

    if len(resp) < 8:
        print(f"  ERROR: Short response ({len(resp)} bytes, expected 8)")
        return False

    if resp[0] != current_id:
        print(f"  ERROR: Slave ID mismatch (expected {current_id}, got {resp[0]})")
        return False

    # Echo: byte[0]=id, byte[1]=0x06, byte[2:4]=addr, byte[4:6]=value, byte[6:8]=CRC
    reg_echo = (resp[2] << 8) | resp[3]
    val_echo = (resp[4] << 8) | resp[5]

    if reg_echo != addr_reg:
        print(f"  ERROR: Register echo mismatch (expected 0x{addr_reg:04X}, got 0x{reg_echo:04X})")
        return False

    if val_echo != new_id:
        print(f"  ERROR: Value echo mismatch (expected {new_id}, got {val_echo})")
        return False

    print(f"  OK: Slave ID changed from {current_id} to {new_id}")
    return True


def write_save_command(ser: serial.Serial, slave_id: int) -> bool:
    """
    Save current settings to user area (register 0x000F, value 0).
    Must be called AFTER the device has switched to the new slave ID.
    Returns True on success.
    """
    save_reg = 0x000F
    req = build_write_request(slave_id, save_reg, 0x0000)
    ser.reset_input_buffer()
    ser.write(req)
    print(f"  TX [{len(req)}]: {hex_str(req)}")

    time.sleep(0.05)
    resp = ser.read(8)
    print(f"  RX [{len(resp)}]: {hex_str(resp)}")

    if len(resp) < 8:
        print(f"  ERROR: Short response ({len(resp)} bytes, expected 8)")
        return False

    if resp[0] != slave_id:
        print(f"  ERROR: Slave ID mismatch (expected {slave_id}, got {resp[0]})")
        return False

    if resp[1] & 0x80:
        print(f"  ERROR: Modbus exception (code {resp[2] if len(resp) > 2 else '?'})")
        return False

    reg_echo = (resp[2] << 8) | resp[3]

    if reg_echo != save_reg:
        print(f"  ERROR: Register echo mismatch (expected 0x{save_reg:04X}, got 0x{reg_echo:04X})")
        return False

    print(f"  OK: Settings saved to user area (register 0x{save_reg:04X})")
    return True


def main():
    if len(sys.argv) != 4:
        print(f"Usage: python {sys.argv[0]} <port> <current_slave_id> <new_slave_id>")
        print(f"Example: python {sys.argv[0]} /dev/ttyAMA3 1 2")
        sys.exit(1)

    port = sys.argv[1]
    current_id = int(sys.argv[2])
    new_id = int(sys.argv[3])

    if new_id < 1 or new_id > 255:
        print("ERROR: Slave ID must be in range [1, 255]")
        sys.exit(1)

    if current_id == new_id:
        print("ERROR: Current ID and new ID are the same — nothing to do.")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  Modbus Slave ID Changer — Level Transmitter")
    print(f"{'='*60}")
    print(f"  Port:      {port}")
    print(f"  Old ID:    {current_id}")
    print(f"  New ID:    {new_id}")
    print(f"{'='*60}\n")

    # ── 1. Open serial port ──
    print("[1/4] Opening serial port...")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )
        print(f"  OK: {port} opened @ 9600, 8N1")
    except Exception as e:
        print(f"  ERROR: Failed to open {port}: {e}")
        sys.exit(1)

    # ── 2. Read current slave ID ──
    print(f"\n[2/4] Verifying current slave ID (register 0x0000)...")
    read_id = read_slave_id(ser, current_id)
    if read_id is None:
        print(f"\n  FAILED to read current slave ID.")
        print(f"  Make sure:")
        print(f"    - The device is powered and connected to {port}")
        print(f"    - The sensor is the ONLY device on this bus (disconnect BMS if needed)")
        print(f"    - The current slave ID is actually {current_id}")
        ser.close()
        sys.exit(1)
    print(f"  OK: Current slave ID = {read_id}")

    if read_id != current_id:
        print(f"  WARNING: Expected {current_id} but device reports {read_id}")
        resp = input(f"  Continue with actual ID {read_id}? [y/N]: ").strip().lower()
        if resp != 'y':
            ser.close()
            sys.exit(1)
        current_id = read_id

    # ── 3. Write new slave ID ──
    print(f"\n[3/4] Writing new slave ID = {new_id}...")
    ok = write_slave_id(ser, current_id, new_id)
    if not ok:
        ser.close()
        print(f"\n  FAILED to change slave ID.")
        sys.exit(1)

    # ── 4. Save settings with new slave ID ──
    # The device responds to the write command with the OLD address,
    # then immediately switches to the NEW address.
    # The save command must be sent using the NEW address.
    print(f"\n[4/4] Saving settings to user area (register 0x000F)...")
    time.sleep(0.1)  # brief pause to let the device switch to new ID
    ok = write_save_command(ser, new_id)
    ser.close()

    if not ok:
        print(f"\n  FAILED to save settings. The slave ID may have changed but won't persist after power cycle.")
        print(f"  Try running again, or manually send: {new_id:02X} 06 00 0F 00 00 [CRC]")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  SUCCESS!")
    print(f"  Slave ID changed: {current_id} → {new_id}  (saved to non-volatile memory)")
    print(f"")
    print(f"  Next steps:")
    print(f"  1. Verify LEVEL_SLAVE_ID = {new_id} in config.py")
    print(f"  2. Restart the application to test")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
