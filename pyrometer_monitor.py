#!/usr/bin/env python3
"""
AST Pyrometer Monitor — Unified MT500 & Modbus RTU Protocol
"""

import serial
import time
import sys
from datetime import datetime

# ─── USER CONFIGURATION ──────────────────────────────────────────────────────
SERIAL_PORT     = "COM3"          # Windows: "COM3" | Linux/macOS: "/dev/ttyUSB0"
BAUD_RATE       = 19200
TIMEOUT         = 0.5             # Serial read timeout (seconds)
POLL_INTERVAL   = 2.0             # Seconds between full scan cycles
STATION_IDS     = [1, 2, 3, 4]    # Ensure these match your sensor IDs

# Toggle between protocols here: "MODBUS" or "MT500"
ACTIVE_PROTOCOL = "MODBUS"        
# ─────────────────────────────────────────────────────────────────────────────

# Table 1 — tau index to human-readable response time
RESPONSE_TIME_MAP = {
    1: "2 ms", 3: "6 ms", 5: "10 ms", 10: "20 ms",
    30: "60 ms", 50: "100 ms", 100: "200 ms", 300: "600 ms",
    500: "1 s", 1000: "2 s", 3000: "6 s", 5000: "10 s",
}

# ANSI colours for terminal output
USE_COLOR = sys.stdout.isatty()
def _c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if USE_COLOR else text

GREEN  = lambda t: _c("92", t)
YELLOW = lambda t: _c("93", t)
RED    = lambda t: _c("91", t)
BOLD   = lambda t: _c("1",  t)
DIM    = lambda t: _c("2",  t)


# ═════════════════════════════════════════════════════════════════════════════
# ─── MODBUS RTU IMPLEMENTATION ───────────────────────────────────────────────
# ═════════════════════════════════════════════════════════════════════════════

def modbus_crc16(data: bytes) -> bytes:
    """Calculate the Modbus RTU CRC-16."""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def read_registers_modbus(ser: serial.Serial, station_id: int, address: int, num_items: int) -> tuple:
    """Read Holding Registers (Function Code 03) via Modbus RTU."""
    # Build Modbus Request: [ID] [FC:03] [Addr Hi] [Addr Lo] [Num Hi] [Num Lo]
    req = bytearray([
        station_id, 
        0x03, 
        (address >> 8) & 0xFF, 
        address & 0xFF, 
        (num_items >> 8) & 0xFF, 
        num_items & 0xFF
    ])
    req += modbus_crc16(req)
    
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()

    # Modbus response length: ID(1) + FC(1) + ByteCount(1) + Data(2*N) + CRC(2)
    expected_len = 5 + (2 * num_items)
    resp = ser.read(expected_len)
    
    if len(resp) < 5:
        # Check if it's a Modbus Exception (Function Code + 0x80)
        if len(resp) >= 3 and resp[1] == 0x83:
            return None, f"Modbus Exception Code {resp[2]}"
        return None, "No response (Timeout)"
        
    if modbus_crc16(resp[:-2]) != resp[-2:]:
        return None, "Modbus CRC mismatch"
        
    values = []
    for i in range(num_items):
        # Extract 16-bit register values
        val = (resp[3 + i*2] << 8) | resp[4 + i*2]
        values.append(val)
        
    return values, None


# ═════════════════════════════════════════════════════════════════════════════
# ─── MT500 IMPLEMENTATION ────────────────────────────────────────────────────
# ═════════════════════════════════════════════════════════════════════════════

def _verify_mt500_checksum(data: bytes) -> bool:
    if len(data) < 4: return False
    try:
        computed = sum(data[1:-2]) & 0xFF
        received = int(data[-2:].decode("ascii"), 16)
        return computed == received
    except Exception:
        return False

def _read_frame_mt500(ser: serial.Serial) -> bytes:
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b: return bytes()
        if b in (b'\x02', b'\x15'):
            buf.extend(b)
            break
    while True:
        b = ser.read(1)
        if not b: return bytes(buf)
        buf.extend(b)
        if b == b'\x03': break
    buf.extend(ser.read(2))
    return bytes(buf)

def read_registers_mt500(ser: serial.Serial, station_id: int, address: int, num_items: int) -> tuple:
    payload = f"{station_id:02X}RD{address:04X}{num_items:02X}"
    etx     = bytes([0x03])
    chksum  = sum(payload.encode("ascii") + etx) & 0xFF
    cmd     = bytes([0x02]) + payload.encode("ascii") + etx + f"{chksum:02X}".encode("ascii")
    
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()

    response = _read_frame_mt500(ser)
    if not response: return None, "No response (Timeout)"
    
    if response[0] == 0x15: return None, f"MT500 NAK Error"
    if not _verify_mt500_checksum(response): return None, "MT500 Checksum mismatch"
    
    values = []
    for i in range(num_items):
        start = 5 + i * 4
        try:
            values.append(int(response[start : start + 4].decode("ascii"), 16))
        except Exception as exc:
            return None, f"Parse error: {exc}"
            
    return values, None


# ═════════════════════════════════════════════════════════════════════════════
# ─── HARDWARE ABSTRACTION LAYER ──────────────────────────────────────────────
# ═════════════════════════════════════════════════════════════════════════════

def read_pyrometer(ser: serial.Serial, station_id: int) -> dict:
    """Reads pyrometer data routing to the correct protocol register map."""
    r: dict = {"station": station_id, "errors": []}

    def _get(address, n, label, use_modbus_address=None):
        addr = use_modbus_address if ACTIVE_PROTOCOL == "MODBUS" else address
        
        if ACTIVE_PROTOCOL == "MODBUS":
            vals, err = read_registers_modbus(ser, station_id, addr, n)
        else:
            vals, err = read_registers_mt500(ser, station_id, addr, n)
            
        if err:
            r["errors"].append(f"[0x{addr:04X}] {label}: {err}")
        return vals

    if ACTIVE_PROTOCOL == "MODBUS":
        # Modbus Register Map
        vals = _get(0x0000, 1, "Temp K", use_modbus_address=0x0000)
        if vals:
            r["temp_k"] = vals[0]
            r["temp_c"] = round(vals[0] - 273.15, 2)
        else:
            r["temp_k"] = r["temp_c"] = None
            
        it = _get(0x0006, 1, "Internal Temp", use_modbus_address=0x0006)
        r["internal_temp_c"] = it[0] if it else None
        
        ht = _get(0x0007, 1, "Head Temp", use_modbus_address=0x0007)
        r["head_temp_c"] = round(ht[0] / 10.0, 2) if ht else None  # Modbus specifies x10 multiplier
        
        em = _get(0x03FC, 1, "Emissivity", use_modbus_address=0x03FC)
        r["emissivity"] = round(em[0] / 1000, 3) if em else None
        
        rt = _get(0x0104, 1, "Response Time", use_modbus_address=0x0104)
        r["response_time"] = RESPONSE_TIME_MAP.get(rt[0], f"τ={rt[0]}") if rt else None

        ls = _get(0x0EF1, 1, "Laser", use_modbus_address=0x0EF1)
        r["laser"] = "ON" if (ls and ls[0] == 1) else "OFF"
        
        r["status_desc"] = "Modbus OK" if vals else "N/A"

    else:
        # MT500 Register Map (from previous implementation)
        vals = _get(0x0000, 2, "Temp + Status")
        if vals:
            r["temp_k"] = vals[0]
            r["temp_c"] = round(vals[0] - 273.15, 2)
            r["status_desc"] = f"Code {vals[1]:#06x}"
        else:
            r["temp_k"] = r["temp_c"] = r["status_desc"] = None
            
        it = _get(0x0006, 2, "Int/Head Temp")
        if it:
            r["internal_temp_c"] = it[0]
            r["head_temp_c"] = round(it[1] / 1000, 2)
        else:
            r["internal_temp_c"] = r["head_temp_c"] = None
            
        em = _get(0x0400, 1, "Emissivity")
        r["emissivity"] = round(em[0] / 1000, 3) if em else None
        
        rt = _get(0x0105, 1, "Response Time")
        r["response_time"] = RESPONSE_TIME_MAP.get(rt[0], f"τ={rt[0]}") if rt else None
        
        ls = _get(0x0F00, 1, "Laser")
        r["laser"] = "ON" if (ls and ls[0] == 1) else "OFF"

    return r

# ═════════════════════════════════════════════════════════════════════════════
# ─── DISPLAY LOGIC ───────────────────────────────────────────────────────────
# ═════════════════════════════════════════════════════════════════════════════

_W = 56
def _row(label: str, value: str) -> str:
    return f"  │  {f'{label:<18}'}: {value}"

def print_pyrometer_block(d: dict) -> None:
    sid = d["station"]
    header = BOLD(f" Pyrometer  #  {sid} ")
    print(f"  ┌{'─' * 5}{header}{'─' * (_W - 5 - len(f' Pyrometer  #  {sid} '))}┐")

    tc, tk = d.get("temp_c"), d.get("temp_k")
    temp_str = f"{tc:>9.2f} °C  /  {tk} K" if tc is not None else RED("N/A")
    print(_row("Temperature", temp_str))

    print(_row("Status", d.get('status_desc', 'N/A')))

    it, ht = d.get("internal_temp_c"), d.get("head_temp_c")
    print(_row("Internal Temp", f"{it} °C" if it is not None else DIM("N/A")))
    print(_row("Head Temp", f"{ht} °C" if ht is not None else DIM("N/A")))

    em = d.get("emissivity")
    print(_row("Emissivity", f"{em}" if em is not None else DIM("N/A")))

    print(_row("Response Time τ", d.get("response_time") or DIM("N/A")))
    print(_row("Laser",           d.get("laser")         or DIM("N/A")))

    if d["errors"]:
        print(f"  │")
        for err in d["errors"]:
            print(f"  │  {RED('✗')} {err}")

    print(f"  └{'─' * (_W + 4)}┘")


def print_summary(all_data: list) -> None:
    temps = [d["temp_c"] for d in all_data if d.get("temp_c") is not None]
    online = len(temps)

    print(f"\n  {BOLD('Summary')}  —  {online}/{len(all_data)} sensors online", end="")
    if temps:
        print(f"   min {GREEN(f'{min(temps):.1f}°C')}   avg {BOLD(f'{sum(temps)/len(temps):.1f}°C')}   max {RED(f'{max(temps):.1f}°C')}")
    else:
        print(f"  {RED('No data')}")


def main() -> None:
    print(BOLD(f"\nAST Pyrometer RS485 Monitor"))
    print(f"Port: {SERIAL_PORT}  │  Baud: {BAUD_RATE}  │  Protocol: {ACTIVE_PROTOCOL}")
    print("─" * 50)

    try:
        ser = serial.Serial(
            port=SERIAL_PORT, baudrate=BAUD_RATE, bytesize=8,
            parity='N', stopbits=1, timeout=TIMEOUT
        )
    except serial.SerialException as exc:
        print(RED(f"\nCould not open {SERIAL_PORT}: {exc}"))
        sys.exit(1)

    print(f"Opened {SERIAL_PORT}  ✓   Polling every {POLL_INTERVAL}s   (Ctrl+C to stop)\n")

    try:
        while True:
            ts = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
            print(f"\n══ {BOLD(ts)} ══")

            all_data = []
            for sid in STATION_IDS:
                data = read_pyrometer(ser, sid)
                all_data.append(data)
                print_pyrometer_block(data)

            print_summary(all_data)
            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print(f"\n\n{BOLD('Stopped.')}  Closing {SERIAL_PORT}.")
    finally:
        if ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
