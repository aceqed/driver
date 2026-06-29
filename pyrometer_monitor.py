#!/usr/bin/env python3
"""
AST Pyrometer Monitor — MT500_AST Protocol
Polls 4 pyrometers over RS485 and prints all readings.

Hardware setup  : All sensors share one RS485 bus (half-duplex).
Serial settings : 19200 baud, 8 data bits, 1 stop bit, no parity.
Protocol        : MT500_AST — only RD (Batch Read) commands are used.

Usage:
    pip install pyserial
    python pyrometer_monitor.py
"""

import serial
import time
import sys
from datetime import datetime

# ─── USER CONFIGURATION ──────────────────────────────────────────────────────
SERIAL_PORT   = "COM3"          # Windows: "COM3" | Linux/macOS: "/dev/ttyUSB0"
BAUD_RATE     = 19200
TIMEOUT       = 0.5             # Serial read timeout (seconds)
POLL_INTERVAL = 2.0             # Seconds between full scan cycles
STATION_IDS   = [1, 2, 3, 4]   # Station IDs of your 4 pyrometers
# ─────────────────────────────────────────────────────────────────────────────

# Appendix A — status codes
STATUS_CODES = {
    0x0000: ("OK",   "No error"),
    0x0001: ("WARN", "Signal lower than sensor sensitivity"),
    0x0002: ("WARN", "Out of range (T brightness minimum)"),
    0x0003: ("WARN", "Too low energy"),
    0x0004: ("WARN", "Signal higher than sensor sensitivity"),
    0x0006: ("WARN", "Sharp brightness jump"),
    0x0007: ("WARN", "Non-stable object measurement"),
    0x0011: ("WARN", "Internal temperature warning"),
    0x0013: ("WARN", "Thermopile ambient temperature too low"),
    0x0014: ("WARN", "Thermopile ambient temperature too high"),
    0x0015: ("INFO", "Pyrometer in testing mode"),
    0x0016: ("INFO", "Pilot light ON"),
    0x0017: ("WARN", "Measurement below lower basic range"),
    0x0018: ("WARN", "Measurement exceeds upper basic range"),
    0x0019: ("WARN", "Pyrometer in warm-up period"),
}

# Error codes returned in NAK replies
NAK_ERROR_CODES = {
    "1": "Invalid checksum",
    "2": "Unknown command",
    "3": "Data length error (items mismatch)",
    "4": "ETX not found",
    "5": "Illegal address or 0 items requested",
    "6": "More than 99 items requested",
    "7": "Unsuccessful write (retry)",
}

# Table 1 — tau index to human-readable response time
RESPONSE_TIME_MAP = {
    1: "2 ms", 3: "6 ms", 5: "10 ms", 10: "20 ms",
    30: "60 ms", 50: "100 ms", 100: "200 ms", 300: "600 ms",
    500: "1 s", 1000: "2 s", 3000: "6 s", 5000: "10 s",
}

# ANSI colours (disabled automatically on non-TTY)
USE_COLOR = sys.stdout.isatty()

def _c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if USE_COLOR else text

GREEN  = lambda t: _c("92", t)
YELLOW = lambda t: _c("93", t)
RED    = lambda t: _c("91", t)
BOLD   = lambda t: _c("1",  t)
DIM    = lambda t: _c("2",  t)


# ─── MT500 PROTOCOL HELPERS ──────────────────────────────────────────────────

def _build_rd(station_id: int, address: int, num_items: int) -> bytes:
    """
    Build a Batch Read (RD) command packet.

    Frame layout (all ASCII except STX/ETX):
      [STX] [StationID 2-hex] [RD] [Address 4-hex] [Items 2-hex] [ETX] [Checksum 2-hex]

    Checksum = lowest byte of sum(bytes 2..L-2, 1-indexed) — i.e. everything
    between STX and the checksum itself, including ETX.
    """
    payload = f"{station_id:02X}RD{address:04X}{num_items:02X}"
    etx     = bytes([0x03])
    chksum  = sum(payload.encode("ascii") + etx) & 0xFF
    return bytes([0x02]) + payload.encode("ascii") + etx + f"{chksum:02X}".encode("ascii")


def _verify_reply_checksum(data: bytes, expected_len: int) -> bool:
    """Verify the checksum embedded in an RD reply."""
    try:
        # Checksum covers bytes 2..L-2 (1-indexed) = indices 1..(L-3) (0-indexed)
        computed = sum(data[1 : expected_len - 2]) & 0xFF
        received = int(data[expected_len - 2 : expected_len].decode("ascii"), 16)
        return computed == received
    except Exception:
        return False


def read_registers(
    ser: serial.Serial,
    station_id: int,
    address: int,
    num_items: int,
) -> tuple:
    """
    Send an RD command and return (list_of_int_values, None) on success
    or (None, error_string) on failure.

    Reply length formula from spec: L = (N × 4) + 8
    Data words (4 ASCII hex chars each) start at byte index 5 (0-indexed).
    """
    expected_len = (num_items * 4) + 8

    cmd = _build_rd(station_id, address, num_items)
    ser.reset_input_buffer()
    ser.write(cmd)

    # Spec mandates a 5 ms slave delay; we wait a bit longer to be safe
    time.sleep(0.05)
    response = ser.read(expected_len + 4)  # small extra buffer

    if not response:
        return None, "No response (timeout — check wiring / station ID)"

    # NAK reply: [NAK=0x15][Station 2][Cmd 2][ErrorCode 1]
    if response[0] == 0x15:
        code = chr(response[5]) if len(response) >= 6 else "?"
        desc = NAK_ERROR_CODES.get(code, "Unknown error")
        return None, f"NAK error {code}: {desc}"

    if response[0] != 0x02:
        return None, f"Unexpected start byte: {response[0]:#04x}"

    if len(response) < expected_len:
        return None, f"Short reply: got {len(response)}, need {expected_len} bytes"

    if not _verify_reply_checksum(response, expected_len):
        return None, "Checksum mismatch in reply"

    # Parse N data words (each is 4 ASCII hex chars) starting at index 5
    values = []
    for i in range(num_items):
        start = 5 + i * 4
        try:
            values.append(int(response[start : start + 4].decode("ascii"), 16))
        except Exception as exc:
            return None, f"Parse error at item {i}: {exc}"

    return values, None


# ─── HIGH-LEVEL PYROMETER READS ──────────────────────────────────────────────

def read_pyrometer(ser: serial.Serial, station_id: int) -> dict:
    """
    Read all useful registers from one pyrometer.
    Returns a dict — failed reads are stored as None with an error note.
    """
    r: dict = {"station": station_id, "errors": []}

    def _get(address, n, label):
        vals, err = read_registers(ser, station_id, address, n)
        if err:
            r["errors"].append(f"[0x{address:04X}] {label}: {err}")
        return vals

    # ── Object temperature (°K) + status code ─────────────────────────────
    #    Address 0x0000 → temperature in °K
    #    Address 0x0001 → status code  (Appendix A)
    vals = _get(0x0000, 2, "Temp + Status")
    if vals:
        temp_k = vals[0]
        status_raw = vals[1]
        r["temp_k"]       = temp_k
        r["temp_c"]       = round(temp_k - 273.15, 2)
        r["temp_f"]       = round(temp_k * 9 / 5 - 459.67, 2)
        status_info       = STATUS_CODES.get(status_raw)
        r["status_code"]  = status_raw
        r["status_label"] = status_info[0] if status_info else "UNKNOWN"
        r["status_desc"]  = status_info[1] if status_info else f"Code {status_raw:#06x}"
    else:
        for k in ("temp_k", "temp_c", "temp_f", "status_code", "status_label", "status_desc"):
            r[k] = None

    # ── Relative energy (2-colour sensors; stored × 1000) ─────────────────
    vals = _get(0x0002, 1, "Rel. energy")
    r["relative_energy"] = round(vals[0] / 1000, 3) if vals else None

    # ── Internal case temp (°C) + optical head temp (m°C) ─────────────────
    vals = _get(0x0006, 2, "Int/Head temp")
    if vals:
        r["internal_temp_c"] = vals[0]
        r["head_temp_c"]     = round(vals[1] / 1000, 3)   # m°C → °C
    else:
        r["internal_temp_c"] = r["head_temp_c"] = None

    # ── Emissivity + slope (stored × 1000) ────────────────────────────────
    vals = _get(0x0400, 2, "Emissivity")
    if vals:
        r["emissivity"]       = round(vals[0] / 1000, 3)
        r["emissivity_slope"] = round(vals[1] / 1000, 3)
    else:
        r["emissivity"] = r["emissivity_slope"] = None

    # ── Response time τ ───────────────────────────────────────────────────
    vals = _get(0x0105, 1, "Response time")
    if vals:
        r["response_time"] = RESPONSE_TIME_MAP.get(vals[0], f"τ={vals[0]}")
    else:
        r["response_time"] = None

    # ── Clear time setting ────────────────────────────────────────────────
    vals = _get(0x0303, 1, "Clear time")
    if vals:
        ct = vals[0]
        r["clear_time"] = "OFF" if ct == 0 else "Auto" if ct == 1 else f"{10 * ct} ms"
    else:
        r["clear_time"] = None

    # ── Laser on/off ──────────────────────────────────────────────────────
    vals = _get(0x0F00, 1, "Laser")
    if vals:
        r["laser"] = "ON" if vals[0] == 1 else "OFF"
    else:
        r["laser"] = None

    return r


# ─── DISPLAY ─────────────────────────────────────────────────────────────────

_W = 56  # box width

def _row(label: str, value: str) -> str:
    label_col = f"{label:<18}"
    return f"  │  {label_col}: {value}"


def print_pyrometer_block(d: dict) -> None:
    sid = d["station"]

    # Colour the status badge
    lbl = d.get("status_label") or "?"
    if lbl == "OK":
        badge = GREEN(f"[{lbl}]")
    elif lbl in ("WARN", "INFO"):
        badge = YELLOW(f"[{lbl}]")
    else:
        badge = RED(f"[{lbl}]")

    header = BOLD(f" Pyrometer  #  {sid} ")
    print(f"  ┌{'─' * 5}{header}{'─' * (_W - 5 - len(f' Pyrometer  #  {sid} '))}┐")

    # Temperature
    tc = d.get("temp_c")
    tf = d.get("temp_f")
    tk = d.get("temp_k")
    if tc is not None:
        temp_str = f"{tc:>9.2f} °C  /  {tf:>9.2f} °F  /  {tk} K"
    else:
        temp_str = RED("N/A")
    print(_row("Temperature", temp_str))

    # Status
    status_str = f"{badge}  {d.get('status_desc', 'N/A')}"
    print(_row("Status", status_str))

    # Internal / head temperature
    it = d.get("internal_temp_c")
    ht = d.get("head_temp_c")
    it_str = f"{it} °C" if it is not None else DIM("N/A")
    ht_str = f"{ht} °C" if ht is not None else DIM("N/A")
    print(_row("Internal Temp", it_str))
    print(_row("Head Temp", ht_str))

    # Emissivity
    em = d.get("emissivity")
    es = d.get("emissivity_slope")
    em_str = f"{em}" if em is not None else DIM("N/A")
    es_str = f"slope {es}" if es is not None else DIM("N/A")
    print(_row("Emissivity", f"{em_str}   {es_str}"))

    # Relative energy
    re = d.get("relative_energy")
    print(_row("Relative Energy", f"{re}" if re is not None else DIM("N/A (single-colour)")))

    # Misc
    print(_row("Response Time τ", d.get("response_time") or DIM("N/A")))
    print(_row("Clear Time",      d.get("clear_time")    or DIM("N/A")))
    print(_row("Laser",           d.get("laser")         or DIM("N/A")))

    # Errors
    if d["errors"]:
        print(f"  │")
        for err in d["errors"]:
            print(f"  │  {RED('✗')} {err}")

    print(f"  └{'─' * (_W + 4)}┘")


def print_summary(all_data: list) -> None:
    temps = [d["temp_c"] for d in all_data if d.get("temp_c") is not None]
    online = sum(1 for d in all_data if d.get("temp_c") is not None)

    print(f"\n  {BOLD('Summary')}  —  {online}/{len(all_data)} sensors online", end="")
    if temps:
        print(
            f"   min {GREEN(f'{min(temps):.1f}°C')} "
            f"  avg {BOLD(f'{sum(temps)/len(temps):.1f}°C')} "
            f"  max {RED(f'{max(temps):.1f}°C')}"
        )
    else:
        print(f"  {RED('No temperature data available')}")


def print_header(ts: str) -> None:
    title = f"  AST Pyrometer Monitor  │  {ts}  "
    border = "═" * (len(title) + 2)
    print(f"\n{border}")
    print(BOLD(title))
    print(border)


# ─── MAIN ────────────────────────────────────────────────────────────────────

def main() -> None:
    print(BOLD(f"\nAST Pyrometer RS485 Monitor"))
    print(f"Port: {SERIAL_PORT}  │  Baud: {BAUD_RATE}  │  Stations: {STATION_IDS}")
    print("─" * 50)

    try:
        ser = serial.Serial(
            port      = SERIAL_PORT,
            baudrate  = BAUD_RATE,
            bytesize  = serial.EIGHTBITS,
            parity    = serial.PARITY_NONE,
            stopbits  = serial.STOPBITS_ONE,
            timeout   = TIMEOUT,
        )
    except serial.SerialException as exc:
        print(RED(f"\nCould not open {SERIAL_PORT}: {exc}"))
        print("Check SERIAL_PORT at the top of the script.")
        sys.exit(1)

    print(f"Opened {SERIAL_PORT}  ✓   Polling every {POLL_INTERVAL}s   (Ctrl+C to stop)\n")

    try:
        while True:
            ts = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
            print_header(ts)

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
