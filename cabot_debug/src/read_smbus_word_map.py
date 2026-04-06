#!/usr/bin/env python3

import argparse
import select
import socket
import struct
import sys
import time
from dataclasses import asdict, dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple


ADDR_SMBUS_REQ = 0x10F
ADDR_SMBUS_RES = 0x521
ADDR_BAT_BASE = 0x518
ADDR_BAT_CAP_BASE = 0x51C
ADDR_BAT_SN = 0x520
CAN_SFF_MASK = 0x7FF
CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)


def decode_hex(value: int) -> str:
    return f"0x{value:04X}"


def decode_signed(value: int) -> str:
    return str(struct.unpack("<h", struct.pack("<H", value))[0])


def decode_minutes(value: int) -> str:
    return "N/A" if value == 0xFFFF else f"{value} min"


def decode_capacity(value: int) -> str:
    return f"{value} mAh"


def decode_voltage(value: int) -> str:
    return f"{value} mV"


def decode_specification_info(value: int) -> str:
    version = (value >> 4) & 0x0F
    revision = (value >> 8) & 0x0F
    vscale = "10mV" if (value & 0x01) else "1mV"
    ipscale = "10mA/10mW" if (value & 0x02) else "1mA/1mW"
    return (
        f"v{version}.{revision}, "
        f"voltage scale={vscale}, "
        f"current/power scale={ipscale}"
    )


def decode_manufacture_date(value: int) -> str:
    day = value & 0x1F
    month = (value >> 5) & 0x0F
    year = 1980 + ((value >> 9) & 0x7F)
    if month == 0 or day == 0:
        return f"invalid ({year:04d}-{month:02d}-{day:02d})"
    return f"{year:04d}-{month:02d}-{day:02d}"


def decode_battery_mode(value: int) -> str:
    flags = []
    bit_names = {
        15: "internal_charge_controller",
        14: "primary_battery_support",
        13: "condition_flag",
        9: "charge_controller_enabled",
        8: "primary_battery",
        7: "alarm_mode",
        1: "capacity_in_mw",
        0: "internal_charger_enabled",
    }
    for bit, name in bit_names.items():
        if value & (1 << bit):
            flags.append(name)
    if not flags:
        flags.append("none")
    return ", ".join(flags)


def decode_battery_status(value: int) -> str:
    flags = []
    error_code = (value >> 8) & 0x0F
    error_names = {
        0x0: "ok",
        0x1: "busy",
        0x2: "reserved_cmd",
        0x3: "unsupported_cmd",
        0x4: "access_denied",
        0x5: "over_underflow",
        0x6: "bad_size",
        0x7: "unknown_error",
    }
    bit_names = {
        15: "over_charged_alarm",
        14: "terminate_charge_alarm",
        12: "over_temp_alarm",
        11: "terminate_discharge_alarm",
        9: "remaining_capacity_alarm",
        8: "remaining_time_alarm",
        7: "initialized",
        6: "discharging",
        5: "fully_charged",
        4: "fully_discharged",
    }
    for bit, name in bit_names.items():
        if value & (1 << bit):
            flags.append(name)
    if not flags:
        flags.append("none")
    return f"error={error_names.get(error_code, f'0x{error_code:X}')}, flags={', '.join(flags)}"


@dataclass(frozen=True)
class RegisterDef:
    addr: int
    name: str
    unit: str
    decoder: Callable[[int], str]


REGISTER_DEFS: List[RegisterDef] = [
    RegisterDef(0x01, "RemainingCapacityAlarm", "mAh", decode_capacity),
    RegisterDef(0x02, "RemainingTimeAlarm", "min", decode_minutes),
    RegisterDef(0x03, "BatteryMode", "-", decode_battery_mode),
    RegisterDef(0x04, "AtRate", "mA", decode_signed),
    RegisterDef(0x05, "AtRateTimeToFull", "min", decode_minutes),
    RegisterDef(0x06, "AtRateTimeToEmpty", "min", decode_minutes),
    RegisterDef(0x07, "AtRateOK", "bool", lambda value: "true" if value else "false"),
    RegisterDef(0x0B, "AverageCurrent", "mA", decode_signed),
    RegisterDef(0x0C, "MaxError", "%", lambda value: f"{value}%"),
    RegisterDef(0x0E, "AbsoluteStateOfCharge", "%", lambda value: f"{value}%"),
    RegisterDef(0x11, "RunTimeToEmpty", "min", decode_minutes),
    RegisterDef(0x12, "AverageTimeToEmpty", "min", decode_minutes),
    RegisterDef(0x13, "AverageTimeToFull", "min", decode_minutes),
    RegisterDef(0x14, "ChargingCurrent", "mA", lambda value: f"{value} mA"),
    RegisterDef(0x15, "ChargingVoltage", "mV", decode_voltage),
    RegisterDef(0x16, "BatteryStatus", "-", decode_battery_status),
    RegisterDef(0x17, "CycleCount", "count", str),
    RegisterDef(0x19, "DesignVoltage", "mV", decode_voltage),
    RegisterDef(0x1A, "SpecificationInfo", "-", decode_specification_info),
    RegisterDef(0x1B, "ManufactureDate", "-", decode_manufacture_date),
]


@dataclass(frozen=True)
class RegisterReadResult:
    location: int
    addr: int
    name: str
    raw: Optional[int]
    value: Optional[int]
    decoded: str
    status: str

    def to_dict(self) -> dict:
        data = asdict(self)
        data["raw_hex"] = decode_hex(self.raw) if self.raw is not None else None
        return data


@dataclass(frozen=True)
class BatteryStateSnapshot:
    location: int
    serial_number: Optional[str]
    percentage: Optional[float]
    voltage: Optional[float]
    current: Optional[float]
    temperature: Optional[float]
    charge: Optional[float]
    capacity: Optional[float]
    design_capacity: Optional[float]

    def to_dict(self) -> dict:
        return asdict(self)


def build_can_frame(can_id: int, payload: bytes) -> bytes:
    data = payload.ljust(8, b"\x00")
    return struct.pack(CAN_FRAME_FORMAT, can_id, len(payload), data)


def parse_can_frame(frame: bytes) -> Tuple[int, bytes]:
    can_id, can_dlc, data = struct.unpack(CAN_FRAME_FORMAT, frame)
    can_id &= CAN_SFF_MASK
    return can_id, data[:can_dlc]


def _decode_optional_ratio(raw: int, scale: float) -> Optional[float]:
    if raw == 0xFFFF:
        return None
    return raw / scale


def _decode_optional_signed_current(raw: int) -> Optional[float]:
    if raw == 0x7FFF:
        return None
    value = struct.unpack("<h", struct.pack("<H", raw))[0]
    return value / 1000.0


def _decode_optional_temperature(raw: int) -> Optional[float]:
    if raw == 0xFFFF:
        return None
    return raw / 10.0 - 273.15


def _decode_optional_voltage(raw: int) -> Optional[float]:
    if raw == 0xFFFF:
        return None
    return raw / 1000.0


def _decode_optional_ah(raw: int) -> Optional[float]:
    if raw == 0xFFFF:
        return None
    return raw / 1000.0


class BatteryStateCanClient:
    def __init__(self, interface: str, timeout: float):
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        filters = b"".join(
            [
                struct.pack("=II", ADDR_BAT_BASE + offset, CAN_SFF_MASK)
                for offset in range(4)
            ] +
            [
                struct.pack("=II", ADDR_BAT_CAP_BASE + offset, CAN_SFF_MASK)
                for offset in range(4)
            ] +
            [
                struct.pack("=II", ADDR_BAT_SN, CAN_SFF_MASK)
            ]
        )
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        self.socket.bind((interface,))

    def close(self) -> None:
        self.socket.close()

    def flush(self) -> None:
        while True:
            readable, _, _ = select.select([self.socket], [], [], 0)
            if not readable:
                return
            self.socket.recv(CAN_FRAME_SIZE)

    def read_snapshots(self, locations: List[int]) -> Dict[int, BatteryStateSnapshot]:
        wanted = set(locations)
        raw_states: Dict[int, dict] = {location: {} for location in locations}
        got_status = set()
        got_capacity = set()
        got_serial = False

        self.flush()
        deadline = time.time() + self.timeout

        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            readable, _, _ = select.select([self.socket], [], [], remaining)
            if not readable:
                break

            can_id, data = parse_can_frame(self.socket.recv(CAN_FRAME_SIZE))
            if ADDR_BAT_BASE <= can_id <= ADDR_BAT_BASE + 3 and len(data) == 8:
                location = can_id - ADDR_BAT_BASE + 1
                if location in wanted:
                    raw_states[location]["voltage_raw"] = data[0] | (data[1] << 8)
                    raw_states[location]["current_raw"] = data[2] | (data[3] << 8)
                    raw_states[location]["percentage_raw"] = data[4] | (data[5] << 8)
                    raw_states[location]["temperature_raw"] = data[6] | (data[7] << 8)
                    got_status.add(location)
            elif ADDR_BAT_CAP_BASE <= can_id <= ADDR_BAT_CAP_BASE + 3 and len(data) == 6:
                location = can_id - ADDR_BAT_CAP_BASE + 1
                if location in wanted:
                    raw_states[location]["charge_raw"] = data[0] | (data[1] << 8)
                    raw_states[location]["capacity_raw"] = data[2] | (data[3] << 8)
                    raw_states[location]["design_capacity_raw"] = data[4] | (data[5] << 8)
                    got_capacity.add(location)
            elif can_id == ADDR_BAT_SN and len(data) == 8:
                for location in wanted:
                    offset = (location - 1) * 2
                    serial_raw = data[offset] | (data[offset + 1] << 8)
                    raw_states[location]["serial_number"] = f"{serial_raw:04x}"
                got_serial = True

            if got_serial and got_status == wanted and got_capacity == wanted:
                break

        snapshots = {}
        for location in locations:
            state = raw_states[location]
            snapshots[location] = BatteryStateSnapshot(
                location=location,
                serial_number=state.get("serial_number"),
                percentage=_decode_optional_ratio(state["percentage_raw"], 100.0) if "percentage_raw" in state else None,
                voltage=_decode_optional_voltage(state["voltage_raw"]) if "voltage_raw" in state else None,
                current=_decode_optional_signed_current(state["current_raw"]) if "current_raw" in state else None,
                temperature=_decode_optional_temperature(state["temperature_raw"]) if "temperature_raw" in state else None,
                charge=_decode_optional_ah(state["charge_raw"]) if "charge_raw" in state else None,
                capacity=_decode_optional_ah(state["capacity_raw"]) if "capacity_raw" in state else None,
                design_capacity=_decode_optional_ah(state["design_capacity_raw"]) if "design_capacity_raw" in state else None,
            )
        return snapshots


class SMBusCanClient:
    def __init__(self, interface: str, timeout: float):
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_filter = struct.pack("=II", ADDR_SMBUS_RES, CAN_SFF_MASK)
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, can_filter)
        self.socket.bind((interface,))

    def close(self) -> None:
        self.socket.close()

    def flush(self) -> None:
        while True:
            readable, _, _ = select.select([self.socket], [], [], 0)
            if not readable:
                return
            self.socket.recv(CAN_FRAME_SIZE)

    def read_word(self, location: int, addr: int) -> Optional[int]:
        self.flush()
        request = build_can_frame(ADDR_SMBUS_REQ, bytes([location, addr]))
        self.socket.send(request)

        while True:
            readable, _, _ = select.select([self.socket], [], [], self.timeout)
            if not readable:
                return None

            can_id, data = parse_can_frame(self.socket.recv(CAN_FRAME_SIZE))
            if can_id != ADDR_SMBUS_RES or len(data) != 4:
                continue
            if data[0] != location or data[1] != addr:
                continue
            return data[2] | (data[3] << 8)


def parse_locations(text: str) -> List[int]:
    locations = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        value = int(part, 10)
        if not 1 <= value <= 4:
            raise argparse.ArgumentTypeError("locations must be in range 1..4")
        locations.append(value)
    if not locations:
        raise argparse.ArgumentTypeError("at least one location is required")
    return locations


def read_registers(client: SMBusCanClient, locations: List[int], timeout: float) -> List[RegisterReadResult]:
    results: List[RegisterReadResult] = []
    for location in locations:
        for register_def in REGISTER_DEFS:
            value = client.read_word(location, register_def.addr)
            if value is None:
                results.append(
                    RegisterReadResult(
                        location=location,
                        addr=register_def.addr,
                        name=register_def.name,
                        raw=None,
                        value=None,
                        decoded=f"no response within {timeout:.3f}s",
                        status="timeout",
                    )
                )
                continue

            results.append(
                RegisterReadResult(
                    location=location,
                    addr=register_def.addr,
                    name=register_def.name,
                    raw=value,
                    value=value,
                    decoded=register_def.decoder(value),
                    status="ok",
                )
            )
    return results


def read_battery_state_snapshots(interface: str, locations: List[int], timeout: float) -> Dict[int, BatteryStateSnapshot]:
    client = BatteryStateCanClient(interface, timeout)
    try:
        return client.read_snapshots(locations)
    finally:
        client.close()


def format_row(columns: Iterable[str], widths: List[int]) -> str:
    return " | ".join(col.ljust(width) for col, width in zip(columns, widths))


def render_location_table(location: int, rows: List[List[str]]) -> str:
    header = ["Addr", "Name", "Raw", "Value", "Decoded"]
    all_rows = [header] + rows
    widths = [max(len(row[i]) for row in all_rows) for i in range(len(header))]
    lines = [f"Location {location}", format_row(header, widths), format_row(["-" * w for w in widths], widths)]
    lines.extend(format_row(row, widths) for row in rows)
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read all SMBus word registers supported by the power board firmware over CAN."
    )
    parser.add_argument("--interface", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument(
        "--locations",
        type=parse_locations,
        default=[1, 2, 3, 4],
        help="Comma-separated battery locations to query (default: 1,2,3,4)",
    )
    parser.add_argument("--timeout", type=float, default=0.5, help="Response timeout in seconds (default: 0.5)")
    args = parser.parse_args()

    client = SMBusCanClient(args.interface, args.timeout)
    try:
        results = read_registers(client, args.locations, args.timeout)
        for location in args.locations:
            rows: List[List[str]] = []
            for result in [item for item in results if item.location == location]:
                rows.append(
                    [
                        f"0x{result.addr:02X}",
                        result.name,
                        decode_hex(result.raw) if result.raw is not None else "timeout",
                        str(result.value) if result.value is not None else "-",
                        result.decoded,
                    ]
                )

            print(render_location_table(location, rows))
            if location != args.locations[-1]:
                print()
    except KeyboardInterrupt:
        return 130
    except OSError as exc:
        print(f"SocketCAN error on {args.interface}: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
