#!/usr/bin/env python3
"""PC host for the Waveshare USB-CAN-A and SmartBias CAN protocol.

The adapter is a USB-serial-CAN device, not a SocketCAN/python-can interface.
This module implements Waveshare's documented variable-length framing. PySerial
is required only for real hardware; codec self-tests have no dependencies.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import struct
import sys
import time
from pathlib import Path
from typing import Callable, Iterable, Optional


CAN_ID_COMMAND = 0x100
CAN_ID_CONFIG_A = 0x101
CAN_ID_CONFIG_B = 0x102
CAN_ID_PID_CONFIG = 0x103
CAN_ID_ACK = 0x180
CAN_ID_STATUS = 0x181
CAN_ID_INA = 0x182
CAN_ID_POINT_A = 0x200
CAN_ID_POINT_B = 0x201
CAN_ID_EVENT = 0x280

CMD_PING = 0x01
CMD_GET_STATUS = 0x02
CMD_START_SWEEP = 0x03
CMD_ABORT = 0x04
CMD_GET_POINT = 0x05
CMD_GET_INA = 0x06
CMD_CLEAR_DATA = 0x07
CMD_START_PID_FF = 0x08
CMD_STOP_OUTPUT = 0x09

STATE_ARMED = 1
STATE_DATA_READY = 4
STATE_FAULT = 6

EVENT_SWEEP_COMPLETE = 3
EVENT_SOFT_CURRENT_STOP = 4
EVENT_SOFT_TEMPERATURE_STOP = 5
EVENT_FAULT = 7

RESULT_OK = 0
PROTOCOL_VERSION = 1

LOCKED_15V_EXPERIMENTS = {
    "pid-no-ff": (0, 100.0, 2015, 0.0),
    "temp-limiter": (1, 100.0, 2015, 0.0),
    "pid-ff": (2, 100.0, 2015, 4.0),
    "open-loop": (3, 100.0, 2015, 0.0),
}

LOCKED_FINAL_MATRIX_EXPERIMENTS = {
    "10": {
        "pid-no-ff": (0, 50.0, 1934, 0.0),
        "temp-limiter": (1, 50.0, 1934, 0.0),
        "pid-ff": (2, 50.0, 1934, 3.0),
        "open-loop": (3, 50.0, 1934, 0.0),
    },
    "15": {
        "pid-no-ff": (0, 100.0, 2062, 0.0),
        "temp-limiter": (1, 100.0, 2062, 0.0),
        "pid-ff": (2, 100.0, 2062, 4.0),
        "open-loop": (3, 100.0, 2062, 0.0),
    },
}

MATRIX_MODE_FILE_STEMS = {
    "pid-no-ff": "01_PID_NO_FF",
    "temp-limiter": "02_TEMP_LIMITER",
    "pid-ff": "03_PID_WITH_NTC_FF",
    "open-loop": "04_OPEN_LOOP",
}

CALIBRATION_CURRENT_STOPS_MA = {"10": 70.0, "15": 120.0}


@dataclasses.dataclass(frozen=True)
class CanFrame:
    identifier: int
    data: bytes
    extended: bool = False
    remote: bool = False

    def __post_init__(self) -> None:
        max_identifier = 0x1FFFFFFF if self.extended else 0x7FF
        if not 0 <= self.identifier <= max_identifier:
            raise ValueError("CAN identifier out of range")
        if len(self.data) > 8:
            raise ValueError("Classic CAN payload exceeds 8 bytes")


def encode_variable_frame(frame: CanFrame) -> bytes:
    """Encode one frame using Waveshare's AA ... 55 variable protocol."""

    type_byte = 0xC0 | len(frame.data)
    if frame.extended:
        type_byte |= 0x20
        identifier = frame.identifier.to_bytes(4, "little")
    else:
        identifier = frame.identifier.to_bytes(2, "little")
    if frame.remote:
        type_byte |= 0x10
    return bytes((0xAA, type_byte)) + identifier + frame.data + bytes((0x55,))


class VariableFrameParser:
    """Incremental parser resilient to partial reads and unrelated bytes."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> list[CanFrame]:
        self._buffer.extend(chunk)
        frames: list[CanFrame] = []
        while True:
            try:
                start = self._buffer.index(0xAA)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 2:
                break

            type_byte = self._buffer[1]
            if (type_byte & 0xC0) != 0xC0:
                del self._buffer[0]
                continue
            data_length = type_byte & 0x0F
            if data_length > 8:
                del self._buffer[0]
                continue
            extended = bool(type_byte & 0x20)
            remote = bool(type_byte & 0x10)
            identifier_length = 4 if extended else 2
            packet_length = 2 + identifier_length + data_length + 1
            if len(self._buffer) < packet_length:
                break
            if self._buffer[packet_length - 1] != 0x55:
                del self._buffer[0]
                continue

            identifier = int.from_bytes(
                self._buffer[2 : 2 + identifier_length], "little"
            )
            data_start = 2 + identifier_length
            data = bytes(self._buffer[data_start : data_start + data_length])
            frames.append(CanFrame(identifier, data, extended, remote))
            del self._buffer[:packet_length]
        return frames


def build_adapter_config_packet(
    can_bitrate: int = 500_000,
    *,
    variable_protocol: bool = True,
    standard_frames: bool = True,
    normal_mode: bool = True,
    auto_retransmit: bool = True,
) -> bytes:
    """Build Waveshare's documented 20-byte adapter configuration command."""

    bitrate_codes = {
        1_000_000: 0x01,
        800_000: 0x02,
        500_000: 0x03,
        400_000: 0x04,
        250_000: 0x05,
        200_000: 0x06,
        125_000: 0x07,
        100_000: 0x08,
        50_000: 0x09,
        20_000: 0x0A,
        10_000: 0x0B,
        5_000: 0x0C,
    }
    if can_bitrate not in bitrate_codes:
        raise ValueError("unsupported USB-CAN-A bitrate")

    packet = bytearray(20)
    packet[0:2] = b"\xAA\x55"
    packet[2] = 0x12 if variable_protocol else 0x02
    packet[3] = bitrate_codes[can_bitrate]
    packet[4] = 0x01 if standard_frames else 0x02
    # Filter ID and mask/block ID are zero: accept all during bring-up.
    packet[13] = 0x00 if normal_mode else 0x03
    packet[14] = 0x00 if auto_retransmit else 0x01
    packet[19] = sum(packet[2:19]) & 0xFF
    return bytes(packet)


def command_frame(opcode: int, token: int, arguments: bytes = b"") -> CanFrame:
    if len(arguments) > 6:
        raise ValueError("command arguments exceed six bytes")
    data = bytes((opcode & 0xFF, token & 0xFF)) + arguments
    return CanFrame(CAN_ID_COMMAND, data.ljust(8, b"\x00"))


def config_a_frame(
    token: int, start_mv: int, stop_mv: int, step_mv: int
) -> CanFrame:
    data = struct.pack("<BBHHH", token & 0xFF, 0, start_mv, stop_mv, step_mv)
    return CanFrame(CAN_ID_CONFIG_A, data)


def config_b_frame(
    token: int,
    sample_count: int,
    settle_ms: int,
    sample_interval_ms: int,
    temperature_stop_c: int,
    current_stop_ma_x10: int,
) -> CanFrame:
    data = struct.pack(
        "<BBHBBH",
        token & 0xFF,
        sample_count,
        settle_ms,
        sample_interval_ms,
        temperature_stop_c,
        current_stop_ma_x10,
    )
    return CanFrame(CAN_ID_CONFIG_B, data)


def pid_config_frame(
    token: int,
    profile: int,
    target_current_ma_x10: int,
    base_dac_mv: int,
    temp_ff_mv_per_c_x100: int,
) -> CanFrame:
    data = struct.pack(
        "<BBHHH",
        token & 0xFF,
        profile & 0xFF,
        target_current_ma_x10,
        base_dac_mv,
        temp_ff_mv_per_c_x100,
    )
    return CanFrame(CAN_ID_PID_CONFIG, data)


def decode_application_frame(frame: CanFrame) -> dict[str, object]:
    data = frame.data
    if len(data) != 8:
        return {"kind": "unexpected_dlc", "id": frame.identifier, "data": data.hex()}
    if frame.identifier == CAN_ID_ACK:
        return {
            "kind": "ack",
            "opcode": data[0],
            "token": data[1],
            "result": data[2],
            "state": data[3],
            "faults": data[4],
            "detail": data[5],
            "version": data[6],
        }
    if frame.identifier == CAN_ID_STATUS:
        state, faults, dac_mv, current_x10, temperature_x10 = struct.unpack(
            "<BBHHh", data
        )
        return {
            "kind": "status",
            "state": state,
            "faults": faults,
            "dac_mv": dac_mv,
            "current_ma": current_x10 / 10.0,
            "temperature_c": temperature_x10 / 10.0,
        }
    if frame.identifier == CAN_ID_INA:
        valid, diagnostics, bus_mv, current_x10, power_mw = struct.unpack(
            "<BBHhH", data
        )
        return {
            "kind": "ina",
            "valid": bool(valid),
            "diagnostics": diagnostics,
            "bus_v": bus_mv / 1000.0,
            "current_ma": current_x10 / 10.0,
            "power_mw": power_mw,
        }
    if frame.identifier == CAN_ID_POINT_A:
        index, dac_mv, current_x10, temperature_x10 = struct.unpack("<HHHh", data)
        return {
            "kind": "point_a",
            "index": index,
            "dac_mv": dac_mv,
            "current_adc_ma": current_x10 / 10.0,
            "temperature_c": temperature_x10 / 10.0,
        }
    if frame.identifier == CAN_ID_POINT_B:
        index, bus_mv, current_x10, flags = struct.unpack("<HHhH", data)
        return {
            "kind": "point_b",
            "index": index,
            "ina_bus_v": bus_mv / 1000.0,
            "ina_current_ma": current_x10 / 10.0,
            "flags": flags,
            "ina_valid": bool(flags & 0x01),
        }
    if frame.identifier == CAN_ID_EVENT:
        point_count = int.from_bytes(data[3:5], "little")
        elapsed_100ms = int.from_bytes(data[5:7], "little")
        return {
            "kind": "event",
            "event": data[0],
            "state": data[1],
            "faults": data[2],
            "point_count": point_count,
            "elapsed_s": elapsed_100ms / 10.0,
            "version": data[7],
        }
    return {"kind": "unknown", "id": frame.identifier, "data": data.hex()}


class SerialCanAdapter:
    def __init__(self, port: str, baudrate: int = 2_000_000) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "PySerial is required for hardware access: python -m pip install pyserial"
            ) from exc
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05,
            write_timeout=1.0,
        )
        self._parser = VariableFrameParser()

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "SerialCanAdapter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def write_raw(self, packet: bytes) -> None:
        self._serial.write(packet)
        self._serial.flush()

    def send(self, frame: CanFrame) -> None:
        self.write_raw(encode_variable_frame(frame))

    def read_frames(self, duration_s: float = 0.1) -> list[CanFrame]:
        deadline = time.monotonic() + duration_s
        frames: list[CanFrame] = []
        while time.monotonic() < deadline:
            count = max(1, int(getattr(self._serial, "in_waiting", 0)))
            chunk = self._serial.read(count)
            if chunk:
                frames.extend(self._parser.feed(chunk))
            elif frames:
                break
        return frames


def wait_for(
    adapter: SerialCanAdapter,
    predicate: Callable[[dict[str, object]], bool],
    timeout_s: float,
    *,
    echo: bool = False,
) -> dict[str, object]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for frame in adapter.read_frames(0.1):
            decoded = decode_application_frame(frame)
            if echo:
                print(decoded)
            if predicate(decoded):
                return decoded
    raise TimeoutError("timed out waiting for expected CAN response")


def configure_adapter(adapter: SerialCanAdapter, can_bitrate: int = 500_000) -> None:
    adapter.write_raw(build_adapter_config_packet(can_bitrate))
    time.sleep(0.25)


def next_token() -> int:
    return int(time.monotonic_ns() >> 12) & 0xFF


def require_ack(
    adapter: SerialCanAdapter, opcode: int, token: int, timeout_s: float = 2.0
) -> dict[str, object]:
    ack = wait_for(
        adapter,
        lambda item: item.get("kind") == "ack"
        and item.get("opcode") == opcode
        and item.get("token") == token,
        timeout_s,
    )
    if ack.get("result") != RESULT_OK:
        raise RuntimeError(f"MCU rejected command/configuration: {ack}")
    return ack


def wait_until_physically_armed(
    adapter: SerialCanAdapter, timeout_s: float
) -> dict[str, object]:
    print("Press the NUCLEO USER button once; CAN cannot bypass physical arming.")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        token = next_token()
        adapter.send(command_frame(CMD_GET_STATUS, token))
        try:
            status = wait_for(
                adapter,
                lambda item: item.get("kind") == "status",
                1.0,
            )
        except TimeoutError:
            continue
        if status.get("state") == STATE_ARMED:
            return status
        if status.get("state") == STATE_FAULT:
            raise RuntimeError(f"MCU is fault-latched: {status}")
        time.sleep(0.25)
    raise TimeoutError("USER-button physical arm was not observed")


def retrieve_point(
    adapter: SerialCanAdapter,
    index: int,
    max_attempts: int = 5,
    response_timeout_s: float = 1.0,
    retry_delay_s: float = 0.05,
) -> dict[str, object]:
    missing = "ACK, point A, point B"
    for attempt in range(1, max_attempts + 1):
        token = next_token()
        adapter.send(command_frame(CMD_GET_POINT, token, struct.pack("<H", index)))
        ack: Optional[dict[str, object]] = None
        part_a: Optional[dict[str, object]] = None
        part_b: Optional[dict[str, object]] = None
        deadline = time.monotonic() + response_timeout_s
        while time.monotonic() < deadline and (
            ack is None or part_a is None or part_b is None
        ):
            for frame in adapter.read_frames(0.1):
                decoded = decode_application_frame(frame)
                if (
                    decoded.get("kind") == "ack"
                    and decoded.get("opcode") == CMD_GET_POINT
                    and decoded.get("token") == token
                ):
                    ack = decoded
                elif (
                    decoded.get("index") == index
                    and decoded.get("kind") == "point_a"
                ):
                    part_a = decoded
                elif (
                    decoded.get("index") == index
                    and decoded.get("kind") == "point_b"
                ):
                    part_b = decoded
        if ack is not None and ack.get("result") != RESULT_OK:
            raise RuntimeError(f"MCU rejected point {index}: {ack}")
        if ack is not None and part_a is not None and part_b is not None:
            return {**part_a, **part_b, "kind": "point"}

        missing_parts = []
        if ack is None:
            missing_parts.append("ACK")
        if part_a is None:
            missing_parts.append("point A")
        if part_b is None:
            missing_parts.append("point B")
        missing = ", ".join(missing_parts)
        if attempt < max_attempts:
            print(
                f"Retrying point {index}: missing {missing} "
                f"(attempt {attempt}/{max_attempts})"
            )
            time.sleep(retry_delay_s)

    raise TimeoutError(
        f"incomplete point response for index {index} after "
        f"{max_attempts} attempts; missing {missing}"
    )


def retrieve_points(
    adapter: SerialCanAdapter,
    point_count: int,
    load_ohm: float,
    sense_ohm: float,
) -> list[dict[str, object]]:
    points = []
    for index in range(point_count):
        points.append(
            add_estimates(retrieve_point(adapter, index), load_ohm, sense_ohm)
        )
        if (index + 1) < point_count:
            time.sleep(0.03)
    return points


def add_estimates(
    point: dict[str, object], load_ohm: float, sense_ohm: float
) -> dict[str, object]:
    current_ma = (
        float(point["ina_current_ma"])
        if point.get("ina_valid")
        else float(point["current_adc_ma"])
    )
    bus_v = float(point.get("ina_bus_v", math.nan))
    current_a = current_ma / 1000.0
    point["selected_current_ma"] = current_ma
    point["vds_est_v"] = (
        bus_v - current_a * (load_ohm + sense_ohm)
        if point.get("ina_valid")
        else math.nan
    )
    # This assumes unity gate-drive gain and is not a measured gate voltage.
    point["vgs_command_est_v"] = (
        float(point["dac_mv"]) / 1000.0 - current_a * sense_ohm
    )
    return point


CSV_FIELDS = [
    "index",
    "dac_mv",
    "current_adc_ma",
    "temperature_c",
    "ina_valid",
    "ina_bus_v",
    "ina_current_ma",
    "selected_current_ma",
    "vds_est_v",
    "vgs_command_est_v",
    "flags",
]


def write_csv(path: Path, points: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(points)


def plot_points(path: Path, points: list[dict[str, object]]) -> None:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is required for --plot: python -m pip install matplotlib"
        ) from exc

    dac_v = [float(point["dac_mv"]) / 1000.0 for point in points]
    current_ma = [float(point["selected_current_ma"]) for point in points]
    vds = [float(point["vds_est_v"]) for point in points]
    valid_vds = [(x, y) for x, y in zip(vds, current_ma) if math.isfinite(x)]

    figure, axes = plt.subplots(1, 2, figsize=(9.0, 3.8))
    axes[0].plot(dac_v, current_ma, marker="o", markersize=3)
    axes[0].set_xlabel("DAC command (V), not measured VGS")
    axes[0].set_ylabel("Drain current (mA)")
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("Gate-bias transfer/load-line sweep")

    if valid_vds:
        axes[1].plot(
            [item[0] for item in valid_vds],
            [item[1] for item in valid_vds],
            marker="o",
            markersize=3,
        )
    axes[1].set_xlabel("Estimated VDS (V)")
    axes[1].set_ylabel("Drain current (mA)")
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title("Single external-load line")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def run_sweep(adapter: SerialCanAdapter, args: argparse.Namespace) -> None:
    # A completed sweep intentionally remains in DATA_READY so its point buffer
    # can be retrieved. Starting a new automated run explicitly discards that
    # old buffer and returns the engine to IDLE before applying new parameters.
    clear_token = next_token()
    adapter.send(command_frame(CMD_CLEAR_DATA, clear_token))
    cleared = require_ack(adapter, CMD_CLEAR_DATA, clear_token)
    print(f"Previous sweep data cleared: {cleared}")

    token = next_token()
    adapter.send(config_a_frame(token, args.start_mv, args.stop_mv, args.step_mv))
    time.sleep(0.05)
    adapter.send(
        config_b_frame(
            token,
            args.samples,
            args.settle_ms,
            args.sample_interval_ms,
            args.temperature_stop_c,
            int(round(args.current_stop_ma * 10.0)),
        )
    )

    # Config A returns a pending ACK (detail=1); Config B returns the applied ACK.
    applied = wait_for(
        adapter,
        lambda item: item.get("kind") == "ack"
        and item.get("opcode") == 0xA2
        and item.get("token") == token,
        2.0,
    )
    if applied.get("result") != RESULT_OK:
        raise RuntimeError(f"sweep configuration rejected: {applied}")

    armed_status = wait_until_physically_armed(adapter, args.arm_timeout)
    print(f"Armed: {armed_status}")
    start_token = next_token()
    adapter.send(command_frame(CMD_START_SWEEP, start_token))
    require_ack(adapter, CMD_START_SWEEP, start_token)

    terminal_events = {
        EVENT_SWEEP_COMPLETE,
        EVENT_SOFT_CURRENT_STOP,
        EVENT_SOFT_TEMPERATURE_STOP,
        EVENT_FAULT,
    }
    event = wait_for(
        adapter,
        lambda item: item.get("kind") == "event"
        and item.get("event") in terminal_events,
        args.sweep_timeout,
        echo=args.verbose,
    )
    print(f"Sweep terminal event: {event}")
    if event.get("event") == EVENT_FAULT or event.get("faults"):
        raise RuntimeError(f"sweep ended in a latched fault: {event}")

    point_count = int(event["point_count"])
    points = retrieve_points(adapter, point_count, args.load_ohm, args.sense_ohm)
    output = Path(args.output).resolve()
    write_csv(output, points)
    print(f"Wrote {point_count} points to {output}")
    if args.plot:
        plot_path = output.with_suffix(".png")
        plot_points(plot_path, points)
        print(f"Wrote plot to {plot_path}")


def apply_calibration_preset(args: argparse.Namespace, *, pulse: bool) -> None:
    voltage = args.voltage
    if args.repeat <= 0:
        raise ValueError("--repeat must be positive")
    if pulse and not 1400 <= args.dac_mv <= 2100:
        raise ValueError("--dac-mv must be between 1400 and 2100")
    args.start_mv = args.dac_mv if pulse else 1800
    args.stop_mv = args.dac_mv if pulse else 2050
    args.step_mv = 5
    args.settle_ms = 500 if pulse else 100
    args.samples = 7 if pulse else 3
    args.sample_interval_ms = 50 if pulse else 40
    args.current_stop_ma = CALIBRATION_CURRENT_STOPS_MA[voltage]
    args.temperature_stop_c = 45
    args.load_ohm = 10.0
    args.sense_ohm = 1.2
    args.arm_timeout = 60.0
    args.sweep_timeout = 90.0
    args.verbose = False
    args.plot = not pulse
    if args.output is None:
        root = Path("lab_data/final_can_ina/calibration")
        if pulse:
            args.output = str(
                root / f"{voltage}V_D{args.dac_mv}_R{args.repeat}.csv"
            )
        else:
            args.output = str(root / f"{voltage}V_bracket_R{args.repeat}.csv")


def fetch_buffered_points(adapter: SerialCanAdapter, args: argparse.Namespace) -> None:
    if args.count <= 0 or args.count > 64:
        raise ValueError("--count must be between 1 and 64")
    points = retrieve_points(adapter, args.count, args.load_ohm, args.sense_ohm)
    output = Path(args.output).resolve()
    write_csv(output, points)
    print(f"Wrote {args.count} buffered points to {output}")
    if args.plot:
        plot_path = output.with_suffix(".png")
        plot_points(plot_path, points)
        print(f"Wrote plot to {plot_path}")


def run_pid_start(adapter: SerialCanAdapter, args: argparse.Namespace) -> None:
    clear_token = next_token()
    adapter.send(command_frame(CMD_CLEAR_DATA, clear_token))
    require_ack(adapter, CMD_CLEAR_DATA, clear_token)

    config_token = next_token()
    adapter.send(
        pid_config_frame(
            config_token,
            args.profile,
            int(round(args.target_ma * 10.0)),
            args.base_mv,
            int(round(args.ff_mv_per_c * 100.0)),
        )
    )
    configured = require_ack(adapter, 0xA3, config_token)
    print(f"Experiment configuration accepted: {configured}")

    armed_status = wait_until_physically_armed(adapter, args.arm_timeout)
    print(f"Armed: {armed_status}")
    started: Optional[dict[str, object]] = None
    for attempt in range(1, 4):
        start_token = next_token()
        adapter.send(command_frame(CMD_START_PID_FF, start_token))
        ack = wait_for(
            adapter,
            lambda item: item.get("kind") == "ack"
            and item.get("opcode") == CMD_START_PID_FF
            and item.get("token") == start_token,
            2.0,
        )
        if ack.get("result") == RESULT_OK:
            started = ack
            break
        if ack.get("result") == 7 and ack.get("state") == STATE_ARMED:
            print(
                f"Experiment start sensor gate retry {attempt}/3: {ack}"
            )
            time.sleep(0.2)
            continue
        raise RuntimeError(f"MCU rejected experiment start: {ack}")
    if started is None:
        raise RuntimeError(
            "Experiment start sensor gate failed after 3 attempts; check the UART "
            "PID_START_REJECT reason and INA bus telemetry"
        )
    if started.get("state") != 5 or started.get("faults"):
        raise RuntimeError(f"Experiment did not enter active state: {started}")
    print(f"Experiment started: {started}")


def apply_locked_15v_experiment(args: argparse.Namespace) -> str:
    mode = args.mode
    profile, target_ma, base_mv, ff_mv_per_c = LOCKED_15V_EXPERIMENTS[mode]
    args.profile = profile
    args.target_ma = target_ma
    args.base_mv = base_mv
    args.ff_mv_per_c = ff_mv_per_c
    return mode


def apply_locked_final_matrix_experiment(args: argparse.Namespace) -> str:
    voltage = str(args.voltage)
    mode = args.mode
    profile, target_ma, base_mv, ff_mv_per_c = (
        LOCKED_FINAL_MATRIX_EXPERIMENTS[voltage][mode]
    )
    args.profile = profile
    args.target_ma = target_ma
    args.base_mv = base_mv
    args.ff_mv_per_c = ff_mv_per_c

    if hasattr(args, "repeat"):
        if args.repeat < 1:
            raise ValueError("--repeat must be at least 1")
        if args.output is None:
            target_text = int(round(target_ma))
            directory = Path(
                f"lab_data/final_can_ina/{voltage}V_{target_text}mA"
            )
            stem = MATRIX_MODE_FILE_STEMS[mode]
            args.output = str(
                directory
                / f"{voltage}V_{target_text}mA_{stem}_R{args.repeat}.csv"
            )
    return f"{voltage}V-{mode}"


def stop_output(adapter: SerialCanAdapter) -> None:
    token = next_token()
    adapter.send(command_frame(CMD_STOP_OUTPUT, token))
    print(require_ack(adapter, CMD_STOP_OUTPUT, token))


LIVE_CSV_FIELDS = [
    "time_s",
    "experiment_mode",
    "state",
    "faults",
    "dac_mv",
    "current_ma",
    "temperature_c",
    "ina_valid",
    "ina_bus_v",
    "ina_current_ma",
    "ina_power_mw",
]


def record_live_characterization(
    adapter: SerialCanAdapter,
    duration_s: float,
    output_name: str,
    target_ma: float,
    show_window: bool,
    experiment_mode: str = "unspecified",
) -> Path:
    if duration_s <= 0.0:
        raise ValueError("--seconds must be greater than zero")
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is required for live plotting: "
            "python -m pip install matplotlib"
        ) from exc

    output = Path(output_name).resolve()
    plot_path = output.with_suffix(".png")
    output.parent.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []
    latest_ina: dict[str, object] = {
        "valid": False,
        "bus_v": math.nan,
        "current_ma": math.nan,
        "power_mw": math.nan,
    }

    figure, axes = plt.subplots(2, 2, figsize=(10.0, 6.5))
    current_line, = axes[0, 0].plot([], [], marker=".", markersize=3)
    axes[0, 0].axhline(target_ma, linestyle="--", linewidth=1, label="Target")
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Drain current (mA)")
    axes[0, 0].set_title("Current regulation")
    axes[0, 0].legend()

    dac_line, = axes[0, 1].plot([], [], marker=".", markersize=3)
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("DAC command (mV)")
    axes[0, 1].set_title("Gate-bias command")

    temperature_line, = axes[1, 0].plot([], [], marker=".", markersize=3)
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("NTC temperature (C)")
    axes[1, 0].set_title("Temperature telemetry")

    transfer_line, = axes[1, 1].plot([], [], marker="o", markersize=3)
    axes[1, 1].set_xlabel("DAC command (V), not measured VGS")
    axes[1, 1].set_ylabel("Drain current (mA)")
    axes[1, 1].set_title("Live bias trajectory")
    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
    figure.tight_layout()
    if show_window:
        plt.ion()
        plt.show(block=False)

    start = time.monotonic()
    deadline = start + duration_s
    next_redraw = start
    try:
        while time.monotonic() < deadline:
            for frame in adapter.read_frames(0.05):
                decoded = decode_application_frame(frame)
                if decoded.get("kind") == "ina":
                    latest_ina = decoded
                elif decoded.get("kind") == "status":
                    record = {
                        "time_s": round(time.monotonic() - start, 3),
                        "experiment_mode": experiment_mode,
                        "state": decoded["state"],
                        "faults": decoded["faults"],
                        "dac_mv": decoded["dac_mv"],
                        "current_ma": decoded["current_ma"],
                        "temperature_c": decoded["temperature_c"],
                        "ina_valid": latest_ina.get("valid", False),
                        "ina_bus_v": latest_ina.get("bus_v", math.nan),
                        "ina_current_ma": latest_ina.get("current_ma", math.nan),
                        "ina_power_mw": latest_ina.get("power_mw", math.nan),
                    }
                    records.append(record)
                    if int(record["faults"]) != 0 or int(record["state"]) == STATE_FAULT:
                        print(f"Live monitor observed fault: {record}")
                        deadline = time.monotonic()
                        break

            now = time.monotonic()
            if records and now >= next_redraw:
                times = [float(item["time_s"]) for item in records]
                currents = [float(item["current_ma"]) for item in records]
                dacs = [float(item["dac_mv"]) for item in records]
                temperatures = [float(item["temperature_c"]) for item in records]
                current_line.set_data(times, currents)
                dac_line.set_data(times, dacs)
                temperature_line.set_data(times, temperatures)
                transfer_line.set_data([value / 1000.0 for value in dacs], currents)
                for axis in axes.flat:
                    axis.relim()
                    axis.autoscale_view()
                if show_window and plt.fignum_exists(figure.number):
                    figure.canvas.draw_idle()
                    plt.pause(0.001)
                next_redraw = now + 0.2
    except KeyboardInterrupt:
        print("Live recording stopped by user; saving captured data.")

    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=LIVE_CSV_FIELDS)
        writer.writeheader()
        writer.writerows(records)
    figure.savefig(plot_path, dpi=180)
    if show_window:
        plt.ioff()
    plt.close(figure)
    print(f"Wrote {len(records)} live status samples to {output}")
    print(f"Wrote live characterization plot to {plot_path}")
    return output


def codec_self_test() -> None:
    official = bytes.fromhex("AA C8 23 01 11 22 33 44 55 66 77 88 55")
    frame = CanFrame(0x123, bytes.fromhex("11 22 33 44 55 66 77 88"))
    assert encode_variable_frame(frame) == official
    parser = VariableFrameParser()
    assert parser.feed(official[:5]) == []
    assert parser.feed(official[5:]) == [frame]
    assert len(build_adapter_config_packet()) == 20
    packet = build_adapter_config_packet()
    assert packet[:5] == bytes.fromhex("AA 55 12 03 01")
    assert packet[-1] == sum(packet[2:19]) & 0xFF
    assert config_a_frame(0x0B, 1800, 2000, 5).data == bytes.fromhex(
        "0B 00 08 07 D0 07 05 00"
    )
    assert config_b_frame(0x0B, 3, 100, 40, 45, 700).data == bytes.fromhex(
        "0B 03 64 00 28 2D BC 02"
    )
    print("codec self-test: PASS")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Windows COM port, for example COM6")
    parser.add_argument("--serial-baud", type=int, default=2_000_000)
    parser.add_argument(
        "--configure-adapter",
        action="store_true",
        help="send the 500 kbit/s/standard/normal/variable adapter command first",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("self-test")
    subparsers.add_parser("adapter-config")
    subparsers.add_parser("ping")
    pid_config = subparsers.add_parser("pid-config")
    pid_config.add_argument("--profile", type=int, default=2)
    pid_config.add_argument("--target-ma", type=float, default=100.0)
    pid_config.add_argument("--base-mv", type=int, default=2050)
    pid_config.add_argument("--ff-mv-per-c", type=float, default=4.0)
    monitor = subparsers.add_parser("monitor")
    monitor.add_argument("--seconds", type=float, default=10.0)

    sweep = subparsers.add_parser("sweep")
    sweep.add_argument("--start-mv", type=int, default=1800)
    sweep.add_argument("--stop-mv", type=int, default=2050)
    sweep.add_argument("--step-mv", type=int, default=5)
    sweep.add_argument("--settle-ms", type=int, default=100)
    sweep.add_argument("--samples", type=int, default=3)
    sweep.add_argument("--sample-interval-ms", type=int, default=40)
    sweep.add_argument("--current-stop-ma", type=float, default=120.0)
    sweep.add_argument("--temperature-stop-c", type=int, default=45)
    sweep.add_argument("--load-ohm", type=float, default=10.0)
    sweep.add_argument("--sense-ohm", type=float, default=1.2)
    sweep.add_argument("--arm-timeout", type=float, default=60.0)
    sweep.add_argument("--sweep-timeout", type=float, default=90.0)
    sweep.add_argument("--output", default="lab_data/can_iv/sweep.csv")
    sweep.add_argument("--plot", action="store_true")
    sweep.add_argument("--verbose", action="store_true")

    cal_sweep = subparsers.add_parser("cal-sweep")
    cal_sweep.add_argument("voltage", choices=("10", "15"))
    cal_sweep.add_argument("--repeat", type=int, default=1)
    cal_sweep.add_argument("--output")

    cal_pulse = subparsers.add_parser("cal-pulse")
    cal_pulse.add_argument("voltage", choices=("10", "15"))
    cal_pulse.add_argument("--dac-mv", type=int, required=True)
    cal_pulse.add_argument("--repeat", type=int, default=1)
    cal_pulse.add_argument("--output")

    fetch = subparsers.add_parser("fetch")
    fetch.add_argument("--count", type=int, required=True)
    fetch.add_argument("--load-ohm", type=float, default=10.0)
    fetch.add_argument("--sense-ohm", type=float, default=1.2)
    fetch.add_argument("--output", default="lab_data/can_iv/buffered_sweep.csv")
    fetch.add_argument("--plot", action="store_true")

    pid_start = subparsers.add_parser("pid-start")
    pid_start.add_argument("--profile", type=int, default=2)
    pid_start.add_argument("--target-ma", type=float, default=100.0)
    pid_start.add_argument("--base-mv", type=int, default=2050)
    pid_start.add_argument("--ff-mv-per-c", type=float, default=4.0)
    pid_start.add_argument("--arm-timeout", type=float, default=60.0)

    experiment_start = subparsers.add_parser("experiment-start")
    experiment_start.add_argument(
        "mode", choices=tuple(LOCKED_15V_EXPERIMENTS)
    )
    experiment_start.add_argument("--arm-timeout", type=float, default=60.0)

    matrix_start = subparsers.add_parser("matrix-start")
    matrix_start.add_argument("voltage", choices=("10", "15"))
    matrix_start.add_argument(
        "mode", choices=tuple(MATRIX_MODE_FILE_STEMS)
    )
    matrix_start.add_argument("--arm-timeout", type=float, default=60.0)

    live_plot = subparsers.add_parser("live-plot")
    live_plot.add_argument("--seconds", type=float, default=60.0)
    live_plot.add_argument("--target-ma", type=float, default=100.0)
    live_plot.add_argument("--mode-label", default="attached-run")
    live_plot.add_argument(
        "--output", default="lab_data/can_iv/live_characterization.csv"
    )
    live_plot.add_argument("--no-window", action="store_true")

    pid_demo = subparsers.add_parser("pid-demo")
    pid_demo.add_argument("--profile", type=int, default=2)
    pid_demo.add_argument("--target-ma", type=float, default=100.0)
    pid_demo.add_argument("--base-mv", type=int, default=2050)
    pid_demo.add_argument("--ff-mv-per-c", type=float, default=4.0)
    pid_demo.add_argument("--arm-timeout", type=float, default=60.0)
    pid_demo.add_argument("--seconds", type=float, default=60.0)
    pid_demo.add_argument(
        "--output", default="lab_data/can_iv/final_pid_live.csv"
    )
    pid_demo.add_argument("--no-window", action="store_true")

    experiment_demo = subparsers.add_parser("experiment-demo")
    experiment_demo.add_argument(
        "mode", choices=tuple(LOCKED_15V_EXPERIMENTS)
    )
    experiment_demo.add_argument("--arm-timeout", type=float, default=60.0)
    experiment_demo.add_argument("--seconds", type=float, default=720.0)
    experiment_demo.add_argument(
        "--output", default="lab_data/new_15V_dac2015mV/15V100_CAN_run.csv"
    )
    experiment_demo.add_argument("--no-window", action="store_true")

    matrix_demo = subparsers.add_parser("matrix-demo")
    matrix_demo.add_argument("voltage", choices=("10", "15"))
    matrix_demo.add_argument(
        "mode", choices=tuple(MATRIX_MODE_FILE_STEMS)
    )
    matrix_demo.add_argument("--repeat", type=int, default=1)
    matrix_demo.add_argument("--arm-timeout", type=float, default=60.0)
    matrix_demo.add_argument("--seconds", type=float, default=600.0)
    matrix_demo.add_argument("--output")
    matrix_demo.add_argument("--no-window", action="store_true")

    subparsers.add_parser("stop")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "self-test":
        codec_self_test()
        return 0
    if not args.port:
        raise SystemExit("--port COMx is required for hardware commands")

    with SerialCanAdapter(args.port, args.serial_baud) as adapter:
        if args.configure_adapter or args.command == "adapter-config":
            configure_adapter(adapter)
            print("USB-CAN-A configured: variable protocol, standard, 500 kbit/s, normal")
            if args.command == "adapter-config":
                return 0

        if args.command == "ping":
            token = next_token()
            adapter.send(command_frame(CMD_PING, token))
            print(require_ack(adapter, CMD_PING, token))
        elif args.command == "pid-config":
            token = next_token()
            adapter.send(
                pid_config_frame(
                    token,
                    args.profile,
                    int(round(args.target_ma * 10.0)),
                    args.base_mv,
                    int(round(args.ff_mv_per_c * 100.0)),
                )
            )
            print(require_ack(adapter, 0xA3, token))
        elif args.command == "monitor":
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                for frame in adapter.read_frames(0.2):
                    print(decode_application_frame(frame))
        elif args.command == "sweep":
            run_sweep(adapter, args)
        elif args.command == "cal-sweep":
            apply_calibration_preset(args, pulse=False)
            run_sweep(adapter, args)
        elif args.command == "cal-pulse":
            apply_calibration_preset(args, pulse=True)
            run_sweep(adapter, args)
        elif args.command == "fetch":
            fetch_buffered_points(adapter, args)
        elif args.command == "pid-start":
            run_pid_start(adapter, args)
        elif args.command == "experiment-start":
            apply_locked_15v_experiment(args)
            run_pid_start(adapter, args)
        elif args.command == "matrix-start":
            apply_locked_final_matrix_experiment(args)
            run_pid_start(adapter, args)
        elif args.command == "live-plot":
            record_live_characterization(
                adapter,
                args.seconds,
                args.output,
                args.target_ma,
                not args.no_window,
                args.mode_label,
            )
        elif args.command == "pid-demo":
            run_pid_start(adapter, args)
            try:
                record_live_characterization(
                    adapter,
                    args.seconds,
                    args.output,
                    args.target_ma,
                    not args.no_window,
                    f"profile-{args.profile}",
                )
            finally:
                stop_output(adapter)
        elif args.command == "experiment-demo":
            mode = apply_locked_15v_experiment(args)
            run_pid_start(adapter, args)
            try:
                record_live_characterization(
                    adapter,
                    args.seconds,
                    args.output,
                    args.target_ma,
                    not args.no_window,
                    mode,
                )
            finally:
                stop_output(adapter)
        elif args.command == "matrix-demo":
            mode = apply_locked_final_matrix_experiment(args)
            run_pid_start(adapter, args)
            try:
                record_live_characterization(
                    adapter,
                    args.seconds,
                    args.output,
                    args.target_ma,
                    not args.no_window,
                    mode,
                )
            finally:
                stop_output(adapter)
        elif args.command == "stop":
            stop_output(adapter)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (TimeoutError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
