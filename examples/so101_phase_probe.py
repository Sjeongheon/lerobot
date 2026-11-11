"""Observe ID2 extended feedback while torque is OFF; restore Phase and Lock.

Only READ and writes to ID2 Phase (18) and Lock (55) are permitted.
No Goal, Home, torque, reset, or calibration writes are implemented.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import serial


def packet(motor: int, instruction: int, parameters: bytes) -> bytes:
    body = bytes([motor, len(parameters) + 2, instruction]) + parameters
    return b"\xff\xff" + body + bytes([(~sum(body)) & 255])


class Probe:
    def __init__(self, port: str, journal: Path) -> None:
        self.log = journal.open("x", encoding="utf-8")
        try:
            self.port = serial.Serial(port, 1_000_000, timeout=0.1, write_timeout=1)
        except BaseException:
            self.log.close()
            raise

    def validate_request(self, motor: int, instruction: int, parameters: bytes) -> None:
        if instruction == 3:
            if motor != 2 or len(parameters) != 2 or parameters[0] not in (18, 55):
                raise ValueError("Only ID2 Phase/Lock writes are permitted")
        elif instruction != 2:
            raise ValueError("Only READ and Phase/Lock WRITE are permitted")

    def exchange(self, motor: int, instruction: int, parameters: bytes) -> bytes:
        self.validate_request(motor, instruction, parameters)
        request = packet(motor, instruction, parameters)
        self.port.reset_input_buffer()
        self.log.write(json.dumps({"time_ns": time.time_ns(), "tx": request.hex()}) + "\n")
        self.log.flush()
        if self.port.write(request) != len(request):
            raise RuntimeError("Incomplete packet write")
        buffer = bytearray()
        deadline = time.monotonic() + getattr(self, "response_timeout_s", 2.0)
        while time.monotonic() < deadline:
            buffer.extend(self.port.read(max(1, self.port.in_waiting)))
            start = buffer.find(b"\xff\xff")
            if start < 0 or len(buffer) < start + 4:
                continue
            if buffer[start + 3] < 2:
                raise RuntimeError("Invalid status frame length")
            size = buffer[start + 3] + 4
            if len(buffer) < start + size:
                continue
            response = bytes(buffer[start:start + size])
            self.log.write(json.dumps({"time_ns": time.time_ns(), "rx": response.hex()}) + "\n")
            self.log.flush()
            if response[2] != motor or sum(response[2:]) & 255 != 255:
                raise RuntimeError(f"Invalid response: {response.hex()}")
            if response[4]:
                raise RuntimeError(f"Motor {motor} status error: {response[4]}")
            return response[5:-1]
        raise TimeoutError(f"No status response from ID {motor}")

    def read(self, motor: int, address: int, count: int) -> bytes:
        result = self.exchange(motor, 2, bytes([address, count]))
        if len(result) != count:
            raise RuntimeError(f"Expected {count} bytes, got {len(result)}")
        return result

    def write(self, address: int, value: int) -> None:
        if self.read(2, 40, 1) != b"\x00":
            raise RuntimeError("ID2 torque is not OFF; refusing configuration write")
        self.exchange(2, 3, bytes([address, value]))
        if self.read(2, address, 1) != bytes([value]):
            raise RuntimeError(f"Read-back failed at address {address}")


def signed(value: int, bit: int = 15) -> int:
    return -(value & ~(1 << bit)) if value & (1 << bit) else value


def decode(raw: bytes) -> dict:
    if len(raw) < 87:
        raise ValueError("A complete 87-byte register snapshot is required")

    def word(address: int) -> int:
        return int.from_bytes(raw[address:address + 2], "little")

    return dict(phase=raw[18], lock=raw[55], torque=raw[40],
                home=signed(word(31), 11), minimum=word(9), maximum=word(11),
                goal=signed(word(42)), present=signed(word(56)),
                private67=signed(word(67)), private71=signed(word(71)),
                raw_hex=raw.hex())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report: dict = {"port": args.port, "motor_id": 2, "samples": []}
    probe = Probe(args.port, args.output / "wire.jsonl")
    original: bytes | None = None
    try:
        report["torque_before"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
        if any(report["torque_before"].values()):
            raise RuntimeError("All six motors must already be torque OFF")
        original = probe.read(2, 0, 87)
        report["before"] = decode(original)
        (args.output / "before.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        probe.write(55, 0)
        probe.write(18, original[18] | 16)
        for _ in range(10):
            report["samples"].append({"time_ns": time.time_ns(), **decode(probe.read(2, 0, 87))})
            time.sleep(0.05)
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            if original is not None:
                errors = []
                for address, value in [(18, original[18]), (55, original[55])]:
                    try:
                        probe.write(address, value)
                    except Exception as exc:
                        errors.append(f"restore {address}: {exc}")
                try:
                    final = probe.read(2, 0, 87)
                    report["after"] = decode(final)
                    fields = [9, 10, 11, 12, 18, 31, 32, 33, 40, 42, 43, 55]
                    report["preserved_registers"] = all(final[a] == original[a] for a in fields)
                    report["torque_after"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
                except Exception as exc:
                    errors.append(f"final read: {exc}")
                report["cleanup_errors"] = errors
        finally:
            probe.port.close()
            probe.log.close()
            (args.output / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    if report.get("cleanup_errors") or not report.get("preserved_registers"):
        raise RuntimeError("Restoration verification failed; inspect result.json")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
