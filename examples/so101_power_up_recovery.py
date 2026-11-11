"""Wait for ID2 to answer again after a power cut, read its state as found, then restore Phase/Lock.

Used when the power-cycle experiment lost its serial handle together with the motor power.
Phase 28 was left in EEPROM on purpose, so the first read after power-up is the multi-turn
position. Read-only until all six motors report torque OFF; then Phase 12 and Lock 1 are
written back with read-back. Never sends a Goal, torque ON or RESET.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import serial

from so101_manual_turn_experiment import ExperimentProbe
from so101_phase_probe import decode

WAIT_S = 900


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stamp = time.time_ns()
    report: dict = {"started_ns": stamp, "silent_polls": 0, "port_errors": 0, "port_reopened": 0}
    probe = ExperimentProbe(args.port, args.output / f"recovery-wire-{stamp}.jsonl")
    probe.response_timeout_s = 0.3

    def save() -> None:
        (args.output / f"recovery-{stamp}.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    def reopen() -> None:
        try:
            probe.port.close()
        except Exception:
            pass
        try:
            probe.port = serial.Serial(args.port, 1_000_000, timeout=0.1, write_timeout=1)
            report["port_reopened"] += 1
        except (OSError, serial.SerialException):
            pass

    try:
        deadline = time.monotonic() + WAIT_S
        raw = None
        next_print = 0.0
        while time.monotonic() < deadline:
            try:
                raw = probe.read(2, 0, 87)
                break
            except (TimeoutError, RuntimeError):
                report["silent_polls"] += 1
            except (OSError, serial.SerialException):
                report["port_errors"] += 1
                reopen()
            if time.monotonic() >= next_print:
                print(json.dumps({"waiting": "motor silent", "silent_polls": report["silent_polls"], "port_errors": report["port_errors"]}), flush=True)
                next_print = time.monotonic() + 5
            time.sleep(1.0)
        if raw is None:
            raise TimeoutError("Motor never answered")
        report["answered_ns"] = time.time_ns()
        probe.response_timeout_s = 2.0
        time.sleep(0.5)
        report["torque"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
        raw = probe.read(2, 0, 87)
        found = decode(raw)
        found.pop("raw_hex")
        report["state_as_found"] = found
        if found["phase"] & 0x10:
            report["verdict"] = {
                "extended_after_power_up": found["present"],
                "display_after_power_up": found["present"] % 4096,
                "turn_after_power_up": found["present"] // 4096,
                "turn_survives_power_cycle": found["present"] // 4096 != 0,
            }
        else:
            report["verdict"] = {"note": "Phase bit 4 not set at power-up; extended position not observable"}
        print(json.dumps(report), flush=True)
        save()
        if any(report["torque"].values()):
            raise RuntimeError("Torque ON after power-up: no writes attempted")
        probe.allowed_writes = {bytes([18, 12]), bytes([55, 1])}
        if raw[18] != 12:
            probe.write_bytes(18, b"\x0c")
        if raw[55] != 1:
            probe.write_bytes(55, b"\x01")
        after = decode(probe.read(2, 0, 87))
        after.pop("raw_hex")
        report["state_after_restore"] = after
        report["torque_after"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        save()
        try:
            probe.port.close()
        except Exception:
            pass
        probe.log.close()
        print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
