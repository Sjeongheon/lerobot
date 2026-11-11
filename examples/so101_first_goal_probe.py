"""One bounded ID2 same-pose hold, with journalled OFF and static restoration.

Requires an onsite operator able to cut power. The 0.3 s / 24 tick cutoffs
are host-side stop requests, not hard real-time physical safety guarantees.
The old Goal is deliberately NOT restored: writing it may itself enable torque.
For expected-turn=1, reset the turn reference once after confirmed torque OFF.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from so101_phase_probe import Probe, decode


class HoldProbe(Probe):
    def validate_request(self, motor: int, instruction: int, parameters: bytes) -> None:
        if instruction == 2 and motor in range(1, 7) and len(parameters) == 2:
            return
        if motor == 2 and instruction == 3:
            if parameters == b"\x28\x00" or parameters in self.config_writes:
                return
            if parameters == bytes([42]) + self.target.to_bytes(2, "little") and self.goal_armed:
                self.goal_armed = False
                return
            if parameters == b"\x28\x01" and self.on_armed:
                self.on_armed = False
                return
        if motor == 2 and instruction == 10 and not parameters and self.reset_armed:
            self.reset_armed = False
            return
        raise ValueError("Packet is outside the single-hold allowlist")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--target", type=int, required=True)
    parser.add_argument("--expected-turn", type=int, choices=(0, 1), required=True)
    parser.add_argument("--reset-before-hold", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.reset_before_hold and args.expected_turn != 0:
        parser.error("RESET-before-hold requires expected-turn=0")
    args.output.mkdir(parents=True, exist_ok=False)
    p = HoldProbe(args.port, args.output / "wire.jsonl")
    p.config_writes = set()
    p.target = args.target
    p.goal_armed = p.on_armed = p.reset_armed = False
    report = dict(target=args.target, expected_turn=args.expected_turn, samples=[],
                  stop_after_s=0.3, stop_displacement_ticks=24, powered_attempted=False)
    original = None
    changed = False
    off_confirmed = False
    started = None

    def save() -> None:
        (args.output / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    def snapshot(label: str) -> dict:
        raw = p.read(2, 0, 87)
        row = dict(label=label, time_ns=time.time_ns(), **decode(raw),
                   speed_raw=int.from_bytes(raw[58:60], "little"),
                   load_raw=int.from_bytes(raw[60:62], "little"),
                   current_raw=int.from_bytes(raw[69:71], "little"), status=raw[65])
        report["samples"].append(row)
        return row

    def config(address: int, value: int) -> None:
        if p.read(2, 40, 1) != b"\x00":
            raise RuntimeError("Configuration requires torque OFF")
        p.exchange(2, 3, bytes([address, value]))
        if p.read(2, address, 1) != bytes([value]):
            raise RuntimeError("Configuration readback failed")

    try:
        report["torque_before"] = {str(i): p.read(i, 40, 1)[0] for i in range(1, 7)}
        if any(report["torque_before"].values()):
            raise RuntimeError("All motors must already be OFF")
        original = p.read(2, 0, 87)
        report["original"] = decode(original)
        if original[18] != 12 or original[33] != 0:
            raise RuntimeError("Expected original Phase12 / position mode0")
        if not report["original"]["minimum"] + 100 < args.target < report["original"]["maximum"] - 100:
            raise RuntimeError("Target must be well inside the calibrated range")
        p.config_writes = {bytes([55,0]), bytes([55,original[55]]), bytes([18,12]), bytes([18,28])}
        changed = True
        config(55, 0)
        config(18, 28)
        if args.reset_before_hold:
            snapshot("extended_before_requested_reset")
            if p.read(2, 40, 1) != b"\x00":
                raise RuntimeError("RESET requires torque OFF")
            p.reset_armed = True
            report["requested_reset_sent"] = True
            save()
            try:
                p.exchange(2, 10, b"")
                report["requested_reset_ack"] = True
            except TimeoutError:
                # No resend or powered continuation after uncertain RESET.
                report["requested_reset_ack"] = False
                raise
            time.sleep(0.2)
            for _ in range(3):
                settled = snapshot("requested_reset_settling")
                if settled["torque"] != 0 or abs(settled["private67"] - settled["present"]) > 3:
                    raise RuntimeError("Post-RESET OFF state is not settled")
                time.sleep(0.05)
        extended = snapshot("extended_before")
        if extended["present"] // 4096 != args.expected_turn:
            raise RuntimeError("Unexpected initial turn reference")
        if abs(extended["present"] % 4096 - args.target) > 5:
            raise RuntimeError("Pose differs from fixed target by more than five ticks")
        if abs(extended["private67"] - extended["present"]) > 3:
            raise RuntimeError("OFF reference is not settled")
        config(18, 12)
        before = snapshot("before_goal")
        save()
        p.response_timeout_s = 0.15
        p.goal_armed = p.on_armed = True
        report["powered_attempted"] = True
        started = time.monotonic()
        p.exchange(2, 3, bytes([42]) + args.target.to_bytes(2, "little"))
        if time.monotonic() - started >= 0.3:
            raise TimeoutError("Goal transaction consumed the hold window")
        p.exchange(2, 3, b"\x28\x01")
        report["stop_reason"] = "time_limit"
        while time.monotonic() - started < 0.3:
            row = snapshot("powered")
            row["elapsed_s"] = time.monotonic() - started
            if row["goal"] != args.target or row["status"]:
                report["stop_reason"] = "goal_or_status_mismatch"
                break
            if abs(row["present"] - before["present"]) >= 24:
                report["stop_reason"] = "displacement_limit"
                break
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        errors = []
        p.goal_armed = p.on_armed = False
        p.response_timeout_s = 0.3
        if report["powered_attempted"]:
            report["off_attempts"] = []
            for _ in range(3):
                try:
                    p.exchange(2, 3, b"\x28\x00")
                    off_confirmed = p.read(2, 40, 1) == b"\x00"
                    report["off_attempts"].append({"confirmed": off_confirmed})
                    if off_confirmed:
                        break
                except Exception as exc:
                    report["off_attempts"].append({"error": str(exc)})
            report["off_confirmed"] = off_confirmed
            report["goal_to_off_confirmation_s"] = time.monotonic() - started
            if not off_confirmed:
                errors.append("TORQUE OFF UNCONFIRMED: onsite power cutoff required")
        else:
            off_confirmed = True
        if off_confirmed and changed:
            try:
                snapshot("after_off")
                config(18, 28)
                snapshot("extended_after_off")
                if args.expected_turn == 1 and report["powered_attempted"]:
                    p.reset_armed = True
                    report["cleanup_reset_sent"] = True
                    try:
                        p.exchange(2, 10, b"")
                        report["cleanup_reset_ack"] = True
                    except TimeoutError:
                        report["cleanup_reset_ack"] = False
                    time.sleep(0.2)
                    snapshot("extended_after_reset")
            except Exception as exc:
                errors.append(str(exc))
            for address, value in [(18,original[18]), (55,original[55])]:
                try:
                    config(address, value)
                except Exception as exc:
                    errors.append(f"restore {address}: {exc}")
            try:
                raw = p.read(2, 0, 87)
                report["final"] = decode(raw)
                report["static_restored"] = all(raw[a] == original[a] for a in [9,10,11,12,18,31,32,33,55])
                report["torque_after"] = {str(i):p.read(i,40,1)[0] for i in range(1,7)}
            except Exception as exc:
                errors.append(f"final read: {exc}")
        report["cleanup_errors"] = errors
        save()
        p.port.close()
        p.log.close()
    print(json.dumps(report), flush=True)
    if errors or report.get("error"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
