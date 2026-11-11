"""Calibrate one SO-101 joint through `SerialMotorsBus` (rustypot) and check its turn count.

Replays the calibration core on shoulder_lift (ID 2) with the real bus: calibration reset
(Home 0), the operator moves the joint to the middle through the encoder's zero, half-turn
homing, a range sweep sampled by this script, then `write_calibration()`, which must zero
the turn count. The multi-turn position is then read with Phase bit 4 set briefly and
compared with the display position. The original calibration of the joint is written back
at the end, torque stays off throughout, and nothing is saved to a calibration file.

Commands in output/command.json: middle, swept, finish. Each wait times out.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode, SerialMotorsBus

WAIT_S = 1800
JOINT = "shoulder_lift"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expect-turn-state", choices=("cleared", "retained"), default="cleared",
                        help="Expected result: cleared for the fix, retained for an unmodified baseline")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report: dict = {"port": args.port, "joint": JOINT, "stages": []}
    samples = (args.output / "samples.jsonl").open("x", encoding="utf-8")

    def save() -> None:
        (args.output / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    bus = SerialMotorsBus(
        port=args.port,
        motors={
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
            "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
    )

    def snapshot(label: str) -> dict:
        row = {
            "time_ns": time.time_ns(),
            "stage": label,
            "torque": int(bus.read("Torque_Enable", JOINT, normalize=False)),
            "phase": int(bus.read("Phase", JOINT, normalize=False)),
            "lock": int(bus.read("Lock", JOINT, normalize=False)),
            "home": int(bus.read("Homing_Offset", JOINT, normalize=False)),
            "minimum": int(bus.read("Min_Position_Limit", JOINT, normalize=False)),
            "maximum": int(bus.read("Max_Position_Limit", JOINT, normalize=False)),
            "present": int(bus.read("Present_Position", JOINT, normalize=False)),
        }
        samples.write(json.dumps(row) + "\n")
        samples.flush()
        if row["torque"] != 0:
            raise RuntimeError("Unexpected torque ON; stopped")
        return row

    def stage(label: str) -> dict:
        row = snapshot(label)
        report["stages"].append(row)
        report["stage"] = label
        save()
        print(json.dumps(row), flush=True)
        return row

    def extended_position(label: str) -> int:
        """Present position with Phase bit 4 set, the multi-turn reading; the bit is cleared again."""
        phase = int(bus.read("Phase", JOINT, normalize=False))
        bus.write("Phase", JOINT, phase | 0x10)
        try:
            row = stage(label)
        finally:
            bus.write("Phase", JOINT, phase)
        return row["present"]

    def wait_for(label: str, accepted: set[str], track: dict | None = None) -> str:
        stage(label)
        deadline = time.monotonic() + WAIT_S
        next_print = 0.0
        while time.monotonic() < deadline:
            position = int(bus.sync_read("Present_Position", [JOINT], normalize=False, num_retry=3)[JOINT])
            if track is not None:
                track["min"] = min(track["min"], position)
                track["max"] = max(track["max"], position)
            samples.write(json.dumps({"time_ns": time.time_ns(), "stage": label, "present": position}) + "\n")
            if time.monotonic() >= next_print:
                print(json.dumps({"stage": label, "present": position, "track": track}), flush=True)
                next_print = time.monotonic() + 2
            command_path = args.output / "command.json"
            if command_path.exists():
                command = json.loads(command_path.read_text(encoding="utf-8-sig"))["command"]
                command_path.unlink()
                if command == "finish" or command in accepted:
                    return command
                raise ValueError(f"Unexpected command {command} in {label}")
            time.sleep(0.05)
        raise TimeoutError(f"Operator timeout in {label}")

    original: MotorCalibration | None = None
    original_phase: int | None = None
    original_lock: int | None = None
    try:
        bus.connect()
        report["torque_before"] = {m: int(v) for m, v in bus.sync_read("Torque_Enable", normalize=False).items()}
        if any(report["torque_before"].values()):
            raise RuntimeError("All motors must already be OFF")
        bus.disable_torque()
        original = bus.read_calibration()[JOINT]
        report["original_calibration"] = original.__dict__
        before = stage("before")
        original_phase, original_lock = before["phase"], before["lock"]
        if before["phase"] & 0x10:
            raise RuntimeError("Phase bit 4 already set; expected the display mode")
        bus.write("Lock", JOINT, 0)
        report["extended_before"] = extended_position("extended_before")

        # The calibration core, as record_calibration() runs it, with the operator's moves
        # waited for through command.json instead of ENTER.
        bus.reset_calibration([JOINT])
        stage("home_zero")
        if wait_for("awaiting_middle", {"middle"}) != "middle":
            raise InterruptedError("finished before the middle")
        homings = bus.set_half_turn_homings([JOINT])
        report["homing_offset"] = homings[JOINT]
        report["extended_after_homing"] = extended_position("extended_after_homing")
        current = int(bus.read("Present_Position", JOINT, normalize=False))
        track = {"min": current, "max": current}
        if wait_for("awaiting_sweep", {"swept"}, track) != "swept":
            raise InterruptedError("finished before the sweep")
        if track["min"] >= track["max"]:
            raise RuntimeError("No range of motion was recorded")
        calibration = {
            JOINT: MotorCalibration(
                id=2, drive_mode=0, homing_offset=int(homings[JOINT]), range_min=track["min"], range_max=track["max"]
            )
        }
        report["recorded_calibration"] = calibration[JOINT].__dict__
        save()

        bus.write_calibration(calibration, cache=False)
        after = stage("after_write_calibration")
        report["extended_after_write_calibration"] = extended_position("extended_after_write_calibration")
        report["verdict"] = {
            "display_after": after["present"],
            "extended_after": report["extended_after_write_calibration"],
            "turn_count_zero_after_write_calibration": report["extended_after_write_calibration"]
            == after["present"],
            "registers_as_written": (after["home"], after["minimum"], after["maximum"])
            == (calibration[JOINT].homing_offset, calibration[JOINT].range_min, calibration[JOINT].range_max),
        }
        save()
        print(json.dumps({"verdict": report["verdict"]}), flush=True)
        report["expected_turn_state"] = args.expect_turn_state
        difference = report["extended_after_write_calibration"] - after["present"]
        matches = difference == 0 if args.expect_turn_state == "cleared" else abs(difference) == 4096
        if not report["verdict"]["registers_as_written"] or not matches:
            raise RuntimeError("Calibration result does not match the expected turn state or register values")
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        errors = []
        try:
            if bus.is_connected and original is not None:
                bus.write("Lock", JOINT, 0)
                try:
                    bus.write_calibration({JOINT: original}, cache=False)
                finally:
                    for register, value in (("Phase", original_phase), ("Lock", original_lock)):
                        if value is not None:
                            try:
                                bus.write(register, JOINT, value)
                            except Exception as exc:
                                errors.append(f"restore {register}: {exc}")
                report["restored"] = stage("restored")
                # A read in extended mode may require EEPROM unlock; restore it afterward.
                bus.write("Lock", JOINT, 0)
                try:
                    report["extended_restored"] = extended_position("extended_restored")
                finally:
                    if original_lock is not None:
                        bus.write("Lock", JOINT, original_lock)
                report["torque_after"] = {
                    m: int(v) for m, v in bus.sync_read("Torque_Enable", normalize=False).items()
                }
                restored = report["restored"]
                if (restored["home"], restored["minimum"], restored["maximum"]) != (
                    original.homing_offset, original.range_min, original.range_max
                ) or any(report["torque_after"].values()):
                    raise RuntimeError("Original calibration or torque OFF verification failed")
        except Exception as exc:
            errors.append(f"restore: {exc}")
        try:
            bus.disconnect(disable_torque=True)
        except Exception as exc:
            errors.append(f"disconnect: {exc}")
        report["cleanup_errors"] = errors
        report["stage"] = "finished"
        save()
        samples.close()
        print(json.dumps({k: v for k, v in report.items() if k != "stages"}), flush=True)
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
