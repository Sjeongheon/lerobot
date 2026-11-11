"""Torque-OFF ID2 power-cycle test of a retained turn.

Precondition: the extended (Phase 28) position already differs from the display
position by a whole turn. Phase 28 stays written across the cycle so the first read
after power-up shows the multi-turn position directly. Commands in output/command.json:
power_cycled, reset, finish. RESET at most once, only when requested. Never sends a Goal
or torque ON; if the motor powers up with torque ON the experiment stops without writing.
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
    parser.add_argument("--allow-no-turn", action="store_true", help="run even without a retained turn")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report: dict = {
        "port": args.port,
        "stages": [],
        "reset_sent": False,
        "unreachable_reads_during_wait": 0,
        "power_down_observed": False,
    }
    original: bytes | None = None
    changed = False
    probe = ExperimentProbe(args.port, args.output / "wire.jsonl")
    probe.allowed_writes = set()
    samples = (args.output / "samples.jsonl").open("x", encoding="utf-8")

    def save() -> None:
        temporary = args.output / "result.tmp"
        temporary.write_text(json.dumps(report, indent=2), encoding="utf-8")
        temporary.replace(args.output / "result.json")

    def sample(stage: str) -> dict:
        row = {"time_ns": time.time_ns(), "stage": stage, **decode(probe.read(2, 0, 87))}
        samples.write(json.dumps(row) + "\n")
        samples.flush()
        if row["torque"] != 0:
            raise RuntimeError("Unexpected torque ON; experiment stopped")
        return row

    def stage(name: str) -> dict:
        row = sample(name)
        report["stage"] = name
        report["stages"].append(row)
        save()
        print(json.dumps({k: v for k, v in row.items() if k != "raw_hex"}), flush=True)
        return row

    def poll_command(accepted: set[str]) -> str | None:
        command_path = args.output / "command.json"
        if not command_path.exists():
            return None
        command = json.loads(command_path.read_text(encoding="utf-8-sig"))["command"]
        command_path.unlink()
        if command == "finish" or command in accepted:
            return command
        raise ValueError(f"Unexpected command {command}")

    def reopen_port() -> bool:
        """The virtual COM port can vanish with the power: drop the dead handle and try again."""
        try:
            probe.port.close()
        except Exception:
            pass
        try:
            probe.port = serial.Serial(args.port, 1_000_000, timeout=0.1, write_timeout=1)
            report["port_reopened"] = report.get("port_reopened", 0) + 1
            return True
        except (OSError, serial.SerialException):
            return False

    def wait_for(name: str, accepted: set[str], tolerate_silence: bool) -> str:
        report["stage"] = name
        save()
        print(json.dumps({"stage": name, "waiting_for": sorted(accepted | {"finish"})}), flush=True)
        deadline = time.monotonic() + WAIT_S
        next_print = 0.0
        while time.monotonic() < deadline:
            try:
                row = sample(name)
                if time.monotonic() >= next_print:
                    keys = ("stage", "present", "private67", "home", "phase", "torque")
                    print(json.dumps({k: row[k] for k in keys}), flush=True)
                    next_print = time.monotonic() + 2
            except (TimeoutError, RuntimeError, OSError, serial.SerialException) as exc:
                if not tolerate_silence or "torque ON" in str(exc):
                    raise
                if isinstance(exc, (OSError, serial.SerialException)):
                    report["port_errors_during_wait"] = report.get("port_errors_during_wait", 0) + 1
                    reopen_port()
                    time.sleep(0.5)
                report["unreachable_reads_during_wait"] += 1
                report["power_down_observed"] = True
                if time.monotonic() >= next_print:
                    count = report["unreachable_reads_during_wait"]
                    print(json.dumps({"stage": name, "motor": "unreachable", "count": count}), flush=True)
                    next_print = time.monotonic() + 2
            command = poll_command(accepted)
            if command is not None:
                return command
            time.sleep(0.1)
        raise TimeoutError(f"Operator timeout in {name}")

    try:
        report["torque_before"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
        if any(report["torque_before"].values()):
            raise RuntimeError("All motors must already be OFF")
        original = probe.read(2, 0, 87)
        report["before"] = decode(original)
        if original[18] != 12 or original[33] != 0:
            raise RuntimeError("Expected Phase 12 / position mode 0 at start")
        save()
        probe.allowed_writes = {bytes([55, 0]), bytes([55, original[55]]), bytes([18, 12]), bytes([18, 28])}
        changed = True
        probe.write_bytes(55, b"\x00")
        probe.write_bytes(18, b"\x1c")
        before = stage("extended_before_power_cycle")
        display_before = before["present"] % 4096
        report["turn_before"] = before["present"] // 4096
        if before["present"] == display_before and not args.allow_no_turn:
            raise RuntimeError("No retained turn to test: extended equals display")
        probe.response_timeout_s = 0.3
        print(
            "OPERATOR: cut the motor power (keep USB), wait 5 s, power it on again, then command power_cycled",
            flush=True,
        )
        if wait_for("awaiting_power_cycle", {"power_cycled"}, tolerate_silence=True) == "power_cycled":
            probe.response_timeout_s = 2.0
            answered = False
            for _ in range(40):
                try:
                    probe.read(2, 0, 87)
                    answered = True
                    break
                except (TimeoutError, RuntimeError):
                    time.sleep(0.5)
                except (OSError, serial.SerialException):
                    reopen_port()
                    time.sleep(0.5)
            if not answered:
                raise RuntimeError("Motor did not answer after the power cycle")
            report["torque_after_power_up"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
            if any(report["torque_after_power_up"].values()):
                raise RuntimeError("Torque ON after power-up: cut power; no writes attempted")
            after = stage("extended_after_power_cycle")
            if not report["power_down_observed"]:
                raise RuntimeError("No communication gap was observed; the power-cycle condition is unverified")
            if not after["phase"] & 0x10:
                raise RuntimeError("Phase bit 4 is clear after power-up; the reading is not an extended position")
            report["verdict"] = {
                "power_down_observed": report["power_down_observed"],
                "unreachable_reads_during_wait": report["unreachable_reads_during_wait"],
                "extended_before": before["present"],
                "extended_after": after["present"],
                "phase_after_power_up": after["phase"],
                "turn_before": report["turn_before"],
                "turn_after": after["present"] // 4096,
                "turn_survives_power_cycle": after["present"] // 4096 != 0,
            }
            save()
            print(json.dumps({"verdict": report["verdict"]}), flush=True)
            if wait_for("awaiting_cleanup", {"reset"}, tolerate_silence=False) == "reset":
                if probe.read(2, 40, 1) != b"\x00":
                    raise RuntimeError("Torque must be OFF before RESET")
                report["reset_sent"] = True
                save()
                probe.reset_allowed = True
                try:
                    probe.exchange(2, 10, b"")
                    report["reset_ack"] = True
                except TimeoutError as exc:
                    # Never resend an uncertain reset.
                    report["reset_ack"] = False
                    report["reset_ack_error"] = str(exc)
                time.sleep(0.2)
                stage("after_reset_extended")
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        errors = []
        probe.response_timeout_s = 2.0
        if changed and original is not None:
            try:
                # Power-up can re-lock EEPROM even though we unlocked before the cycle.
                probe.write_bytes(55, b"\x00")
            except Exception as exc:
                errors.append(f"unlock for restoration: {exc}")
            for address, value in [(18, original[18:19]), (55, original[55:56])]:
                try:
                    if probe.read(2, address, len(value)) != value:
                        probe.write_bytes(address, value)
                except Exception as exc:
                    errors.append(f"restore {address}: {exc}")
            try:
                final = probe.read(2, 0, 87)
                report["after"] = decode(final)
                # Goal (42-43) may legitimately change across a power cycle: reported, not asserted.
                fields = [9, 10, 11, 12, 18, 31, 32, 33, 40, 55]
                report["preserved_registers"] = all(final[a] == original[a] for a in fields)
                report["goal_preserved"] = final[42:44] == original[42:44]
                report["torque_after"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
            except Exception as exc:
                errors.append(f"verification: {exc}")
        report["cleanup_errors"] = errors
        report["stage"] = "finished"
        save()
        samples.close()
        probe.port.close()
        probe.log.close()
        print(json.dumps({k: v for k, v in report.items() if k != "stages"}), flush=True)
    if errors or not report.get("preserved_registers"):
        raise RuntimeError("Restoration verification failed")


if __name__ == "__main__":
    main()
