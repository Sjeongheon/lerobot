"""Torque-OFF ID2 turn-retention experiment with the Phase multi-turn bit CLEAR during the move.

Question: does the turn counter accumulate while Phase bit 4 is clear (Phase 12)?
Commands in output/command.json: restore_home, reset, finish. The operator moves only
in awaiting_manual_move, with Home 0 and Phase 12. Phase 28 is written only for the
snapshots taken before and after the move. RESET is transmitted at most once, only when
requested; "finish" in awaiting_cleanup leaves any retained turn in place for a later
power-cycle test. Never sends a Goal or torque ON.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from so101_manual_turn_experiment import ExperimentProbe
from so101_phase_probe import decode

WAIT_S = 900


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report: dict = {
        "port": args.port,
        "stages": [],
        "crossings": [],
        "reset_sent": False,
        "scope": "ID2 torque-OFF; manual move under Phase 12 / Home 0; Phase 28 only for snapshots",
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

    def wait_for(name: str, accepted: set[str]) -> str:
        previous = stage(name)["present"]
        deadline = time.monotonic() + WAIT_S
        next_print = 0.0
        while time.monotonic() < deadline:
            row = sample(name)
            delta = row["present"] - previous
            if abs(delta) > 2048:
                report["crossings"].append(
                    {"stage": name, "from": previous, "to": row["present"], "phase": row["phase"]}
                )
                save()
            previous = row["present"]
            if time.monotonic() >= next_print:
                keys = ("stage", "present", "private67", "home", "phase", "torque")
                print(json.dumps({k: row[k] for k in keys}), flush=True)
                next_print = time.monotonic() + 2
            command_path = args.output / "command.json"
            if command_path.exists():
                command = json.loads(command_path.read_text(encoding="utf-8-sig"))["command"]
                command_path.unlink()
                if command == "finish" or command in accepted:
                    return command
                raise ValueError(f"Unexpected command {command} in {name}")
            time.sleep(0.05)
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
        probe.allowed_writes = {
            bytes([55, 0]),
            bytes([55, original[55]]),
            bytes([18, 12]),
            bytes([18, 28]),
            b"\x1f\x00\x00",
            bytes([31]) + original[31:33],
        }
        changed = True
        probe.write_bytes(55, b"\x00")
        probe.write_bytes(18, b"\x1c")
        stage("extended_before")
        probe.write_bytes(18, b"\x0c")
        stage("display_before")
        probe.write_bytes(31, b"\x00\x00")
        stage("home_zero_display")
        if wait_for("awaiting_manual_move", {"restore_home"}) == "restore_home":
            stage("manual_motion_complete_display")
            probe.write_bytes(18, b"\x1c")
            home_zero_ext = stage("home_zero_extended")
            probe.write_bytes(31, original[31:33])
            time.sleep(0.2)
            restored_ext = stage("home_restored_extended")
            probe.write_bytes(18, b"\x0c")
            restored_disp = stage("home_restored_display")
            report["verdict"] = {
                "crossings_seen_under_phase12": len(report["crossings"]),
                "home_zero_extended_present": home_zero_ext["present"],
                "home_restored_extended_present": restored_ext["present"],
                "home_restored_display_present": restored_disp["present"],
                "extended_minus_display": restored_ext["present"] - restored_disp["present"],
                "turn_retained_after_phase12_move": restored_ext["present"] != restored_disp["present"],
            }
            save()
            print(json.dumps({"verdict": report["verdict"]}), flush=True)
            if wait_for("awaiting_cleanup", {"reset"}) == "reset":
                if probe.read(2, 40, 1) != b"\x00":
                    raise RuntimeError("Torque must be OFF before RESET")
                probe.write_bytes(18, b"\x1c")
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
                probe.write_bytes(18, b"\x0c")
                stage("after_reset_display")
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        errors = []
        if changed and original is not None:
            for address, value in [(31, original[31:33]), (18, original[18:19]), (55, original[55:56])]:
                try:
                    if probe.read(2, address, len(value)) != value:
                        probe.write_bytes(address, value)
                except Exception as exc:
                    errors.append(f"restore {address}: {exc}")
            try:
                final = probe.read(2, 0, 87)
                report["after"] = decode(final)
                fields = [9, 10, 11, 12, 18, 31, 32, 33, 40, 42, 43, 55]
                report["preserved_registers"] = all(final[a] == original[a] for a in fields)
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
