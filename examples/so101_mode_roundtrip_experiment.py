"""Staged, torque-OFF ID2 experiment: does an Operating_Mode round-trip clear the turn count?

Same shape as so101_manual_turn_experiment.py. Never sends a Goal or torque ON.
Stages: Lock 0, Phase bit 4 set, Home 0, the operator moves the joint by hand through
the encoder's zero, Home is written back (the count is now visible as a +-4096
difference between the normal and the extended reading), then on command the script
writes Operating_Mode 1 (velocity), waits, writes Operating_Mode 0 (position) and reads
both positions again. RESET (0x0A) is available afterwards as the fallback, at most once.

With --lock-during-switch, Lock is 1 while Operating_Mode is written (no EEPROM save) and 0 again after.
Commands in output/command.json: restore_home, roundtrip, reset, finish. Each waiting
stage times out after 300 seconds and restores Home/Mode/Phase/Lock.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from so101_phase_probe import Probe, decode

WAIT_S = 300
MODE_DWELL_S = 0.1


class ExperimentProbe(Probe):
    allowed_writes: set[bytes]
    reset_allowed: bool = False

    def validate_request(self, motor: int, instruction: int, parameters: bytes) -> None:
        if instruction == 2 and 1 <= motor <= 6 and len(parameters) == 2:
            return
        if motor == 2 and instruction == 3 and parameters in self.allowed_writes:
            return
        if motor == 2 and instruction == 10 and not parameters and self.reset_allowed:
            self.reset_allowed = False
            return
        raise ValueError("Packet not permitted by the experiment")

    def write_bytes(self, address: int, value: bytes) -> None:
        if self.read(2, 40, 1) != b"\x00":
            raise RuntimeError("Torque must remain OFF")
        self.exchange(2, 3, bytes([address]) + value)
        if self.read(2, address, len(value)) != value:
            raise RuntimeError(f"Readback mismatch at {address}")


def decode_mode(raw: bytes) -> dict:
    return {**decode(raw), "mode": raw[33]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lock-during-switch", action="store_true",
                        help="write Lock 1 before the Operating_Mode writes and Lock 0 after, so no EEPROM save happens")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report: dict = {"port": args.port, "stages": [], "pairs": {}, "roundtrips": [], "reset_sent": False}
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
        row = {"time_ns": time.time_ns(), "stage": stage, **decode_mode(probe.read(2, 0, 87))}
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

    def pair(name: str) -> dict:
        """Normal (Phase bit 4 clear) and extended (bit 4 set) readings, moments apart."""
        assert original is not None
        probe.write_bytes(18, bytes([original[18] & ~16]))
        normal = sample(f"{name}_normal")
        probe.write_bytes(18, bytes([original[18] | 16]))
        extended = sample(f"{name}_extended")
        result = {"normal": normal["present"], "extended": extended["present"],
                  "difference": extended["present"] - normal["present"],
                  "home": extended["home"], "mode": extended["mode"], "phase": extended["phase"]}
        report["pairs"][name] = result
        save()
        print(json.dumps({"pair": name, **result}), flush=True)
        return result

    def wait_for(name: str, accepted: set[str]) -> str:
        stage(name)
        deadline = time.monotonic() + WAIT_S
        next_print = 0.0
        while time.monotonic() < deadline:
            row = sample(name)
            if time.monotonic() >= next_print:
                print(json.dumps({k: row[k] for k in ("stage", "present", "mode", "home", "torque")}), flush=True)
                next_print = time.monotonic() + 2
            command_path = args.output / "command.json"
            if command_path.exists():
                command = json.loads(command_path.read_text(encoding="utf-8-sig"))["command"]
                command_path.unlink()
                if command == "finish" or command in accepted:
                    return command
                raise ValueError(f"Unexpected command {command} in {name}")
            time.sleep(0.1)
        raise TimeoutError(f"Operator timeout in {name}")

    def roundtrip(index: int) -> None:
        if probe.read(2, 40, 1) != b"\x00":
            raise RuntimeError("Torque must be OFF before the mode change")
        before = pair(f"roundtrip{index}_before")
        if args.lock_during_switch:
            probe.write_bytes(55, b"\x01")
        t0 = time.time_ns()
        probe.write_bytes(33, b"\x01")
        in_velocity = []
        for _ in range(3):
            in_velocity.append(sample(f"roundtrip{index}_in_velocity_mode"))
            time.sleep(MODE_DWELL_S / 3)
        probe.write_bytes(33, b"\x00")
        t1 = time.time_ns()
        if args.lock_during_switch:
            probe.write_bytes(55, b"\x00")
        time.sleep(0.2)
        after = pair(f"roundtrip{index}_after")
        report["roundtrips"].append({
            "index": index, "before": before, "after": after,
            "velocity_mode_ms": (t1 - t0) / 1e6, "lock_during_switch": args.lock_during_switch,
            "in_velocity_mode": [{k: r[k] for k in ("present", "mode", "torque", "goal")} for r in in_velocity],
            "cleared": abs(after["difference"]) < 100 and abs(before["difference"]) >= 4000,
        })
        save()
        print(json.dumps(report["roundtrips"][-1]), flush=True)

    try:
        report["torque_before"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
        if any(report["torque_before"].values()):
            raise RuntimeError("All motors must already be OFF")
        original = probe.read(2, 0, 87)
        report["before"] = decode_mode(original)
        if original[33] != 0:
            raise RuntimeError(f"Operating_Mode is {original[33]}, expected 0 (position)")
        save()
        probe.allowed_writes = {bytes([55, 0]), bytes([55, original[55]]),
                                bytes([18, original[18] & ~16]), bytes([18, original[18] | 16]),
                                b"\x1f\x00\x00", bytes([31]) + original[31:33],
                                bytes([33, 0]), bytes([33, 1]), bytes([55, 1])}
        changed = True
        probe.write_bytes(55, b"\x00")
        probe.write_bytes(18, bytes([original[18] | 16]))
        pair("original_home")
        probe.write_bytes(31, b"\x00\x00")
        command = wait_for("awaiting_manual_move", {"restore_home"})
        if command == "restore_home":
            stage("manual_motion_complete_home_zero")
            probe.write_bytes(31, original[31:33])
            time.sleep(0.2)
            pair("original_home_restored")
            index = 0
            while True:
                command = wait_for("awaiting_command", {"roundtrip", "reset"})
                if command == "roundtrip":
                    index += 1
                    roundtrip(index)
                elif command == "reset":
                    if probe.read(2, 40, 1) != b"\x00":
                        raise RuntimeError("Torque must be OFF before RESET")
                    report["reset_sent"] = True
                    save()
                    probe.reset_allowed = True
                    try:
                        probe.exchange(2, 10, b"")
                        report["reset_ack"] = True
                    except TimeoutError as exc:
                        report["reset_ack"] = False
                        report["reset_ack_error"] = str(exc)
                    time.sleep(0.2)
                    pair("after_reset")
                    break
                else:
                    break
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        errors = []
        if changed and original is not None:
            for address, value in [(31, original[31:33]), (33, original[33:34]), (18, original[18:19]), (55, original[55:56])]:
                try:
                    if probe.read(2, address, len(value)) != value:
                        probe.write_bytes(address, value)
                except Exception as exc:
                    errors.append(f"restore {address}: {exc}")
            try:
                final = probe.read(2, 0, 87)
                report["after"] = decode_mode(final)
                report["preserved_registers"] = all(final[a] == original[a] for a in [9, 10, 11, 12, 18, 31, 32, 33, 40, 42, 43, 55])
                report["torque_after"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
            except Exception as exc:
                errors.append(f"verification: {exc}")
        report["cleanup_errors"] = errors
        report["stage"] = "finished"
        save()
        samples.close()
        probe.port.close()
        probe.log.close()
        print(json.dumps(report), flush=True)
    if errors or not report.get("preserved_registers"):
        raise RuntimeError("Restoration verification failed")


if __name__ == "__main__":
    main()
