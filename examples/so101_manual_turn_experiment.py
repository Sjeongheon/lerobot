"""Staged, torque-OFF ID2 coordinate experiment. Never sends a Goal or torque ON.

Commands in output/command.json: restore_home, reset, finish. The operator moves
only in awaiting_manual_move. Each waiting stage times out after 180 seconds and
restores Home/Phase/Lock. RESET is transmitted at most once, only when requested.
Private registers 67/71 are recorded as indicators, not firmware ground truth.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from so101_phase_probe import Probe, decode


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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report: dict = {"port": args.port, "stages": [], "reset_sent": False}
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
        stage(name)
        deadline = time.monotonic() + 180
        next_print = 0.0
        while time.monotonic() < deadline:
            row = sample(name)
            if time.monotonic() >= next_print:
                print(json.dumps({k: row[k] for k in ("stage", "present", "private67", "home", "torque")}), flush=True)
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

    try:
        report["torque_before"] = {str(i): probe.read(i, 40, 1)[0] for i in range(1, 7)}
        if any(report["torque_before"].values()):
            raise RuntimeError("All motors must already be OFF")
        original = probe.read(2, 0, 87)
        report["before"] = decode(original)
        save()
        probe.allowed_writes = {bytes([55, 0]), bytes([55, original[55]]),
                                bytes([18, original[18]]), bytes([18, original[18] | 16]),
                                b"\x1f\x00\x00", bytes([31]) + original[31:33]}
        changed = True
        probe.write_bytes(55, b"\x00")
        probe.write_bytes(18, bytes([original[18] | 16]))
        stage("original_home_extended")
        probe.write_bytes(31, b"\x00\x00")
        if wait_for("awaiting_manual_move", {"restore_home"}) == "restore_home":
            stage("manual_motion_complete_home_zero")
            probe.write_bytes(31, original[31:33])
            time.sleep(0.2)
            stage("original_home_restored")
            if wait_for("awaiting_reset_comparison", {"reset"}) == "reset":
                if probe.read(2, 40, 1) != b"\x00":
                    raise RuntimeError("Torque must be OFF before RESET")
                report["reset_sent"] = True
                save()
                probe.reset_allowed = True
                try:
                    probe.exchange(2, 10, b"")
                    report["reset_ack"] = True
                except TimeoutError as exc:
                    # Never resend an uncertain reset. Read state once settled.
                    report["reset_ack"] = False
                    report["reset_ack_error"] = str(exc)
                time.sleep(0.2)
                stage("after_reset")
                for _ in range(20):
                    sample("after_reset_settling")
                    time.sleep(0.1)
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
                report["preserved_registers"] = all(final[a] == original[a] for a in [9,10,11,12,18,31,32,33,40,42,43,55])
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
