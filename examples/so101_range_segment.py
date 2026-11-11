"""One slow, stepped ID2 range segment; every segment ends torque OFF.

Requires onsite support and a clear sweep. Host monitoring is not a physical
emergency stop. Restores speed/Phase/Lock, leaves the final commanded Goal.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from so101_first_goal_probe import HoldProbe
from so101_phase_probe import decode, signed


def tracking_action(error: int, stalled_seconds: float) -> str:
    """Small errors and stalls do not alter the scheduled command ramp."""
    return "stop" if abs(error) > 40 else "advance"


class RangeProbe(HoldProbe):
    def validate_request(self, motor: int, instruction: int, parameters: bytes) -> None:
        if motor in (1, 3, 4, 5) and instruction == 3:
            if parameters == b"\x28\x00":
                return
            if (motor, parameters) in getattr(self, "hold_packets", set()):
                # Goal and torque ON are one-shot. Configuration may be restored.
                if parameters[0] in (40, 42):
                    self.hold_packets.remove((motor, parameters))
                return
            raise ValueError("Unapproved holding-joint write")
        if instruction == 3 and motor == 2 and parameters[:1] == b"\x2a":
            if not self.goal_armed or len(parameters) != 3:
                raise ValueError("Goal must be explicitly armed")
            goal = int.from_bytes(parameters[1:], "little")
            if not self.minimum <= goal <= self.maximum or abs(goal-self.last_goal) > 6:
                raise ValueError("Goal outside range or larger than six-tick step")
            self.goal_armed = False
            self.last_goal = goal
            return
        super().validate_request(motor, instruction, parameters)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--target", type=int, required=True)
    parser.add_argument("--speed", type=int, default=60, choices=(30, 60))
    parser.add_argument("--hold-others", action="store_true")
    parser.add_argument("--full-cycle", action="store_true")
    parser.add_argument("--reset-before", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    p = RangeProbe(args.port, args.output / "wire.jsonl")
    p.config_writes = set()
    p.goal_armed = p.on_armed = p.reset_armed = False
    p.target = args.target
    p.hold_packets = set()
    hold_originals: dict[int, bytes] = {}
    hold_targets: dict[int, int] = {}
    report = dict(target=args.target, speed=args.speed, powered_attempted=False, reached=False)
    original = None
    changed = False
    log = (args.output / "samples.jsonl").open("x", encoding="utf-8")

    def read(label: str) -> dict:
        raw = p.read(2, 0, 87)
        row = dict(label=label, time_ns=time.time_ns(), **decode(raw),
                   speed=signed(int.from_bytes(raw[58:60],"little")),
                   load=signed(int.from_bytes(raw[60:62],"little"),10),
                   current_raw=int.from_bytes(raw[69:71],"little"), status=raw[65])
        log.write(json.dumps(row)+"\n"); log.flush()
        return row

    def config(address: int, data: bytes) -> None:
        if p.read(2,40,1) != b"\x00":
            raise RuntimeError("Configuration requires torque OFF")
        p.exchange(2,3,bytes([address])+data)
        if p.read(2,address,len(data)) != data:
            raise RuntimeError(f"Readback failed at {address}")

    def hold_config(motor: int, address: int, value: int) -> None:
        if p.read(motor,40,1) != b"\x00":
            raise RuntimeError("Holding-joint configuration requires OFF")
        p.exchange(motor,3,bytes([address,value]))
        if p.read(motor,address,1) != bytes([value]):
            raise RuntimeError("Holding-joint configuration readback failed")

    def monitor_holds() -> None:
        for motor, target in hold_targets.items():
            raw = p.read(motor,0,87)
            row = dict(label="holding_joint", id=motor, time_ns=time.time_ns(), **decode(raw),
                       load=signed(int.from_bytes(raw[60:62],"little"),10), status=raw[65])
            log.write(json.dumps(row)+"\n"); log.flush()
            if row["torque"] != 1 or row["goal"] != target or row["status"]:
                raise RuntimeError(f"Holding joint {motor} status/torque/goal mismatch")
            if abs(row["present"]-target) > 24 or abs(row["load"]) > 600:
                raise RuntimeError(f"Holding joint {motor} moved or exceeded load limit")

    try:
        report["torque_before"] = {str(i):p.read(i,40,1)[0] for i in range(1,7)}
        if any(report["torque_before"].values()):
            raise RuntimeError("All motors must already be OFF")
        original = p.read(2,0,87)
        report["before"] = decode(original)
        report["original_velocity"] = int.from_bytes(original[46:48], "little")
        (args.output/"preflight.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        p.minimum, p.maximum = report["before"]["minimum"], report["before"]["maximum"]
        if not p.minimum <= args.target <= p.maximum or original[18] != 12 or original[33] != 0:
            raise RuntimeError("Invalid target, Phase, or operating mode")
        p.config_writes = {bytes([55,0]),bytes([55,original[55]]),bytes([18,12]),bytes([18,28]),
                           bytes([46])+args.speed.to_bytes(2,"little"),bytes([46])+original[46:48]}
        changed = True
        config(55,b"\x00"); config(18,b"\x1c")
        if args.reset_before:
            report["before_reset"] = read("before_reset")
            if p.read(2,40,1) != b"\x00":
                raise RuntimeError("RESET requires torque OFF")
            p.reset_armed = True
            p.exchange(2,10,b"")
            report["reset_acknowledged"] = True
            time.sleep(0.2)
        extended = read("extended_before")
        if not p.minimum-5 <= extended["present"] <= p.maximum+5 or abs(extended["present"]-extended["private67"]) > 3:
            raise RuntimeError("Extended position/reference is not ready")
        config(18,b"\x0c"); config(46,args.speed.to_bytes(2,"little"))
        if args.hold_others:
            for motor in (1,3,4,5):
                raw = p.read(motor,0,87)
                if raw[40] != 0 or raw[18] != 12 or raw[33] != 0:
                    raise RuntimeError(f"Holding joint {motor} is not ready")
                hold_originals[motor] = raw
                p.hold_packets.update((motor,b) for b in (bytes([55,0]),bytes([55,raw[55]]),bytes([18,12]),bytes([18,28])))
                hold_config(motor,55,0); hold_config(motor,18,28)
                extended_hold = decode(p.read(motor,0,87))
                position = extended_hold["present"]
                if not extended_hold["minimum"] <= position <= extended_hold["maximum"] or abs(position-extended_hold["private67"]) > 3:
                    raise RuntimeError(f"Holding joint {motor} has inconsistent extended coordinates")
                hold_targets[motor] = position
                hold_config(motor,18,12)
            report["hold_targets"] = hold_targets
            report["hold_originals"] = {str(i):decode(raw) for i,raw in hold_originals.items()}
            (args.output/"preflight.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
            for motor,target in hold_targets.items():
                # Recheck posture before arming; never send an old cached Goal.
                if abs(decode(p.read(motor,0,87))["present"]-target) > 3:
                    raise RuntimeError(f"Holding joint {motor} moved before arming")
                p.hold_packets.add((motor,bytes([42])+target.to_bytes(2,"little")))
                p.hold_packets.add((motor,b"\x28\x01"))
                p.exchange(motor,3,bytes([42])+target.to_bytes(2,"little"))
                p.exchange(motor,3,b"\x28\x01")
            time.sleep(0.1)
            monitor_holds()
        start = read("before_goal")
        if not p.minimum-5 <= start["present"] <= p.maximum+5 or abs(start["present"]-start["private67"]) > 3:
            raise RuntimeError("Shoulder moved outside the starting tolerance")
        p.last_goal = start["present"]
        initial_goal = max(p.minimum, min(p.maximum, p.last_goal))
        # Unlike the short hold probe, each command advances at most six ticks.
        # Allow bridge/network jitter without replaying uncertain commands.
        p.response_timeout_s = 0.5
        report["powered_attempted"] = True
        p.goal_armed = True
        p.exchange(2,3,bytes([42])+initial_goal.to_bytes(2,"little"))
        p.on_armed = True
        p.exchange(2,3,b"\x28\x01")
        begin = time.monotonic(); next_goal = begin; next_print = begin; next_hold_check = begin
        last_progress = begin; progress_position = start["present"]
        target_since = None
        targets = [p.minimum, p.maximum, p.minimum] if args.full_cycle else [args.target]
        report["planned_targets"] = targets.copy()
        report["endpoints"] = []
        active_target = targets.pop(0)
        leg_begin = begin
        report["target_observation_seconds"] = 3.0
        while time.monotonic()-leg_begin < 120:
            if (args.output/"stop.request").exists():
                raise InterruptedError("Operator stop requested")
            row = read("powered")
            now = time.monotonic()
            if now >= next_hold_check:
                monitor_holds()
                next_hold_check = time.monotonic()+0.5
            if abs(row["present"]-progress_position) >= 2:
                last_progress, progress_position = now, row["present"]
            
            if p.last_goal == active_target:
                if target_since is None:
                    target_since = now
                if now-target_since >= report["target_observation_seconds"]:
                    report["endpoints"].append(dict(target=active_target, sample=row,
                                                   error_ticks=row["present"]-active_target))
                    if not targets:
                        report["observation_complete"] = True
                        report["final_powered_sample"] = row
                        break
                    active_target = targets.pop(0)
                    target_since = None
                    leg_begin = now
            if now >= next_goal:
                delta = max(-6,min(6,active_target-p.last_goal))
                if delta:
                    p.goal_armed = True
                    p.exchange(2,3,bytes([42])+(p.last_goal+delta).to_bytes(2,"little"))
                    next_goal = time.monotonic()+6/args.speed
            if now >= next_print:
                print(json.dumps({k:row[k] for k in ("present","goal","speed","load","current_raw")}),flush=True)
                next_print = now+2
            time.sleep(0.02)
        if not report.get("observation_complete"):
            raise TimeoutError("Segment deadline reached")
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        p.goal_armed = p.on_armed = False
        p.response_timeout_s = 0.3
        errors = []; off = not report["powered_attempted"]
        if report["powered_attempted"]:
            for _ in range(3):
                try:
                    p.exchange(2,3,b"\x28\x00")
                    off = p.read(2,40,1) == b"\x00"
                    if off: break
                except Exception as exc:
                    report["off_last_error"] = str(exc)
        report["off_confirmed"] = off
        if not off:
            errors.append("TORQUE OFF UNCONFIRMED: onsite power cutoff required")
        report["holding_off"] = {}
        for motor,raw in hold_originals.items():
            confirmed = False
            for _ in range(3):
                try:
                    p.exchange(motor,3,b"\x28\x00")
                    confirmed = p.read(motor,40,1) == b"\x00"
                    if confirmed: break
                except Exception as exc:
                    report[f"holding_{motor}_off_error"] = str(exc)
            report["holding_off"][str(motor)] = confirmed
            if not confirmed:
                errors.append(f"Holding joint {motor} OFF unconfirmed")
                continue
            for address,value in [(18,raw[18]),(55,raw[55])]:
                try: hold_config(motor,address,value)
                except Exception as exc: errors.append(f"restore holding {motor}/{address}: {exc}")
        if off and changed:
            for address,data in [(46,original[46:48]),(18,original[18:19]),(55,original[55:56])]:
                try: config(address,data)
                except Exception as exc: errors.append(f"restore {address}: {exc}")
            try:
                final = p.read(2,0,87)
                report["after"] = decode(final)
                report["static_restored"] = all(final[a] == original[a] for a in [9,10,11,12,18,31,32,33,41,46,47,48,49,55])
                report["torque_after"] = {str(i):p.read(i,40,1)[0] for i in range(1,7)}
            except Exception as exc: errors.append(str(exc))
        report["cleanup_errors"] = errors
        (args.output/"result.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        log.close(); p.port.close(); p.log.close()
    print(json.dumps(report),flush=True)
    if report.get("error") or errors or not off: raise SystemExit(1)


if __name__ == "__main__":
    main()
