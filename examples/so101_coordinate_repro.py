"""Offline coordinate experiment; no serial imports or hardware access.

Ports encoder_multiturn_track and same_home_event coordinate semantics from
the supplied calibration_turn_replay.c. This is a conditional reference model,
not stock firmware or a simulation of mechanics, PID dynamics, or rustypot I/O.
Uses the current repository's actual _get_half_turn_homings calculation via AST.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import re
import zipfile
from pathlib import Path
from types import SimpleNamespace


class Encoder:
    def __init__(self, raw: int, home: int) -> None:
        self.raw = raw
        self.home = home
        self.turns = 0
        self.previous = (raw - home) % 4096
        self.position = self.previous

    def update(self, raw: int) -> None:
        self.raw = raw % 4096
        corrected = (self.raw - self.home) % 4096
        delta = corrected - self.previous
        if delta > 4076:
            self.turns -= 1
        elif delta < -4076:
            self.turns += 1
        self.previous = corrected
        self.position = corrected + 4096 * self.turns

    def set_home(self, home: int) -> None:
        # Reference setter updates the encoder without clearing the turn count.
        self.home = home
        self.update(self.raw)


def host_home_function(source: str):
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SerialMotorsBus")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_get_half_turn_homings")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), method], type_ignores=[])
    namespace = {}
    exec(compile(ast.fix_missing_locations(module), "motors_bus.py:extracted", "exec"), namespace)
    return namespace[method.name]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/so101_coordinate_repro.json"))
    args = parser.parse_args()
    host = Path(__file__).resolve().parents[1] / "src/lerobot/motors/motors_bus.py"
    host_bytes = host.read_bytes()
    calculate_home = host_home_function(host_bytes.decode("utf-8"))
    with zipfile.ZipFile(args.archive) as archive:
        name = next(n for n in archive.namelist() if n.endswith("/calibration_turn_replay.c"))
        reference = archive.read(name)
    text = reference.decode("utf-8")
    assert "delta > (int32_t)ENCODER_WRAP_THRESH" in text
    assert "param[2] = param[2] + 1" in text
    stub = SimpleNamespace(
        motors={"joint": SimpleNamespace(model="sts3215")},
        resolution=lambda model: 4096,
        _definition=lambda motor: SimpleNamespace(register=lambda name: SimpleNamespace(sign_bit=11)),
    )
    results = []
    for joint, home, minimum, maximum in [("shoulder_lift", -1772, 867, 3261), ("elbow_flex", -1219, 684, 2903)]:
        match = re.search(rf"static const int32_t {joint}_samples\[\]=\{{([^}}]+)\}};", text)
        assert match is not None
        samples = [int(v) for v in match[1].split(",")]
        assert len(samples) == 6452
        for condition, start, old_home in [("middle_start", 2047, 0), ("cross_boundary", minimum, 0), ("upper_start", maximum, 0), ("saved_home_at_boot", minimum, home)]:
            encoder = Encoder((start + home) % 4096, old_home)
            step = 1 if start < 2047 else -1
            for position in range(start, 2047, step):
                encoder.update(position + home)
            encoder.update(2047 + home)
            turns_before_home = encoder.turns
            encoder.set_home(0)
            calculated = calculate_home(stub, {"joint": encoder.position % 4096}, {"joint": -1})["joint"]
            assert calculated == home
            encoder.set_home(calculated)
            middle_full = encoder.position
            for position in samples:
                encoder.update(position + home)
            encoder.set_home(calculated)
            # In the reference, settled OFF seeds profile integrator to full
            # current position. Mode 0 uses the public goal as an absolute target.
            integrator = encoder.position
            goal = samples[-1]
            delta = goal - integrator
            expected = -4096 if condition == "cross_boundary" else 0
            assert delta == expected, (joint, condition, delta)
            assert encoder.turns == turns_before_home
            results.append(dict(joint=joint, condition=condition,
                manual_motion_degrees=round((2047-start)*360/4096, 3),
                home=calculated, turns_before_home=turns_before_home,
                middle_full=middle_full, range_samples=len(samples),
                final_display=encoder.position % 4096, final_full=encoder.position,
                first_same_pose_goal=goal, profile_target_minus_integrator=delta))
    report = dict(scope="Python coordinate port of supplied reference; actual host Home calculation; not firmware or hardware validation",
        host_sha256=hashlib.sha256(host_bytes).hexdigest(),
        reference_entry=name, reference_sha256=hashlib.sha256(reference).hexdigest(),
        cases=results, assertions_passed=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    for row in results:
        print(f"{row['joint']:14} {row['condition']:20} motion={row['manual_motion_degrees']:8.3f} deg  N={row['turns_before_home']}  delta={row['profile_target_minus_integrator']}")
    print(f"PASS: {len(results)} cases; report: {args.output}")


if __name__ == "__main__":
    main()
