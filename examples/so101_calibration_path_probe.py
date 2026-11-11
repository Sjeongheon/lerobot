"""ID2 torque-OFF calibration core via current source methods and a serial adapter.

No Goal, torque ON, RESET, or calibration-file writes. This does not exercise
rustypot transport, robot.connect/configure, or the complete robot CLI.
Create command.json containing {"command":"finish"} after the manual sweep.
Use --source with the pre-fix motors_bus.py (recorded commit 986b51ff).
The adapter deliberately does not implement the post-fix RESET operation.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import time
from pathlib import Path
from types import SimpleNamespace

from so101_manual_turn_experiment import ExperimentProbe
from so101_phase_probe import decode, signed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--home-zero-start', action='store_true')
    parser.add_argument('--source', type=Path, help='Pre-fix motors_bus.py to execute; no source is modified')
    args = parser.parse_args()
    source = args.source or Path(__file__).resolve().parents[1] / 'src/lerobot/motors/motors_bus.py'
    code = source.read_text(encoding='utf-8')
    cls = next(n for n in ast.parse(code).body if isinstance(n, ast.ClassDef) and n.name == 'SerialMotorsBus')
    names = {'reset_calibration', 'set_half_turn_homings', '_get_half_turn_homings',
             'record_ranges_of_motion', 'write_calibration', '_calibration_register_values',
             '_calibration_mismatches'}
    namespace = {'time': time, 'pformat': repr}
    nodes = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    if len(nodes) != len(names):
        parser.error('Calibration source does not provide the seven expected methods')
    if any(isinstance(node, ast.Attribute) and node.attr in ('reset_turn_count', 'supports_turn_count_reset')
           for method in nodes for node in ast.walk(method)):
        parser.error('This source includes the RESET fix. Pass --source pointing to pre-fix motors_bus.py (986b51ff); use so101_bus_calibration_check.py for the fixed path.')
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), *nodes], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), namespace)
    args.output.mkdir(parents=True, exist_ok=False)
    p = ExperimentProbe(args.port, args.output/'wire.jsonl')
    p.allowed_writes = set()
    report = {'source_sha256': hashlib.sha256(code.encode()).hexdigest(), 'scope': __doc__}
    original = None
    changed = False
    samples = (args.output/'samples.jsonl').open('x', encoding='utf-8')
    registers = {'Homing_Offset': (31,2), 'Min_Position_Limit': (9,2), 'Max_Position_Limit': (11,2), 'Present_Position': (56,2)}

    def save() -> None:
        (args.output/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')

    def snapshot(label: str) -> dict:
        row = dict(stage=label, time_ns=time.time_ns(), **decode(p.read(2,0,87)))
        if row['torque'] != 0:
            raise RuntimeError('Unexpected torque ON')
        samples.write(json.dumps(row)+'\n'); samples.flush()
        return row

    def write_register(self, name: str, motor: str, value: int, **kwargs) -> None:
        address, width = registers[name]
        if name == 'Present_Position':
            raise ValueError('Read only')
        value = int(value)
        if name == 'Homing_Offset':
            if abs(value) > 2047: raise ValueError('Invalid Home')
            value = abs(value) | (2048 if value < 0 else 0)
        elif not 0 <= value <= 4095:
            raise ValueError('Invalid limit')
        data = value.to_bytes(width,'little')
        p.allowed_writes.add(bytes([address])+data)
        p.write_bytes(address,data)

    def read_register(self, name: str, motor: str, **kwargs) -> int:
        address, width = registers[name]
        value = int.from_bytes(p.read(2,address,width),'little')
        return signed(value,11) if name == 'Homing_Offset' else value

    def sync_read(self, name: str, *args, **kwargs) -> dict:
        row = snapshot('range' if report.get('stage') == 'awaiting_range' else 'homing')
        return {'shoulder': row['present']}

    def finished() -> bool:
        if time.monotonic() > deadline:
            raise TimeoutError('Manual range timeout; restoring original calibration')
        command = args.output/'command.json'
        return command.exists() and json.loads(command.read_text(encoding='utf-8-sig'))['command'] == 'finish'

    namespace['enter_pressed'] = finished
    adapter = type('CalibrationAdapter', (), {name: namespace[name] for name in names})
    adapter.write = write_register; adapter.read = read_register; adapter.sync_read = sync_read
    adapter._get_motors_list = lambda self, motors=None: ['shoulder']
    adapter._get_motor_model = lambda self, motor: 'sts3215'
    adapter._has_register = lambda self, motor, name: name in registers
    adapter.resolution = lambda self, model: 4096
    adapter._definition = lambda self, motor: SimpleNamespace(homing_offset_sign=-1, register=lambda name: SimpleNamespace(sign_bit=11))
    bus = adapter(); bus.motors = {'shoulder': SimpleNamespace(model='sts3215')}
    try:
        report['torque_before'] = {str(i):p.read(i,40,1)[0] for i in range(1,7)}
        if any(report['torque_before'].values()): raise RuntimeError('All motors must be OFF')
        original = p.read(2,0,87); report['before'] = decode(original)
        if original[18] != 12 or original[33] != 0: raise RuntimeError('Expected Phase12/position mode')
        if args.home_zero_start:
            if not 850 <= report['before']['present'] <= 1200:
                raise RuntimeError('Low starting posture not confirmed')
        elif not 1700 <= report['before']['present'] <= 2400:
            raise RuntimeError('Middle posture not confirmed')
        p.allowed_writes.update({bytes([55,0]), bytes([55,original[55]]), bytes([18,12]),bytes([18,28])})
        changed = True
        p.write_bytes(55,b'\x00')
        p.write_bytes(18,b'\x1c'); report['extended_before'] = snapshot('extended_before')
        if args.home_zero_start:
            write_register(bus,'Homing_Offset','shoulder',0)
            report['home_zero_start'] = snapshot('home_zero_start')
            report['stage'] = 'awaiting_middle'; save()
            print(json.dumps(report),flush=True)
            middle_deadline = time.monotonic()+600
            while True:
                row = snapshot('awaiting_middle')
                command = args.output/'command.json'
                if command.exists():
                    action = json.loads(command.read_text(encoding='utf-8-sig'))['command']
                    command.unlink()
                    if action != 'middle':
                        raise InterruptedError('Operator cancelled middle movement')
                    physical_position = (row['present']-report['before']['home']) % 4096
                    if not 1700 <= physical_position <= 2400:
                        raise RuntimeError('Middle posture not confirmed after Home0 movement')
                    report['middle_home_zero'] = row
                    break
                if time.monotonic() > middle_deadline:
                    raise TimeoutError('Middle movement timed out')
                time.sleep(0.05)
        p.write_bytes(18,b'\x0c')
        homes = bus.set_half_turn_homings()
        report['homes'] = homes
        p.write_bytes(18,b'\x1c'); report['extended_after_homing'] = snapshot('extended_after_homing')
        p.write_bytes(18,b'\x0c')
        report['stage'] = 'awaiting_range'; save()
        print(json.dumps(report),flush=True)
        deadline = time.monotonic()+600
        mins,maxes = bus.record_ranges_of_motion(display_values=False)
        calibration = {'shoulder': SimpleNamespace(homing_offset=homes['shoulder'],range_min=mins['shoulder'],range_max=maxes['shoulder'])}
        bus.write_calibration(calibration)
        report['recorded_range'] = [mins['shoulder'],maxes['shoulder']]
        report['public_after'] = snapshot('public_after')
        p.write_bytes(18,b'\x1c'); report['extended_after'] = snapshot('extended_after')
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        errors = []
        if changed and original is not None:
            for address,width in [(31,2),(9,2),(11,2),(18,1),(55,1)]:
                try:
                    data = original[address:address+width]
                    p.allowed_writes.add(bytes([address])+data)
                    p.write_bytes(address,data)
                except Exception as exc: errors.append(str(exc))
            try:
                report['after_restore'] = snapshot('after_restore')
                report['torque_after'] = {str(i):p.read(i,40,1)[0] for i in range(1,7)}
            except Exception as exc: errors.append(str(exc))
        report['cleanup_errors'] = errors; report['stage'] = 'finished'; save()
        samples.close(); p.port.close(); p.log.close()
    print(json.dumps(report),flush=True)
    if report.get('error') or errors: raise SystemExit(1)


if __name__ == '__main__':
    main()
