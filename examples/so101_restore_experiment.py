"""Restore ID2 experiment settings and clear turns, with all motors OFF."""
import json
import time
from pathlib import Path

from so101_manual_turn_experiment import ExperimentProbe
from so101_phase_probe import decode


class RestoreProbe(ExperimentProbe):
    def validate_request(self, motor: int, instruction: int, parameters: bytes) -> None:
        if motor in range(1, 7) and instruction == 3 and parameters == b'\x28\x00':
            return
        super().validate_request(motor, instruction, parameters)


def main() -> None:
    root = Path('outputs/so101-experiment-20261111')
    out = root / ('restore-' + time.strftime('%Y%m%d-%H%M%S'))
    out.mkdir()
    reference = json.loads((root/'calibration-path-home-zero-1/result.json').read_text())
    original = bytes.fromhex(reference['before']['raw_hex'])
    p = RestoreProbe('COM21',out/'wire.jsonl')
    p.allowed_writes = {bytes([55,0]),bytes([18,28])}
    settings = [(31,2),(9,2),(11,2),(46,2),(18,1),(55,1)]
    p.allowed_writes.update(bytes([a])+original[a:a+n] for a,n in settings)
    report = {}
    try:
        report['before'] = {str(i):decode(p.read(i,0,87)) for i in range(1,7)}
        for i in range(1,7):
            if p.read(i,40,1) != b'\x00':
                p.exchange(i,3,b'\x28\x00')
            if p.read(i,40,1) != b'\x00': raise RuntimeError(f'ID{i} OFF unconfirmed')
        p.write_bytes(55,b'\x00')
        for a,n in settings[:-2]:
            if p.read(2,a,n) != original[a:a+n]:
                p.write_bytes(a,original[a:a+n])
        p.write_bytes(18,b'\x1c')
        report['extended_before_reset'] = decode(p.read(2,0,87))
        if p.read(2,40,1) != b'\x00': raise RuntimeError('RESET requires OFF')
        p.reset_allowed = True
        report['reset_sent'] = True
        p.exchange(2,10,b'')
        report['reset_ack'] = True
        time.sleep(0.2)
        report['extended_after_reset'] = decode(p.read(2,0,87))
        pos = report['extended_after_reset']['present']
        if not 0 <= pos <= 4095: raise RuntimeError('Extended position still outside one turn')
    except BaseException as exc:
        report['error'] = repr(exc)
    finally:
        errors = []
        for a in (18,55):
            try: p.write_bytes(a,original[a:a+1])
            except Exception as exc: errors.append(repr(exc))
        try:
            report['after'] = {str(i):decode(p.read(i,0,87)) for i in range(1,7)}
            report['settings_verified'] = all(p.read(2,a,n)==original[a:a+n] for a,n in settings)
            report['all_off'] = all(row['torque']==0 for row in report['after'].values())
        except Exception as exc: errors.append(repr(exc))
        report['cleanup_errors'] = errors
        (out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        p.port.close(); p.log.close()
    print(json.dumps(report))
    if report.get('error') or errors or not report.get('all_off') or not report.get('settings_verified'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
