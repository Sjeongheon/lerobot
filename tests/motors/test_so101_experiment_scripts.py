"""Offline execution checks, not a model validating STS3215 firmware behavior.

Every serial open and rustypot bus open is intercepted. Script workflows run
against a deterministic register fixture, with operator commands and faults.
"""
from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("serial")
pytest.importorskip("rustypot")

import rustypot
import serial

from lerobot.motors import SerialMotorsBus
from tests.mocks.mock_motors_bus import MockBus

pytestmark = pytest.mark.hardware
EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
SCRIPTS = (
    "so101_phase_probe",
    "so101_manual_turn_experiment",
    "so101_first_goal_probe",
    "so101_calibration_path_probe",
    "so101_phase12_crossing_experiment",
    "so101_power_cycle_experiment",
    "so101_bus_calibration_check",
)
sys.path.insert(0, str(EXAMPLES))
MODULES = {name: importlib.import_module(name) for name in SCRIPTS}


class Clock:
    def __init__(self) -> None:
        self.value = 1.0

    def monotonic(self) -> float:
        self.value += 0.005
        return self.value

    def sleep(self, seconds: float) -> None:
        self.value += seconds


class Registers:
    """Minimal workflow fixture; its turn behavior is assumed, not evidence."""

    def __init__(self, *, position: int = 2070, turns: int = 0) -> None:
        self.data = {i: bytearray(87) for i in range(1, 7)}
        self.physical = (position - 1772) % 4096
        self.turns = turns
        self.previous = position
        self.writes: list[tuple[int, int, bytes]] = []
        self.resets = 0
        self.raw_reads = 0
        self.closed = False
        self.hook: Any = None
        self.drop_reset_ack = False
        self.powered_drift = False
        self.inject_read: Any = None
        for data in self.data.values():
            data[18], data[55] = 12, 1
            self.word(data, 9, 867)
            self.word(data, 11, 3261)
            self.word(data, 31, 2048 + 1772)
            self.word(data, 42, 887)
            self.word(data, 46, 800)
        self.original = bytes(self.data[2])

    @staticmethod
    def word(data: bytearray, address: int, value: int) -> None:
        data[address : address + 2] = value.to_bytes(2, "little")

    def move_to(self, position: int) -> None:
        if position - self.previous < -2048:
            self.turns += 1
        elif position - self.previous > 2048:
            self.turns -= 1
        self.physical = (position + self.home) % 4096
        self.previous = position

    @property
    def home(self) -> int:
        value = int.from_bytes(self.data[2][31:33], "little")
        return -(value & 2047) if value & 2048 else value

    @property
    def present(self) -> int:
        return (self.physical - self.home) % 4096

    def read(self, motor: int, address: int, size: int) -> bytes | None:
        if self.hook:
            self.hook(motor, address, size)
        if self.inject_read:
            self.inject_read(motor, address, size)
        data = self.data[motor]
        if motor == 2 and address == 0 and size == 87:
            self.raw_reads += 1
            if data[40] and self.powered_drift:
                self.physical = (self.physical - 10) % 4096
        position = self.present + (4096 * self.turns if data[18] & 16 else 0)
        self.word(data, 56, position)
        self.word(data, 67, position)
        return bytes(data[address : address + size])

    def write(self, motor: int, instruction: int, params: bytes) -> None:
        if instruction == 10:
            assert motor == 2 and not params
            assert self.data[2][40] == 0
            self.resets += 1
            self.turns = 0
            return
        assert instruction == 3
        address, value = params[0], params[1:]
        self.writes.append((motor, address, value))
        if address not in (40, 42):
            assert self.data[motor][40] == 0, "Configuration while powered"
        if address in (9, 11, 18, 31) and self.data[motor][55]:
            raise RuntimeError("EEPROM is locked")
        self.data[motor][address : address + len(value)] = value
        if address == 42:
            self.data[motor][40] = 1  # exercise the observed Goal-write side effect
        if address == 31:
            self.previous = self.present


class FakeSerial:
    def __init__(self, registers: Registers, clock: Clock) -> None:
        self.registers, self.clock = registers, clock
        self.pending = bytearray()
        self.closed = False
        self.corrupt: Any = None

    @property
    def in_waiting(self) -> int:
        return len(self.pending)

    def reset_input_buffer(self) -> None:
        self.pending.clear()

    def write(self, request: bytes) -> int:
        assert request[:2] == b"\xff\xff" and sum(request[2:]) & 255 == 255
        motor, instruction = request[2], request[4]
        params = request[5:-1]
        if instruction == 2:
            value = self.registers.read(motor, *params)
            if value is None:
                return len(request)
        else:
            self.registers.write(motor, instruction, params)
            if instruction == 10 and self.registers.drop_reset_ack:
                return len(request)
            value = b""
        body = bytes([motor, len(value) + 2, 0]) + value
        response = b"\xff\xff" + body + bytes([~sum(body) & 255])
        self.pending.extend(self.corrupt(response) if self.corrupt else response)
        return len(request)

    def read(self, size: int) -> bytes:
        self.clock.value += 0.02
        value = bytes(self.pending[:size])
        del self.pending[:size]
        return value

    def close(self) -> None:
        self.closed = True


class Operator:
    def __init__(self, output: Path, registers: Registers, mode: str) -> None:
        self.output, self.registers, self.mode = output, registers, mode
        self.counts: dict[str, int] = {}
        self.completed: set[str] = set()

    def __call__(self, motor: int, address: int, size: int) -> None:
        if motor != 2 or address != 0 or size != 87:
            return
        result = self.output / "result.json"
        if not result.exists():
            return
        stage = json.loads(result.read_text(encoding="utf-8")).get("stage", "")
        if stage in self.completed:
            return
        self.counts[stage] = n = self.counts.get(stage, 0) + 1
        action = None
        if stage == "awaiting_manual_move":
            self.registers.move_to(4090 if n == 1 else 300)
            if n >= 2:
                action = "restore_home"
        elif stage in ("awaiting_reset_comparison", "awaiting_cleanup"):
            action = "finish" if self.mode in ("phase12", "power") else "reset"
        elif stage == "awaiting_middle":
            self.registers.move_to(300)
            action = "middle"
        elif stage in ("awaiting_range", "awaiting_sweep"):
            self.registers.move_to((900, 2000, 3100, 2047)[min(n - 1, 3)])
            if n >= 4:
                action = "swept" if stage == "awaiting_sweep" else "finish"
        elif stage == "awaiting_power_cycle":
            if n == 1:
                raise TimeoutError("Simulated communication gap")
            self.registers.turns = 0
            self.registers.data[2][42:44] = b"\x00\x00"
            self.registers.data[2][55] = 1
            action = "power_cycled"
        if action and not (self.output / "command.json").exists():
            (self.output / "command.json").write_text(json.dumps({"command": action}), encoding="utf-8")
            self.completed.add(stage)


class FakeNativeBus(MockBus):
    """Actual Python bus methods over register fixtures, with no rustypot I/O."""

    addresses = {
        "homing_offset": (31, 2), "min_position_limit": (9, 2), "max_position_limit": (11, 2),
        "phase": (18, 1), "lock": (55, 1), "torque_enable": (40, 1), "present_position": (56, 2),
    }

    def __init__(self, motors, registers: Registers, clear_on_reset: bool = True):
        super().__init__(motors=motors)
        self.fixture = registers
        self.clear_on_reset = clear_on_reset
        for motor in range(1, 7):
            self.seed(motor, "model_number", 777)
            self.seed(motor, "firmware_major_version", 3)
            self.seed(motor, "firmware_minor_version", 10)

    def stored(self, motor_id: int, register: str) -> int:
        if register in self.addresses:
            address, count = self.addresses[register]
            if register == "present_position" and motor_id == 2 and self.fixture.hook:
                self.fixture.hook(2, 0, 87)
            value = int.from_bytes(self.fixture.read(motor_id, address, count), "little")
            if register == "homing_offset" and value & 2048:
                return -(value & 2047)
            return value
        return super().stored(motor_id, register)

    def write_register_with_error(self, motor_id: int, register: str, value: int, retries: int = 0) -> int:
        status = super().write_register_with_error(motor_id, register, value, retries)
        if register in self.addresses:
            address, count = self.addresses[register]
            encoded = abs(value) | (2048 if value < 0 else 0) if register == "homing_offset" else value
            self.fixture.write(motor_id, 3, bytes([address]) + encoded.to_bytes(count, "little"))
        return status

    def set_torque(self, motor_ids, enabled: bool, retries: int = 0):
        failed = super().set_torque(motor_ids, enabled, retries)
        for motor in motor_ids:
            if motor not in failed:
                self.fixture.data[motor][40] = int(enabled)
        return failed

    def reset_turn_count(self, motor_ids, retries: int = 0):
        failed = super().reset_turn_count(motor_ids, retries)
        if 2 in motor_ids and 2 not in failed and self.clear_on_reset:
            self.fixture.turns = 0
        return failed


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    def blocked(*args, **kwargs):
        raise AssertionError("An offline test tried opening real hardware")

    monkeypatch.setattr(serial, "Serial", blocked)
    monkeypatch.setattr(SerialMotorsBus, "_open_bus", blocked)
    monkeypatch.setattr(rustypot, "Bus", blocked)


@pytest.fixture
def harness(monkeypatch, tmp_path):
    clock = Clock()
    regs = Registers()
    ports = []

    def open_serial(*args, **kwargs):
        port = FakeSerial(regs, clock)
        ports.append(port)
        return port

    monkeypatch.setattr(serial, "Serial", open_serial)
    for module in MODULES.values():
        if hasattr(module, "time"):
            monkeypatch.setattr(module, "time", clock)
    clock.time_ns = lambda: int(clock.value * 1_000_000_000)

    def run(name: str, extra: list[str] | None = None, mode: str | None = None):
        output = tmp_path / f"run-{len(list(tmp_path.glob('run-*')))}"
        regs.hook = Operator(output, regs, mode) if mode else None
        monkeypatch.setattr(sys, "argv", [name, "--port", "OFFLINE", "--output", str(output), *(extra or [])])
        MODULES[name].main()
        return json.loads((output / "result.json").read_text(encoding="utf-8"))

    return regs, ports, run, clock


@pytest.fixture
def baseline_source():
    """The motors_bus.py the calibration-path records were taken with (commit 986b51ff, before the reset)."""
    return EXAMPLES.parent / "experiments" / "so101-turn-count-20261111" / "fixtures" / "motors_bus_986b51ff.py"


@pytest.mark.parametrize("name", SCRIPTS)
def test_help_does_not_open_hardware(monkeypatch, name):
    monkeypatch.setattr(sys, "argv", [name, "--help"])
    with pytest.raises(SystemExit) as exc:
        MODULES[name].main()
    assert exc.value.code == 0


def test_phase_probe_round_trip(harness):
    regs, ports, run, _ = harness
    result = run("so101_phase_probe")
    assert result["preserved_registers"]
    assert all(v == 0 for v in result["torque_after"].values())
    assert {address for _, address, _ in regs.writes} <= {18, 55}
    assert all(p.closed for p in ports)


def test_manual_home_reset_and_restore(harness):
    regs, ports, run, _ = harness
    regs.move_to(900)
    result = run("so101_manual_turn_experiment", mode="manual")
    assert result["reset_ack"] and regs.resets == 1
    assert result["preserved_registers"] and not result["cleanup_errors"]
    assert not any(address in (40, 42) for _, address, _ in regs.writes)
    assert all(p.closed for p in ports)


def test_phase12_crossing_is_observed_without_powering(harness):
    regs, ports, run, _ = harness
    regs.move_to(900)
    result = run("so101_phase12_crossing_experiment", mode="phase12")
    assert result["verdict"]["crossings_seen_under_phase12"] >= 1
    assert result["verdict"]["extended_minus_display"] == 4096
    assert regs.resets == 0
    assert not any(address in (40, 42) for _, address, _ in regs.writes)
    assert result["preserved_registers"] and all(p.closed for p in ports)


@pytest.mark.parametrize("turn,drift,reset_first", [(0, False, False), (1, True, False), (1, False, True)])
def test_hold_powers_off_and_never_replays_old_goal(harness, turn, drift, reset_first):
    regs, ports, run, _ = harness
    regs.turns, regs.powered_drift = turn, drift
    args = ["--target", "2069", "--expected-turn", "0" if reset_first else str(turn)]
    if reset_first:
        args.append("--reset-before-hold")
    result = run("so101_first_goal_probe", args)
    assert result["off_confirmed"] and result["static_restored"]
    assert all(v == 0 for v in result["torque_after"].values())
    assert [value for _, address, value in regs.writes if address == 42] == [int(2069).to_bytes(2, "little")]
    assert regs.resets == int(turn == 1)
    if drift:
        assert result["stop_reason"] == "displacement_limit"
    assert all(p.closed for p in ports)


def test_power_cycle_workflow_keeps_goal_reset_separate(harness):
    regs, ports, run, _ = harness
    regs.turns = 1
    result = run("so101_power_cycle_experiment", mode="power")
    assert result["verdict"]["power_down_observed"]
    assert not result["verdict"]["turn_survives_power_cycle"]
    assert result["preserved_registers"] and not result["goal_preserved"]
    assert regs.resets == 0
    assert not any(address in (40, 42) for _, address, _ in regs.writes)
    assert all(p.closed for p in ports)


def test_calibration_adapter_executes_recorded_source(harness, baseline_source):
    regs, ports, run, _ = harness
    result = run("so101_calibration_path_probe", ["--source", str(baseline_source)], mode="calibration")
    assert not result.get("error") and result["recorded_range"] == [900, 3100]
    assert result["source_sha256"] == "fc0875d8c4ff98f302ac7ac0313703b47b2aa28c9f9a9f3afe2692366f7bae2c"
    assert result["after_restore"]["home"] == -1772
    assert all(v == 0 for v in result["torque_after"].values())
    assert not any(address in (40, 42) for _, address, _ in regs.writes)
    assert regs.resets == 0 and all(p.closed for p in ports)


def test_adapter_rejects_incompatible_source_before_serial_open(harness):
    _, ports, run, _ = harness
    with pytest.raises(SystemExit):
        run("so101_calibration_path_probe")
    assert ports == []


def test_malformed_status_frame_is_reported(harness, tmp_path):
    _, _, _, _ = harness
    module = MODULES["so101_phase_probe"]
    p = module.Probe("OFFLINE", tmp_path / "wire.jsonl")
    p.port.corrupt = lambda _: b"\xff\xff\x02\x00\xfd"
    try:
        with pytest.raises(RuntimeError, match="length|frame"):
            p.read(2, 40, 1)
    finally:
        p.port.close()
        p.log.close()


def test_unarmed_packets_are_rejected():
    module = MODULES["so101_first_goal_probe"]
    p = object.__new__(module.HoldProbe)
    p.config_writes = set()
    p.target = 2069
    p.goal_armed = p.on_armed = p.reset_armed = False
    for motor, instruction, data in [(2, 3, b"\x28\x01"), (2, 10, b""), (3, 3, b"\x28\x01"), (254, 10, b"")]:
        with pytest.raises(ValueError):
            p.validate_request(motor, instruction, data)


@pytest.mark.parametrize("clears,expected,success", [(True, "cleared", True), (False, "cleared", False), (False, "retained", True)])
def test_bus_calibration_result_and_restoration(harness, monkeypatch, clears, expected, success):
    regs, _, run, _ = harness
    regs.move_to(900)
    native = []

    def open_bus(bus):
        transport = FakeNativeBus(bus._definitions, regs, clears)
        native.append(transport)
        return transport

    monkeypatch.setattr(SerialMotorsBus, "_open_bus", open_bus)
    if success:
        result = run("so101_bus_calibration_check", ["--expect-turn-state", expected], mode="bus")
        assert result["verdict"]["turn_count_zero_after_write_calibration"] is clears
        assert not result["cleanup_errors"]
        assert all(v == 0 for v in result["torque_after"].values())
    else:
        with pytest.raises(RuntimeError, match="expected turn state"):
            run("so101_bus_calibration_check", ["--expect-turn-state", expected], mode="bus")
    assert native and native[0].closed
    assert int.from_bytes(regs.data[2][31:33], "little") == 2048 + 1772
    assert int.from_bytes(regs.data[2][9:11], "little") == 867
    assert int.from_bytes(regs.data[2][11:13], "little") == 3261
    assert regs.data[2][18] == 12 and regs.data[2][55] == 1


@pytest.mark.parametrize("name", ["so101_phase_probe", "so101_manual_turn_experiment", "so101_phase12_crossing_experiment", "so101_power_cycle_experiment"])
def test_nonzero_preflight_torque_sends_no_configuration(harness, name):
    regs, ports, run, _ = harness
    regs.data[3][40] = 1
    with pytest.raises(RuntimeError, match="OFF"):
        run(name)
    assert regs.writes == [] and all(p.closed for p in ports)


@pytest.mark.parametrize("failure", [TimeoutError("Injected read timeout"), KeyboardInterrupt()])
def test_phase_probe_restores_after_interrupted_measurement(harness, failure):
    regs, ports, run, _ = harness
    failed = False

    def fail_once(motor, address, size):
        nonlocal failed
        if motor == 2 and address == 0 and regs.data[2][18] == 28 and not failed:
            failed = True
            raise failure

    regs.inject_read = fail_once
    with pytest.raises(type(failure)):
        run("so101_phase_probe")
    assert regs.data[2][18] == 12 and regs.data[2][55] == 1
    assert all(p.closed for p in ports)


def test_uncertain_hold_reset_never_powers_or_retries(harness):
    regs, ports, run, _ = harness
    regs.turns = 1
    regs.drop_reset_ack = True
    with pytest.raises(SystemExit):
        run("so101_first_goal_probe", ["--target", "2069", "--expected-turn", "0", "--reset-before-hold"])
    assert regs.resets == 1
    assert not any(address in (40, 42) for _, address, _ in regs.writes)
    assert all(p.closed for p in ports)


def test_hold_goal_timeout_still_sends_off(harness):
    regs, ports, run, _ = harness
    failed = False

    def fail_once(motor, address, size):
        nonlocal failed
        if regs.data[2][40] and not failed:
            failed = True
            raise TimeoutError("Lost response after Goal enabled torque")

    regs.inject_read = fail_once
    with pytest.raises(SystemExit):
        run("so101_first_goal_probe", ["--target", "2069", "--expected-turn", "0"])
    assert regs.data[2][40] == 0
    assert (2, 40, b"\x00") in regs.writes
    assert all(p.closed for p in ports)


@pytest.mark.parametrize("fault", ["checksum", "wrong_id", "short_payload", "status"])
def test_bad_serial_reply_is_not_accepted(harness, tmp_path, fault):
    _, _, _, _ = harness
    module = MODULES["so101_phase_probe"]
    p = module.Probe("OFFLINE", tmp_path / "reply.jsonl")

    def corrupt(response):
        body = bytearray(response[2:-1])
        if fault == "checksum":
            return response[:-1] + bytes([response[-1] ^ 1])
        if fault == "wrong_id":
            body[0] = 3
        elif fault == "short_payload":
            body = body[:3]
            body[1] = 2
        else:
            body[2] = 32
        return b"\xff\xff" + body + bytes([~sum(body) & 255])

    p.port.corrupt = corrupt
    try:
        with pytest.raises(RuntimeError):
            p.read(2, 40, 1)
    finally:
        p.port.close()
        p.log.close()


@pytest.mark.parametrize("condition", ["no_gap", "display_mode"])
def test_power_cycle_does_not_claim_success_without_valid_observation(harness, condition):
    regs, ports, run, _ = harness
    regs.turns = 1
    base_read = regs.read

    def read(motor, address, size):
        try:
            value = base_read(motor, address, size)
        except TimeoutError:
            if condition != "no_gap":
                raise
            return bytes(regs.data[motor][address:address+size])
        if condition == "display_mode" and regs.turns == 0:
            regs.data[2][18] = 12
            if motor == 2 and address == 0:
                value = bytes(regs.data[2])
        return value

    regs.read = read
    with pytest.raises(RuntimeError, match="communication gap|not an extended"):
        run("so101_power_cycle_experiment", mode="power")
    assert all(p.closed for p in ports)


@pytest.mark.parametrize("name", ["so101_manual_turn_experiment", "so101_phase12_crossing_experiment"])
def test_manual_wait_timeout_restores_configuration(harness, name):
    regs, ports, run, clock = harness
    regs.move_to(900)
    clock.sleep = lambda seconds: setattr(clock, "value", clock.value + 1000)
    with pytest.raises(TimeoutError):
        run(name)
    assert regs.data[2][31:33] == regs.original[31:33]
    assert regs.data[2][18] == 12 and regs.data[2][55] == 1
    assert not any(address in (40, 42) for _, address, _ in regs.writes)
    assert all(p.closed for p in ports)


def test_adapter_home_zero_workflow_retains_turn_in_fixture(harness, baseline_source):
    regs, _, run, _ = harness
    regs.move_to(900)
    result = run("so101_calibration_path_probe", ["--source", str(baseline_source), "--home-zero-start"], mode="calibration")
    assert result["extended_after"]["present"] - result["public_after"]["present"] == 4096
    assert result["recorded_range"] == [900, 3100]
    assert not result["cleanup_errors"] and regs.resets == 0


def test_adapter_preflight_failure_does_not_write(harness, baseline_source):
    regs, ports, run, _ = harness
    regs.move_to(900)
    with pytest.raises(SystemExit):
        run("so101_calibration_path_probe", ["--source", str(baseline_source)])
    assert regs.writes == [] and all(p.closed for p in ports)


def test_decode_rejects_incomplete_snapshot():
    with pytest.raises(ValueError, match="87-byte"):
        MODULES["so101_phase_probe"].decode(bytes(86))
