# SO-101 shoulder_lift: the turn count an STS3215 keeps across a calibration

Records of the experiments behind the lerobot issue on the Feetech STS3215 retaining a one-turn
position offset after calibration. Every run was made on one SO-101 follower arm, on its
shoulder_lift motor (ID 2, STS3215, firmware 3.10), on 2026-11-11 unless noted. Nothing here
is needed to run LeRobot; the branch exists so the numbers in the issue can be checked against
the raw packets.

The symptom is not particular to this arm. Before this investigation, at least three other SO-101
arms had shown the same thing, a joint driving off at its first goal after a calibration, and the
cause was not identified at the time; the earliest record dates from 2026-10-17. Those earlier cases were not recorded at the packet level
and are not part of this branch; the arm here is the first one where the count was measured.

The branch follows the order of events: the experiments, then the three commits of the RESET
form of the fix (now `fix/feetech-turn-count-reset-instruction`), then B01, which ran
`write_calibration()` with that fix in place, then the round-trip runs (W01, W02) and B02, which
ran `write_calibration()` with the round-trip form of the fix, the one on the PR branch
(`fix/feetech-turn-count-reset`, commit f1c437a5). That commit is not on this branch, so the
`motors_bus.py` it ran with is kept as a fixture. There is no other code change.

## Layout

| Path | What it is |
|---|---|
| `examples/so101_*.py` | The scripts, byte for byte as they ran (not formatted to the repo's style on purpose). They import each other by module name, so run them from `examples/`. |
| `records/<run>/` | One folder per run: `result.json` (verdict and restoration state), `samples.jsonl` (position, torque, Home, Phase over time), `wire.jsonl` (every packet sent and received), sometimes `audit.json`. |
| `records/*.log` | Console output of W01 and W02; `<repo>` replaces the local checkout path. |
| `fixtures/motors_bus_986b51ff.py` | The `motors_bus.py` the calibration-path records were taken with (commit 986b51ff, read-back verification but no reset). |
| `fixtures/motors_bus_f1c437a5.py` | The `motors_bus.py` B02 was taken with (commit f1c437a5 of the PR branch: read-back verification and the `Operating_Mode` round-trip). |
| `tests/motors/test_so101_experiment_scripts.py` | Offline checks of the scripts over a register fixture; no hardware, no rustypot I/O. |

How to read a run: `result.json` first, then `samples.jsonl`, then `wire.jsonl` when a packet
matters. Logs are context, not measurements.

Setup: the motors were reached through a virtual COM pair (Robot IO Bridge; the scripts see
`COM21` or `COM22`), at 1 Mbps, with raw Feetech packets over pyserial except for B01 and B02,
which go through `SerialMotorsBus` and rustypot. Positions are ticks (0..4095 per turn; 2047
is the half-turn target). "Normal" is `Present_Position` with Phase bit 4 clear, "multi-turn"
the same register with the bit set; the two were read moments apart, not at the same instant.

## Runs

Only runs whose observation completed are kept; attempts that timed out, were refused by the
serial bridge or were aborted before anything was measured are not on the branch.

### Reproduction and the first goal

| ID | Record | Summary |
|---|---|---|
| T01 | `manual-turn` | Home 0, joint moved by hand through the encoder's zero: 3197 to 4393 multi-turn; 6165 once the original Home is written back; about 2070 after RESET (0x0A). Registers unchanged by RESET. |
| H01 | `hold-normal` | Normal reference: 2070 / 2070, goal 2069, stays at 2070, load 0. |
| H02 | `hold-retained-turn` | 2073 normal / 6169 multi-turn, goal 2069: moves 2073 to 2039 in 0.22 s, load 56 to 508, until the harness's 24-tick stop cuts the torque. Bounded by the stop, not the motor. |
| H03 | `hold-after-reset` | After RESET: 1996 / 1996, goal 1996, stays. Different starting position from H01. |
| | `first-goal-comparison.json` | Packets, settings and torque-off timing of H01 against H02. |

### Calibration path

| ID | Record | Summary |
|---|---|---|
| M01 | `coordinate-repro.json`, `current-calibration-model.json` | Offline port of the reference turn-tracking rule to the host's half-turn homing calculation, 8 conditions (2026-11-09). A model, not a measurement. |
| C01 | `calibration-path-baseline` | Existing Home before the move: range 859..3180, final 2115 / 2115. |
| C02 | `calibration-path-home-zero` | Home 0, zero crossed before homing: range 826..3208, final 1971 normal / 6067 multi-turn, every register as written. |
| R01 | `restore-then-reset` | Settings restored, then RESET: 1107 / 5203 to 1107 / 1107. |

C01 and C02 run the calibration core of `fixtures/motors_bus_986b51ff.py` (`set_half_turn_homings`,
`record_ranges_of_motion`, `write_calibration`) over a raw serial adapter, torque off, with no
Goal, RESET or calibration file. They do not exercise the LeRobot CLI or rustypot.

### Where the count lives

| ID | Record | Summary |
|---|---|---|
| P01 | `phase12-crossing` | Same crossing with Phase 12 (bit 4 clear) during the move: one 4080 to 21 wrap seen; 2500 normal / 6596 multi-turn after the original Home is written back. The count does not depend on the bit. |
| P02 | `power-cycle` | 12 V cut only: 17 unanswered reads during the cut; 6596 before, 2500 after, Phase still 28. The count is in RAM. |

### The fix on the real bus

| ID | Record | Summary |
|---|---|---|
| B01 | `bus-calibration-check-reset` | RESET form of the fix, through `SerialMotorsBus` and the rustypot branch that sends the instruction: 6143 multi-turn after homing, 731 / 731 after `write_calibration()`, registers as written, torque off; 868 / 868 after the original calibration is written back. |
| B02 | `bus-calibration-check-roundtrip` | 2026-11-12, round-trip form of the fix (`fixtures/motors_bus_f1c437a5.py`), through `SerialMotorsBus` and the released rustypot 1.13.1: 2047 normal / 6143 multi-turn after homing, 851 / 851 after `write_calibration()`, registers as written (Home -1752, 848 to 3234), torque off; 871 / 871 after the original calibration is written back. |

Same procedure, same result for the two forms of the fix.

### The workaround: an Operating_Mode round-trip (2026-11-12)

A comment on the issue suggested that switching the servo to velocity mode and back might
clear the count, since the fault had cleared on that arm without a power cycle. The
reimplementation agrees (`encoder_read()` re-initialises the tracker on the first read back in
position mode), and these runs check it on the motor. Torque off throughout; no Goal, no RESET.

| ID | Record | Summary |
|---|---|---|
| W01 | `mode-roundtrip-1` | Lock 0 throughout. 868 / 868 at the original Home; Home 0, joint moved by hand up through the wrap (4871); Home back: 2548 normal / 6644 multi-turn; `Operating_Mode` 1, about 0.5 s, `Operating_Mode` 0: 2548 / 2548. Registers as before. |
| W02 | `mode-roundtrip-2` | Same with Lock 1 written before the two mode writes and Lock 0 after, so nothing is saved to EEPROM; crossing downward (-902): 870 normal / -3226 multi-turn; after the round-trip 870 / 870. Registers as before. |

While the servo is in velocity mode, `Present_Position` reads the raw angle (776 in W01, which is
2548 - 1772; 3194 in W02): neither the homing offset nor the turn is applied in that mode.

## The issue's narrative, by record

1. A one-turn difference between the normal and multi-turn reading after moving through the
   encoder's zero: T01, P01.
2. It survives the calibration core, with every register as written: C02 against C01.
3. A goal four ticks away drives the motor a turn: H02 against H01 and H03.
4. The count is not the Phase bit and not EEPROM: P01, P02.
5. RESET clears it and keeps the registers: T01, R01; and through the bus after
   `write_calibration()`: B01.
6. An `Operating_Mode` round-trip (position, velocity, position) clears it the same way,
   with or without the EEPROM lock: W01, W02; and through the bus after
   `write_calibration()`: B02.

## Limits

- The measurements are from one arm, one joint and one firmware version; the earlier arms that
  showed the symptom were not measured this way. Private registers 67 and 71 were recorded as
  indicators only.
- The two readings of a position (bit clear, bit set) are consecutive reads, not simultaneous.
- Powered movement is measured between torque-on samples; sag after torque off is excluded.
- The scripts are experiment harnesses with operator steps (`command.json` in the output
  folder) and are not meant for reuse on an arm without someone at the power switch.
