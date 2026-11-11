# SO-101 shoulder_lift: the turn count an STS3215 keeps across a calibration

Records of the experiments behind the lerobot issue on the Feetech STS3215 retaining a one-turn
position offset after calibration. Every run was made on one SO-101 follower arm, on its
shoulder_lift motor (ID 2, STS3215, firmware 3.10), on 2026-11-11 unless noted. Nothing here
is needed to run LeRobot; the branch exists so the numbers in the issue can be checked against
the raw packets.

The symptom is not particular to this arm. Before this investigation, at least three other SO-101
arms had shown the same thing, a joint driving off at its first goal after a calibration, and the
cause was not identified at the time. Those earlier cases were not recorded at the packet level
and are not part of this branch; the arm here is the first one where the count was measured.

The branch follows the order of events: the experiments, then the three commits of the fix
branch (`fix/feetech-turn-count-reset`, the same commits as its PR), then B02, which ran
`write_calibration()` with the fix in place. There is no other code change.

## Layout

| Path | What it is |
|---|---|
| `examples/so101_*.py` | The scripts, byte for byte as they ran (not formatted to the repo's style on purpose). They import each other by module name, so run them from `examples/`. |
| `records/<run>/` | One folder per run: `result.json` (verdict, error, restoration state), `samples.jsonl` (position, torque, Home, Phase over time), `wire.jsonl` (every packet sent and received), sometimes `audit.json`. |
| `records/*.log` | Console output of the runs that had it; `<repo>` replaces the local checkout path. |
| `fixtures/motors_bus_986b51ff.py` | The `motors_bus.py` the calibration-path records were taken with (commit 986b51ff, read-back verification but no reset). |
| `tests/motors/test_so101_experiment_scripts.py` | Offline checks of the scripts over a register fixture; no hardware, no rustypot I/O. |

How to read a run: `result.json` first, then `samples.jsonl`, then `wire.jsonl` when a packet
matters. Logs and drafts are context, not measurements.

Setup: the motors were reached through a virtual COM pair (Robot IO Bridge; the scripts see
`COM21` or `COM22`), at 1 Mbps, with raw Feetech packets over pyserial except for B01/B02,
which go through `SerialMotorsBus` and rustypot. Positions are ticks (0..4095 per turn; 2047
is the half-turn target). "Normal" is `Present_Position` with Phase bit 4 clear, "multi-turn"
the same register with the bit set; the two were read moments apart, not at the same instant.

## Runs

Status says whether the procedure's observation completed, nothing more.

### Reproduction and the first goal

| ID | Status | Record | Summary |
|---|---|---|---|
| S06 | probe | `phase-probe`, `20261110-phase-probe` | Reading Phase bit 4 and restoring it; the 11-10 folder is the rehearsal over the bridge. |
| T01 | done | `manual-turn` | Home 0, joint moved by hand through the encoder's zero: 3197 to 4393 multi-turn; 6165 once the original Home is written back; about 2070 after RESET (0x0A). Registers unchanged by RESET. |
| H01 | done | `hold-normal` | Normal reference: 2070 / 2070, goal 2069, stays at 2070, load 0. |
| H02a | prepared | `prepare-retained-hold` | Recreates the retained turn for H02; not a powered test itself. |
| H02 | done | `hold-retained-turn` | 2073 normal / 6169 multi-turn, goal 2069: moves 2073 to 2039 in 0.22 s, load 56 to 508, until the harness's 24-tick stop cuts the torque. Bounded by the stop, not the motor. |
| H03 | done | `hold-after-reset` | After RESET: 1996 / 1996, goal 1996, stays. Different starting position from H01. |
| | | `first-goal-comparison.json` | Packets, settings and torque-off timing of H01 against H02. |

### Calibration path

| ID | Status | Record | Summary |
|---|---|---|---|
| M01 | model | `coordinate-repro.json`, `current-calibration-model.json` | Offline port of the reference turn-tracking rule to the host's half-turn homing calculation, 8 conditions (2026-11-09). A model, not a measurement. |
| C00 | aborted | `recreate-retained-v2` | Direction changed while waiting at Home 0; nothing driven. |
| C01 | timed out | `calibration-path-baseline-1` | 3103 samples, 2044..3319 ticks; restored before the final writes. |
| C01a | check | `calibration-baseline-after-timeout` | Multi-turn 2054 and settings after the C01 restoration. |
| C02 | done | `calibration-path-baseline-2` | Existing Home before the move: range 859..3180, final 2115 / 2115. |
| C03 | done | `calibration-path-home-zero-1` | Home 0, zero crossed before homing: range 826..3208, final 1971 normal / 6067 multi-turn, every register as written. |
| R01 | restored | `restore-20261111-111803` | Settings restored, then RESET: 1107 / 5203 to 1107 / 1107. |
| S01, S02 | preflight | `actual-calibration-baseline`, `calibration-home-zero-preflight` | State reads before C02 and C03. |

C01 to C03 run the calibration core of `fixtures/motors_bus_986b51ff.py` (`set_half_turn_homings`,
`record_ranges_of_motion`, `write_calibration`) over a raw serial adapter, torque off, with no
Goal, RESET or calibration file. They do not exercise the LeRobot CLI or rustypot.

### Where the count lives

| ID | Status | Record | Summary |
|---|---|---|---|
| P01 | done | `phase12-crossing-1` | Same crossing with Phase 12 (bit 4 clear) during the move: one 4080 to 21 wrap seen; 2500 normal / 6596 multi-turn after the original Home is written back. The count does not depend on the bit. |
| P02 | invalid | `power-cycle-1` | The USB bridge was cut with the motor power and the serial handle died; `power-cycle-1-recovery*.log` and the bridge probes are the recovery. Not evidence. |
| P03 | done | `power-cycle-2` | 12 V cut only: 17 unanswered reads during the cut; 6596 before, 2500 after, Phase still 28. The count is in RAM. |

### The fix on the real bus

| ID | Status | Record | Summary |
|---|---|---|---|
| B01 | timed out | `bus-calibration-check-1` | Waiting for the middle pose; restoration checked. |
| B02 | done | `bus-calibration-check-2` | Through `SerialMotorsBus` and the rustypot branch: 6143 multi-turn after homing, 731 / 731 after `write_calibration()`, registers as written, torque off; 868 / 868 after the original calibration is written back. |

### Range attempts, excluded from the evidence

| ID | Status | Record | Summary |
|---|---|---|---|
| G01 | harness stop | `range-lower-interior` | Gap between the 8-tick progress and 12-tick stall rules; separate torque-off recovery. |
| G02 | segment done | `range-lower-interior-v2` | 874 to about 912, target 917. Not a full descent. |
| G03 | timeout | `range-upper-interior` | About 912 to 1202; status response timeout. |
| G04 | stall | `range-upper-interior-v2` | About 1202 to 1366; interference at the arm reported. |
| G05 | not started | `range-held-upper-v1` | Start 866 below the 867 lower limit. |
| G06 | stall | `range-held-small-v1` | Other joints held; 866 to 900, target 917. |
| G07 | observed | `range-held-observe-v1` | 875 to 902, target 917, 3 s of observation; a small steady tracking error, separate from the 4096-tick offset. |
| | | `full-range-1022*`, `range-progress.json` | Three earlier full-range attempts (threshold stop, interrupted, status error 32) and the progress notes. |

No run completed a full powered sweep of the joint; these are kept so the exclusion can be
checked.

## The issue's narrative, by record

1. A one-turn difference between the normal and multi-turn reading after moving through the
   encoder's zero: T01, P01.
2. It survives the calibration core, with every register as written: C03 against C02.
3. A goal four ticks away drives the motor a turn: H02 against H01 and H03.
4. The count is not the Phase bit and not EEPROM: P01, P03.
5. RESET clears it and keeps the registers: T01, R01; and through the bus after
   `write_calibration()`: B02.

## Limits

- The measurements are from one arm, one joint and one firmware version; the earlier arms that
  showed the symptom were not measured this way. Private registers 67 and 71 were recorded as
  indicators only.
- The two readings of a position (bit clear, bit set) are consecutive reads, not simultaneous.
- Powered movement is measured between torque-on samples; sag after torque off is excluded.
- The scripts are experiment harnesses with operator steps (`command.json` in the output
  folder) and are not meant for reuse on an arm without someone at the power switch.
