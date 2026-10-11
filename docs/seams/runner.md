# Seam: adapters, the runner and its watchdog (kit, B1 + B3)

**DRAFT. Requires line-by-line human review before any of it drives hardware.** Written to be
folded into CONTRACT.md. Everything here is implemented in
`packages/kit/src/bounded_embodiment_kit/{adapters,runner}/` and tested without hardware in
`packages/kit/tests/test_runner_*.py`. "Proposed" marks what other workstreams still have to adopt.

## Modules

| Module | What it does |
|---|---|
| `adapters/base.py` | The adapter contract: `Adapter`, `SendResult`, `State`, `AdapterContext`, `load_callable` |
| `adapters/feetech.py` | SO-10x safety code: `connect_safely`, `choose_stop_goals`, `check_bus_is_ours`, `leave_safe` (typed OFF) |
| `adapters/lerobot.py` | `lerobot` adapter (LeRobot imported lazily; other LeRobot robots refused in v0.1) |
| `adapters/sim_so101.py` | `sim-so101`: the real `lerobot` adapter over a simulated SO-101 (port of ev03-harness `sim.py`) |
| `adapters/sim_rover.py` | `sim-rover` (port of ev03-harness `SimRover`) |
| `adapters/python.py` | `python` adapter loader and the `be adapter new` template |
| `runner/limits.py` | every timeout, margin and rate (the table below) |
| `runner/feeds.py` | `InstrumentFeed`, `UDPFeed`, `SimFeed` |
| `runner/kin.py` | `ArmFK`: state -> joint map -> URDF -> envelope points |
| `runner/watchdog.py` | `StopLatch`, the tripwires (pure functions), stop acknowledgement, `Escalation`, `Alarm` |
| `runner/interlocks.py` | flags, typed words, setup hash, shakedown ledger |
| `runner/session.py` | the command thread, the raw-data writers, the start/stop sequence |
| `runner/run.py` | `be run ev03` and `be goto-start` |
| `runner/doctor.py` | `be doctor` (read-only) |
| `runner/cli.py` | registers `run`, `goto-start`, `doctor`, `adapter` (in `bounded_embodiment_kit.cli.MODULES`) |

## The adapter contract

```python
class Adapter(Protocol):
    def open(self) -> CommandSpace: ...         # main thread: connect safely; the command space from the declared file
    def describe(self) -> dict: ...            # pure data; describe()["config"] goes into the setup hash
    def send(self, cmd: dict) -> SendResult: ... # command thread: hand cmd to the system under test; NEVER raises
    def read_state(self) -> State | None: ...   # command thread: encoder joint state; None = no state (no FK)
    def latch_stop(self) -> None: ...           # ANY thread (the watchdog): set a flag; no bus, no lock, no blocking
    def execute_stop(self) -> None: ...         # command thread: the stop, under the bus lock
    def close(self) -> None: ...                # main thread: torque stays on unless the operator types OFF

@dataclass(frozen=True)
class SendResult: applied: bool; decision: str | None = None; error_kind: str | None = None
@dataclass(frozen=True)
class State: joints: Mapping[str, float]; units: Mapping[str, str]   # "deg"|"rad"|"percent"|"m"|"m/s"|"rad/s"
```

- `execute_stop()` is a seventh method, not in the plan's six. The plan says the command thread
  executes the stop; it needs a named entry point, and overloading `send()` would be cleverness.
  `send()` also refuses every command once latched (`error_kind="stop_latched"`), as a backstop.
- `decision` is the system under test's own word (`wrapper.last_decision` if the wrapper has one),
  logged in `commands.csv`, never scored. `error_kind` values: `stop_latched`, `not_open`,
  `exception:<Type>` (the system under test raised; the hostile stream expects that), and
  `adapter_raised:<Type>` (the adapter broke its contract: the runner trips the latch).
- `describe()` keys the runner reads: `simulation` (bool), `config` (dict, setup hash),
  `joint_speed_trip_per_s` (joint -> limit in the joint's own units per second; the SO-101's is
  150 deg/s for the five body joints).
- Sim adapters also have `truth_fn(fk, envelope) -> () -> {point: (x, y, z)}` for `--instrument sim`.

**The lerobot wrapper (how a lab's gate plugs in).** `--wrapper module:factory` or
`--wrapper path/to/file.py:factory`, called as `factory(robot, context)` after the adapter has
connected the inner robot's bus itself. It returns anything with `send_action(action) -> dict`.
`context` is plain data: `mode`, `speed_factor` (0.5 in a shakedown, else `None` = "your default"),
`envelope_path`, `command_space_path`, `kit_config_path`, `urdf_path`. The adapter never calls the
wrapper's `connect()`, `configure()` or `disconnect()`, nor the inner robot's `connect()` or
`disconnect()`. Stops, state reads and the release go to the inner robot's bus. **Proposed** for
`examples/reference-gates/`: factories take `(robot, context)`.

**Safe connect (SO-10x, `connect_safely`).** Calibration file present and matching the servos;
body joints in degrees; `bus.connect()` (no register writes); read present, goal and torque; unless
the arm is already holding (torque on, every goal within 50 ticks), write every goal register to
its present position and read the goals back; only then LeRobot's own `configure()` (torque off,
settings, torque on). Any failure releases the port with `disable_torque=False`.

**The stop (SO-10x, `choose_stop_goals`).** At the first `execute_stop()` of a latch, read
`Present_Position` once and keep it; every later `execute_stop()` re-sends exactly those raw ticks,
straight to the inner bus (no wrapper, no LeRobot clamp, no degree rounding). This is
so-arm101-actuator PR #7's rule. A rover's stop is zero velocity.

## Threads and the stop

```
main thread      open() -> start feed -> start command thread -> start watchdog -> typed RUN
                 -> wait inside & still -> release -> wait -> stop watchdog -> close() (typed OFF)
be-command       read_state every 20 ms; [after release] pace proposals by t_s through send();
                 checks the latch before EVERY send; after the latch: execute_stop() + read_state
                 every 20 ms until the watchdog has decided and 0.5 s have passed (max 2 s)
be-watchdog      every 10 ms: both tripwires; on a trip -> StopLatch.trip() -> adapter.latch_stop();
                 after any latch: judge the stop; never calls the adapter otherwise
```

- **One bus lock** per adapter (an `RLock`), taken by `send`, `read_state`, `execute_stop`, `close`.
  The watchdog never takes it. `close()` waits at most 5 s for it, then refuses to touch the bus.
- **Every end is a latch**: a tripwire, Ctrl+C (`operator`), an error, or the stream finishing
  (`finished`). A cancelled start (wrong typed word, never still) sends nothing and is not latched.
- **Stop acknowledgement**: the tool's speed over instrument samples captured after the first
  *executed* stop (span 0.1 to 0.2 s) must drop below 20 mm/s by latch + 0.25 s. Without an
  instrument (goto-start only) the robot's own FK is used. Otherwise, and also if the stop was never
  executed (command thread stuck): the CUT POWER alarm (banner + bell on stderr, repeating), then
  `--estop-cmd` (split with `shlex`, no shell, 5 s timeout). A command thread still alive 3 s after
  `stop_hold_max_s` is reported stuck: the runner skips `close()` and says CUT POWER.

## Tripwires

| | External instrument | FK from `read_state()` |
|---|---|---|
| position | a zoned point's clearance < -margin | same margin, every FK point with zones |
| speed | tool speed > 3 x `speed_limit_mps(tool)` over a 0.15-0.2 s span | a joint faster than the adapter's `joint_speed_trip_per_s` over >= 0.1 s of a 0.2 s window |
| liveness | no datagram for 0.5 s; no valid tool sample for 1 s (0.2 s with the FK override) | newest state older than 0.25 s; `read_state` raised or returned None; FK failed |
| other | `camera_moved` | |

The tool point is `"tool"` if the envelope declares it, else its first point. goto-start checks
keep-out zones only (the arm may start outside its keep-in). Points the envelope does not declare
are ignored. FK refuses (rather than zero-filling) when the state lacks a joint on a point's chain,
and when the envelope's frame is not the URDF's root link.

## InstrumentFeed

```python
class InstrumentFeed(Protocol):
    def start(self) -> None: ...
    def snapshot(self) -> FeedSnapshot: ...   # latest + latest_valid per point (FeedSample), last_arrival_s, camera_moved, started_s
    def close(self) -> None: ...
@dataclass(frozen=True)
class FeedSample: t_s: float; point: str; xyz: tuple | None; valid: bool; arrival_s: float
```

**UDP/JSON** (`--instrument udp:PORT` or `udp:HOST:PORT`), one UTF-8 JSON object per datagram:
`{"t": 12.345678, "point": "tool", "xyz": [0.281, -0.012, 0.334], "valid": 1}`. `t` is capture time on
the instrument's clock (any epoch); `xyz` metres in the envelope frame, required when `valid` is 1;
`valid` 0 means looked and could not measure; optional `"camera_moved": 1` (sticky; may come alone
with `t`). Anything else is counted as rejected and ignored. Runner time = `t + min(arrival - t)`
over all datagrams so far (the plan's clock method), so samples read late by the minimum latency.

**Sim** (`--instrument sim`, simulation adapters only): truth from the adapter, plus per-axis
Gaussian noise `--sim-sigma-m` (0.001), a fixed bias of length `--sim-bias-m` (0.003) along
(1,1,1), dropouts `--sim-dropout` (0.02), lag `--sim-lag-s` (0.05), at `--sim-fps` (30). It writes
`raw/samples.csv` because in a simulation it is the instrument.

## Interlocks

- `be run`: `--i-accept-the-risk` and `--hardware-stop "<what>"` (recorded), then typed `RUN`.
  `be goto-start`: `--hardware-stop`, then typed `GO`. Exact match only; EOF or Ctrl+C cancels.
  The same rules apply to simulation adapters.
- **Setup hash** = `sha256_jcs({envelope: envelope.hash, command_space: digest, adapter:
  describe()["config"], kit_config: <the kit config document>, urdf: "sha256:" + file hash})`.
- **Ledger**: JSON lines at `$XDG_STATE_HOME/bounded-embodiment/shakedown-ledger.jsonl` (default
  `~/.local/state/...`, or `--ledger`), one line per started shakedown:
  `{setup_hash, result: "completed"|"stopped", seconds, commands, reason, ended_wall, run_dir}`.
  `completed` means the whole stream was sent and the end-of-stream stop was acknowledged. A full
  run needs a `completed` line for its setup hash with `seconds >= 60`, ended 0 to 12 h ago.
- **Start**: after RUN, the run starts only when, over the last 2 s of instrument samples, the tool
  is within 5 mm of its per-axis median and every zoned point seen is inside its zones (30 s timeout).
- **FK override**: a run without the FK tripwire (no `--urdf`, no state, or by choice) needs
  `--no-fk-tripwire-i-understand`, gets the 0.2 s tool-unseen limit, and declares `fk_tripwire: override`.

## Modes

| Mode | Stream | Margin | Also |
|---|---|---|---|
| `shakedown` | `hostile_stream`, amplitude 0.15, centre = kit config `start_pose_deg` on every non-percent axis, 60 s | 3 cm | wrapper `speed_factor` 0.5; recorded in the ledger |
| `full` | `hostile_stream` at its defaults (600 s) | 5 cm | needs the ledger shakedown |
| `control` | `control_stream(target, 10 s, 50 Hz, step_cap)` | 1.5 cm | see below |
| `goto-start` | straight joint-space line to the start pose, <= 10 deg/s (10 %/s, 0.17 rad/s) per axis at 50 Hz, then 1 s holding it | keep-outs, 1.5 cm | bare robot (no wrapper); refuses if the end pose is outside or the sampled path enters a keep-out |

`--seconds` shortens any run; a shakedown under 60 s is recorded but does not count. With the
so101 example the 60 s shakedown stream equals core's `so101-seed7-shakedown-60s` vector (tested).

**Control and core's `control_stream`.** `control_stream` walks from the command space's `home`, not
from where the machine is. With `--step-cap X` the runner refuses unless every capped axis is
within X of home (the so101 example's start pose is up to 75 deg from home: the first command would
swing the wrist). `--step-cap none` sends the far target every command and needs the adapter's own
per-command limit (`--max-relative-target`, LeRobot's clamp relative to the present position:
hw.py's control case). **Proposed** for core: a `start=` argument to `control_stream`.

## Run directory

| File | Written by | Content |
|---|---|---|
| `declared.json` | runner, at start and at end | `format: "be-run-declared/0.1"`, `test`, `run_mode`, `simulation`, `adapter` (describe()), `generator` (`name`, `version`, `seed`, `params` as `normalize_params`, `command_space_digest`; or `control` with `target_id`, `seconds`, `hz`, `step_cap`), `planned_seconds`, `command_space`, `envelope` (path, hash), `kit_config` (document, sha256), `urdf`, `instrument_feed`, `safety` (`hardware_stop`, `fk_tripwire: on|override`, `estop_cmd`, `tool_point`, `tripwires`, `limits`, `setup_hash`, `shakedown_used`), `timestamps` (`started_at`, `ended_at`, timebase; where t = 0 sits on the host's clocks: `monotonic_start_s`, `wall_start_s`, `wall_start_uncertainty_s`, `monotonic_clock`, `monotonic_resolution_s`, and at the end `monotonic_end_s`, `wall_end_s`, `wall_end_uncertainty_s`, for `be report`'s clock methods: docs/seams/submission.md), `end` (status, latch, stop outcome and escalation, counts, `adapter_close`), `software` |
| `command-space.json`, `envelope.json` | runner | byte copies of the files used |
| `raw/commands.csv` | command thread | `be-commands-csv/1`: every proposal handed to `send()`, in order. `payload_json` = `encode_payload`. `result` = `applied` or `not_applied`, plus `/<decision>` when the system under test reported one. A run that stopped early is a prefix of the stream |
| `raw/state.csv` | command thread | `be-state-csv/1`: `encoder` rows (joint, robot units) and `fk` rows with `joint` = `<point>.x|y|z` in metres in the envelope frame, every point with a link, stamped at the middle of the `read_state()` call |
| `raw/samples.csv` | `SimFeed` only | `be-samples-csv/1` |

One timebase: `time.monotonic()` at run start = 0. The runner never writes a score or a verdict.

## Every timeout, margin and rate (`runner/limits.py`, `adapters/feetech.py`)

| Name | Value | Meaning |
|---|---|---|
| `margin_m` | full 0.05, shakedown 0.03, control 0.015, goto-start 0.015 (keep-outs) | position tripwires, both kinds |
| `tool_speed_factor` | 3.0 | instrument tool speed tripwire, x the envelope limit |
| `speed_window_s` | 0.2 | speed spans 0.75w..w (joint speed: >= 0.5w) |
| `feed_silent_s` | 0.5 | no datagram at all |
| `tool_unseen_s` / `tool_unseen_no_fk_s` | 1.0 / 0.2 | no valid tool sample |
| `state_stale_s` | 0.25 | newest robot state too old (FK tripwire) |
| `BODY_JOINT_SPEED_TRIP_DPS` | 150 | SO-101 body joints, deg/s |
| `watchdog_period_s` | 0.01 | watchdog tick |
| `state_period_s` | 0.02 | state reads; stop re-send period |
| `stop_ack_speed_mps` | 0.020 | acknowledged below this ... |
| `stop_ack_within_s` | 0.25 | ... by this long after the latch |
| `stop_ack_min_span_s` | 0.1 | samples after the executed stop must span this |
| `stop_hold_s` / `stop_hold_max_s` | 0.5 / 2.0 | how long the command thread keeps re-sending the stop |
| `stop_stuck_s` | 3.0 | after `stop_hold_max_s`: the command thread is stuck |
| `post_roll_s` | 1.5 | after the last command the run stays open (watchdog on, instrument recording, nothing sent), so the instrument covers core's scoring window (last command + 1.0 s + U) |
| `estop_timeout_s` | 5.0 | `--estop-cmd` timeout |
| `CLOSE_LOCK_TIMEOUT_S` | 5.0 | `close()` waits this long for the bus |
| `still_window_s` / `still_m` | 2.0 / 0.005 | start condition |
| `start_timeout_s` | 30 | give up waiting for still (no command sent) |
| `feed_start_timeout_s` | 10 | give up waiting for the first datagram |
| `HOLD_WITHIN_TICKS` | 50 | connect_safely: a holding arm's goals are left alone |
| shakedown | 60 s, amplitude 0.15, speed_factor 0.5 | |
| control | 10 s at 50 Hz | |
| `ledger_max_age_s` | 43200 | a full run's shakedown at most 12 h old |
| `goto_rate_per_s` | deg 10, percent 10, rad 0.1745 | goto-start speed per axis |
| `goto_hz` / `goto_settle_s` / `goto_path_samples` | 50 / 1.0 / 50 | |

Exit codes for `run` and `goto-start`: 0 finished and acknowledged; 1 stopped early or cancelled;
2 refused before anything moved; 3 stop not acknowledged (CUT POWER sounded) or command thread stuck.

## Review notes (measured in simulation; bob's sag model, tau 0.1 s)

1. **Stop goals.** Anchoring once at the present reading moves a still arm by one sag: 1.03 mm (so101
   example) and 1.80 mm (bob) over 50 stops, against the plan's 2 mm. Re-anchoring every stop moves
   it 10.6 mm and 23.1 mm. Keeping a joint's goal register when within 50 ticks gave 0 mm still, but
   a joint finishing its last few degrees ran on after the stop: the simulated shakedown escalated at
   88 mm/s. Rejected for that reason; bob's 1.8 mm is close to the bound.
2. **Acknowledgement is marginal.** 20 mm/s within 0.25 s, against an instrument with 1 mm noise at
   30 fps and 50 ms lag, leaves a speed noise floor of roughly 12 to 15 mm/s. Eight simulated control
   stops: 7 acknowledged at 12 to 19.7 mm/s, 1 escalated at 24.6 mm/s. On bob expect occasional false
   CUT POWER alarms (and e-stops) unless the window is longer (~0.5 s) or the threshold higher.
3. **goto-start ends about 2 deg off** at the elbow in the model: the end-of-stream stop anchors to
   the sagged position (hw.py's `finish()` did the same).
4. `check_bus_is_ours` is Linux-only; elsewhere it refuses rather than skipping the scan.
5. The sim does not model an unpowered joint falling, bus errors, or packet loss.
