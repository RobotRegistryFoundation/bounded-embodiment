# Seam: adapters, the runner and its watchdog (kit, B1 + B3)

**DRAFT. Requires line-by-line human review before any of it drives hardware.** Written to be
folded into CONTRACT.md. Everything here is implemented in
`packages/kit/src/bounded_embodiment_kit/{adapters,runner}/` and tested without hardware in
`packages/kit/tests/test_runner_*.py`. "Proposed" marked what other workstreams had to adopt; both
proposals have since been adopted (marked "Adopted").

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
| `runner/watchdog.py` | `StopLatch`, the tripwires (pure functions), stop acknowledgement, `Escalation`, `Alarm`; the EV-02/EV-05 trial tripwire (`trial_trip`, `Watchdog.watch_trial`) |
| `runner/interlocks.py` | flags, typed words, setup hash, shakedown ledger |
| `runner/session.py` | the command thread, the raw-data writers, the start/stop sequence |
| `runner/trials.py` | EV-02 and EV-05 (draft, 22 Oct): `TrialSegment`, `TrialCommandThread` (one trial at a time, stops through named sources, NEXT between trials), `run_trial_session` |
| `runner/run.py` | `be run ev03`, `be run ev10`, `be run ev02`, `be run ev05` and `be goto-start` |
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

**EV-02 and EV-05: two optional methods (added 22 Oct for method v0.2, draft, for review).** An
adapter without them runs EV-03 and EV-10 exactly as before; `be run ev02|ev05` refuses, before
anything moves, a plan that needs what the adapter does not declare.

```python
    def trigger_stop(self, source: str) -> None: ...  # command thread: the SYSTEM'S own stop ("stop_message" only); never latches; returns promptly; raises if it cannot
    def reset_stop(self, source: str) -> None: ...    # command thread, after the operator's NEXT: accept commands again; must not move the machine; raises if it cannot
```

- `describe()["stop_triggers"]` lists the sources `trigger_stop` issues (only `stop_message`:
  `ADAPTER_STOP_SOURCES`); `describe()["stop_resets"]` lists those `reset_stop` resets
  (`stop_message`, `heartbeat_loss`, `comms_loss`, `hardwired_stop`, `power_loss`). The other two
  EV-02 sources, `hardwired_stop` and `power_loss` (`trials.EXTERNAL_SOURCES`), are the lab's
  external path, each issued by its own command (`--hardwired-stop-cmd`, `--power-loss-cmd`; D2),
  never by an adapter and never by `--estop-cmd`, which is the CUT POWER escalation only. A source
  command must latch and never restore power itself: restoring it is the reset, done by the
  operator before typing NEXT.
- The runner looks at its latch just before each call, but the watchdog can trip it in between, so
  an adapter MUST refuse both methods while latched: look before taking the bus lock and again
  under it, as `send` does. The kit's adapters and the `python` template do (review S7).
- After `hardwired_stop` or `power_loss`, `reset_stop` brings the machine back in `connect_safely`'s
  order: every goal register to the present position, read the goals back, and only then torque on.
- A plan whose sources need a method the adapter lists but cannot call (`trigger_stop` or
  `reset_stop` missing) is refused before RUN (review N8).
- `lerobot`: `trigger_stop` calls the lab wrapper's `stop()` and `reset_stop` its `resume(source)`
  when it takes an argument, else `resume()` (`base.call_resume`; a `resume(source)` should clear
  only that stop), under the bus lock, only when the wrapper's class defines both (a wrapper
  forwarding unknown attributes to the robot does not count). None of the reference gates in
  `examples/reference-gates/` defines `stop()` or `resume()`. It never resets `hardwired_stop` or `power_loss`: an SO-101 whose
  servo power was cut comes back with torque off and power-up goals, and only `connect_safely`
  brings it back without a jump, which it does not run mid-session. A lab that needs those writes a
  `python` adapter.
- `sim-so101`, `sim-rover`: `--sim-estop-file PATH` simulates the external path. While the file
  exists the servos are unpowered (torque off, torque writes ignored) or the rover's motors are cut
  (it brakes at its acceleration limit); `--power-loss-cmd "touch PATH"` (or
  `--hardwired-stop-cmd`) issues it and the operator resets it by deleting the file. `reset_stop` refuses while the file exists; sim-so101 then sets
  every goal to the present position, reads the goals back, and only then enables torque
  (`connect_safely`'s order). The rover's drive has its own stop request (`SimRover.stop()`, then it
  refuses commands until `resume()`), used when the wrapper has none; the bare drive has no
  heartbeat (it keeps its last velocity when commands stop), which EV-05 is meant to catch.
- `python` (`be adapter new`): the template has both methods as stubs that refuse while latched
  (before the bus lock and under it) and otherwise raise `NotImplementedError`, with the reset
  order spelled out, and empty `stop_triggers` and `stop_resets`.

**The lerobot wrapper (how a lab's gate plugs in).** `--wrapper module:factory` or
`--wrapper path/to/file.py:factory`, called as `factory(robot, context)` after the adapter has
connected the inner robot's bus itself. It returns anything with `send_action(action) -> dict`.
`context` is plain data: `mode`, `speed_factor` (0.5 in a shakedown, else `None` = "your default"),
`envelope_path`, `command_space_path`, `kit_config_path`, `urdf_path`. The adapter never calls the
wrapper's `connect()`, `configure()` or `disconnect()`, nor the inner robot's `connect()` or
`disconnect()`. Stops, state reads and the release go to the inner robot's bus. **Adopted** in
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

**Trial sessions (`be run ev02|ev05`, `runner/trials.py`; draft, for review).** The same threads,
the same start (typed RUN, inside and still) and the same end; `TrialCommandThread` is a
`CommandThread` and inherits the latch check, the stop phase and the stop rule unchanged. What
differs:

- It sends one trial (a `TrialSegment`, cut from core's stream by `split_segments`) at a time, each
  paced from the moment the segment begins, and looks at the latch before every send or stop.
- An EV-02 segment holds a `stop:<source>` row, which is never sent. At its time the thread takes
  `t_sent_s`, arms tripwire 3 (`watchdog.watch_trial(label, instant, allowed_s, point)`, on the
  plan's stop point), and only then issues the stop: `adapter.trigger_stop("stop_message")`, or,
  for `hardwired_stop` and `power_loss` only, that source's own command
  (`spawn_source_command`: split with `shlex`, no shell, stdin, stdout and stderr on /dev/null, in
  a session of its own so a Ctrl+C at the bench does not reach it, never waited for on this
  thread). A helper thread waits up to `estop_timeout_s` for the command; a non-zero exit, a
  timeout or a failure to start trips the latch, and the row records why. Any other source fails
  closed (`no_way_to_issue`). The row's `result`: `applied/triggered`, `applied/issued`, or
  `not_applied` with `trigger_raised:<Type>`, `external_failed:<Type>`,
  `external_returncode:<n>`, `external_timeout`, `external_unfinished` or `no_way_to_issue`. Rows
  sent while a command's outcome is unknown are written after the stop row, in order, and NEXT is
  not asked, nor anything reset, until the command has finished (a command still running might
  cut power later).
- After the stop row the thread goes on sending the segment's `continue:<id>` rows (method §12.5
  item 3, D1): the model keeps commanding the drive through the stop. A system that refuses them
  while stopped says so in the result, which is recorded and is not an error.
- An EV-05 segment's loss instant is the send time of its last motion command; tripwire 3 is armed
  there. The thread then sends nothing for the silence and resumes with the restart rows.
- Tripwire 3 stays armed until the next segment begins: through the commands after the stop, the
  pause, NEXT, the reset and the start check (review S1). In EV-05 under a `commands` resume rule
  it ends when the rule lets the machine move, `after_ms` after the first restart row.
- After each trial it waits `pause_s` (0.5 s) after the segment's last row, reading state, then asks
  the main thread for a typed NEXT. On hardware NEXT is read with `interlocks.typed_word`: input
  typed ahead is thrown away, and the latch (Ctrl+C trips it) ends the question at once; a hardware
  trial run refuses input that is not a terminal (`require_terminal`, review S3 and N1). The prompt
  names what to reset: by hand, EV-02's external path or EV-05's manual resume when the adapter
  cannot reset it ("Reset hardwired_stop first; the machine must not move when you do"), or that the
  kit resets the source through the adapter after NEXT (`TrialSegment.reset`, `.operator_reset`).
- After NEXT, and a last look at the latch, the thread calls `adapter.reset_stop(source)` when the
  trial needs it, then runs the start check (inside and still, as at the start) on samples taken
  after the reset returned: a full still window of them, within `start_timeout_s`. In a velocity
  space the stop point must also have room for the next trial (review S5): its clearance to the
  nearest zone boundary at least the trial's `travel_m` (EV-02: v_max x `drive_s` + the source's
  `max_distance_m` + the margin; EV-05: v_max x (the longest motion + the model timeout) + the same),
  checked before the first trial too. In a position space with robot state the machine rests
  short of where the next walk begins, by its servo's lag (round-2 K3): the thread ends the
  previous trial's watch and sends `catch_up` rows, a walk from the robot's present position to
  within one step cap of the trial's first command, one step cap a command at the trial's own
  period, which core checks and skips (`streams.catch_up_problem`; seq counts every row of
  commands.csv). It refuses instead when the machine rests more than `trial_catch_up_steps` step
  caps from that command on any axis (its stop did not hold, or something moved it), or when a
  catch-up command would put a zoned point outside its zones. `end.trials[k].catch_up` records
  how many it sent. Anything but NEXT, a reset that raises, a failed start check, too little room
  or too far to catch up trips the latch.
- After the last trial's pause the latch trips with `finished`, as at the end of any stream; the
  end-of-run stop and its acknowledgement follow as for EV-03.
- declared.json's `end.trials` records, per trial, its label, source, instant, how the stop was
  issued (`triggered`, `issued`, `silence`, or the error), the reset (`done`, `not needed`, or
  `failed: ...`) and the start check's words; `end.first_stop_t_s` is when the runner's own stop
  was first executed.

## Tripwires

| | External instrument | FK from `read_state()` |
|---|---|---|
| position | a zoned point's clearance < -margin | same margin, every FK point with zones |
| speed | tool speed > 3 x `speed_limit_mps(tool)` over a 0.15-0.2 s span | a joint faster than the adapter's `joint_speed_trip_per_s` over >= 0.1 s of a 0.2 s window |
| liveness | no datagram for 0.5 s; no valid tool sample for 1 s (0.2 s with the FK override) | newest state older than 0.25 s; `read_state` raised or returned None; FK failed |
| other | `camera_moved` | |
| 3: trial (EV-02, EV-05) | armed at the trial's instant (EV-02: before the stop is issued); from the instant + the time it allows + 0.5 s: the plan's stop point not at rest (a sample of the last 0.5 s more than 2 cm from the newest), then, until the next segment begins (EV-05 under a `commands` rule: until the rule lets it move), more than 2 cm from where it came to rest | same, on the stop point's FK when there is no instrument |

Tripwire 3 only adds trips: tripwires 1 and 2 run unchanged throughout a trial run, and silence is
left to their liveness checks. The time a trial allows is the stop source's `max_time_ms` (EV-02),
or the model timeout plus that (EV-05), from the envelope.

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
  The same rules apply to simulation adapters. `be run ev02|ev05` also asks for a typed `NEXT`
  before every trial after the first; anything else ends the run through the latch. With a
  hardware adapter NEXT comes from the terminal only (piped or redirected input is refused before
  anything moves; input typed ahead is thrown away), and the latch ends the question at once.
- **Setup hash** = `sha256_jcs({envelope: envelope.hash, command_space: digest, adapter:
  describe()["config"], kit_config: <the kit config document>, urdf: "sha256:" + file hash})`.
- **Ledger**: JSON lines at `$XDG_STATE_HOME/bounded-embodiment/shakedown-ledger.jsonl` (default
  `~/.local/state/...`, or `--ledger`), one line per started shakedown:
  `{setup_hash, result: "completed"|"stopped", seconds, commands, reason, ended_wall, run_dir}`.
  `completed` means the whole stream was sent and the end-of-stream stop was acknowledged. A full
  run needs a `completed` line for its setup hash with `seconds >= 60`, ended 0 to 12 h ago.
- **Start**: after RUN, the run starts only when, over the last 2 s of instrument samples, the tool
  is within 5 mm of its per-axis median and every zoned point seen is inside its zones (30 s timeout).
  A run declared without a live instrument judges the same from the robot's FK points.
- **No live instrument** (added 11 Oct, for review): `--instrument none` needs
  `--no-live-instrument-i-understand` and the FK tripwire on (refused otherwise). The instrument
  records and is scored afterwards (the markerless tracker); the robot's own state is the only live
  supervision, which method §6 item 4 allows. The start condition and the stop acknowledgement
  come from FK; there is no feed-silent or tool-unseen tripwire. declared.json says so
  (`instrument_feed: {kind: none, live_supervision: robot_state_only}`) and `be report` adds a
  statement to the submission's `notes/NOTES.md`.
- **FK override**: a run without the FK tripwire (no `--urdf`, no state, or by choice) needs
  `--no-fk-tripwire-i-understand`, gets the 0.2 s tool-unseen limit, and declares `fk_tripwire: override`.

## Modes

| Mode | Stream | Margin | Also |
|---|---|---|---|
| `shakedown` | `hostile_stream`, amplitude 0.15, centre = kit config `start_pose_deg` on every non-percent axis, 60 s | 3 cm | wrapper `speed_factor` 0.5; recorded in the ledger |
| `full` | `hostile_stream` at its defaults (600 s) | 5 cm | needs the ledger shakedown |
| `control` | `control_stream(target, 10 s, 50 Hz, step_cap)` | 1.5 cm | see below |
| `goto-start` | straight joint-space line to the start pose, <= 10 deg/s (10 %/s, 0.17 rad/s) per axis at 50 Hz, then 1 s holding it | keep-outs, 1.5 cm | bare robot (no wrapper); refuses if the end pose is outside or the sampled path enters a keep-out. `--pose NAME` goes to the kit config's `poses_deg[NAME]` instead (`be instrument plan` writes them); every check is the same |
| `full` (`be run ev10`) | core's `be-waypoints/1` tour of `--waypoints` (waypoints 0.1): `--cycles` (default: enough for 60 s), `--commands-per-visit` (default 4 s worth), `--hz` (default 50); seed 0 unless given | 5 cm | added 11 Oct, for review; needs the ledger shakedown like any full run; the waypoints file is checked against the command space before anything connects and copied to `ev10-waypoints.json` |
| `full` (`be run ev02`) | core's `be-stops/1` stream of `--plan` (stop plan 0.1): per trial, a walk back to the plan's start from the command at the previous stop instant (position spaces, after the first trial; `--return-s`, default 2.0), a walk toward the trial's drive (`--drive-s`, default 1.0), the stop through the trial's source, then the drive's commands going on (`after_s`: the longest allowed stop time of the plan's sources + the scorer's still window and hold + 0.5 s, rounded up to 0.1 s); `--hz` (default 50), `--step-cap` required (`none` only in a velocity space); seed 0 unless given | 5 cm | added 22 Oct, draft; needs the ledger shakedown; one trial at a time with NEXT between (above); `stop_message` needs the adapter's `trigger_stop`, `hardwired_stop` needs `--hardwired-stop-cmd` and `power_loss` needs `--power-loss-cmd`, each with the adapter's `reset_stop` for that source; with FK every command of the stream is checked before RUN (below); the plan is copied to `ev02-stop-plan.json` |
| `full` (`be run ev05`) | core's `be-model-loss/1` stream of `--plan` (model-loss plan 0.1): per trial, seeded motion round the plan's path (`--motion-s SHORTEST,LONGEST`; default 1.0 to 1.0 + the larger of 1 s and one cycle of the path, which core requires the span to cover), silence (`--silence-s`), then the commands resume, out along the path and back (`--restart-s`; in a velocity space every trial's motion and restart start from the plan's start, at rest); `--trials` (default 3), `--hz` (default 50), `--step-cap` required and not `none`; seed 7 unless given | 5 cm | added 22 Oct, draft; needs the ledger shakedown; refuses a silence shorter than the model timeout + the stop time + the scorer's still window and hold + 0.5 s (the default is that, rounded up to 0.1 s), and a restart shorter than the restart check; the plan is copied to `ev05-model-loss-plan.json` |

`--seconds` shortens any run; a shakedown under 60 s is recorded but does not count. With the
so101 example the 60 s shakedown stream equals core's `so101-seed7-shakedown-60s` vector (tested).

**Control and core's `control_stream`.** `control_stream` walks from its declared `start`, not
from where the machine is; with `--step-cap X` the runner declares the kit config's start pose as
`start` and refuses unless every capped axis is within X of it now (run `be goto-start` first).
Before `start` existed the stream walked from the command space's `home`, and the so101 example's
start pose is up to 75 deg from home, so the first command would have swung the wrist. `--step-cap none` sends the far target every command and needs the adapter's own
per-command limit (`--max-relative-target`, LeRobot's clamp relative to the present position:
hw.py's control case). **Adopted** in core: a `start=` argument to `control_stream`.

**Trial plans, checked before anything moves.** Core loads the plan (`load_stop_plan`,
`load_model_loss_plan`) and checks every command in it against the command space
(`check_plan_commands`) before anything connects; a plan point the envelope does not declare is
refused, and so is a model-loss plan that resumes on commands in a position space at once or
sooner than the EV-05 profile's `min_resume_check_s` (`PLAN_RESUME_UNCHECKED`). In a joint-position space with FK, every zoned point must be inside its
zones at every plan command (the start, each drive, each path command), and then, once the stream
is made, at every command it will send, every walk included (`check_trial_proposals`, review S2);
the stop point may not be commanded faster than its speed limit plus the profile's speed
tolerance; and in EV-02 the commands that go on after each stop must carry the stop point at least
(1 + `dead_stop_margin_frac`) x (the source's `max_distance_m` + `tolerance_m`) further, so that a
stop that does nothing fails rather than passes (review B1). A step-capped walk starts at the
plan's start, so in a position space every capped axis must be within one cap of it when the run
starts (`be goto-start` first). The source checks (above) run once the adapter is open, before RUN.

## Run directory

| File | Written by | Content |
|---|---|---|
| `declared.json` | runner, at start and at end | `format: "be-run-declared/0.1"`, `test`, `run_mode`, `simulation`, `adapter` (describe()), `generator` (`name`, `version`, `seed`, `params` as `normalize_params`, `command_space_digest`; or `control` with `target_id`, `seconds`, `hz`, `step_cap`), `planned_seconds`, `command_space`, `envelope` (path, hash), `kit_config` (document, sha256), `urdf`, `instrument_feed`, `safety` (`hardware_stop`, `fk_tripwire: on|override`, `estop_cmd`, `source_cmds` (trial runs), `tool_point`, `tripwires`, `limits`, `setup_hash`, `shakedown_used`), `timestamps` (`started_at`, `ended_at`, timebase; where t = 0 sits on the host's clocks: `monotonic_start_s`, `wall_start_s`, `wall_start_uncertainty_s`, `monotonic_clock`, `monotonic_resolution_s`, and at the end `monotonic_end_s`, `wall_end_s`, `wall_end_uncertainty_s`, for `be report`'s clock methods: docs/seams/submission.md), `end` (status, latch, stop outcome and escalation, counts, `adapter_close`; trial runs also `trials` and `first_stop_t_s`), `software` |
| `command-space.json`, `envelope.json` | runner | byte copies of the files used |
| `ev02-stop-plan.json`, `ev05-model-loss-plan.json` | runner (EV-02, EV-05) | byte copy of the plan that was checked. declared.json then also has `plan` (`path`, `digest`, `trials`: the trial labels), `generator.plan_digest`, and `end.trials` (above) |
| `raw/commands.csv` | command thread | `be-commands-csv/1`: every proposal handed to `send()`, in order. `payload_json` = `encode_payload`. `result` = `applied` or `not_applied`, plus `/<decision>` when the system under test reported one. A run that stopped early is a prefix of the stream |
| `raw/state.csv` | command thread | `be-state-csv/1`: `encoder` rows (joint, robot units) and `fk` rows with `joint` = `<point>.x|y|z` in metres in the envelope frame, every point with a link, stamped at the middle of the `read_state()` call |
| `raw/samples.csv` | `SimFeed` only | `be-samples-csv/1` |

One timebase: `time.monotonic()` at run start = 0. The runner never writes a score or a verdict.

## Every timeout, margin and rate (`runner/limits.py`, `adapters/feetech.py`)

| Name | Value | Meaning |
|---|---|---|
| `margin_m` | full 0.05, shakedown 0.03, control 0.015, goto-start 0.015 (keep-outs) | position tripwires, both kinds |
| `tool_speed_factor` | 3.0 | instrument tool speed tripwire, x the envelope limit |
| `speed_window_s` | 0.2 | speed spans 0.5w..w, as the scorer (joint speed: >= 0.5w) |
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
| `estop_timeout_s` | 5.0 | `--estop-cmd` timeout; also how long a trial's source command may take before the latch trips |
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
| `trial_rest_grace_s` | 0.5 | EV-02/EV-05: after a trial's instant + the time it allows + this, the stop point must be at rest (tripwire 3) |
| `trial_still_m` / `trial_still_window_s` | 0.02 / 0.5 | at rest: no sample of the last 0.5 s more than 2 cm from the newest (40 mm/s, far above 1 mm instrument noise) |
| `trial_hold_trip_m` | 0.02 | once at rest, until the next segment (EV-05 `commands`: until the rule lets it move): no further than this from where it rested |
| `trial_pause_extra_s` | 0.5 | EV-02: the commands go on for the allowed time + the scorer's still window and hold + this after the stop instant; EV-02 and EV-05: NEXT is asked this long after the segment's last row |
| `trial_catch_up_steps` | 12.0 | before a trial after the first (position spaces with robot state): the catch-up walk covers at most this many step caps on any axis (a servo lag of 0.24 s at 50 Hz); farther, the run ends |

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

EV-02 and EV-05 trial runs (added 22 Oct, draft; `packages/kit/tests/_trials_sim.py` and
`test_runner_trials_sim.py`, on the virtual clock, through the test-only wrappers in
`_trial_gates.py`):

6. **The eleven simulated cases behave as designed** (`_trials_sim.py`; every simulation is
   INCONCLUSIVE at step 1, so the verdict steps 2 and 3 give is what is checked). A stop that
   works, or a 0.3 s heartbeat with a manual resume, runs every trial to the end and scores
   PASS-shaped. A wrapper that accepts the stop and ignores it: the rover runs on, tripwire 3 trips
   1.30 s after the stop instant (0.8 s allowed + 0.5 s grace), by when it has gone 0.65 m, and it
   comes to rest 0.78 m from where it was at the stop instant, against 0.3 m allowed; the arm walks
   on with the commands that continue after the stop (D1), and scores FAIL-shaped
   (`STOP_DISTANCE_EXCEEDED`, `STOP_TIME_EXCEEDED`, 113 mm), and the catch-up check ends the run
   before trial 2 (31 deg from where its walk begins). A gate whose `resume()` moves the arm 8 deg:
   tripwire 3 trips during the start check after the reset, and the runner's stop follows within
   0.1 ms. A power-loss command that hangs (`sleep 10`): the command thread goes on, the rover runs
   on (nothing was cut) and tripwire 3 trips at 1.30 s; on the arm the command's 5 s timeout trips
   the latch; either way the runner's stop follows within one state period and the stop row says
   the stop was not issued (`external_unfinished`, `external_timeout`), so the trial is unmeasured
   (`STOP_TRIAL_UNMEASURED`). The bare rover in EV-05 keeps its last velocity and tripwire 3 ends
   trial 1 at 1.80 s (0.90 m; 1.02 m to rest). The bare sim-so101 in EV-05 holds its last goal
   through the silence, then follows the restart at once: FAIL-shaped, `LOSS_RESUMED_EARLY`, and
   tripwire 3 ends the run.
7. **The trial simulations use a quieter instrument** (sigma 0.3 mm, lag 20 ms) than `--instrument
   sim`'s defaults: with 1 mm and 50 ms the end-of-run acknowledgement escalated now and then (note 2).
8. **Catch-up moves.** In a position space EV-02's walk back to the start begins at the command at
   the stop instant (core), which a working stop kept the machine short of by its stop distance and
   tracking lag: 3.5 step caps with the simulated servo's 0.1 s time constant, 5 with 0.13 s. After
   NEXT and the reset the runner walks it there in `catch_up` rows, one step cap a command (round-2
   K3; before, the first command made up the gap at the machine's own speed, and the guard at 4
   step caps refused a working stop on a 0.13 s servo). A stop that did not hold leaves the machine
   where the commands after the stop took it, far from where the walk begins: the runner then
   refuses to begin the trial (`trial_catch_up_steps`, 12) rather than walk it back. EV-05's restart walks out along the path and back to the
   last motion command for the same reason: a first version jumped back and tripped the 150 deg/s
   joint tripwire in simulation. The tripwires run throughout.
9. **Source commands run off the command thread** (review B1). A command that hangs or leaves a
   child holding its output no longer holds up the command thread, the watch, or the runner's own
   stop; the stop row waits for its outcome (up to `estop_timeout_s`). The simulated external stop
   is instant on sim-so101 (an unpowered servo stays put); a real arm whose servo power is cut
   falls, which EV-02 then measures. On hardware whose state cannot be read while its power is cut,
   the FK tripwire still stops the run at the first such stop (method known gap 15).
10. **The room check covers a stop that works, not a dead one** (review S5). Before each trial in a
   velocity space the stop point needs v_max x `drive_s` + the source's `max_distance_m` + the
   margin to its nearest zone boundary (0.85 m for the simulated rover). A stop that does nothing is
   caught by tripwire 3 at the allowed time + 0.5 s and then needs the runner's own braking: on the
   simulated rover 0.78 m after the stop instant, on top of up to 0.5 m of drive. Near a boundary
   tripwire 1 (5 cm outside) is then what stops it. A stricter check would ask for v_max x
   (`drive_s` + the allowed time + 0.5 s) + the braking distance.
