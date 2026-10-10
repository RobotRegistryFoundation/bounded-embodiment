# Seam: the command space and the hostile generator (A2)

Written to be folded into CONTRACT.md. Everything here is implemented in `packages/core/src/bounded_embodiment/{commandspace,generator}.py` and pinned by tests and vectors. "Proposed" marks the parts other workstreams still have to adopt.

## Core module surface

**`bounded_embodiment.commandspace`**
```python
load_command_space(src: str | os.PathLike | dict) -> CommandSpace   # raises CommandSpaceError (ValueError) with .findings

@dataclass(frozen=True)
class Axis:        key: str; unit: str; lo: float; hi: float; home: float
@dataclass(frozen=True)
class FarTarget:   id: str; values: Mapping[str, float]; description: str = ""   # values is read-only
@dataclass(frozen=True, eq=False)
class CommandSpace:
    kind: str                         # "joint_position" | "cartesian_position" | "velocity"
    axes: tuple[Axis, ...]            # declared order
    far_targets: tuple[FarTarget, ...]  # declared order
    malformed: tuple[str, ...]        # declared kinds, in MALFORMED_KINDS order; all nine when omitted
    document: dict                    # the dict as given
    digest: str                       # sha256_jcs(document), computed at construction
    keys -> tuple[str, ...]; axis(key) -> Axis; far_target(id) -> FarTarget   # KeyError if unknown

KINDS, UNITS, MALFORMED_KINDS, UNKNOWN_AXIS_KEY = "unknown_axis", SCHEMA_NAME = "command-space-0.1"
```

**`bounded_embodiment.generator`**
```python
GENERATOR = "be-hostile"; GENERATOR_VERSION = "1"
STRATEGIES = ("uniform", "boundary", "max", "sign_flip", "drive_out", "fast", "partial", "malformed")
SHAKEDOWN_STRATEGIES = ("uniform", "boundary", "sign_flip", "partial")

@dataclass(frozen=True)
class Proposal: seq: int; t_s: float; strategy: str; payload: dict

default_params() -> dict
normalize_params(params: Mapping | None = None) -> dict          # ValueError on any unknown key or bad value
hostile_stream(space, seed: int, params: Mapping | None = None) -> Iterator[Proposal]   # checks eagerly, generates lazily
control_stream(space, target_id: str, seconds: float, hz: float = 50.0,
               step_cap: Mapping[str, float] | None = None,
               start: Mapping[str, float] | None = None) -> Iterator[Proposal]
far_target_pool(space, seed: int) -> list[tuple[str, dict[str, float]]]
encode_payload(payload: dict) -> str
decode_payload(text: str) -> dict
stream_sha256(space, seed: int, params: Mapping | None = None) -> str   # "sha256:" + 64 lower-hex
compare_to_commands(space, seed: int, params: Mapping | None,
                    rows: Iterable[tuple[int | str, str, str]]) -> list[Finding]
```
- Seeds are integers from 0 to 2^53 inclusive. Negative seeds are refused because `random.Random(-7)` is `random.Random(7)`. The upper bound keeps a seed representable in an RFC 8785 report.
- Every bad argument raises `ValueError`, a wrong type included, because parameters arrive as JSON from a report. Unknown ids raise `KeyError` (`control_stream`, `CommandSpace.axis`, `CommandSpace.far_target`).

## Command space 0.1 (`schemas/command-space-0.1.json`, CC BY 4.0)

```jsonc
{
  "command_space_version": "0.1",
  "kind": "joint_position" | "cartesian_position" | "velocity",
  "axes": [                                   // >= 1, in a meaningful order (draws follow it)
    {"key": "shoulder_pan.pos", "unit": "deg|rad|m|m/s|rad/s|percent", "lo": -110.0, "hi": 110.0, "home": 0.0}
  ],
  "far_targets": [                            // optional
    {"id": "into_table", "values": {"<every axis key>": 0.0}, "description": "optional"}
  ],
  "malformed": ["nan", "inf", "neg_inf", "string", "null", "bool", "missing_key", "unknown_key", "empty"],  // optional, >= 1, unique; omitted = all
  "description": "optional free text"
}
```
- `additionalProperties: false` on every structural object. `far_targets[].values` maps axis keys to values.
- The schema checks these:
  - keys are 1 to 128 printable ASCII characters with no spaces (`^[!-~]{1,128}$`);
  - far-target ids match `^[A-Za-z0-9_.-]{1,64}$`;
  - every number is finite with magnitude below 10^6, so the `max` strategy's 10^6 is always out of range.
- The loader checks the rules a schema cannot express:
  - axis keys are unique;
  - `unknown_axis` is reserved;
  - `lo < hi`, and `lo <= home <= hi`;
  - far-target ids are unique;
  - each far target names every axis and nothing else.
- The loader reads every number as a float, and reads -0.0 as 0.0. Documents with the same digest therefore always give the same stream.
- Finding codes:
  - `COMMAND_SPACE_UNREADABLE`, `_PARSE_ERROR`, `_NOT_OBJECT`, `_VERSION_UNSUPPORTED`, `_NOT_FINITE`, `_NOT_CANONICAL`;
  - `_DUPLICATE_AXIS`, `_RESERVED_KEY`, `_RANGE_INVERTED`, `_HOME_OUTSIDE`;
  - `_DUPLICATE_FAR_TARGET`, `_FAR_TARGET_UNKNOWN_AXIS`, `_FAR_TARGET_MISSING_AXIS`;
  - schema violations stay `SCHEMA_INVALID`.
- Examples: `examples/command-spaces/so101.command-space.json` (URDF limits, seven far targets) and `rover.command-space.json` (±1.5 m/s, ±1.5 rad/s, one far target, `straight_out`).

## Parameters

`default_params()`:
```json
{"seconds": 600.0, "base_hz": 50.0, "burst_hz": 500.0, "segment_s": [0.5, 4.0],
 "amplitude": 1.0, "center": null, "strategies": ["uniform","boundary","max","sign_flip","drive_out","fast","partial","malformed"]}
```
`normalize_params` fills in defaults, converts numbers to floats (-0.0 becomes 0.0), and puts `strategies` in `STRATEGIES` order. Normalizing twice changes nothing. It refuses:
- `seconds` outside (0, 86400], and `base_hz` or `burst_hz` outside (0, 10000];
- `segment_s` that is not `[min, max]` with `0 < min <= max`;
- `amplitude` outside (0, 1];
- `center` that is neither null nor `{key: finite number}`;
- `strategies` that are empty, repeated or unknown.

With `amplitude < 1` (hardware shakedown):
- the default pool is `SHAKEDOWN_STRATEGIES`, and any other strategy is refused;
- each axis is confined to the band `lo' = c + (lo - c)·amplitude`, `hi' = c + (hi - c)·amplitude`, clipped to [lo, hi];
- `c` is `center[key]`, or `home` when the key is absent; `hostile_stream` checks that center keys are axes and that each value is within its axis's limits.

## The stream (generator `be-hostile/1`)

**Randomness.** One `random.Random(seed)`. Only `.random()` is called, through these helpers:
- `_uniform(a, b) = a + (b - a)·random()`;
- `_index(n) = min(int(random()·n), n - 1)`;
- `_choice(seq) = seq[_index(len(seq))]`;
- `_coin() = random() < 0.5` (True = hi side, or + sign).

**Draws, in order** (axes always in declared order):

| when | draws |
|---|---|
| stream start | 8 × len(axes): the box corners, corner by corner, axis by axis (`_coin`) |
| segment start (`t >= seg_end`) | strategy `_choice(pool)`; `seg_end = t + _uniform(*segment_s)`; far target `_choice(far_pool)` |
| uniform | one `_uniform` per axis |
| boundary | k `_choice(BOUNDARY_K)` (no draw when amplitude < 1), then one `_coin` per axis |
| max | magnitude `_choice((1e3, 1e6))`, then one `_coin` per axis for the sign |
| sign_flip, drive_out, fast | none |
| partial | axis `_index(len(axes))`, then its value |
| malformed | kind `_choice(space.malformed)`, then axis `_choice(axes)` (no axis draw for `empty`) |

**Time.**
- `t` starts at 0. After each command, `t += 1/burst_hz` if that command was `fast`, else `t += 1/base_hz`. This is accumulated in stream order.
- The stream runs while `t < seconds`.
- `Proposal.t_s = t` is the schedule the kit paces by. Nothing compares it.
- `seq` counts from 0.

**Strategies.** `lo`/`hi` below are the axis limits, or the band at amplitude < 1.
- **uniform:** each axis `_uniform(lo - 0.25·span, hi + 0.25·span)`; at amplitude < 1, uniform in [lo', hi'] and clamped into it.
- **boundary:** one k ∈ {−0.001, 0, +0.001, +0.05} per command. Each axis takes `hi + k·half_span` or `lo − k·half_span`, where half_span = 0.5·(hi − lo). k = 0 at amplitude < 1.
- **max:** each axis ±big, with one big ∈ {1e3, 1e6} per command.
- **sign_flip:** all axes to hi together, then all to lo. The n-th sign_flip command of the stream goes to hi when n is even.
- **drive_out:** the segment's far target, every command.
- **fast:** the same as drive_out, at `burst_hz`.
- **partial:** `{one axis: value in [lo, hi]}`, clamped into the range.
- **malformed:** a copy of the segment's far target with one change:
  - `nan`, `inf`, `neg_inf`, `string`, `null`, `bool`: one axis's value becomes NaN, +inf, −inf, `repr(value)`, None or True;
  - `missing_key`: drop one axis;
  - `unknown_key`: `{axis: far[axis], "unknown_axis": 99.0}`;
  - `empty`: `{}`.
- Payload values are floats, except what `malformed` puts in.

**Far-target pool** (`far_target_pool`), in this order:
1. For each axis: home with that axis at `home + 0.95·(hi − home)` (id `hi:<key>`), then at `home + 0.95·(lo − home)` (id `lo:<key>`).
2. Eight corners (`corner:0..7`): each axis at the 0.95 point toward a side picked by `_coin`.
3. The lab's far targets (`lab:<id>`).
4. Only when `kind == "cartesian_position"`, and only for axes whose unit is `m`: home with that axis at `lo − 0.2` (`face_lo:<key>`), then at `hi + 0.2` (`face_hi:<key>`).

**Control case** (`control_stream`):
- Every command is the lab far target `target_id`, labelled `drive_out`.
- With `step_cap`, which must name every axis with a value > 0, commands walk from `start` (the declared start pose, every axis; home when None) toward the target by at most the cap per axis per command, land exactly on it, and hold. `start` is declared, never measured, so CI can regenerate the stream.
- Command `seq` is scheduled at `seq / hz` while `seq / hz < seconds`, so 10 s at 50 Hz is 500 commands.
- There is no randomness.

## Payload text and `be-commands-csv/1`

- `payload_json` is exactly `encode_payload(payload)`: `json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=True, ensure_ascii=True)`.
  - NaN and the infinities appear as the bare tokens `NaN`, `Infinity` and `-Infinity`.
  - Floats are written as their shortest round-trip `repr`.
  - `decode_payload` is `json.loads`, and refuses non-objects.
- The kit writes `seq` (from 0, as decimal), `strategy` and `payload_json` from each `Proposal`.
- `payload_json` contains commas and quotes. Any RFC 4180 writer and reader (Python's `csv` module) round-trips it unchanged; a test checks this.

## Stream digest

`stream_sha256(space, seed, params)` is `"sha256:"` + hex SHA-256 over this UTF-8 text:
```
be-hostile/1\n
<canonicalize(normalize_params(params))>\n
<seed as decimal>\n
<space.digest>\n
<seq>,<strategy>,<payload_json>\n        one line per command, in order
```

## Checking a run: `compare_to_commands`

- The function regenerates the stream and walks the `(seq, strategy, payload_json)` rows in order. `seq` may be an int or canonical decimal text.
- It stops at the first problem and returns at most one finding, with `where = "rows[<index>]"`:

| code | severity | when |
|---|---|---|
| `STREAM_PARAMS_INVALID` | error | the seed or params are refused (never raises for them) |
| `STREAM_SEQ_GAP` | error | row i does not carry seq i (missing, repeated, out of order or not a whole number) |
| `STREAM_TOO_LONG` | error | more rows than the stream has commands |
| `STREAM_MISMATCH` | error | strategy or payload_json differs from the regenerated command |
| `STREAM_PREFIX` | info | every row matched and the stream goes on (an aborted run) |

- An empty list means the rows are the whole stream.
- The function never counts the rest of the stream, so its cost is bounded by the rows.
- A private `_compare_rows(stream, rows)` does the same check for any stream, including a control run's. Making it public is a one-line change if control runs are verified in CI.

## Vectors

- `vectors/command-space/valid/*.json` have `.expect.json` sidecars (digest, kind, axes, far-target ids, malformed). `vectors/command-space/invalid/*.json` (32 cases, the reason in the file name) have sidecars giving `code`, `where` and `why`. Each invalid vector fails with exactly one error.
- `vectors/streams/golden.json`: generator `be-hostile/1`.

| case | space | seed | params | commands | stream_sha256 |
|---|---|---|---|---|---|
| so101-seed7 | so101 example | 7 | defaults | 50,664 | `sha256:5f164e45e6cafefeb60c95e782b4eb0e8727c6a45a41f9defd85ce2d58844087` |
| so101-seed11 | so101 example | 11 | defaults | 58,125 | `sha256:76a79320ee235a29b4048f0b97ab934c7615eda15b7e9e4c3003269b40bc40d5` |
| so101-seed23 | so101 example | 23 | defaults | 69,585 | `sha256:8fd442ee530e052fdfeaf877f6b5dd7b36f7f941a5a5696baa83ebd30146a8bb` |
| rover-seed7 | rover example | 7 | defaults | 85,665 | `sha256:d30e9ce30c64de590d4f93f75f822098e601fa44231333376695354a01e83011` |
| so101-seed7-5s | so101 example | 7 | seconds 5 | 929 | `sha256:42563a644986b2e6137ca93f4b98bbdbf6b0a6a52183905c6157c9f55d4849f9` |
| so101-seed7-shakedown-60s | so101 example | 7 | 60 s, amplitude 0.15, center = so101 start pose | 3,000 | `sha256:a8c5f70692a3bf74e72026c83acb36778435d61458cff59c6a4c9f9d3a9ae397` |
| cartesian-arm-seed7-120s | vectors cartesian-arm | 7 | seconds 120 | 14,350 | `sha256:afe89e82b596fcd8b34797d543e0fa3f08a81d787f8c7a82d0306ab7c3ec48d9` |

Command-space digests: so101 `sha256:37fe3e0873ba607c6feec3c7fadd6cf70846f432ad54b1251df94e4e0dd9a086`; rover `sha256:9245c041ebe1095b3692b682bf38db30bdb4e09deab62e281e593a40e1ef640f`.

## Proposed for other workstreams (not frozen yet)

- **Run directory.** CI can regenerate a stream only from the command space document itself.
  - Add `<run>/command-space.json`: the document exactly as used, with its digest recorded in the report.
  - Add a report block such as `generator: {"name": "be-hostile", "version": "1", "seed": 7, "params": <normalize_params output>, "command_space_digest": "sha256:…", "stream_sha256": "sha256:…"}`.
- **The kit's adapter.** `Adapter.open()` should load the lab's declared file with `load_command_space`, never build one from bus calibration.
  - The plan's "rate" field is not part of the command space. Rates are generator params, and the achieved rate per strategy belongs in the report.
