# Seams for v0.1 (frozen 10 Oct 2026)

Every workstream builds against these names. If a name here is wrong, say so and get the file changed before you build on a different one. The approved plan is `claude/bounded-embodiment-kit-plan.md` in the project.

## Distributions

| | core | kit |
|---|---|---|
| PyPI name | `bounded-embodiment` | `bounded-embodiment-kit` |
| Import name | `bounded_embodiment` | `bounded_embodiment_kit` |
| Path | `packages/core` | `packages/kit` |
| Python | ≥ 3.10 | ≥ 3.10 (the `lerobot` extra needs ≥ 3.12) |
| Runtime deps | `jsonschema>=4.18`, `cryptography>=42`, nothing else | core at the same minor version |
| Extras | none | `lerobot`; `camera` (`opencv-python-headless>=4.9,<4.14`); `oakd` (`depthai>=2.24`) |

- **Core must not import** numpy, the kit, lerobot, cv2, serial, depthai, any gate, or anything named `rcan*`.
- **The kit must not import** any gate or anything named `rcan*`.
- **Gates** live in `examples/reference-gates/`. Neither package ever imports them.

**CLI.** `be`, with a `bounded-embodiment` alias.
- Core registers the subcommands `verify`, `keygen`, `envelope` (`validate`, `convert`, `hash`), and `canon` (debug).
- The kit adds subcommands through the entry-point group `bounded_embodiment.cli`.
- `be run`, `be record`, `be fit`, `be instrument`, `be report`, `be submit` and `be doctor` all come from the kit.

## Core module surface

**`bounded_embodiment.canonical`**
```python
canonicalize(obj) -> bytes                 # RFC 8785 (JCS)
loads_strict(data: bytes | str) -> object  # rejects duplicate keys, NaN, ±Infinity, lone surrogates
sha256_jcs(obj) -> str                     # "sha256:" + 64 lower-hex
class CanonicalError(ValueError)
```
- Numbers are serialized with ECMAScript `Number.prototype.toString`, as in RFC 8785 §3.2.2.3.
- Integers above 2^53 in magnitude raise `CanonicalError`.
- Object keys sort by UTF-16 code units.

**`bounded_embodiment.signing`**
```python
generate_key() -> tuple[dict, dict]   # (private_jwk, public_jwk); OKP, crv Ed25519
thumbprint(public_jwk) -> str         # RFC 7638 over {"crv","kty","x"}; base64url, no padding
sign(doc: dict, private_jwk: dict) -> dict
verify(doc: dict, keyring: dict[str, dict]) -> list[Finding]   # keyring: kid -> public_jwk
```
- `sign` returns a copy of the document with `signature: [{"alg": "Ed25519", "kid": <thumbprint>, "sig": <base64url>}]` appended.
- The signature covers `canonicalize(doc minus its top-level "signature" member)`.
- An existing `signature` array is kept and appended to.
- `verify` never trusts a public key embedded in the document; it only uses the keyring.
- Unknown `alg` values give a warning finding, not an error. An entry with `alg: "ML-DSA-65"` is verified if the `cryptography` build supports it, and otherwise skipped with a warning.

**`bounded_embodiment.findings`**
```python
@dataclass(frozen=True)
class Finding:
    code: str        # STABLE_UPPER_SNAKE
    severity: str    # "error" | "warn" | "info"
    detail: str
    where: str = ""
```

**`bounded_embodiment.schema`**
```python
validate(doc, schema_name: str) -> list[Finding]   # e.g. "envelope-0.2", "test-report-0.1", "key-0.1"
```
- Schemas ship inside the package at `bounded_embodiment/schemas/*.json`, copied from the repo-root `schemas/` at build time.
- Validation never fetches anything over the network.

**`bounded_embodiment.envelope`**
```python
load(src: str | Path | dict) -> Envelope   # accepts 0.1 (upgraded in memory) and 0.2; raises EnvelopeError with findings
```

| `Envelope` member | Meaning |
|---|---|
| `version` | `"0.2"`, after any upgrade |
| `document` | the dict as given |
| `hash` | `sha256_jcs` of the document minus `signature` |
| `points` | `dict[str, Point]` |
| `zones` | `list[Zone]` |
| `zones_for(point_id)` | the zones whose `applies_to` includes the point or `"*"` |
| `clearance(point_id, xyz)` | signed metres; positive means compliant, negative means outside by that much (see below) |
| `faces(point_id)` | `[(face_id, zone_id, fn(xyz) -> distance_to_face_m)]`, used for coverage |
| `speed_limit_mps(point_id)` | the point's `max_speed_mps`, falling back to `motion.max_speed_mps` |

`clearance(point_id, xyz)` is the minimum of two kinds of term:
- for each keep-in zone, the distance inside it (negative if outside);
- for each keep-out zone, the distance outside it (negative if inside).

## Envelope 0.2 (`schemas/envelope-0.2.json`, CC BY 4.0)

It extends rcan-spec envelope 0.1 (copied unchanged to `schemas/envelope-0.1.json`) and uses `additionalProperties: false` throughout.
```jsonc
{
  "envelope_version": "0.2",
  "machine": {"id": "...", "class": "arm|mobile_base|mobile_manipulator|legged|other", "description": "..."},
  "declared_level": "BE-1|BE-2|BE-3",          // OPTIONAL, self-declared, never shown by results tooling
  "frame": "base_link",                          // every coordinate below is in metres in this frame
  "points": [                                    // >= 1
    {"id": "tool", "description": "centre of the gripper fingertips, closed", "link": "gripper", "offset_m": [0,0,-0.03], "max_speed_mps": 0.25}
  ],
  "workspace": {"zones": [                       // >= 1 keep_in
    {"id": "box", "kind": "keep_in", "applies_to": ["tool"],
     "box": {"min_m": [0.21,-0.10,0.26], "max_m": [0.36,0.10,0.42]}},
    {"id": "post", "kind": "keep_out", "applies_to": ["*"],
     "prism": {"polygon_m": [[x,y],...], "z_m": [zmin, zmax]}}   // a zone has exactly one of box | prism
  ]},
  "motion": {"max_speed_mps": 0.25, "max_turn_radps": 1.0, "max_accel_mps2": 2.0,
             "joints": [{"name": "elbow_flex", "position_rad": [-1.6, 1.6], "max_speed_radps": 1.0}]},
  "force": {...}, "proximity": [...], "sensing": {"max_state_age_ms": 100},   // as in 0.1
  "stop": {"category": 1, "max_time_ms": 300, "max_distance_m": 0.05,
           "sources": [{"source": "stop_message|hardwired_stop|heartbeat_loss|comms_loss|power_loss|gate_loss",
                        "category": 1, "max_time_ms": 300, "max_distance_m": 0.05}]},
  "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
  "authority": {...},                                                     // as in 0.1
  "notes": "free text",
  "signature": "legacy-string" | [{"alg","kid","sig"}]
}
```

**Upgrading 0.1 to 0.2 (in memory only):**
- `workspace.keep_in` becomes `{"id": "keep_in", "kind": "keep_in", "applies_to": ["*"], "prism": {"polygon_m": <polygon>, "z_m": <z_range_m or omitted>}}`.
- Each `keep_out[i]` becomes `{"id": "keep_out_<i>", "kind": "keep_out", "applies_to": ["*"], "prism": {"polygon_m": <polygon>, "z_m": <same z_range_m or omitted>}}`.
- `frame` is copied up from `workspace.frame`.
- `points` defaults to `[{"id": "machine", "description": "the machine's reference point as declared in 0.1"}]`.
- `level` becomes `declared_level`.
- The original 0.1 document is what gets hashed.

**Two rules the loader enforces beyond the schema:**
- Each `min_m` is below its `max_m`, and polygons are simple (no self-intersection).
- No `stop.sources[]` entry exceeds the top-level `stop` values.

**Kept out of the envelope (they live in kit or profile config):** tolerance, start pose, joint map, URDF path.

## Profiles (`profiles/*.json`, CC BY 4.0)

`ev03-illustrative-0.1.json`:
```json
{"profile":"ev03-illustrative-0.1","thresholds_are_illustrative":true,
 "tolerance_m":0.01,"tolerance_speed_frac":0.05,"speed_window_s":0.2,
 "min_duration_s":600,"min_base_rate_hz":45,"burst_rate_hz":500,
 "required_strategies":["uniform","boundary","max","sign_flip","drive_out","fast","partial","malformed"],
 "gap_min_s":0.1,"gap_max_s":1.0,"gap_max_fraction":0.05,
 "motion_window_s":1.0,"motion_min_displacement_m":0.01,"motion_sigma_factor":6,"motion_min_window_fraction":0.05,
 "accuracy_p95_max_tolerance_frac":1.0,"accuracy_max_max_tolerance_frac":1.5,"post_check_drift_max_tolerance_frac":0.5,
 "accuracy_check_max_age_s":43200}
```

## Raw data formats (all UTF-8 CSV with a header row; compressed with gzip in a submission)

- **`be-samples-csv/1`:** `t_s,point,x_m,y_m,z_m,valid,residual_m`. Coordinates are in the envelope `frame`. `valid` is 0 or 1. `residual_m` is the instrument's own fit residual, or empty.
- **`be-commands-csv/1`:** `seq,t_sent_s,t_done_s,strategy,payload_json,result,error_kind`.
- **`be-state-csv/1`:** `t_s,source,joint,value` (long format); `source` is `encoder` or `fk_tool`.
- **`be-accuracy-csv/1`:** `phase,label,ref_x_m,ref_y_m,ref_z_m,meas_x_m,meas_y_m,meas_z_m,n`. `phase` is `pre`, `post` or `jitter`.

Every `t_s` is seconds on the run's single timebase: host `time.monotonic()` at run start equals 0. Instruments convert their device timestamps to this timebase, and record the offset and uncertainty in the run's `clock` block.

## Run directory (the unit of evidence)

```
<run>/report.json            signed test report 0.1
<run>/envelope.json          the envelope exactly as used (its hash is in the report)
<run>/raw/samples.csv(.gz)   be-samples-csv/1
<run>/raw/commands.csv(.gz)  be-commands-csv/1
<run>/raw/state.csv(.gz)     be-state-csv/1
<run>/raw/accuracy.csv       be-accuracy-csv/1
<run>/raw/instrument/...     calibration and setup files (listed in report.raw_data)
<run>/notes/NOTES.md, <run>/notes/photos/*.jpg   optional; added by `be report` before signing
```

## Instrument kinds (`report.instruments[].kind`)

- `markerless_model_tracker`: a multi-view or depth fit of the machine's own geometry.
  - It is `external: true` only when `independent_of_robot_state: true`, meaning it is never seeded from encoder or commanded values.
- `optical_marker_tracker`, `motion_capture`, `laser_tracker`, `depth_camera`.
- `robot_encoders_fk` is always `external: false`.
- `simulation_ground_truth` is always `external: false`.
- `other`.

## Verdicts

The verdict is `PASS`, `FAIL` or `INCONCLUSIVE`, and `report.build` always works it out from the raw data.
- The kit never writes a verdict.
- Reason codes are `UPPER_SNAKE` and stable once released.
