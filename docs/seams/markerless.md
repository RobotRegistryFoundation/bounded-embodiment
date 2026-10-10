# Seam: the markerless model tracker (kit, `be fit`)

The instrument that measures where the SO-101's tool, wrist and elbow are without anything on the arm:
the URDF's own meshes fitted to the OAK-D's depth from a fixed camera. It reads a recording made by
`be record` (`be-camera-recording/1`; the recorder's `--help` and `record.py`'s module docstring describe a session) and writes
`be-samples-csv/1`. Kind `markerless_model_tracker`, `external: true`, `independent_of_robot_state: true`.
Everything is in `packages/kit/src/bounded_embodiment_kit/instruments/` (`session.py` and `markerless/`);
core is untouched, and nothing here imports a gate or anything named `rcan*`.

## The rules, and where they are enforced

- **Independence.** The inputs are the session's camera files (`meta.json`, `events.csv`,
  `measurements.txt`, `board.json`, `oakd/`, `wrist/`), the URDF and its meshes, the envelope, the
  registration and the background. Nothing opens encoder or commanded values, harness logs or probe files,
  even inside the session directory. Frame k starts from the fit of frame k-1 (only if that frame was
  valid); the first frame, and the first after an invalid one, start from a global search.
  `test_never_opens_robot_state_or_the_truth` plants `results/arm-hw/*.json` and `probe*.json` in a
  session, logs every file opened by `register`, `background` and `run` (patched `open`, `io.open`,
  `Path.open`, plus an audit hook), and checks none of them (nor the renderer's truth file) is opened.
- **No smoothing.** The cost has no term involving another frame; the previous estimate is only where
  the optimiser starts (`test_the_estimate_does_not_depend_on_the_starting_guess`). Reported points
  are each frame's own fit.
- **Honest validity.** Every row has `valid` 0/1 and `residual_m` (the frame's inlier RMS). A doubtful
  frame or point is invalid, with empty coordinates, and `fit_log.csv` says why.
- **Determinism.** Seeded sampling (meshes, pixel priority), fixed grids, fixed iteration caps;
  two runs write identical `samples.csv` bytes (`test_two_runs_write_identical_bytes`).

## Commands

```bash
pip install -e packages/core -e "packages/kit[fit]"     # fit extra: opencv-python-headless, scipy
python tools/fetch_robot_assets.py examples/robots/so101 --from-dir <SO-ARM100>/Simulation/SO101

be fit register SESSION                 # -> SESSION/fit/registration.json (exit 1 if it has warnings)
be fit background SESSION [--window START:END ...]          # -> SESSION/fit/background.npz, .json
be fit run SESSION --envelope examples/envelopes/bob.envelope.json \
    [--from LABEL] [--to LABEL] [--every N] [--out DIR] [--debug-every N]   # -> samples.csv, fit_log.csv, fit_meta.json
be fit selftest [--quick] [--sensitivity] [--keep DIR]      # synthetic end to end, prints accuracy and ms/frame
```

An arm carrying the wrist camera on the printed mount is `so101_new_calib_camera.urdf` (the same links
plus the mount and the camera): give it with `--urdf` to `register` and `run`.

Times (`--from`, `--to`, `--window`) are an event label (its last occurrence), `LABEL+S`/`LABEL-S`, or
seconds since the recording started. `--urdf` defaults to the URDF named in `registration.json` (if its
sha256 still matches), else `examples/robots/so101/so101_new_calib.urdf`.

## Python API

```python
from bounded_embodiment_kit.instruments.session import Session, parse_measurements
from bounded_embodiment_kit.instruments.markerless.register import register_session, RegistrationError
from bounded_embodiment_kit.instruments.markerless.background import build_background
from bounded_embodiment_kit.instruments.markerless.fit import fit_session, FitConfig, FitError, Tracker
from bounded_embodiment_kit.instruments.markerless.model import ArmModel, base_offset

register_session(session, urdf, *, board_spec=None, out=None, window_s=(1.0, 9.0), measurements=None) -> dict
build_background(session, *, windows=None, out=None) -> dict
fit_session(session, urdf, envelope, registration=None, *, background=None, config=None, out=None,
            t_from=None, t_to=None, every=1, debug_every=0, model=None, max_frames=None) -> dict   # fit_meta
```

`fit_session` takes only the session directory, the URDF (path or a prebuilt `ArmModel`), the envelope
(its points that name a link), the registration (path or dict) and options. `Tracker(model, camera,
points, config, background).step(depth_mm, t)` is the per-frame core. `Session` reads a recording with
`record.py`'s own readers (`_streams`, `_Frames`, `_load_events`, `_read_rows`), so the two cannot
disagree about the files; colour and depth pair by `(oak_session, device_seq)` when their timestamps
agree to half a frame, else by nearest `t_mono_s`. `synth.py` renders recordings with known truth in the
same format (`write_session`, `write_motion_session`).

## Frames: base_link is not F

The tape measurements are in F: origin on the table directly under the shoulder_pan axis, +x forward,
+y left, +z up. The URDF's base_link is not F. `base_offset(urdf)` computes the difference from the
URDF: shoulder_pan's joint origin is (0.0388353, -9e-9, 0.0624) m in base_link and its axis is
vertical, and the lowest point of base_link's meshes (its underside, on the table) is z = -0.0024 m.
So **base_link = F + (38.8 mm, 0.0 mm, -2.4 mm)**: base_link's origin is 38.8 mm behind the axis and
2.4 mm above the table. `register` prints this, and stores it in `registration.json`
(`steps.base_offset`). The runbook's "the base frame ... origin on the table, directly under the
shoulder_pan axis" describes F, not the envelope's `base_link`; the fitter reports in `base_link`.

## Registration (`be fit register`)

`T_bl_cam = T_bl_F . T_F_board . inverse(T_cam_board)`:

1. `T_cam_board`: ChArUco corners (`cv2.aruco.CharucoDetector`, in both OpenCV 4.13 and 5.0) in every
   colour frame from +1 s to +9 s after the last `board_placed` (ending early at the next event), at
   1280x720 with meta.json's intrinsics and distortion. All observations go into one PnP: IPPE on the
   frame with most corners to start, then iterative on the stack, then `solvePnPRefineLM`. The board's
   print scale comes from the two measured 100 mm bars (refused beyond +-3 mm, warned beyond 0.5 mm).
2. `T_F_board`: x = unit(X - O), y = (Y - O) orthogonalised, z = x cross y, origin (O, h). With the
   board face up, z must come out (0, 0, -1); otherwise (X and Y swapped, a y measured with the wrong
   sign, `board_face` not `up`) it is refused.
3. `T_bl_F`: the base offset above.

What each check in `registration.json` means:

| check | meaning | warning when |
|---|---|---|
| `pnp.reprojection_rms_px` | how well one board pose explains every corner seen | > 1 px (board or camera moved, intrinsics off) |
| `measurements.O_to_X_mm`, `O_to_Y_mm` | the tape's corner distances vs 150 / 210 mm (scaled) | either off by > 3 mm |
| `measurements.angle_XOY_deg` | the tape's corner angle | off 90 by > 1 deg |
| `measurements.z_axis_in_F` | the board normal from the tape | not (0, 0, -1): refused |
| `measurements.yaw_difference_deg` | yaw from both edges (least squares) minus yaw from O->X alone | reported |
| `depth.signed_offset_from_pnp_plane_mm`, `angle_between_planes_deg` | depth inside the board outline vs the PnP plane: depth scale and colour/depth alignment | > 10 mm or > 3 deg; never applied |
| `camera_vs_tape` | camera distance/height from the registration vs the rough tape numbers | reported |

## Background (`be fit background`)

Per pixel, the median of the valid depth over a still window (default `background_start` ..
`background_end`; without them, the first 10 s, reported as a fallback); a pixel valid in fewer than
half the frames has none (0). Several `--window`s combine as the farthest per pixel (unknown if
unknown in any). The 10 Oct background has the arm resting in view; adding a hold window
(`--window background_start:background_end --window hold_1:hold_1_end`) uncovers what is behind the
rest pose. The fitter's foreground test is "nearer than the background", so the arm is not lost where
it once rested.

## The fit (`be fit run`)

**Observation.** Depth (640x360 uint16 mm, aligned to colour, colour lens model) back-projected into
base_link. Kept: valid, >= 0.30 m from the camera, inside the URDF-derived ROI (a vertical cylinder
about the shoulder_pan axis: the farthest model point over 2000 seeded poses + 50 mm; up to the highest
+ 50 mm), >= 8 mm above the table, and nearer than the background by max(15 mm, 3 sigma_z) (sigma_z =
1.5 mm (z / 0.6 m)^2) or with no background. At most 4000 points, by a fixed per-pixel priority.

**Cost.** (a) Point-to-plane distance from each point to the nearest dense model sample (1.5 mm apart,
outward normals) of the nearest link, Huber delta 5 mm; each point's weight ramps from 1 at 15 mm to 0 at
the 25 mm gate (40 mm in the first step after a global search). Association is a gather from per-link 2 mm
voxel fields built by an exact distance transform; an exact k-d tree polishes global-search candidates.
(b) Free space: the moving links' samples 9 mm apart that face the camera, z-buffered, against the
observed depth eroded 5x5; where the camera sees > 20 mm (ramping over 10 mm) beyond a sample, it costs
its image distance to the nearest data pixel (in metres at its depth), capped at 20 mm.

**Optimiser.** Levenberg-Marquardt over the six joints (URDF limits widened by 5 deg), analytic
Jacobians, re-association at every evaluation, <= 15 steps per frame; converged when the undamped
Gauss-Newton step would move no link origin more than 0.1 mm. Global search: a 15 deg grid over the
first three joints scored by render-and-compare, a grid over the distal joints for the best ten, the
robust cost for the best 24, LM (on 1500 points) from the best six and from the best in each quarter of
the wrist_flex and wrist_roll ranges, 1-D scans of wrist_flex, wrist_roll and the gripper, wrist_roll
+-180 deg (the closed gripper is nearly symmetric about it), then the best fit and the strongest distinct
alternative polished on all points.

**Tracker.** TRACK: LM from the previous valid frame's estimate. Every 15 tracked frames, a flip test
(wrist_roll +-180 deg from the current fit; switch if it is cheaper). REINIT: a global search (the first
frame, after an invalid frame, or a recheck). LOST: the frame is invalid; the next frame starts over.

**Validity.** All thresholds are fields of `FitConfig` (recorded in `fit_meta.json`):

| check (frame) | threshold | what it catches |
|---|---|---|
| enough data | >= 300 kept points | arm out of view, blackout, occlusion |
| converged | LM settled (see above) | a fit still moving when its steps ran out |
| inliers | >= 300 within 10 mm, and >= 50 % of the points | the model explains only part of the data |
| `rms_m` | inlier point-to-plane RMS <= 6 mm | wrong pose, wrong model, bad depth |
| `free_space_frac` | <= 10 % of the visible model samples seen through | model where the camera sees past it |
| jump | no point moved > 1.5 m/s x dt + 30 mm since the last valid frame | a flip to another basin |

| check (point) | threshold | what it catches |
|---|---|---|
| formal sigma | <= 2 mm (1-sigma through the normal equations) | the joints this point depends on are unseen |
| ambiguity | no other fit within 1.05x the cost puts it > 3 mm away | two explanations of the data (a mirrored gripper); the point stays invalid until a global search, every 15 frames, settles it |

The formal sigma underestimates real error (about 5x on synthetic data); it is an observability check,
not an error bar.

## Files

| file | content |
|---|---|
| `fit/registration.json` | `T_bl_cam`, `T_cam_bl`, `camera_in_base_link`, `chain`, `steps` (each transform, `base_offset` with its explanation), `inputs` (board spec and scale, measurements, URDF path+sha256, intrinsics, window), `checks` (above), `warnings` |
| `fit/background.npz` | `background_mm` (uint16), `iqr_mm`, `valid_frac`, `windows` (t_mono_s), `frames`; `background.json` the report |
| `fit/samples.csv` | `be-samples-csv/1`: `t_s,point,x_m,y_m,z_m,valid,residual_m` per depth frame and envelope point, base_link; invalid rows have empty x/y/z |
| `fit/fit_log.csv` | per frame: `frame, depth_index, t_s`, the six joints (rad), `cost, alt_cost, points, inliers, rms_m, free_space_frac, iterations, status, valid, reason, ms` |
| `fit/fit_meta.json` | instrument, versions, inputs (URDF, mesh, registration and background sha256, envelope hash, intrinsics), range, `FitConfig`, model description, counts, ms/frame |
| `fit/debug/frame_NNNNNN.png` | `--debug-every N`: the paired colour frame, the fitted outline, the points and the data |

`t_s` is the depth frame's `t_mono_s`, the recording host's CLOCK_MONOTONIC. It is not yet the run
timebase of CONTRACT.md (zero at run start): converting needs the run's start on the same clock
(`clock.csv` maps wall time to it).

## Accuracy and speed (synthetic)

`be fit selftest` renders the 10 Oct procedure (board, background with the arm resting, a sweep to
READY, 10 s of the pick-and-place at 30 fps) with a passive-stereo noise model (sigma_z 1.5 mm at
0.6 m growing with z^2, half spatially correlated; 15 % holes in blobs on flat faces; flying pixels at
depth edges; mm quantisation), registers from the rendered board, and fits. With the exact model:

| point | p50 | p95 | max | valid |
|---|---|---|---|---|
| tool | 0.6 mm | 1.0-1.1 mm | 1.3-1.6 mm | 300/300 |
| elbow | 0.2 mm | 0.4-0.5 mm | 0.6-0.7 mm | 300/300 |
| wrist | 0.4 mm | 0.8 mm | 1.2 mm | 300/300 |

Registration from the rendered board: 0.20 mm, 0.02 deg (0.54 mm, 0.06 deg with the corners taped to
the nearest mm). 20 % holes: p95 <= 1.0 mm. A 1 s blackout: exactly those 30 frames invalid, the next one a
valid re-init. A 40 mm sphere (a hand) beside the shoulder, appearing after the background: points move
p95 0.3 mm, max 0.5 mm. Measured, not asserted: a registration yawed 1 deg about the board's corner O
moves the tool 7-8 mm and the elbow and wrist 5-6.5 mm; an arm whose upper arm and forearm are each 2 mm
longer than the URDF gives tool p95 1.8 mm, elbow 2.3 mm, wrist 1.6 mm.

Speed (a 2-core x86-64 VM, <= 4000 points): mean ~95 ms/frame while tracking (p50 ~72 ms); the
flip test every 15 frames takes ~300 ms (so p95 ~320 ms); a global search 3-5 s. Over the 10 s run with
its one re-init: mean ~110 ms/frame.

## Tests

`packages/kit/tests/test_markerless_{mesh,session,register,fit}.py`, fixtures in `conftest.py`.
The end-to-end tests are marked `slow` (about 5 minutes; `-m "not slow"` leaves them out) and skip,
saying how to fetch them, when the meshes are absent.
