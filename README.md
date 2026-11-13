# HySlam

A modularized monocular visual SLAM demo built with OpenTK (OpenGL) and OpenCVSharp.
It reads frames from a video file, converts them to grayscale for all vision processing,
tracks near-field road features with pyramidal Lucas-Kanade optical flow, estimates a
metric 3-DOF camera motion (yaw + translation) on the road plane, triangulates above-road
structure, colors each point from the source video, and renders a single full-window
free-camera view of the resulting map. An optional headless `--bake` mode processes
the whole clip in several passes and exports the best-scoring result. A read-only
`--load` viewer and a headless `--verify` check reopen that export without the video.

> **Experimental project.** HySlam is a research and teaching prototype, not a
> production SLAM system. It deliberately keeps a small, inspectable pipeline
> instead of optimizing for robustness. Trajectories and point clouds can drift,
> and the absolute scale can be wrong even when the visualization looks plausible.
> Do not use it for navigation, safety-critical decisions, surveying or
> autonomous control.

## Scope and assumptions

| Topic | Assumption / limitation |
| --- | --- |
| Camera | Monocular, rigidly mounted, known height above the ground plane |
| Motion | 3-DOF planar motion: yaw plus forward/lateral translation. Roll, pitch changes and vertical motion are not modelled |
| Ground | Locally planar and visible in the lower part of the frame |
| Scale | Fixed by `--camera-height` and the focal length; a wrong height or focal length scales the whole trajectory |
| Mapping | Sparse points from recent keyframes (multi-keyframe triangulation); no loop closure, relocalization or bundle adjustment |
| Modes | Interactive OpenGL viewer, headless multi-pass `--bake`, read-only `--load` viewer, headless `--verify` |
| Output | Approximate trajectory and point cloud for inspection, not metrology or control |

## Quick start

From the repository root:

```powershell
dotnet build HySlam.slnx
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4
```

The repository does not include videos or images, so `--path` must point to a
local clip. Press `Esc` to close the interactive viewer. Show every option with:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --help
```

For a short, repeatable run without wall-clock pacing:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --preset:road --max-frames 120 --deterministic
```

For a headless multi-pass export of the whole clip:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --preset:road --bake --bake-passes 5
```

To inspect or validate an existing export without the video:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --load C:\runs\bake_road
dotnet run --project HySlam\HySlam.csproj -- --verify C:\runs\bake_road
```

## Project Structure

```
HySlam.slnx
HySlam/
├── Program.cs                      // Entry point (Main) and CLI parsing
├── usage.txt                       // Help text printed by --help
├── HySlam.csproj
│
├── Core/
│   ├── WindowManager.cs            // OpenTK lifecycle and interactive loop
│   ├── LoadedMapWindow.cs          // Read-only viewer for an exported map
│   ├── RunOptions.cs               // Parsed command line configuration
│   │
│   ├── Bake/
│   │   ├── BakeOptions.cs          // Pass count, output directory, keyframe budget
│   │   ├── BakeRunner.cs           // Headless multi-pass orchestration and export
│   │   ├── BakePassResult.cs       // Per-pass map, steps, frame metrics, settings
│   │   ├── BakeScorer.cs           // Comparable score for each pass
│   │   └── ProgressBar.cs          // Percentage bar with ETA and redirected fallback
│   │
│   ├── Slam/
│   │   ├── SlamEngine.cs           // OpenCV pipeline (optical flow, pose, triangulation)
│   │   ├── MotionPredictor.cs      // Constant-velocity forecast and outlier gate
│   │   ├── PosePriorStep.cs        // Previous-pass motion prior for one frame
│   │   ├── CameraCalibration.cs    // Intrinsics/distortion loading and scaling
│   │   ├── CameraPose.cs           // Camera rotation + translation
│   │   └── KeyframeManager.cs      // Bounded keyframe history
│   │
│   ├── Diagnostics/
│   │   ├── RunRecorder.cs          // metrics.csv, points.csv, trajectory.csv, summary.txt
│   │   ├── MapFileReader.cs        // Reads an exported map back into MapData
│   │   ├── BakeVerifier.cs         // Headless cross-check of a bake output
│   │   └── LogControl.cs           // Runtime Serilog level switch used by the bake bar
│   │
│   ├── Data/
│   │   ├── MapPoint.cs             // A single 3D map point
│   │   ├── MapData.cs              // Map collections (points + trajectory)
│   │   └── VoxelGrid.cs            // Spatial hash that prevents duplicate points
│   │
│   └── Rendering/
│       ├── MapRenderer.cs          // Single free-camera OpenGL view (VAO/VBO, draw calls)
│       ├── Shader.cs               // Shader compilation/linking and uniform handling
│       └── Shaders.glsl            // Vertex + fragment shader (#vertex / #fragment)
│
└── UI/
    └── CameraFeedWindow.cs         // VideoCapture wrapper (no OpenCV preview window)
```

## Requirements

- .NET 10 SDK
- 64-bit Windows (the native OpenCV runtime ships with the `OpenCvSharp5.runtime.win` package)
- A GPU/driver with OpenGL 3.3 Core support

NuGet packages:

| Package | Purpose |
| --- | --- |
| `OpenTK` | Window, OpenGL context, input |
| `OpenCvSharp5` | Image processing, ORB, optical flow, geometry |
| `OpenCvSharp5.runtime.win` | Native OpenCV DLLs |

## Build

```powershell
dotnet build HySlam.slnx
```

## Run

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4
```

Or run the built executable directly:

```powershell
.\HySlam\bin\Debug\net10.0\HySlam.exe --source:video --path C:\clips\road.mp4
```

`--path` accepts any local video file supported by OpenCV. The repository does
not track source footage; use your own clip or the path to a locally stored demo
video.

### Detailed run commands

All commands are PowerShell examples from the repository root. The `--` token
separates `dotnet run` options from application options; everything after it is
parsed by HySlam.

```powershell
# Restore and build the Debug configuration
dotnet restore HySlam.slnx
dotnet build HySlam.slnx

# Show the built-in help; no video is opened
dotnet run --project HySlam\HySlam.csproj -- --help

# Run from source with a local clip
dotnet run --project HySlam\HySlam.csproj -- `
  --source:video --path C:\clips\road.mp4 --preset:road --camera-height 1.3

# Run the built executable directly
.\HySlam\bin\Debug\net10.0\HySlam.exe `
  --source:video --path C:\clips\road.mp4 --preset:road --camera-height 1.3

# Build and run a Release build
dotnet build HySlam.slnx -c Release
.\HySlam\bin\Release\net10.0\HySlam.exe `
  --source:video --path C:\clips\highway.mp4 --preset:highway --camera-height 1.3

# Write a deterministic dump for inspection
dotnet run --project HySlam\HySlam.csproj -- `
  --source:video --path C:\clips\short.mp4 `
  --preset:road --deterministic --max-frames 300 --dump C:\runs\baseline

# Use a real calibration file and override one preset value
dotnet run --project HySlam\HySlam.csproj -- `
  --source:video --path C:\clips\road.mp4 --calib C:\clips\camera.txt `
  --preset:road --camera-height 1.25 --band-top 0.72
```

Quote paths that contain spaces:

```powershell
dotnet run --project HySlam\HySlam.csproj -- `
  --source:video --path "C:\My Clips\road test.mp4" --preset:road
```

The dump directory is created automatically when it does not exist. The process
exits after the windows are closed and writes the final `points.csv`,
`trajectory.csv` and `summary.txt` at that point.

Publishing uses the project defaults, including `PublishAot=true`:

```powershell
dotnet publish HySlam\HySlam.csproj -c Release -r win-x64
.\HySlam\bin\Release\net10.0\win-x64\publish\HySlam.exe --help
```

Native AOT publishing needs the matching MSVC C++ toolchain; a normal
`dotnet build` does not. If the AOT toolchain is not installed, use the
framework-dependent build output above.

### Command line

```text
HySlam.exe --source:video --path <file> [options]
HySlam.exe --help
```

Every option that takes a value accepts all three forms: `--path C:\clips\road.mp4`,
`--path:C:\clips\road.mp4` and `--path=C:\clips\road.mp4`. `--deterministic` is a flag and
rejects a value. Option names are case-sensitive; preset names are not.

| Option | Value | Default | Accepted range | Effect |
| --- | --- | --- | --- | --- |
| `--source` | `video` | `video` | only `video` | Frame source. Anything else exits with an error |
| `--path` | file path | required for video runs | existing, readable file | Video decoded by OpenCV. Required unless `--load` or `--verify` is used. Relative paths resolve against the current working directory |
| `--calib` | file path | none | valid `key=value` file | Real intrinsics and distortion. Overrides `--fov`; see [Calibration](#calibration) |
| `--dump` | directory | none | any writable path | Writes `metrics.csv`, `points.csv`, `trajectory.csv` and `summary.txt`. The directory is created when missing |
| `--load` | directory | none | existing map directory | Opens `points.csv` and `trajectory.csv` in the read-only viewer; no video is decoded. Mutually exclusive with `--verify` and `--bake` |
| `--verify` | directory | none | existing bake output | Validates the five bake files headlessly and exits with code `0` or `1`. Mutually exclusive with `--load` and `--bake` |
| `--max-frames` | integer | `0` (unlimited) | `>= 1` | Stop after `N` decoded source frames. Only every third frame is processed, so `--max-frames 120` feeds about 40 frames to the pipeline |
| `--deterministic` | flag | off | no value | Read one source frame per update instead of wall-clock pacing; also seeds OpenCV RNG |
| `--camera-height` | meters | `1.3` | `> 0` | Camera height above the road. Sets the metric scale of the whole trajectory |
| `--pitch` | degrees | horizon-derived | `-45 .. 45` | Camera tilt; positive looks down. Indoor presets default to `5` when the option is omitted |
| `--fov` | degrees | fallback `fx = fy = 1.1 * width` | `> 0` and `< 180` | Horizontal FOV for the fallback intrinsics. Omit it to keep the width-based fallback; a value of `0` is rejected |
| `--preset` | name | `road` | `road`, `highway`, `fast`, `mountain`, `city`, `office`, `home` plus aliases | Applies a scene tuning table; see [Scene presets](#scene-presets) |
| `--min-depth` | meters | preset (road `6`) | `> 0` and `< --max-depth` | Near limit of the ground-plane depth window |
| `--max-depth` | meters | preset (road `26`) | `> 0` and `> --min-depth` | Far limit of the ground-plane depth window |
| `--structure-depth` | meters | preset (road `45`) | `> 0` | Maximum camera depth for triangulated above-road structure |
| `--band-top` | fraction | preset (road `0.70`) | `0 .. 1` | Top of the road detection band as a fraction of frame height |
| `--band-bottom` | fraction | preset (road `0.99`) | `0 .. 1` | Bottom of the road detection band. Requires `bottom - top >= 0.05` |
| `--keyframe-frames` | integer | preset (road `30`) | `>= 1` | Maximum processed frames a keyframe stays active before it is replaced |
| `--max-step` | meters | preset (road `3.0`) | `> 0` | Maximum camera translation accepted for one processed frame |
| `--lk-window` | pixels | preset (road `21`) | odd, `5 .. 61` | Lucas-Kanade search window size |
| `--lk-levels` | integer | preset (road `3`) | `0 .. 6` | Lucas-Kanade pyramid levels |
| `--lk-error` | pixels | preset (road `20`) | `> 0` | Maximum per-point LK error for a match to count |
| `--bake` | flag | off | no value | Run headless and process the whole clip in multiple passes. Implies `--deterministic` and replaces `--dump` |
| `--bake-passes` | integer | `5` | `1 .. 10` | Number of passes. `1` is a plain baseline; `5` is the recommended full bake |
| `--bake-output` | directory | `runs/bake_<timestamp>` | any writable path | Directory for the selected pass's CSV files and reports |
| `--bake-max-keyframes` | integer | `512` | `8 .. 8192` | Keyframe history kept per pass; a larger history allows triangulation from older baselines |
| `--help`, `-h` | none | - | - | Print the built-in `usage.txt` and exit with code `0` |

The three `--bake-*` options require `--bake`. A bake writes only to
`--bake-output`; `--dump` is ignored in that mode. `--load` and `--verify` are
mutually exclusive and cannot be combined with `--bake`; `--path` and `--dump`
are ignored in read-back mode.

Bounds are validated after the preset and the explicit options are merged, not while each
argument is read. For example `--preset:city --min-depth 25 --max-depth 26` succeeds because
the resulting window is valid, while `--preset:city --min-depth 25` fails because the preset
`--max-depth 20` is then smaller than the explicit minimum.

When the video ends (or `--max-frames` is reached), the window stays open so the map can be
inspected and the dump is finalized. The process exits with code `0` on success or after
`--help`, and with code `1` on invalid arguments, an unknown option or a missing file.
`--verify` returns `0` only when every cross-check passes.

### Settings and precedence

A setting is resolved in three layers:

1. **Built-in default** - `--camera-height 1.3`, fallback intrinsics, no dump, unlimited frames.
2. **Preset** - `--preset` fills only the scene-dependent values listed in
   [Scene presets](#scene-presets).
3. **Explicit option** - any option passed on the command line wins, regardless of where it
   appears relative to `--preset`.

Consequences worth remembering:

- `--preset:city --band-top 0.74` keeps every other `city` value and changes only the band top.
- Options are applied left to right, so repeating one uses the **last** occurrence:
  `--camera-height 1.1 --camera-height 1.4` runs with `1.4`.
- `--help` exits as soon as it is reached: `--bogus --help` reports the unknown option,
  while `--help --bogus` prints the help text and exits with code `0`.
- Numbers use the invariant culture: write `1.25` and `0.70`, never `1,25` or `0,70`.
  Angles, distances, integers and band fractions are parsed the same way in every locale.
- Relative paths for `--path`, `--calib`, `--dump`, `--load` and `--verify` are converted
  to absolute paths against the directory you start the process from, not the project
  directory.
- `--calib` replaces the intrinsics from `--fov`, so a calibration run does not need `--fov`.
- A preset changes only numeric scene settings. It never overrides `--camera-height`,
  `--dump`, `--max-frames`, `--deterministic` or the input path.

Use these units consistently: meters for `--camera-height`, `--min-depth`, `--max-depth`,
`--structure-depth` and `--max-step`; degrees for `--pitch` and `--fov`; a fraction of the
frame height for `--band-top` and `--band-bottom`; pixels for `--lk-window` and `--lk-error`.

#### Tuning by symptom

| Symptom | Setting to inspect first |
| --- | --- |
| Path length is off by a constant factor | `--camera-height`, then `--calib` or `--fov` |
| Speed looks plausible but the map drifts | `--keyframe-frames`, `--max-step`, preset mismatch |
| Tracking collapses on fast footage | `highway` / `fast` preset, then `--lk-window`, `--lk-levels`, `--lk-error` |
| Road fit is pulled towards walls, cars or sky | narrow `--band-top` / `--band-bottom`; shorten `--max-depth` |
| Almost no road inliers | widen the band, raise `--min-depth` only if the near field is noisy |
| Very few structure points | shorter `--keyframe-frames`, larger `--structure-depth`, check `--max-step` |
| Trajectory jumps during a hand-held pan | smaller `--max-step`, shorter keyframe interval, keep the camera level |
| Indoor path is far too short or too long | `office` / `home` preset, measured `--camera-height`, then `--fov` |
| Runs cannot be compared between machines | add `--deterministic --max-frames N --dump <dir>` |
| No dump appears | check that `--dump` is present and the directory is writable |
| Window opens but no frames | check the codec and `--path`; OpenCV must be able to decode the file |

Change one setting at a time and compare `summary.txt` plus `metrics.csv` between runs. A
single run cannot tell you whether a change helped; a deterministic baseline can.

## Usage scenarios

The commands below use `C:\clips\<name>.mp4` as a placeholder. Replace it with a
local file; video and image files are intentionally not tracked by git. Presets are
starting points, not calibration: pass a measured `--camera-height` and, when
possible, a real calibration file or a measured `--fov`.

### 1. Minimal smoke test

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --preset:road --max-frames 120 --deterministic --dump C:\runs\smoke
```

This reads 120 source frames with deterministic pacing and writes the CSV dump. Use
it to verify the build, OpenGL context, video decoding and dump path before tuning
anything.

### 2. Dashcam on a normal road (default)

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --preset:road --camera-height 1.3
```

Expected behaviour: near-field ground band `0.70-0.99`, depth window `6-26 m`, step
gate `3 m`. This is the reference scenario for rigid dashcams, lane markings and
moderate speeds.

### 3. Motorway and high-speed footage

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\highway.mp4 `
  --preset:highway --camera-height 1.3
```

Use `--preset:fast` when the camera moves farther between frames than the default
Lucas-Kanade search can follow, for example low-FPS or very high-speed footage:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\fast.mp4 `
  --preset:fast --camera-height 1.3
```

### 4. Mountain road

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\mountain.mp4 `
  --preset:mountain --camera-height 1.3
```

This preset uses a shorter keyframe interval and a wider LK error gate than `road`,
which helps on winding roads with trees, rock and weak lane markings.

### 5. Dense city / stop-and-go

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\city.mp4 `
  --preset:city --camera-height 1.3
```

The `city` preset narrows the depth window and the detection band because buildings,
parked cars and other vertical surfaces contaminate the ground plane.

### 6. Hand-held indoor clip

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\room.mp4 `
  --preset:home --camera-height 1.3
```

Use `--preset:office` for larger rooms or a camera held at desk height:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\office.mp4 `
  --preset:office --camera-height 1.5
```

The planar model cannot represent roll, pitch changes or vertical bobbing. Avoid
panning while walking; see [Home and office limitations](#home-and-office-limitations).

### 7. Calibrated run

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --calib C:\clips\camera.txt --camera-height 1.3
```

Prefer a real calibration file over `--fov` when one is available: it fixes the
intrinsics and supplies the OpenCV distortion coefficients. The file format is
documented in [Calibration](#calibration).

### 8. Reproducible regression run

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\short.mp4 `
  --preset:road --deterministic --max-frames 300 --dump C:\runs\baseline
```

Repeat the command with the same clip and options to get byte-identical CSV dumps.
Keep `trajectory.csv`, `summary.txt` and `metrics.csv` as a baseline and compare
them after changing the pipeline.

### 9. Parameter sweep

Presets are only defaults; any explicit option overrides the preset value:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\city.mp4 `
  --preset:city --band-top 0.74 --max-step 1.5 --camera-height 1.3
```

Change one parameter at a time. Read `summary.txt` first, then use `metrics.csv` to
see where tracking, the RANSAC fit or the structure triangulation starts to fail.

### 10. Diagnosing wrong scale or lost tracking

- **Path length off by a constant factor:** check `--camera-height`, then adjust
  `--fov` or provide `--calib`. The product of height and focal length sets every
  forward distance.
- **Tracking drops on fast footage:** use `highway` or `fast`, widen `--lk-window`,
  add `--lk-levels`, or raise `--lk-error`.
- **Ground plane contaminated by walls or sky:** narrow the detection band with
  `--band-top` / `--band-bottom`, or shorten `--min-depth` / `--max-depth`.
- **Indoor trajectory drifts during a pan:** the planar model cannot represent the
  motion. Keep the camera level, avoid panning while walking, and use a shorter
  keyframe interval or a smaller `--max-step`.

### 11. Offline multi-pass bake

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\mountain.mp4 `
  --preset:mountain --camera-height 1.3 --bake --bake-passes 5
```

No window opens. Each pass is scored, and the best result is written to
`runs/bake_<timestamp>`. Use `--bake-passes 2` for a quick preview; see
[Offline bake](#offline-bake) for the pass plan and output files.

### 12. Inspect or validate an export

```powershell
dotnet run --project HySlam\HySlam.csproj -- --load C:\runs\bake_road
dotnet run --project HySlam\HySlam.csproj -- --verify C:\runs\bake_road
```

`--load` reopens the exported point cloud and trajectory in the free-camera
viewer without the video. `--verify` checks the five bake files against
`summary.txt` and exits non-zero on a mismatch. See
[Reading a bake output](#reading-a-bake-output).

## Offline bake

`--bake` turns HySlam into a headless batch tool. It processes the complete clip
several times instead of showing the interactive viewer, keeps the best result and
writes it to disk. Use it when the window is only needed for inspection and the
final map or trajectory matters.

```powershell
.\HySlam\bin\Debug\net10.0\HySlam.exe --source:video --path C:\clips\road.mp4 `
  --preset:road --camera-height 1.3 --bake --bake-passes 5 `
  --bake-output C:\runs\bake_road
```

### Pass plan

| Pass | Name | What changes | Why |
| --- | --- | --- | --- |
| 1 | `baseline` | Preset parameters, no prior | Produces the reference trajectory and the per-frame metrics |
| 2 | `adaptive` | Wider LK search, one more pyramid level, 25% higher LK error and a 20% larger step cap when the baseline left gaps | Recovers frames where tracking collapsed |
| 3 | `guided` | Wider LK window, 10% stricter LK error and the best trajectory so far as a per-frame motion prior | Rejects jumps and keeps the trajectory smooth through weak segments |
| 4 | `structure` | Half the keyframe interval, up to five structure keyframes and a 400-point triangulation budget (default 220) | Fills map gaps with more and denser triangulation baselines |
| 5 | `cleanup` | Structure settings plus removal of points observed only once, when at least 30% of the points are confirmed | Removes noise and decides the final result |

Passes beyond five repeat the cleanup plan with a slightly wider LK search. They
are accepted (`--bake-passes 1..10`) but rarely improve the result.

### Scoring and selection

Every pass gets one comparable score:

```text
score = 0.35 * mean_inlier_ratio
      + 0.25 * coverage
      + 0.20 * density
      + 0.20 * smoothness
```

- `mean_inlier_ratio` is the average `inliers / matches` over processed frames.
- `coverage` is the fraction of processed frames with at least 30 inliers.
- `density` is confirmed map points per processed frame, saturated at 250. A point
  is confirmed when it was observed at least twice.
- `smoothness` is `1 - normalized_step_jerk`; sudden step changes lower it.

`passes.csv` lists every pass and marks the selected one. The selected pass is not
necessarily the last pass: a pass that lost tracking or produced a noisy map
cannot win only because it ran later.

### Progress and logging

In a terminal the bake draws one line:

```text
Bake  [##########--------------]  48%  pass 3/5  frame 512/848  guided  ETA 00:37
```

When stdout is redirected, the same information is written every five percent as
a normal line, so `> bake.log` stays readable. Per-frame engine traces are
suppressed while the bar is active; pass summaries are logged between passes.

### Output files

| File | Contents |
| --- | --- |
| `points.csv` | Selected map: `x,y,z,r,g,b` |
| `trajectory.csv` | Selected camera centers: `x,y,z` |
| `metrics.csv` | Selected pass per-frame metrics: `frame,processed,matches,inliers,inlier_ratio,triangulated,total_points,motion_valid,step_m,yaw_deg` |
| `passes.csv` | One row per pass with score components, point count, path length and duration |
| `summary.txt` | Selected pass, score, map size, confirmed points, effective settings and timing |

### Bake limitations

- There is still no loop closure, relocalization or bundle adjustment. A bake can
  improve density and robustness on weak frames, but it cannot remove drift that
  accumulated in every pass.
- A bake is deterministic and roughly `passes` times slower than one interactive
  run. The 5-pass default on a few minutes of 30 fps footage takes several minutes.
- Each pass keeps up to `--bake-max-keyframes` keyframes. Raising the limit uses
  more memory; lowering it reduces the baselines available to the structure pass.
- The planar-motion and metric-scale assumptions are unchanged. A wrong
  `--camera-height` or a wrong horizon scales every pass in the same way.

## Reading a bake output

### Viewer (`--load`)

```powershell
.\HySlam\bin\Debug\net10.0\HySlam.exe --load C:\runs\bake_road
```

`--load` opens `points.csv` and `trajectory.csv` from the given directory in the
same free-camera viewer used by a live run. No video is decoded and no tracking
runs: the map is static, and the camera is framed on the whole point cloud and
trajectory at startup. Left-drag orbits, the wheel zooms and `Esc` closes the
window. The title shows the point count, trajectory sample count and path length.

The same command opens a `--dump` directory, because the interactive recorder
writes the same two CSV files. Without `summary.txt` the viewer still works.

A point row may be `x,y,z` or `x,y,z,r,g,b`; an optional seventh `observations`
column is read as well. Missing colors default to white. Values must use the
invariant culture (`1.25`, not `1,25`) and be finite; a malformed row aborts the
load with the file name and line number.

### Verification (`--verify`)

```powershell
.\HySlam\bin\Debug\net10.0\HySlam.exe --verify C:\runs\bake_road
```

`--verify` is a headless consistency check for a bake output. It requires all
five files, then reports one line per check:

```text
Verification of C:\runs\bake_road
  [ok]   map loaded: 35896 points, 818 trajectory samples, path 339.25 m, span 325.15 m.
  [ok]   summary.txt parsed: 34 keys.
  [ok]   points.csv count 35896 matches summary.txt (35896).
  [ok]   trajectory path 339.25 m matches summary.txt (339.25 m).
  ...
  [ok]   passes.csv selected name 'cleanup' matches summary.txt.
  [ok]   passes.csv selected points match the exported map and summary.txt.
  [ok]   passes.csv selected score matches summary.txt.
  [warn] points.csv has no 'observations' column; confirmed points were not cross-checked.
  Verification passed: 16 checks, 1 warnings.
```

The checks cover:

- all five files exist and both map CSVs parse into finite numbers;
- `points.csv` and `trajectory.csv` counts match `summary.txt`;
- the recomputed path and span match the reported values within a small rounding
  tolerance;
- the confirmed-point count matches when an `observations` column is present;
- `metrics.csv` has the expected columns and one row per processed frame;
- `passes.csv` has one row per bake pass and exactly one selected row, whose pass
  number, name, point count and score match `summary.txt`.

The exit code is `0` when every check passes and `1` when a file is missing, a CSV
is malformed or any value disagrees. A warning does not fail verification; the
current bake export has no observations column, so the confirmed-point comparison
is normally reported as a warning.

`--verify` is specific to a bake output because it expects `passes.csv`. Use
`--load` to inspect a plain `--dump` run.

## Calibration

A calibration file is a plain `key=value` text file. `fx`, `fy`, `cx`, `cy` are required;
`width`, `height` and the OpenCV distortion coefficients are optional:

```ini
# camera.txt
fx=1050
fy=1050
cx=960
cy=540
width=1920
height=1080
k1=-0.12
k2=0.08
p1=0
p2=0
k3=0
```

If `width`/`height` are present, the intrinsics are scaled automatically to the video
resolution. When distortion coefficients are non-zero, `Cv2.Undistort` is applied before
optical flow, pose estimation and triangulation, and point colors are sampled from the
undistorted frame.

Without `--calib`, the fallback matches the original intrinsics: `fx = fy = 1.1 * width`
and the principal point at the image center. `--fov` overrides the focal length with
`f = width / (2 * tan(fov / 2))`.

The camera pitch is not assumed to be zero. Unless `--pitch` is given, the road horizon
is placed at `0.665 * height` of the frame, a value measured from the lane-line vanishing
point and the optical-flow focus of expansion on dashcam footage, and the pitch is derived
as `pitch = -atan((horizon - cy) / fy)`. This matters: using a centred horizon mis-scales
every projected depth and collapses the translation estimate.

## Scene presets

Different scenes need different geometry assumptions: how far the ground plane is
usable, which part of the frame contains it, how much motion happens between frames and
how far the optical-flow search must reach. `--preset` sets all of those at once. The
preset is resolved after parsing, so option order does not matter, and any explicit
option overrides the preset value (`--preset:city --band-top 0.74` is valid).

| Preset | Use for | Aliases |
| --- | --- | --- |
| `road` | dashcam on a normal road, rigid mount (default) | - |
| `highway` | motorway at cruise speed | `motorway`, `expressway` |
| `fast` | very high speed or low frame rate | `high-speed`, `highspeed`, `speed` |
| `mountain` | winding mountain road, trees and rock, no lane markings | `mountain-road` |
| `city` | dense urban street, near buildings, stop-and-go | `urban` |
| `office` | office room, hand-held, desk height | - |
| `home` | home / narrow room, hand-held walking | `indoor`, `house`, `room` |

| Preset | `--min-depth` / `--max-depth` | `--structure-depth` | `--band-top` / `--band-bottom` | `--keyframe-frames` | `--max-step` | `--pitch` | `--lk-window` / `--lk-levels` / `--lk-error` |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `road` | `6` / `26` m | `45` m | `0.70` / `0.99` | `30` | `3.0` m | horizon | `21` / `3` / `20` |
| `highway` | `10` / `55` m | `120` m | `0.68` / `0.86` | `12` | `10.0` m | horizon | `27` / `4` / `30` |
| `fast` | `12` / `70` m | `120` m | `0.65` / `0.82` | `6` | `15.0` m | horizon | `31` / `4` / `35` |
| `mountain` | `5` / `30` m | `70` m | `0.70` / `0.99` | `25` | `4.0` m | horizon | `21` / `3` / `25` |
| `city` | `3` / `18` m | `40` m | `0.72` / `0.98` | `10` | `3.0` m | horizon | `21` / `3` / `20` |
| `office` | `1.5` / `9` m | `18` m | `0.72` / `0.96` | `8` | `0.8` m | `5 deg` down | `21` / `3` / `20` |
| `home` | `1.2` / `7` m | `12` m | `0.78` / `0.97` | `8` | `0.6` m | `5 deg` down | `21` / `3` / `20` |

### Motion prediction (always on)

A short history of recent yaw and translation steps is kept. Before trusting a
planar-motion estimate, it is compared against the constant-velocity forecast:

- **High-speed tracks:** the forecast prevents a single bad LK match from jumping
  the camera tens of meters. Estimates that deviate beyond `maxTranslationStep * 1.5`
  are rejected.
- **Low-texture segments:** when no usable fit exists, the smoothed prediction is
  applied so the trajectory does not freeze.

### Multi-keyframe structure

Structure points are triangulated from the last three keyframes instead of a
single one. Multiple baselines give a denser and more consistent point cloud on
complex city roads where a single baseline is too short or too noisy.

Why the values differ:

- **Depth window.** Pixel error grows quadratically with ground distance. A road has a
  usable plane out to tens of meters, a room only a few meters, and `city` needs a
  shorter window than open road because buildings and parked cars break the plane.
- **Detection band.** The band must contain ground, not walls or sky. `home` and
  `office` use the lower part of the frame; `fast` keeps a thin band just below the
  horizon, where the inter-frame displacement is still small enough for optical flow.
- **Keyframe interval and step gate.** A walking camera needs a short keyframe baseline
  to triangulate anything, and a tight step limit so a tilt during a pan cannot be
  fitted as a jump. High-speed footage needs the opposite: a larger step gate.
- **Lucas-Kanade range.** At high speed one frame moves features farther than a `21x21`
  window with 3 pyramid levels can search, so `highway` and `fast` widen the window and
  add a level. The per-point error gate scales with it.

Example commands for every preset are in [Usage scenarios](#usage-scenarios).

A preset is a starting point, not a calibration. Always pass `--camera-height` when it is
known, because it sets the metric scale, and calibrate `--fov` or supply `--calib` when
the path length is off by a constant factor.

### Home and office limitations

The planar model allows only yaw, forward/backward and lateral motion at a constant
height with a constant pitch. A person walking and panning introduces roll, pitch and
vertical bobbing that the model cannot represent, so a pan is partly absorbed as a
spurious translation and the trajectory can drift sideways. On a 21.6 s room clip
(`478x850`, walking from a hallway into a bedroom) the `home` preset produces about
`18 m` of path and `184` poses, but the last third of the walk pans across the room and
the trajectory drifts several meters to the side. A rigid mount, a level camera, slow
motion and avoiding panning while walking all improve the result far more than any
parameter. `office` is slightly less aggressive than `home` because office floors are
usually larger and the camera may be held at desk height.

**Focal length dominates the forward scale.** For a monocular camera on a known plane the
product `height * focal` sets every forward distance, and a room does not contain enough
information to separate the two. A portrait phone clip is a good example: the fallback
`f = 1.1 * width` is `526 px` for a `478 px` wide frame, while a 26 mm-equivalent main
camera cropped to 9:16 is closer to `637 px` (`--fov 41`). That is a `20 %` difference in
path length. Film a tape-measured straight line (3-5 m is enough) and adjust `--fov` until
`path_meters` matches, or supply a real `--calib` file.

## Reproducibility and metrics

Run a short, deterministic pass and inspect the CSV output:

```powershell
.\HySlam\bin\Debug\net10.0\HySlam.exe --source:video --path C:\clips\short.mp4 `
  --preset:road --deterministic --max-frames 300 --dump .\runs\baseline
```

The dump directory then contains:

| File | Contents |
| --- | --- |
| `metrics.csv` | Per processed frame: `frame,matches,inliers,inlier_ratio,triangulated,total_points,keyframe_id,x,y,z` |
| `points.csv` | Final map points: `x,y,z,r,g,b` |
| `trajectory.csv` | Camera centers appended to the trajectory: `x,y,z` |
| `summary.txt` | Video path, frame counts, map size, path length, span, duration and average speed |

The same files can be reopened without the video with `--load <directory>`.
`--verify` expects the bake-only `passes.csv`, so it validates a bake directory,
not a plain dump.

RANSAC is seeded with `Cv2.SetTheRNG(...)` at startup, so `--deterministic` runs of the
same clip and options produce the same frame sequence and the same counters. Wall-clock
pacing is disabled in this mode; one source frame is read per update.

## Controls

| Input | Action |
| --- | --- |
| Left mouse drag | Orbit the free camera (anywhere in the window) |
| Mouse wheel | Zoom in / out (anywhere in the window) |
| `Esc` | Quit |

The interactive viewer is a single full-window free camera. There is no separate
top-down view and no OpenCV preview window. The `--load` viewer uses the same
controls and frames the complete map when it opens.

## Pipeline

1. `CameraFeedWindow` reads frames from the video file. `SlamEngine` optionally
   undistorts the frame, converts it to grayscale for every vision step, and samples
   point colors from the same frame.
2. `WindowManager` paces playback using the video FPS and processes every third frame
   (`FrameSkip = 3`). With `--deterministic` it instead reads exactly one source frame
   per update. A bake uses the same sampling but runs headless until the video ends.
3. `KeyframeManager` detects ORB features when a keyframe is created (`ORB.Create(2000)`)
   over the drivable part of the frame and keeps a bounded history of keyframes with their
   grayscale images, 2D feature points and world poses. The interactive viewer keeps the
   last 8 keyframes; a bake can keep up to `--bake-max-keyframes` keyframes. A keyframe
   is created at the start, after `--keyframe-frames` processed frames, or when the camera
   has rotated `>= 15 deg` / moved `>= 1.5 m` since the current keyframe.
4. `SlamEngine` estimates motion from the road plane instead of a general essential
   matrix:
   - **Near-field detection:** `BuildGroundMask()` restricts corner detection to the
     bottom of the image (`0.70-0.99` of the height). Fresh features are detected every
     processed frame, so the point pool cannot drift into the far end of the band where
     the depth gate rejects everything.
   - **Tracking:** pyramidal Lucas-Kanade flow (`21x21` window, 3 levels) plus a
     forward-backward error check (`<= 1.0 px`). Corner sub-pixel refinement is
     deliberately *not* used here, because on road texture it snaps points onto the
     nearest lane marking and adds a systematic yaw bias.
   - **Planar fit:** each match is back-projected onto the road plane using the camera
     height and the horizon-derived pitch, keeping only depths of `6-26 m` that lie inside
     the visible road wedge. A deterministic 2-D rigid RANSAC fits yaw + translation,
     with a `0.25 m` inlier tolerance, at least 20 inliers and a `0.40` inlier ratio.
   - **Smoothing and prediction:** yaw and translation steps are smoothed with an
     exponential filter (`0.20` / `0.30`). If a frame is rejected, the last valid step is
     extrapolated for at most 2 frames.
   - **Metric pose:** the fitted step is composed with the current heading and the camera
     center, so `CameraPose` is metric and includes the camera pitch. Camera centers are
     appended to the trajectory once per processed frame.
5. The map combines two point sources:
   - **Road points** come directly from the ground-plane projection of inlier matches.
     They are placed exactly on the road (`y = 0`) using level road coordinates.
   - **Structure points** are triangulated between the recent keyframes and the current
     frame with the pitch-aware relative pose (the last 3 keyframes interactively, up to 5
     in the bake structure pass). Points are kept with camera depth
     `0.5-45 m`, parallax `>= 0.3 deg`, reprojection error `<= 2.0 px`, at least `0.4 m`
     above the road and within `30 m` lateral / `80 m` longitudinal of the camera.
   - Both kinds use a `0.3 m` voxel spacing to avoid duplicate clusters.
6. `MapRenderer` uploads the current map each frame and draws one full-window
   perspective free camera. It uses a pure black background. Map point colors are
   sampled from the source video; the view adds a major/minor ground grid, a vertical
   height axis with 5-unit ticks and a camera frustum.

## Repository and data policy

The repository is source-only. Source footage, calibration photos and generated
runs are intentionally excluded from version control. The `.gitignore` covers:

- build output and IDE state (`bin/`, `obj/`, `artifacts/`, `publish/`, `.vs/`,
  `.vscode/`, `.idea/`, `*.user`, `*.suo`)
- test results, coverage and logs (`TestResults/`, `*.trx`, `*.log`)
- generated dumps and tables (`*.csv`, `runs/`, `dump*/`)
- common video formats (`*.mp4`, `*.avi`, `*.mov`, `*.mkv`, `*.webm`, ...)
- common image formats (`*.jpg`, `*.jpeg`, `*.png`, `*.bmp`, `*.tif`, `*.webp`, ...)
- OS metadata (`Thumbs.db`, `desktop.ini`, `.DS_Store`)

Keep client footage, personal images and anything that identifies a location out
of the repository. If a small redistributable sample is required, add it
explicitly with `git add -f` only after checking its license and privacy.

## Notes

- **Metric scale:** the planar odometry removes the monocular scale ambiguity by
  projecting every road match onto the ground plane with a known camera height
  (`--camera-height`, default `1.3 m`). The result is metric, but its absolute accuracy
  depends on that height, the focal length and the horizon/pitch. For reference, the
  640x360, 84.9 s dashcam clip used during development yields roughly `333 m` of path
  at `14.1 km/h` with the default settings; a wheel-speed or GPS reference is still the
  only way to validate absolute speed independently.
- **Calibration:** with `--calib` the intrinsics and distortion coefficients come from the
  calibration file (scaled to the video resolution when `width`/`height` are given).
  Without it, intrinsics are estimated from the frame size (`fx = fy = 1.1 * width`,
  principal point at the image center) and distortion is assumed to be zero. The pitch
  comes from the measured road horizon unless `--pitch` is supplied.
- **Reproducibility:** `--deterministic` disables wall-clock pacing and reads one source
  frame per update. `Cv2.SetTheRNG(...)` seeds OpenCV's RANSAC at startup, and the
  custom odometry RANSAC uses its own fixed seed, so repeated runs with the same input
  produce byte-identical CSV dumps. `--bake` always enables deterministic mode.
- **Offline bake:** `--bake` runs the whole clip in 1-10 passes without an OpenGL window,
  chains the best trajectory into the next pass as a motion prior, raises the structure
  triangulation budget in the completion pass and exports only the best-scoring pass. See
  [Offline bake](#offline-bake).
- **Read-back:** `--load` reads the exported `points.csv`/`trajectory.csv` back into the
  viewer without a video, and `--verify` cross-checks a bake directory against
  `summary.txt`. See [Reading a bake output](#reading-a-bake-output).
- **Tracking:** odometry uses near-field road features only; the keyframe path is used for
  structure triangulation. There is no descriptor matching at runtime, no covisibility
  graph, relocalization, loop closure or bundle adjustment.
- **Grayscale processing:** feature detection, optical flow and pose estimation are all
  grayscale. Map point colors are sampled from the source video; the trajectory remains
  green.
- **No distance fade:** points keep their full color in the free view, even when the
  camera is zoomed far out.
- **Depth/height cues:** the free view draws a brighter major grid every 25 units, a
  dimmer 5-unit minor grid, and a vertical Y axis with 5-unit ticks. The window requests
  4x MSAA for smoother point edges.
- **Sparse map:** the sky is masked out and structure triangulations are filtered by
  depth, parallax, reprojection error and distance from the camera. The remaining points
  use a `0.3`-unit voxel spacing, so the cloud stays sparse and clean instead of
  collapsing into duplicate clusters.
- **Limits:** the map is capped at 150,000 points (trimmed to 100,000) to keep rendering
  responsive.
