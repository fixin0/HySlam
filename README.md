# HySlam

A modularized monocular visual SLAM demo built with OpenTK (OpenGL) and OpenCVSharp.
It reads frames from a video file, converts them to grayscale for all vision processing,
tracks near-field road features with pyramidal Lucas-Kanade optical flow, estimates a
metric 3-DOF camera motion (yaw + translation) on the road plane, triangulates above-road
structure, colors each point from the source video, and renders a dual view (top-down +
free camera) of the resulting map.

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
| Mapping | Sparse points from one active keyframe; no loop closure, relocalization or bundle adjustment |
| Output | Approximate trajectory and point cloud for inspection, not metrology or control |

## Quick start

From the repository root:

```powershell
dotnet build HySlam.slnx
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4
```

The repository does not include videos or images, so `--path` must point to a
local clip. Press `Esc` to close the preview and the OpenGL window. Show every
option with:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --help
```

For a short, repeatable run without wall-clock pacing:

```powershell
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --preset:road --max-frames 120 --deterministic
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
│   ├── WindowManager.cs            // OpenTK lifecycle (OnLoad, OnUpdate, OnRender, OnUnload)
│   ├── RunOptions.cs               // Parsed command line configuration
│   │
│   ├── Slam/
│   │   ├── SlamEngine.cs           // OpenCV pipeline (optical flow, pose, triangulation)
│   │   ├── CameraCalibration.cs    // Intrinsics/distortion loading and scaling
│   │   ├── CameraPose.cs           // Camera rotation + translation
│   │   └── KeyframeManager.cs      // Keyframe creation and management
│   │
│   ├── Diagnostics/
│   │   └── RunRecorder.cs          // metrics.csv, points.csv, trajectory.csv, summary.txt
│   │
│   ├── Data/
│   │   ├── MapPoint.cs             // A single 3D map point
│   │   ├── MapData.cs              // Map collections (points + trajectory)
│   │   └── VoxelGrid.cs            // Spatial hash that prevents duplicate points
│   │
│   └── Rendering/
│       ├── MapRenderer.cs          // All OpenGL drawing (VAO/VBO, draw calls, dual view)
│       ├── Shader.cs               // Shader compilation/linking and uniform handling
│       └── Shaders.glsl            // Vertex + fragment shader (#vertex / #fragment)
│
└── UI/
    └── CameraFeedWindow.cs         // VideoCapture + Cv2.ImShow (2D video window)
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
| `--path` | file path | required | existing, readable file | Video decoded by OpenCV. Relative paths resolve against the current working directory |
| `--calib` | file path | none | valid `key=value` file | Real intrinsics and distortion. Overrides `--fov`; see [Calibration](#calibration) |
| `--dump` | directory | none | any writable path | Writes `metrics.csv`, `points.csv`, `trajectory.csv` and `summary.txt`. The directory is created when missing |
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
| `--help`, `-h` | none | - | - | Print the built-in `usage.txt` and exit with code `0` |

Bounds are validated after the preset and the explicit options are merged, not while each
argument is read. For example `--preset:city --min-depth 25 --max-depth 26` succeeds because
the resulting window is valid, while `--preset:city --min-depth 25` fails because the preset
`--max-depth 20` is then smaller than the explicit minimum.

When the video ends (or `--max-frames` is reached), the window stays open so the map can be
inspected and the dump is finalized. The process exits with code `0` on success or after
`--help`, and with code `1` on invalid arguments, an unknown option or a missing file.

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
- Relative paths for `--path`, `--calib` and `--dump` are converted to absolute paths against
  the directory you start the process from, not the project directory.
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
| `highway` | `10` / `55` m | `120` m | `0.68` / `0.86` | `12` | `12.0` m | horizon | `27` / `4` / `30` |
| `fast` | `15` / `80` m | `160` m | `0.67` / `0.80` | `8` | `25.0` m | horizon | `31` / `4` / `40` |
| `mountain` | `5` / `30` m | `70` m | `0.70` / `0.99` | `25` | `4.0` m | horizon | `21` / `3` / `25` |
| `city` | `4` / `20` m | `45` m | `0.76` / `0.97` | `12` | `2.0` m | horizon | `21` / `3` / `20` |
| `office` | `1.5` / `9` m | `18` m | `0.72` / `0.96` | `8` | `0.8` m | `5 deg` down | `21` / `3` / `20` |
| `home` | `1.2` / `7` m | `12` m | `0.78` / `0.97` | `8` | `0.6` m | `5 deg` down | `21` / `3` / `20` |

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

RANSAC is seeded with `Cv2.SetTheRNG(...)` at startup, so `--deterministic` runs of the
same clip and options produce the same frame sequence and the same counters. Wall-clock
pacing is disabled in this mode; one source frame is read per update.

## Controls

| Input | Action |
| --- | --- |
| Left mouse drag (right half) | Orbit the free camera |
| Mouse wheel (right half) | Zoom in / out |
| `Esc` | Quit |

In the OpenCV preview window, `Esc` also quits.

## Pipeline

1. `CameraFeedWindow` reads frames from the video file. `SlamEngine` optionally
   undistorts the frame, converts it to grayscale for every vision step, and samples
   point colors from the same frame. The preview is drawn from the frame used by the
   tracker so overlays line up.
2. `WindowManager` paces playback using the video FPS and processes every third frame
   (`FrameSkip = 3`). With `--deterministic` it instead reads exactly one source frame
   per update.
3. `KeyframeManager` detects ORB features when a keyframe is created (`ORB.Create(2000)`)
   over the drivable part of the frame and keeps a single current keyframe with its
   grayscale image, 2D feature points and world pose. A new keyframe replaces the previous
   one. A keyframe is created at the start, after 30 processed frames, or when the camera
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
   - **Structure points** are triangulated between the current keyframe and the current
     frame with the pitch-aware relative pose. Points are kept with camera depth
     `0.5-45 m`, parallax `>= 0.3 deg`, reprojection error `<= 2.0 px`, at least `0.4 m`
     above the road and within `30 m` lateral / `80 m` longitudinal of the camera.
   - Both kinds use a `0.3 m` voxel spacing to avoid duplicate clusters.
6. `MapRenderer` uploads the current map each frame and draws two viewports:
   a top-down orthographic view and a perspective free view. Both use a pure black
   background. Map point colors are sampled from the source video; the free view adds a
   major/minor ground grid and a vertical height axis with 5-unit ticks.

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
  produce byte-identical CSV dumps.
- **Tracking:** odometry uses near-field road features only; the keyframe path is used for
  structure triangulation. There is no descriptor matching at runtime, no covisibility
  graph, relocalization, loop closure or bundle adjustment.
- **Grayscale processing:** the video preview, feature detection, optical flow and pose
  estimation are all grayscale. Map point colors are sampled from the source video;
  the trajectory remains green.
- **No distance fade:** points keep their full color in the free view, even when the
  camera is zoomed far out.
- **Depth/height cues:** the free view draws a brighter major grid every 25 units, a
  dimmer 5-unit minor grid, and a vertical Y axis with 5-unit ticks. The window requests
  4x MSAA for smoother point edges.
- **Sparse map:** the sky is masked out and structure triangulations are filtered by
  depth, parallax, reprojection error and distance from the camera. The remaining points
  use a `0.3`-unit voxel spacing, so the cloud stays sparse and clean instead of
  collapsing into duplicate clusters.
- **Overlay colors:** green circles are the features tracked from the previous frame, red
  circles are the features in the current frame.
- **Limits:** the map is capped at 150,000 points (trimmed to 100,000) to keep rendering
  responsive.
