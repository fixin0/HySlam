# HySlam

A modularized monocular visual SLAM demo built with raylib (`Raylib-cs`) and OpenCVSharp.
It reads frames from a video, tracks near-field road features with pyramidal Lucas–Kanade
optical flow, estimates metric 3-DOF camera motion (yaw + translation) on the road plane,
triangulates above-road structure, and renders the map in a free-camera raylib window.
A headless `--bake` mode processes the whole clip in several passes, `--load` reopens an
export, and `--verify` checks it.

> **Experimental project.** HySlam is a research and teaching prototype, not a production
> SLAM system. Trajectories and point clouds can drift, and absolute scale can be wrong
> even when the visualization looks plausible. Do not use it for navigation,
> safety-critical decisions, surveying or autonomous control.

## Requirements

- .NET 10 SDK
- 64-bit Windows (the native raylib and OpenCV runtimes ship with the NuGet packages)
- A GPU/driver with OpenGL 3.3 Core support

| Package | Purpose |
| --- | --- |
| `Raylib-cs` | Window, input, 3D rendering |
| `OpenCvSharp5` | Image processing, ORB, optical flow, geometry |
| `OpenCvSharp5.runtime.win` | Native OpenCV DLLs |

## Quick start

```powershell
dotnet build HySlam.slnx

# Interactive viewer
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4

# Deterministic short run with a CSV dump
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --max-frames 120 --deterministic --dump C:\runs\baseline

# Headless multi-pass bake
dotnet run --project HySlam\HySlam.csproj -- --source:video --path C:\clips\road.mp4 `
  --bake --bake-passes 5 --bake-output C:\runs\bake_road

# Inspect / validate an export
dotnet run --project HySlam\HySlam.csproj -- --load C:\runs\bake_road
dotnet run --project HySlam\HySlam.csproj -- --verify C:\runs\bake_road
```

The repository does not ship footage, so `--path` must point to a local clip. Press `Esc`
to close the viewer. Run with `--help` for every option (the same text is in
`HySlam/usage.txt`).

## Modes

| Mode | Description |
| --- | --- |
| Interactive | Live SLAM plus the raylib viewer, with a source-video window and lane overlay |
| `--bake` | Headless 1–10 pass run that exports the best-scoring pass |
| `--load <dir>` | Read-only viewer for `points.csv` and `trajectory.csv` |
| `--verify <dir>` | Headless cross-check of a bake output (exit code `0` / `1`) |

## Common options

| Option | Default | Effect |
| --- | --- | --- |
| `--path <file>` | required | Input video |
| `--camera-height <m>` | `1.3` | Camera height above the road; sets the metric scale |
| `--calib <file>` | none | Intrinsics and distortion (`key=value` file) |
| `--fov <deg>` | `fx = fy = 1.1 * width` | Horizontal FOV for the fallback intrinsics |
| `--pitch <deg>` | horizon-derived | Camera tilt; positive looks down |
| `--min-depth` / `--max-depth` | `10` / `55` m | Ground-plane depth window |
| `--structure-depth <m>` | `120` | Maximum depth for triangulated structure |
| `--band-top` / `--band-bottom` | `0.68` / `0.86` | Road detection band as a frame-height fraction |
| `--keyframe-frames <n>` | `12` | Processed frames before a keyframe is replaced |
| `--max-step <m>` | `10` | Maximum translation accepted per processed frame |
| `--lk-window` / `--lk-levels` / `--lk-error` | `27` / `4` / `30` | Lucas–Kanade window, levels, error limit |
| `--color-mode` | `height` | `height`, `sampled`, `semantic`, `confidence` |
| `--theme` | `dark` | `dark` or `light` |
| `--grid` | off | Show the ground grid and height axis |
| `--no-video` | off | Hide the source-video window |
| `--loop-closure` | off | ORB loop closure and drift correction (opt-in) |
| `--dump <dir>` | none | Write metrics, points, trajectory and summary CSVs |
| `--max-frames <n>` | unlimited | Stop after `N` decoded source frames |
| `--deterministic` | off | One source frame per update; reproducible runs |

All values accept `--opt value`, `--opt:value` and `--opt=value`. Numbers use the
invariant culture (`1.25`, not `1,25`). Invalid values exit with code `1`.

## Project structure

```
HySlam/
├── Program.cs                     // Entry point and CLI parsing
├── usage.txt                      // --help text
├── Core/
│   ├── WindowManager.cs           // raylib interactive loop
│   ├── LoadedMapWindow.cs         // Read-only map viewer
│   ├── Bake/                      // Headless multi-pass orchestration and scoring
│   ├── Slam/                      // Optical flow, pose, triangulation, calibration
│   ├── Diagnostics/               // CSV recording, map reader, verifier
│   ├── Data/                      // Map points, map data, voxel grid
│   └── Rendering/                 // MapRenderer (raylib)
└── UI/                            // Video capture and OpenCV controls panel
```

## How it works

1. `CameraFeedWindow` reads frames from the video; all vision steps run on grayscale.
2. `WindowManager` processes every third frame (one per update with `--deterministic`).
3. `SlamEngine` detects near-field road features, tracks them with pyramidal Lucas–Kanade
   and a forward–backward check, back-projects matches onto the road plane and fits a
   planar rigid transform with RANSAC.
4. The fitted step is smoothed and composed into a metric `CameraPose`. Road points and
   multi-keyframe triangulated structure points are added with a `0.3 m` voxel spacing.
5. Every `StructureUpdateInterval` frames a local bundle adjustment refines recent poses
   and points. `--loop-closure` adds optional ORB place recognition.
6. `MapRenderer` draws the map with one free camera; each point is a camera-facing quad.

## Calibration

`--calib` takes a plain `key=value` file. `fx`, `fy`, `cx`, `cy` are required; `width`,
`height` and the OpenCV distortion coefficients are optional:

```ini
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

With `width`/`height` the intrinsics are scaled to the video resolution. Non-zero
distortion coefficients enable `Cv2.Undistort` before tracking. Without `--calib` the
fallback is `fx = fy = 1.1 * width` with the principal point at the image center, and
`--fov` overrides the focal length. Unless `--pitch` is given, the pitch is derived from a
measured road horizon at `0.665 * height`.

## Output

| File | Contents |
| --- | --- |
| `points.csv` | `x,y,z,r,g,b`, plus optional normals, confidence and semantic label |
| `trajectory.csv` | Camera centers `x,y,z` |
| `metrics.csv` | Per-frame tracking counters and pose |
| `summary.txt` | Counts, path length, span, duration, average speed, effective settings |
| `passes.csv` | Bake only: one row per pass, selected pass marked |

A bake always runs deterministically, writes only to `--bake-output`, and `--verify`
expects all five files. Use `--load` for a plain `--dump` directory.

## Controls

| Input | Action |
| --- | --- |
| Left mouse drag | Orbit the free camera |
| Mouse wheel | Zoom |
| `Esc` | Quit |

A **HySlam Controls** panel provides live trackbars for theme, visibility, colour mode,
grid, lane lines, trajectory and point size. The `--load` viewer uses the same controls
and frames the whole map at startup.

## Notes

- **Metric scale:** a wrong `--camera-height`, focal length or horizon scales every
  distance; pass a measured height and, when possible, `--calib`.
- **Reproducibility:** `--deterministic` seeds OpenCV's RNG and disables wall-clock
  pacing, so repeated runs produce identical CSV dumps. `--bake` implies it.
- **Map size:** the map is capped at 150,000 points (trimmed to 100,000).
- **Tuning:** change one option at a time and compare `summary.txt` / `metrics.csv`
  against a deterministic baseline.

## Contributing

Contributions are welcome. Please follow these rules:

1. **Discuss first.** Open an issue for anything larger than a small fix so the approach
   can be agreed before code is written.
2. **One change per pull request.** Keep diffs focused; unrelated formatting or renames
   make review harder.
3. **Branch from `main`.** Use a descriptive topic branch such as
   `fix/pose-drift` or `feature/semantic-filter`, and open the PR against `main`.
4. **Write clear commit messages.** Use the imperative mood and explain *why*:
   `Fix yaw sign when composing the planar step`.
5. **Describe the PR.** Say what changed, why, how it was tested, and which clip and
   options were used. Include before/after numbers from `summary.txt` when tuning.
6. **Match the existing style.** C# with nullable reference types, `internal sealed`
   classes, XML comments on non-obvious logic, and no new dependencies without a reason.
7. **Build and smoke-test locally.** `dotnet build HySlam.slnx` must pass without new
   warnings, and a deterministic run (`--max-frames 120 --deterministic --dump ...`)
   should complete and write its files.
8. **Do not commit media or generated data.** Videos, images, calibration photos,
   `runs/` and dumps are git-ignored; keep client footage and location-identifying
   material out of the repository.
9. **Document user-visible changes** in `README.md` and `HySlam/usage.txt`.
10. **Be respectful.** Assume good intent and keep reviews technical and constructive.
