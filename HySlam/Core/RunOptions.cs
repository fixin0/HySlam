using HySlam.Core.Bake;
using HySlam.Core.Rendering;
using HySlam.Core.Slam;

namespace HySlam.Core
{
    /// <summary>
    /// Immutable run configuration produced by the command line parser and consumed
    /// by the window manager, the bake runner and the read-back diagnostics.
    /// </summary>
    internal sealed class RunOptions
    {
        /// <summary>
        /// Video decoded by the interactive viewer or the bake. Null in read-back
        /// mode (--load / --verify), where only exported files are used.
        /// </summary>
        public string? VideoPath { get; init; }

        /// <summary>
        /// Map directory opened by the read-only viewer. The directory must contain
        /// points.csv and trajectory.csv.
        /// </summary>
        public string? LoadDirectory { get; init; }

        /// <summary>
        /// Bake output directory validated headlessly. All five output files must
        /// be present.
        /// </summary>
        public string? VerifyDirectory { get; init; }

        /// <summary>Optional real camera calibration. Null means use the FOV fallback.</summary>
        public CameraCalibration? Calibration { get; init; }

        /// <summary>Optional directory for metrics and map dumps.</summary>
        public string? DumpDirectory { get; init; }

        /// <summary>Stop after reading this many source frames. Zero means unlimited.</summary>
        public int MaxFrames { get; init; }

        /// <summary>Read one frame per update so a run is not affected by wall-clock timing.</summary>
        public bool Deterministic { get; init; }

        /// <summary>
        /// Height of the camera above the road surface in meters. This fixes the metric
        /// scale of the planar visual odometry and of the road-surface point cloud.
        /// </summary>
        public double CameraHeight { get; init; } = 1.3;

        /// <summary>
        /// Optional downward tilt of the camera in degrees. Positive values mean the
        /// optical axis points slightly below the horizon. When null the tilt is
        /// derived from the calibrated ground horizon (see SlamEngine).
        /// </summary>
        public double? PitchDegrees { get; init; }

        /// <summary>
        /// Optional horizontal field of view in degrees. When zero the built-in
        /// fallback intrinsics are used. Ignored when a calibration file is supplied.
        /// </summary>
        public double HorizontalFovDegrees { get; init; }

        /// <summary>
        /// Near limit (meters) of the ground-plane depth window used by the planar
        /// odometry. Defaults are tuned for motorway footage.
        /// </summary>
        public double MinGroundDepth { get; init; } = 10.0;

        /// <summary>
        /// Far limit (meters) of the ground-plane depth window. Beyond this the metric
        /// error of a pixel grows quadratically, so the fit is restricted to the near
        /// field. Defaults are tuned for motorway footage.
        /// </summary>
        public double MaxGroundDepth { get; init; } = 55.0;

        /// <summary>
        /// Maximum camera depth (meters) for triangulated above-ground structure.
        /// Defaults are tuned for motorway footage.
        /// </summary>
        public double MaxStructureDepth { get; init; } = 120.0;

        /// <summary>
        /// Top of the image band used for ground-plane odometry, as a fraction of the
        /// frame height. The band must contain the ground surface only.
        /// </summary>
        public double GroundBandTop { get; init; } = 0.68;

        /// <summary>
        /// Bottom of the ground-plane odometry band, as a fraction of the frame height.
        /// </summary>
        public double GroundBandBottom { get; init; } = 0.86;

        /// <summary>
        /// Maximum number of processed frames a keyframe stays active. The motorway
        /// default keeps a short interval so structure has enough baseline.
        /// </summary>
        public int KeyframeFrames { get; init; } = 12;

        /// <summary>
        /// Maximum camera translation (meters) accepted for one processed frame.
        /// Defaults are tuned for motorway footage.
        /// </summary>
        public double MaxTranslationStep { get; init; } = 10.0;

        /// <summary>Lucas-Kanade window size in pixels (odd).</summary>
        public int LkWindowSize { get; init; } = 27;

        /// <summary>Lucas-Kanade pyramid levels.</summary>
        public int LkLevels { get; init; } = 4;

        /// <summary>Maximum LK per-point error accepted as a valid match.</summary>
        public double LkError { get; init; } = 30.0;

        /// <summary>
        /// Enables loop closure detection and drift correction. Disabled by default:
        /// a forward-driving clip with repeated road texture can produce false
        /// positives. Enable it when the route revisits previously seen places.
        /// </summary>
        public bool EnableLoopClosure { get; init; }

        /// <summary>
        /// Point colour mode for the viewer. Defaults to the LiDAR-style height
        /// palette; use Sampled to keep the colours from the source video.
        /// </summary>
        public RenderColorMode ColorMode { get; init; } = RenderColorMode.Height;

        /// <summary>Viewer background and grid theme.</summary>
        public RenderTheme Theme { get; init; } = RenderTheme.Dark;

        /// <summary>Draw the ground grid and height axis in the viewer.</summary>
        public bool ShowGrid { get; init; }

        /// <summary>
        /// Show the source video in a separate window with the active lane overlay.
        /// Enabled by default for interactive runs; use --no-video to disable it.
        /// </summary>
        public bool ShowVideo { get; init; } = true;

        /// <summary>
        /// Offline bake configuration. When set the program runs headless, processes
        /// the clip several times and writes the best result to the bake directory
        /// instead of opening the interactive window.
        /// </summary>
        public BakeOptions? Bake { get; init; }
    }
}
