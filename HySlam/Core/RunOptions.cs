using HySlam.Core.Slam;

namespace HySlam.Core
{
    /// <summary>
    /// Immutable run configuration produced by the command line parser and consumed
    /// by the window manager.
    /// </summary>
    internal sealed class RunOptions
    {
        public required string VideoPath { get; init; }

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
        /// odometry. The road preset uses 6 m; an indoor room needs roughly 1.5 m.
        /// </summary>
        public double MinGroundDepth { get; init; } = 6.0;

        /// <summary>
        /// Far limit (meters) of the ground-plane depth window. Beyond this the metric
        /// error of a pixel grows quadratically, so the fit is restricted to the near
        /// field. The road preset uses 26 m; an indoor room needs roughly 7 m.
        /// </summary>
        public double MaxGroundDepth { get; init; } = 26.0;

        /// <summary>
        /// Maximum camera depth (meters) for triangulated above-ground structure.
        /// The road preset uses 45 m; an indoor room needs roughly 12 m.
        /// </summary>
        public double MaxStructureDepth { get; init; } = 45.0;

        /// <summary>
        /// Top of the image band used for ground-plane odometry, as a fraction of the
        /// frame height. The band must contain the ground surface only.
        /// </summary>
        public double GroundBandTop { get; init; } = 0.70;

        /// <summary>
        /// Bottom of the ground-plane odometry band, as a fraction of the frame height.
        /// </summary>
        public double GroundBandBottom { get; init; } = 0.99;

        /// <summary>
        /// Maximum number of processed frames a keyframe stays active. Structure
        /// triangulation needs a small baseline, so an indoor scene uses a much
        /// shorter interval than the road preset.
        /// </summary>
        public int KeyframeFrames { get; init; } = 30;

        /// <summary>
        /// Maximum camera translation (meters) accepted for one processed frame. A
        /// hand-held indoor clip needs a much tighter limit than a road clip, because
        /// a tilt during a pan can otherwise be fitted as a spontaneous jump.
        /// </summary>
        public double MaxTranslationStep { get; init; } = 3.0;

        /// <summary>Name of the scene preset that produced the effective values.</summary>
        public string ScenePreset { get; init; } = "road";

        /// <summary>Lucas-Kanade window size in pixels (odd).</summary>
        public int LkWindowSize { get; init; } = 21;

        /// <summary>Lucas-Kanade pyramid levels.</summary>
        public int LkLevels { get; init; } = 3;

        /// <summary>Maximum LK per-point error accepted as a valid match.</summary>
        public double LkError { get; init; } = 20.0;
    }
}
