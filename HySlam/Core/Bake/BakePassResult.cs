using HySlam.Core.Data;
using HySlam.Core.Slam;

namespace HySlam.Core.Bake
{
    /// <summary>
    /// One tracking sample collected during a bake pass. The values mirror the
    /// per-frame metrics written by the interactive recorder, plus the motion step
    /// used for prior chaining and smoothness scoring.
    /// </summary>
    internal readonly record struct FrameMetric(
        int Frame,
        int Processed,
        int Matches,
        int Inliers,
        int Triangulated,
        int TotalPoints,
        bool MotionValid,
        double StepMeters,
        double YawDegrees)
    {
        public double InlierRatio => Matches > 0 ? Inliers / (double)Matches : 0.0;
    }

    /// <summary>
    /// Engine parameters that may differ between bake passes. Scene geometry and
    /// intrinsics come from <see cref="RunOptions"/> and stay fixed inside one bake.
    /// </summary>
    internal readonly record struct BakePassSettings(
        int LkWindow,
        int LkLevels,
        double LkError,
        double MaxStep,
        int KeyframeFrames,
        int StructureKeyframes,
        int StructurePoints)
    {
        public const int DefaultStructureKeyframes = 3;
        public const int DefaultStructurePoints = 220;

        public static BakePassSettings From(RunOptions options) => new(
            options.LkWindowSize,
            options.LkLevels,
            options.LkError,
            options.MaxTranslationStep,
            options.KeyframeFrames,
            DefaultStructureKeyframes,
            DefaultStructurePoints);
    }

    /// <summary>
    /// Complete output of one bake pass: the map it produced, the motion steps used
    /// as the prior for the next pass, the per-frame metrics and its score.
    /// </summary>
    internal sealed class BakePassResult
    {
        public required int Index { get; init; }
        public required string Name { get; init; }
        public required MapData Map { get; init; }
        public required List<PosePriorStep> Steps { get; init; }
        public required List<FrameMetric> Frames { get; init; }
        public required BakePassSettings Settings { get; init; }
        public required int FramesRead { get; init; }
        public required int FramesProcessed { get; init; }
        public required double Fps { get; init; }
        public required TimeSpan Elapsed { get; init; }

        /// <summary>True when low-observation points were removed by the cleanup pass.</summary>
        public bool Pruned { get; set; }

        public BakeScore Score { get; set; }

        public int ConfirmedPoints
        {
            get
            {
                int confirmed = 0;
                foreach (var point in Map.Points)
                    if (point.Observations >= 2)
                        confirmed++;

                return confirmed;
            }
        }

        public double PathMeters
        {
            get
            {
                double path = 0.0;
                var trajectory = Map.Trajectory;
                for (int i = 1; i < trajectory.Count; i++)
                    path += (trajectory[i] - trajectory[i - 1]).Length;

                return path;
            }
        }
    }
}
