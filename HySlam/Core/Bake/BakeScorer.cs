namespace HySlam.Core.Bake
{
    /// <summary>
    /// Score of one bake pass. Every component is normalized to 0..1 so the total
    /// can be compared between passes and reported in passes.csv.
    /// </summary>
    internal readonly record struct BakeScore(
        double Total,
        double InlierRatio,
        double Coverage,
        double Density,
        double Smoothness)
    {
        public static readonly BakeScore Empty = new(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>
    /// Turns a pass into a single comparable number. The weights favor tracking
    /// quality first, then how much of the clip was actually observed, then map
    /// density, and finally trajectory smoothness.
    /// </summary>
    internal static class BakeScorer
    {
        private const double DensitySaturationPointsPerFrame = 250.0;
        private const int CoverageMinInliers = 30;

        public static BakeScore Score(BakePassResult pass)
        {
            var frames = pass.Frames;
            if (frames.Count == 0)
                return BakeScore.Empty;

            double inlierRatio = 0.0;
            int coveredFrames = 0;

            foreach (var frame in frames)
            {
                inlierRatio += frame.InlierRatio;
                if (frame.Inliers >= CoverageMinInliers)
                    coveredFrames++;
            }

            inlierRatio /= frames.Count;
            double coverage = coveredFrames / (double)frames.Count;

            // Count only points that were observed at least twice. A cleanup pass
            // must not be punished for removing single-frame noise, and a cloud that
            // only exists for one frame is not useful structure.
            double density = Math.Clamp(
                pass.ConfirmedPoints / (double)frames.Count / DensitySaturationPointsPerFrame,
                0.0,
                1.0);

            double smoothness = Smoothness(frames);

            double total =
                0.35 * inlierRatio +
                0.25 * coverage +
                0.20 * density +
                0.20 * smoothness;

            return new BakeScore(total, inlierRatio, coverage, density, smoothness);
        }

        /// <summary>
        /// Compares consecutive motion steps. A trajectory whose step length changes
        /// abruptly scores lower, which marks passes that survived only by jumping.
        /// </summary>
        private static double Smoothness(IReadOnlyList<FrameMetric> frames)
        {
            if (frames.Count < 3)
                return 1.0;

            double mean = 0.0;
            for (int i = 0; i < frames.Count; i++)
                mean += frames[i].StepMeters;
            mean /= frames.Count;

            if (mean <= 1e-6)
                return 1.0;

            double jerk = 0.0;
            for (int i = 2; i < frames.Count; i++)
                jerk += Math.Abs(
                    frames[i].StepMeters -
                    2.0 * frames[i - 1].StepMeters +
                    frames[i - 2].StepMeters);

            jerk /= frames.Count - 2;
            double roughness = Math.Clamp(jerk / (mean * 2.0), 0.0, 1.0);
            return 1.0 - roughness;
        }
    }
}
