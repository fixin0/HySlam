using System.Collections.Generic;
using OpenCvSharp;

namespace HySlam.Core.Slam
{
    /// <summary>
    /// Minimal loop closure detector.
    ///
    /// Compares the current keyframe's ORB descriptors against a history of past
    /// keyframes using Hamming distance matching. When enough good matches are
    /// found with an older keyframe, a loop closure is reported. The relative
    /// transform between the two keyframes is computed by the caller from their
    /// stored poses.
    /// </summary>
    internal sealed class LoopClosureDetector
    {
        private const int MaxHistoryKeyframes = 64;
        private const int MinMatchCount = 20;
        private const double MinInlierRatio = 0.30;
        private const double MinLoopBaselineMeters = 3.0;

        private readonly DescriptorMatcher matcher;
        private readonly List<Keyframe> history = new();

        public LoopClosureDetector()
        {
            matcher = BFMatcher.Create("BruteForce-Hamming");
        }

        /// <summary>
        /// Checks whether <paramref name="current"/> forms a loop closure with any
        /// keyframe in the history. Returns the matched keyframe when a loop is found.
        /// </summary>
        public Keyframe? Detect(Keyframe current)
        {
            if (current.Descriptors is null || current.Descriptors.Rows < MinMatchCount)
                return null;

            foreach (var past in history)
            {
                if (past.Descriptors is null || past.Descriptors.Rows < MinMatchCount)
                    continue;

                // Skip very recent keyframes (they are part of the local window).
                if (current.Id - past.Id < 5)
                    continue;

                // Skip if the two keyframes are too close in space (not a real loop).
                var d = current.Pose.GetCameraPositionGL() - past.Pose.GetCameraPositionGL();
                if (d.Length < MinLoopBaselineMeters)
                    continue;

                if (HasEnoughMatches(current, past))
                    return past;
            }

            // Add current keyframe to history (bounded).
            AddToHistory(current);

            return null;
        }

        private bool HasEnoughMatches(Keyframe current, Keyframe past)
        {
            using var matches = new Mat();
            matcher.Match(current.Descriptors!, past.Descriptors!, matches);

            DMatch[] matchArray;
            matches.GetArray<DMatch>(out matchArray);

            if (matchArray.Length < MinMatchCount)
                return false;

            // Sort by distance and keep the best matches.
            System.Array.Sort(matchArray, (a, b) => a.Distance.CompareTo(b.Distance));
            int keep = System.Math.Min(matchArray.Length, 100);
            int goodCount = 0;

            // Lowe's ratio test: a match is good if its distance is much smaller
            // than the second-best match's distance.
            const double ratioThreshold = 0.8;
            for (int i = 0; i < keep; i++)
            {
                if (i + 1 < matchArray.Length &&
                    matchArray[i].Distance <= ratioThreshold * matchArray[i + 1].Distance)
                {
                    goodCount++;
                }
            }

            return goodCount >= MinMatchCount * MinInlierRatio;
        }

        private void AddToHistory(Keyframe kf)
        {
            history.Add(kf);
            if (history.Count > MaxHistoryKeyframes)
                history.RemoveAt(0);
        }

        public void Dispose()
        {
            matcher?.Dispose();
        }
    }
}
