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
        private const int MinMatchCount = 30;
        private const double MinInlierRatio = 0.45;
        private const double MinLoopBaselineMeters = 5.0;
        private const int MinKeyframeGap = 10;

        // A real loop travels a long path but returns close to the start. A straight
        // road has arc length roughly equal to the straight-line distance, so the
        // ratio check rejects most false positives. A large heading change also
        // indicates that the camera turned around rather than drove straight on.
        private const double MinLoopPathMeters = 20.0;
        private const double MinLoopPathRatio = 1.8;
        private const double MinLoopHeadingDegrees = 30.0;

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
            if (current.Descriptors is null || current.Descriptors.IsDisposed ||
                current.Descriptors.Rows < MinMatchCount)
                return null;

            foreach (var past in history)
            {
                if (past.Descriptors is null || past.Descriptors.IsDisposed ||
                    past.Descriptors.Rows < MinMatchCount)
                    continue;

                // Skip very recent keyframes (they are part of the local window).
                if (current.Id - past.Id < MinKeyframeGap)
                    continue;

                // Skip if the two keyframes are too close in space (not a real loop).
                var d = current.Pose.GetCameraPositionGL() - past.Pose.GetCameraPositionGL();
                if (d.Length < MinLoopBaselineMeters)
                    continue;

                // Geometric gate: a loop travels a long path and/or turns around.
                if (!LooksLikeLoop(past, current))
                    continue;

                if (HasEnoughMatches(current, past))
                    return past;
            }

            // Add current keyframe to history (bounded).
            AddToHistory(current);

            return null;
        }

        private bool LooksLikeLoop(Keyframe past, Keyframe current)
        {
            int start = history.IndexOf(past);
            if (start < 0)
                return false;

            // Arc length along the keyframe history from the matched keyframe to the
            // current one.
            double arc = 0.0;
            var prev = history[start].Pose.GetCameraPositionGL();
            for (int i = start + 1; i < history.Count; i++)
            {
                var p = history[i].Pose.GetCameraPositionGL();
                arc += (p - prev).Length;
                prev = p;
            }

            var currentPos = current.Pose.GetCameraPositionGL();
            arc += (currentPos - prev).Length;

            double straight = (currentPos - history[start].Pose.GetCameraPositionGL()).Length;
            if (straight < 1e-3)
                return false;
            if (arc < MinLoopPathMeters)
                return false;

            // A true loop either travels much farther than the straight-line distance
            // or turns the camera by a significant angle.
            double ratio = arc / straight;
            double headingDiff = HeadingDifferenceDegrees(past, current);
            return ratio >= MinLoopPathRatio || headingDiff >= MinLoopHeadingDegrees;
        }

        private static double HeadingDifferenceDegrees(Keyframe a, Keyframe b)
        {
            double diff = Math.Abs(b.Pose.ExtractYawRadians() - a.Pose.ExtractYawRadians());
            while (diff > Math.PI)
                diff = Math.Abs(diff - 2.0 * Math.PI);
            return diff * 180.0 / Math.PI;
        }

        private bool HasEnoughMatches(Keyframe current, Keyframe past)
        {
            // KnnMatch returns one array per query descriptor with the two best
            // candidates, which lets us apply a proper Lowe's ratio test.
            DMatch[][] matches = matcher.KnnMatch(current.Descriptors!, past.Descriptors!, 2);

            if (matches.Length < MinMatchCount)
                return false;

            const double ratioThreshold = 0.75;
            int goodCount = 0;

            foreach (var pair in matches)
            {
                if (pair.Length < 2)
                    continue;

                if (pair[0].Distance <= ratioThreshold * pair[1].Distance)
                    goodCount++;
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
