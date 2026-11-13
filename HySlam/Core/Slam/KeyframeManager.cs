using OpenCvSharp;
using System.Collections.Generic;

namespace HySlam.Core.Slam
{
    internal sealed class Keyframe
    {
        public int Id;
        public List<Point2f> Points2D = new();
        public Mat? Gray;
        public CameraPose Pose = new();
        public Mat? Descriptors;

        public void Dispose()
        {
            Gray?.Dispose();
            Descriptors?.Dispose();
        }
    }

    internal sealed class KeyframeManager : IDisposable
    {
        public const int MinKeyframePoints = 50;
        public const int DefaultHistoryLimit = 8;

        private readonly ORB orb;
        private readonly List<Keyframe> keyframes = new();
        private readonly int historyLimit;
        private Keyframe? currentKeyframe;
        private int nextKeyframeId = 1;

        /// <summary>
        /// Creates a keyframe history. The interactive viewer keeps a small window; the
        /// offline bake uses a much longer one so structure can be triangulated from
        /// distant baselines.
        /// </summary>
        public KeyframeManager(int historyLimit = DefaultHistoryLimit)
        {
            this.historyLimit = Math.Max(2, historyLimit);
            orb = ORB.Create(2000);
        }

        public int HistoryLimit => historyLimit;

        public Keyframe? Current => currentKeyframe;

        public int CurrentId => currentKeyframe?.Id ?? -1;

        /// <summary>
        /// Returns the most recent keyframes (including the current one), newest first.
        /// </summary>
        public IReadOnlyList<Keyframe> Recent(int count)
        {
            int take = System.Math.Clamp(count, 1, keyframes.Count);
            var result = new List<Keyframe>(take);
            for (int i = keyframes.Count - 1; i >= 0 && result.Count < take; i--)
                result.Add(keyframes[i]);
            return result;
        }

        /// <summary>
        /// Returns all keyframes currently in the history. Some older keyframes may
        /// have been disposed when the history limit was reached; their descriptors
        /// are null but their pose data remains valid.
        /// </summary>
        public IReadOnlyList<Keyframe> GetAll()
        {
            return keyframes.AsReadOnly();
        }

        public bool CreateNewKeyframe(Mat gray, Mat mask, CameraPose worldPose)
        {
            // The keyframe owns the descriptor matrix, so it must not be disposed here.
            var descriptors = new Mat();
            orb.DetectAndCompute(gray, mask, out KeyPoint[] kps, descriptors);

            if (kps.Length < MinKeyframePoints)
            {
                descriptors.Dispose();
                return false;
            }

            var newKf = new Keyframe
            {
                Id = nextKeyframeId++,
                Gray = gray.Clone(),
                Pose = worldPose.Clone(),
                Descriptors = descriptors
            };

            foreach (var kp in kps) newKf.Points2D.Add(kp.Pt);
            
            // Keep only the most recent keyframes so memory stays bounded.
            if (keyframes.Count >= historyLimit)
            {
                keyframes[0].Dispose();
                keyframes.RemoveAt(0);
            }

            currentKeyframe = newKf;
            keyframes.Add(newKf);
            return true;
        }

        public void Dispose()
        {
            foreach (var kf in keyframes) kf.Dispose();
            keyframes.Clear();
            currentKeyframe = null;
            orb.Dispose();
        }
    }
}
