using OpenCvSharp;

namespace HySlam.Core.Slam
{
    internal sealed class Keyframe
    {
        public int Id;
        public List<Point2f> Points2D = new();
        public Mat? Gray;
        public CameraPose Pose = new();
    }

    internal sealed class KeyframeManager : IDisposable
    {
        public const int MinKeyframePoints = 50;

        private readonly ORB orb;
        private Keyframe? currentKeyframe;
        private int nextKeyframeId = 1;

        public KeyframeManager()
        {
            orb = ORB.Create(2000);
        }

        public Keyframe? Current => currentKeyframe;

        public int CurrentId => currentKeyframe?.Id ?? -1;

        public bool CreateNewKeyframe(Mat gray, Mat mask, CameraPose worldPose)
        {
            using var descriptors = new Mat();
            orb.DetectAndCompute(gray, mask, out KeyPoint[] kps, descriptors);

            if (kps.Length < MinKeyframePoints)
                return false;

            var newKf = new Keyframe
            {
                Id = nextKeyframeId++,
                Gray = gray.Clone(),
                Pose = worldPose.Clone()
            };

            foreach (var kp in kps) newKf.Points2D.Add(kp.Pt);

            currentKeyframe?.Gray?.Dispose();
            currentKeyframe = newKf;
            return true;
        }

        public void Dispose()
        {
            currentKeyframe?.Gray?.Dispose();
            orb.Dispose();
        }
    }
}
