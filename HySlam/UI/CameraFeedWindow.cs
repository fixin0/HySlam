using OpenCvSharp;

namespace HySlam.UI
{
    internal sealed class CameraFeedWindow : IDisposable
    {
        private readonly VideoCapture capture;
        private readonly Mat frame = new();

        public Mat Frame => frame;
        public double FrameWidth => capture.FrameWidth;
        public double FrameHeight => capture.FrameHeight;
        public double Fps => capture.Fps;

        /// <summary>Source frame count reported by the container, or 0 when unknown.</summary>
        public int FrameCount => capture.FrameCount;

        public CameraFeedWindow(string videoPath)
        {
            capture = new VideoCapture(videoPath);
            if (!capture.IsOpened())
                throw new Exception("Video could not be opened: " + videoPath);
        }

        public bool ReadFrame()
        {
            return capture.Read(frame) && !frame.Empty();
        }

        public void Dispose()
        {
            capture?.Dispose();
            frame?.Dispose();

            try { Cv2.DestroyAllWindows(); } catch { }
        }
    }
}
