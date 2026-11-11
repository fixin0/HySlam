using OpenCvSharp;

namespace HySlam.UI
{
    internal sealed class CameraFeedWindow : IDisposable
    {
        private const string WindowName = "HySlam - Camera Feed";

        private readonly VideoCapture capture;
        private readonly Mat frame = new();
        private readonly Mat grayFrame = new();
        private readonly Mat displayFrame = new();

        public Mat Frame => frame;
        public double FrameWidth => capture.FrameWidth;
        public double FrameHeight => capture.FrameHeight;
        public double Fps => capture.Fps;
        public bool IsHighGuiClosed { get; private set; }

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

        public void ShowFrame(List<Point2f> kfPts, List<Point2f> currPts, Mat? source = null)
        {
            if (IsHighGuiClosed) return;

            // Vision processing is grayscale; the preview is shown without color too.
            // When undistortion is active, use the same rectified frame as the tracker
            // so the overlay coordinates match the displayed pixels.
            Mat input = source is not null && !source.Empty() ? source : frame;

            if (input.Channels() == 3)
                Cv2.CvtColor(input, grayFrame, ColorConversionCodes.BGR2GRAY);
            else
                input.CopyTo(grayFrame);

            // Convert back to three channels so the overlay keeps its colors.
            Cv2.CvtColor(grayFrame, displayFrame, ColorConversionCodes.GRAY2BGR);
            DrawCameraOverlay(displayFrame, kfPts, currPts);
            Cv2.ImShow(WindowName, displayFrame);
        }

        public bool PumpEvents()
        {
            if (IsHighGuiClosed) return false;

            int key = Cv2.WaitKey(1);
            if (key == 27)
            {
                IsHighGuiClosed = true;
                return true;
            }

            return false;
        }

        private static void DrawCameraOverlay(Mat target, List<Point2f> kfPts, List<Point2f> currPts)
        {
            foreach (var p in kfPts) Cv2.Circle(target, new Point((int)p.X, (int)p.Y), 2, new Scalar(0, 255, 0), -1);
            foreach (var p in currPts) Cv2.Circle(target, new Point((int)p.X, (int)p.Y), 3, new Scalar(0, 0, 255), -1);
        }

        public void Dispose()
        {
            capture?.Dispose();
            frame?.Dispose();
            grayFrame?.Dispose();
            displayFrame?.Dispose();

            try { Cv2.DestroyAllWindows(); } catch { }
        }
    }
}
