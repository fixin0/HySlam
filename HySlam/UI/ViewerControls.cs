using HySlam.Core.Rendering;
using OpenCvSharp;
using Serilog;

namespace HySlam.UI
{
    /// <summary>
    /// OpenCV trackbar panel for the 3D viewer. Provides live controls for theme,
    /// visibility, colour mode, grid, lane lines, trajectory and point size.
    /// The panel is intentionally dependency-free: OpenCV highgui is already part
    /// of the application, so no extra UI toolkit is required.
    /// </summary>
    internal sealed class ViewerControls : IDisposable
    {
        private const string WindowName = "HySlam Controls";
        private const string ThemeTrack = "Theme";
        private const string ShowTrack = "Show";
        private const string ColorTrack = "Color";
        private const string GridTrack = "Grid";
        private const string LaneTrack = "Lanes";
        private const string PathTrack = "Path";
        private const string SizeTrack = "Point size";

        private readonly Mat panel = new Mat(250, 430, MatType.CV_8UC3, Scalar.All(30));
        private bool disposed;
        private bool available = true;

        public ViewerControls(RenderOptions options)
        {
            Cv2.NamedWindow(WindowName, WindowFlags.AutoSize);
            Cv2.MoveWindow(WindowName, 860, 40);

            Log.Information(
                "Viewer controls: theme {Theme}, colour {Color}, grid {Grid}, point size {Size}.",
                options.Theme,
                options.ColorMode,
                options.ShowGrid,
                options.PointSize);

            int theme = options.Theme == RenderTheme.Light ? 1 : 0;
            int show = options.Visibility switch
            {
                VisibilityFilter.RoadsOnly => 1,
                VisibilityFilter.ObjectsOnly => 2,
                VisibilityFilter.BuildingsOnly => 3,
                _ => 0,
            };
            int color = options.ColorMode switch
            {
                RenderColorMode.Sampled => 1,
                RenderColorMode.Semantic => 2,
                RenderColorMode.Confidence => 3,
                _ => 0,
            };
            int grid = options.ShowGrid ? 1 : 0;
            int lanes = options.ShowLaneLines ? 1 : 0;
            int path = options.ShowTrajectory ? 1 : 0;
            int size = Math.Clamp((int)options.PointSize, 1, 12);

            Cv2.CreateTrackbar(ThemeTrack, WindowName, ref theme, 1, OnTrackbar);
            Cv2.CreateTrackbar(ShowTrack, WindowName, ref show, 3, OnTrackbar);
            Cv2.CreateTrackbar(ColorTrack, WindowName, ref color, 3, OnTrackbar);
            Cv2.CreateTrackbar(GridTrack, WindowName, ref grid, 1, OnTrackbar);
            Cv2.CreateTrackbar(LaneTrack, WindowName, ref lanes, 1, OnTrackbar);
            Cv2.CreateTrackbar(PathTrack, WindowName, ref path, 1, OnTrackbar);
            Cv2.CreateTrackbar(SizeTrack, WindowName, ref size, 12, OnTrackbar);

            Cv2.ImShow(WindowName, panel);
            Cv2.WaitKey(1);

            // Set positions after the window is realised: some highgui backends
            // ignore trackbar positions set before the first show.
            Cv2.SetTrackbarPos(ThemeTrack, WindowName, options.Theme == RenderTheme.Light ? 1 : 0);
            Cv2.SetTrackbarPos(ShowTrack, WindowName, show);
            Cv2.SetTrackbarPos(ColorTrack, WindowName, color);
            Cv2.SetTrackbarPos(GridTrack, WindowName, options.ShowGrid ? 1 : 0);
            Cv2.SetTrackbarPos(LaneTrack, WindowName, options.ShowLaneLines ? 1 : 0);
            Cv2.SetTrackbarPos(PathTrack, WindowName, options.ShowTrajectory ? 1 : 0);
            Cv2.SetTrackbarPos(SizeTrack, WindowName, Math.Clamp((int)options.PointSize, 1, 12));
            Cv2.WaitKey(1);

            RefreshPanel();
            Cv2.ImShow(WindowName, panel);
            Cv2.SetWindowProperty(WindowName, WindowPropertyFlags.Topmost, 1);
            Cv2.WaitKey(1);
        }

        private static void OnTrackbar(int pos, IntPtr userdata)
        {
            // Values are polled in Update; the callback only keeps highgui happy.
        }

        /// <summary>
        /// Reads the trackbar values and applies them to the renderer options.
        /// </summary>
        public void Update(RenderOptions options)
        {
            if (disposed || !available)
                return;

            try
            {
                int theme = Cv2.GetTrackbarPos(ThemeTrack, WindowName);
                int show = Cv2.GetTrackbarPos(ShowTrack, WindowName);
                int color = Cv2.GetTrackbarPos(ColorTrack, WindowName);
                int grid = Cv2.GetTrackbarPos(GridTrack, WindowName);
                int lanes = Cv2.GetTrackbarPos(LaneTrack, WindowName);
                int path = Cv2.GetTrackbarPos(PathTrack, WindowName);
                int size = Cv2.GetTrackbarPos(SizeTrack, WindowName);

                options.Theme = theme == 1 ? RenderTheme.Light : RenderTheme.Dark;
                options.Visibility = show switch
                {
                    1 => VisibilityFilter.RoadsOnly,
                    2 => VisibilityFilter.ObjectsOnly,
                    3 => VisibilityFilter.BuildingsOnly,
                    _ => VisibilityFilter.All,
                };
                options.ColorMode = color switch
                {
                    1 => RenderColorMode.Sampled,
                    2 => RenderColorMode.Semantic,
                    3 => RenderColorMode.Confidence,
                    _ => RenderColorMode.Height,
                };
                options.ShowGrid = grid != 0;
                options.ShowLaneLines = lanes != 0;
                options.ShowTrajectory = path != 0;
                options.PointSize = Math.Max(1, size);

                RefreshPanel();
                Cv2.ImShow(WindowName, panel);
                Cv2.WaitKey(1);
            }
            catch (OpenCVException)
            {
                // The user closed the controls window; keep the viewer running
                // with the last settings instead of terminating the render loop.
                available = false;
            }
        }

        private void RefreshPanel()
        {
            int theme = Cv2.GetTrackbarPos(ThemeTrack, WindowName);
            int show = Cv2.GetTrackbarPos(ShowTrack, WindowName);
            int color = Cv2.GetTrackbarPos(ColorTrack, WindowName);
            int grid = Cv2.GetTrackbarPos(GridTrack, WindowName);
            int lanes = Cv2.GetTrackbarPos(LaneTrack, WindowName);
            int path = Cv2.GetTrackbarPos(PathTrack, WindowName);
            int size = Cv2.GetTrackbarPos(SizeTrack, WindowName);

            bool light = theme == 1;
            var background = light ? new Scalar(238, 238, 238) : new Scalar(28, 28, 28);
            var text = light ? new Scalar(20, 20, 20) : new Scalar(235, 235, 235);
            var accent = light ? new Scalar(20, 90, 160) : new Scalar(80, 200, 255);
            panel.SetTo(background);

            string showName = show switch
            {
                1 => "Roads only",
                2 => "Objects only",
                3 => "Buildings only",
                _ => "All",
            };
            string colorName = color switch
            {
                1 => "Sampled",
                2 => "Semantic",
                3 => "Confidence",
                _ => "Height",
            };

            int y = 34;
            Cv2.PutText(panel, "HySlam controls", new Point(14, y),
                HersheyFonts.HersheySimplex, 0.7, accent, 2);
            y += 34;
            Cv2.PutText(panel, $"Theme: {(light ? "Light" : "Dark")}", new Point(14, y),
                HersheyFonts.HersheySimplex, 0.5, text, 1);
            y += 26;
            Cv2.PutText(panel, $"Show: {showName}", new Point(14, y),
                HersheyFonts.HersheySimplex, 0.5, text, 1);
            y += 26;
            Cv2.PutText(panel, $"Color: {colorName}", new Point(14, y),
                HersheyFonts.HersheySimplex, 0.5, text, 1);
            y += 26;
            Cv2.PutText(panel,
                $"Grid: {(grid != 0 ? "on" : "off")}   Lanes: {(lanes != 0 ? "on" : "off")}   Path: {(path != 0 ? "on" : "off")}",
                new Point(14, y), HersheyFonts.HersheySimplex, 0.5, text, 1);
            y += 26;
            Cv2.PutText(panel, $"Point size: {Math.Max(1, size)}", new Point(14, y),
                HersheyFonts.HersheySimplex, 0.5, text, 1);
            y += 36;
            Cv2.PutText(panel, "Show: 0 All  1 Roads  2 Objects  3 Buildings",
                new Point(14, y), HersheyFonts.HersheySimplex, 0.4, text, 1);
            y += 22;
            Cv2.PutText(panel, "Color: 0 Height  1 Sampled  2 Semantic  3 Confidence",
                new Point(14, y), HersheyFonts.HersheySimplex, 0.4, text, 1);
        }

        public void Dispose()
        {
            if (disposed)
                return;

            disposed = true;
            try { Cv2.DestroyWindow(WindowName); } catch { }
            panel.Dispose();
        }
    }
}
