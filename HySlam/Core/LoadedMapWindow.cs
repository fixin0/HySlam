using HySlam.Core.Data;
using HySlam.Core.Rendering;
using HySlam.UI;
using Raylib_cs;
using Serilog;

namespace HySlam.Core
{
    /// <summary>
    /// Read-only viewer for a map directory written by --bake or --dump.
    ///
    /// No video is opened and no tracking runs; the point cloud and trajectory are
    /// loaded once and rendered with the same free-camera controls as the
    /// interactive window. The camera is framed on the whole map at startup.
    /// </summary>
    internal sealed class LoadedMapWindow : IDisposable
    {
        private const int WindowWidth = 1280;
        private const int WindowHeight = 720;
        private const int TargetFps = 60;

        private readonly string directory;
        private readonly MapData map;
        private readonly RenderColorMode colorMode;
        private readonly RenderTheme theme;
        private readonly bool showGrid;

        private MapRenderer renderer = null!;
        private ViewerControls controls = null!;
        private int renderFrames;
        private bool closeRequested;
        private bool initialized;
        private bool disposed;

        public LoadedMapWindow(string directory, MapData map, RenderColorMode colorMode, RenderTheme theme, bool showGrid)
        {
            this.directory = directory;
            this.map = map;
            this.colorMode = colorMode;
            this.theme = theme;
            this.showGrid = showGrid;
        }

        /// <summary>
        /// Creates the raylib window and runs the read-only viewer until it is closed.
        /// </summary>
        public void Run()
        {
            Raylib.SetConfigFlags(ConfigFlags.Msaa4xHint | ConfigFlags.ResizableWindow);
            Raylib.SetTraceLogLevel(TraceLogLevel.Warning);
            Raylib.InitWindow(WindowWidth, WindowHeight, "HySlam Map | " + DescribeDirectory(directory));
            Raylib.SetTargetFPS(TargetFps);

            try
            {
                OnLoad();

                while (!closeRequested && !Raylib.WindowShouldClose())
                {
                    if (Raylib.IsKeyDown(KeyboardKey.Escape))
                    {
                        closeRequested = true;
                        break;
                    }

                    OnUpdateFrame();

                    if (closeRequested)
                        break;

                    OnRenderFrame();
                }
            }
            finally
            {
                OnUnload();

                if (Raylib.IsWindowReady())
                    Raylib.CloseWindow();
            }
        }

        private static string DescribeDirectory(string directory)
        {
            string trimmed = Path.TrimEndingDirectorySeparator(directory);
            string name = Path.GetFileName(trimmed);
            return string.IsNullOrEmpty(name) ? trimmed : name;
        }

        private void OnLoad()
        {
            try
            {
                renderer = new MapRenderer();
                renderer.Options.ColorMode = colorMode;
                renderer.Options.Theme = theme;
                renderer.Options.ShowGrid = showGrid;
                renderer.FrameMap(map);
                controls = new ViewerControls(renderer.Options);

                double pathMeters = 0.0;
                for (int i = 1; i < map.Trajectory.Count; i++)
                    pathMeters += (map.Trajectory[i] - map.Trajectory[i - 1]).Length();

                Raylib.SetWindowTitle(
                    $"HySlam Map | {map.Points.Count} points | " +
                    $"{map.Trajectory.Count} samples | {pathMeters:0.0} m");

                Log.Information(
                    "Loaded map from {Directory}: {Points} points, {Trajectory} trajectory samples, {Path:0.0} m path.",
                    directory,
                    map.Points.Count,
                    map.Trajectory.Count,
                    pathMeters);

                initialized = true;
            }
            catch (Exception exception)
            {
                Log.Fatal(exception, "HySlam failed to open the map viewer.");
                closeRequested = true;
            }
        }

        private void OnUpdateFrame()
        {
            try
            {
                if (Raylib.IsMouseButtonDown(MouseButton.Left))
                {
                    var delta = Raylib.GetMouseDelta();
                    renderer.Orbit(delta.X, delta.Y);
                }

                float scroll = Raylib.GetMouseWheelMove();
                if (scroll != 0f)
                    renderer.Zoom(scroll);

                controls.Update(renderer.Options);
            }
            catch (Exception exception)
            {
                Log.Fatal(exception, "HySlam map viewer update failed.");
                closeRequested = true;
            }
        }

        private void OnRenderFrame()
        {
            try
            {
                renderFrames++;

                Raylib.BeginDrawing();
                renderer.Render(map, pose: null);
                Raylib.EndDrawing();
            }
            catch (Exception exception)
            {
                Log.Fatal(exception, "HySlam map viewer render failed.");
                closeRequested = true;
            }
        }

        private void OnUnload()
        {
            if (initialized)
            {
                controls?.Dispose();
                renderer?.Dispose();
                Log.Information("Map viewer closed after {Frames} frames.", renderFrames);
            }
            else
            {
                controls?.Dispose();
                renderer?.Dispose();
            }
        }

        public void Dispose()
        {
            if (disposed)
                return;

            disposed = true;
            if (Raylib.IsWindowReady())
                Raylib.CloseWindow();
        }
    }
}
