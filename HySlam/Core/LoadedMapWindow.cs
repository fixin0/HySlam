using HySlam.Core.Data;
using HySlam.Core.Rendering;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
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
    internal sealed class LoadedMapWindow : GameWindow
    {
        private readonly string directory;
        private readonly MapData map;

        private MapRenderer renderer = null!;
        private int renderFrames;

        public LoadedMapWindow(string directory, MapData map)
            : base(
                CreateGameWindowSettings(),
                CreateNativeWindowSettings(directory))
        {
            this.directory = directory;
            this.map = map;
        }

        private static GameWindowSettings CreateGameWindowSettings()
        {
            var settings = GameWindowSettings.Default;
            settings.UpdateFrequency = 60;
            return settings;
        }

        private static NativeWindowSettings CreateNativeWindowSettings(string directory)
        {
            return new NativeWindowSettings
            {
                ClientSize = new Vector2i(1280, 720),
                Title = "HySlam Map | " + DescribeDirectory(directory),
                APIVersion = new Version(3, 3),
                Profile = ContextProfile.Core,
                NumberOfSamples = 4
            };
        }

        private static string DescribeDirectory(string directory)
        {
            string trimmed = Path.TrimEndingDirectorySeparator(directory);
            string name = Path.GetFileName(trimmed);
            return string.IsNullOrEmpty(name) ? trimmed : name;
        }

        protected override void OnLoad()
        {
            try
            {
                base.OnLoad();

                GL.Enable(EnableCap.DepthTest);
                GL.Enable(EnableCap.ProgramPointSize);
                GL.Enable(EnableCap.ScissorTest);

                Log.Information(
                    "OpenGL {Version} on {Renderer} ({Vendor}).",
                    GL.GetString(StringName.Version) ?? "unknown",
                    GL.GetString(StringName.Renderer) ?? "unknown",
                    GL.GetString(StringName.Vendor) ?? "unknown");

                renderer = new MapRenderer();
                renderer.FrameMap(map);

                double pathMeters = 0.0;
                for (int i = 1; i < map.Trajectory.Count; i++)
                    pathMeters += (map.Trajectory[i] - map.Trajectory[i - 1]).Length;

                Title = $"HySlam Map | {map.Points.Count} points | " +
                        $"{map.Trajectory.Count} samples | {pathMeters:0.0} m";

                Log.Information(
                    "Loaded map from {Directory}: {Points} points, {Trajectory} trajectory samples, {Path:0.0} m path.",
                    directory,
                    map.Points.Count,
                    map.Trajectory.Count,
                    pathMeters);
            }
            catch (Exception exception)
            {
                Log.Fatal(exception, "HySlam failed to open the map viewer.");
                Close();
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            try
            {
                base.OnUpdateFrame(e);

                if (KeyboardState.IsKeyDown(Keys.Escape))
                {
                    Close();
                    return;
                }

                var mouse = MouseState;

                if (mouse.IsButtonDown(MouseButton.Left))
                    renderer.Orbit(mouse.Delta.X, mouse.Delta.Y);

                float scroll = mouse.ScrollDelta.Y;
                if (scroll != 0)
                    renderer.Zoom(scroll);
            }
            catch (Exception exception)
            {
                Log.Fatal(exception, "HySlam map viewer update failed.");
                Close();
            }
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            try
            {
                base.OnRenderFrame(e);

                renderFrames++;
                renderer.Render(Size, map, pose: null);
                SwapBuffers();
            }
            catch (Exception exception)
            {
                Log.Fatal(exception, "HySlam map viewer render failed.");
                Close();
            }
        }

        protected override void OnResize(ResizeEventArgs e)
        {
            Log.Information("OnResize: {Width}x{Height}.", e.Width, e.Height);
            base.OnResize(e);
        }

        protected override void OnUnload()
        {
            base.OnUnload();

            renderer?.Dispose();
            Log.Information("Map viewer closed after {Frames} frames.", renderFrames);
        }
    }
}
