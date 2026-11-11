using HySlam.Core.Data;
using HySlam.Core.Diagnostics;
using HySlam.Core.Rendering;
using HySlam.Core.Slam;
using HySlam.UI;
using OpenCvSharp;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using Serilog;

namespace HySlam.Core
{
    internal sealed class WindowManager : GameWindow
    {
        private readonly RunOptions options;

        private readonly MapData map = new();

        private CameraFeedWindow capture = null!;
        private KeyframeManager keyframes = null!;
        private SlamEngine slam = null!;
        private MapRenderer renderer = null!;
        private RunRecorder recorder = null!;

        private int frameIndex;
        private int framesRead;
        private int framesProcessed;
        private bool videoEnded, highGuiClosed, recordingFinished;

        // === Quality and Visualization Settings ===
        private const int FrameSkip = 3;

        private double frameInterval = 1.0 / 30.0;
        private double frameRate = 30.0;
        private double accumulatedTime;

        public WindowManager(RunOptions options)
            : base(
                CreateGameWindowSettings(),
                new NativeWindowSettings
                {
                    ClientSize = new Vector2i(1280, 720),
                    Title = "HySlam Window + " + options.VideoPath,
                    APIVersion = new Version(3, 3),
                    Profile = ContextProfile.Core,
                    NumberOfSamples = 4
                })
        {
            this.options = options;
        }

        private static GameWindowSettings CreateGameWindowSettings()
        {
            var s = GameWindowSettings.Default;
            s.UpdateFrequency = 60;
            return s;
        }

        protected override void OnLoad()
        {
            base.OnLoad();

            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.ProgramPointSize);
            GL.Enable(EnableCap.ScissorTest);

            InitializeRenderer();
            InitializeSLAM();

           
            Log.Information("HySlam initialized -> {VideoPath}", options.VideoPath);
        }

        private void InitializeRenderer()
        {
            renderer = new MapRenderer();
            Log.Information("Map Renderer initialized.");
            if(renderer == null)
            {
                Log.Error("Failed to initialize Map Renderer.");
                throw new Exception("Failed to initialize Map Renderer.");
            }

        }

        private void InitializeSLAM()
        {
            capture = new CameraFeedWindow(options.VideoPath);

            keyframes = new KeyframeManager();
            slam = new SlamEngine(map, keyframes);

            double width = capture.FrameWidth;
            double height = capture.FrameHeight;

            slam.Initialize(
                width,
                height,
                options.Calibration,
                options.CameraHeight,
                options.PitchDegrees,
                options.HorizontalFovDegrees,
                options.MinGroundDepth,
                options.MaxGroundDepth,
                options.MaxStructureDepth,
                options.GroundBandTop,
                options.GroundBandBottom,
                options.KeyframeFrames,
                options.MaxTranslationStep,
                options.LkWindowSize,
                options.LkLevels,
                options.LkError);
            recorder = new RunRecorder(options.DumpDirectory);

            double fps = capture.Fps;
            if (fps <= 0.1 || fps > 240) fps = 30.0;
            frameRate = fps;
            frameInterval = 1.0 / fps;

            frameIndex = FrameSkip - 1;
            Log.Information("SLAM initialized.");
            Log.Information(
                "Preset '{Preset}': depth {MinDepth:0.#}-{MaxDepth:0.#} m, band {BandTop:0.00}-{BandBottom:0.00}, " +
                "keyframe {Keyframes} frames, max step {MaxStep:0.#} m, LK window {Window}, levels {Levels}.",
                options.ScenePreset,
                options.MinGroundDepth,
                options.MaxGroundDepth,
                options.GroundBandTop,
                options.GroundBandBottom,
                options.KeyframeFrames,
                options.MaxTranslationStep,
                options.LkWindowSize,
                options.LkLevels);
            if(slam == null)
            {
                Log.Error("Failed to initialize SLAM Engine.");
                throw new Exception("Failed to initialize SLAM Engine.");
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);

            if (KeyboardState.IsKeyDown(Keys.Escape))
            {
                Close();
                return;
            }

            // --- FREE VIEW MOUSE CONTROL (Right Side of the Screen) ---
            var mouse = MouseState;
            bool inFreeView = mouse.X > Size.X / 2;

            if (inFreeView)
            {
                if (mouse.IsButtonDown(MouseButton.Left))
                {
                    renderer.Orbit(mouse.Delta.X, mouse.Delta.Y);
                }

                float scroll = mouse.ScrollDelta.Y;
                if (scroll != 0)
                {
                    renderer.Zoom(scroll);
                }
            }

            if (!videoEnded)
            {
                if (options.Deterministic)
                {
                    // One source frame per update: no wall-clock pacing, so two runs
                    // with the same input and options process the same frame sequence.
                    ProcessVideoFrame();
                }
                else
                {
                    accumulatedTime += e.Time;
                    const double maxAccum = 0.5;
                    if (accumulatedTime > maxAccum) accumulatedTime = maxAccum;

                    while (accumulatedTime >= frameInterval && !videoEnded)
                    {
                        ProcessVideoFrame();
                        accumulatedTime -= frameInterval;
                    }
                }
            }

            if (!highGuiClosed)
            {
                if (capture.PumpEvents())
                {
                    highGuiClosed = true;
                    Close();
                    return;
                }
            }

            Title = $"HySlam | Inliers: {slam.InlierCount} | +3D: {slam.TriangulatedCount} | Total: {map.Points.Count}";
        }

        private void ProcessVideoFrame()
        {
            if (!capture.ReadFrame())
            {
                EndVideo("Video finished.");
                return;
            }

            framesRead++;
            frameIndex++;

            if (frameIndex % FrameSkip != 0)
            {
                FinishIfFrameLimitReached();
                return;
            }

            bool processed = slam.ProcessFrame(capture.Frame, out var kfFiltered, out var currFiltered);
            framesProcessed++;

            if (recorder.Enabled)
                recorder.Record(framesRead, slam, map, keyframes.CurrentId);

            if (processed && !highGuiClosed)
            {
                capture.ShowFrame(kfFiltered, currFiltered, slam.UndistortedFrame);
            }

            FinishIfFrameLimitReached();
        }

        private void FinishIfFrameLimitReached()
        {
            if (options.MaxFrames > 0 && framesRead >= options.MaxFrames)
                EndVideo($"Reached --max-frames ({options.MaxFrames}).");
        }

        private void EndVideo(string message)
        {
            if (videoEnded)
                return;

            videoEnded = true;
            Log.Information("{Message}", message);
            FinishRecording();
        }

        private void FinishRecording()
        {
            if (recordingFinished)
                return;

            recordingFinished = true;
            recorder?.Finish(
                map.Points,
                map.Trajectory,
                options.VideoPath,
                framesRead,
                framesProcessed,
                options.MaxFrames,
                options.Deterministic,
                frameRate);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);

            renderer.Render(Size, map, slam.Pose);

            SwapBuffers();

            
        }

        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);
        }

        protected override void OnUnload()
        {
            base.OnUnload();

            FinishRecording();

            capture?.Dispose();
            keyframes?.Dispose();
            slam?.Dispose();
            recorder?.Dispose();

            try { Cv2.DestroyAllWindows(); } catch { }

            renderer?.Dispose();
        }
    }
}
