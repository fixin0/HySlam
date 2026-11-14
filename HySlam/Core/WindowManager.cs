using HySlam.Core.Data;
using HySlam.Core.Diagnostics;
using HySlam.Core.Rendering;
using HySlam.Core.Slam;
using HySlam.UI;
using System.ComponentModel;
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
        private readonly string videoPath;

        private readonly MapData map = new();

        private CameraFeedWindow capture = null!;
        private KeyframeManager keyframes = null!;
        private SlamEngine slam = null!;
        private MapRenderer renderer = null!;
        private RunRecorder recorder = null!;
        private ViewerControls controls = null!;

        private int frameIndex;
        private int framesRead;
        private int framesProcessed;
        private bool videoEnded, recordingFinished;

        // Diagnostics: log every step of the first few frames so a native crash can
        // be located from the last line that reached the log sink.
        private const int TraceFrames = 3;
        private int updateFrames;
        private int renderFrames;

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
            videoPath = options.VideoPath
                ?? throw new InvalidOperationException("The interactive viewer requires a video path.");
        }

        private static GameWindowSettings CreateGameWindowSettings()
        {
            var s = GameWindowSettings.Default;
            s.UpdateFrequency = 60;
            return s;
        }

        protected override void OnLoad()
        {
            try
            {
                Log.Information("OnLoad: begin.");

                base.OnLoad();
                Log.Information("OnLoad: base.OnLoad completed.");

                GL.Enable(EnableCap.DepthTest);
                GL.Enable(EnableCap.ProgramPointSize);
                GL.Enable(EnableCap.ScissorTest);

                Log.Information(
                    "OpenGL {Version} on {Renderer} ({Vendor}).",
                    GL.GetString(StringName.Version) ?? "unknown",
                    GL.GetString(StringName.Renderer) ?? "unknown",
                    GL.GetString(StringName.Vendor) ?? "unknown");

                InitializeRenderer();
                InitializeSLAM();
                controls = new ViewerControls(renderer.Options);

                Log.Information("HySlam initialized -> {VideoPath}", videoPath);
                Log.Information("OnLoad: end.");
            }
            catch (Exception ex)
            {
                Log.Fatal(ex, "HySlam failed to initialize.");
                Close();
            }
        }

        private void InitializeRenderer()
        {
            renderer = new MapRenderer();
            renderer.Options.ColorMode = options.ColorMode;
            renderer.Options.Theme = options.Theme;
            renderer.Options.ShowGrid = options.ShowGrid;
            Log.Information("Map Renderer initialized.");
            if(renderer == null)
            {
                Log.Error("Failed to initialize Map Renderer.");
                throw new Exception("Failed to initialize Map Renderer.");
            }

        }

        private void InitializeSLAM()
        {
            Log.Information("InitializeSLAM: opening video capture.");
            capture = new CameraFeedWindow(videoPath);

            Log.Information(
                "InitializeSLAM: video {Width}x{Height} @ {Fps:0.##} fps.",
                capture.FrameWidth,
                capture.FrameHeight,
                capture.Fps);

            // Loop closure needs a longer keyframe history to compare against
            // distant places. Without it the default 8-keyframe window is enough.
            keyframes = new KeyframeManager(
                options.EnableLoopClosure ? 64 : KeyframeManager.DefaultHistoryLimit);
            slam = new SlamEngine(map, keyframes);

            double width = capture.FrameWidth;
            double height = capture.FrameHeight;

            Log.Information("InitializeSLAM: configuring the SLAM engine.");
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
            slam.LoopClosureEnabled = options.EnableLoopClosure;
            recorder = new RunRecorder(options.DumpDirectory);

            double fps = capture.Fps;
            if (fps <= 0.1 || fps > 240) fps = 30.0;
            frameRate = fps;
            frameInterval = 1.0 / fps;

            frameIndex = FrameSkip - 1;
            Log.Information("SLAM initialized.");
            Log.Information(
                "Highway defaults: depth {MinDepth:0.#}-{MaxDepth:0.#} m, band {BandTop:0.00}-{BandBottom:0.00}, " +
                "keyframe {Keyframes} frames, max step {MaxStep:0.#} m, LK window {Window}, levels {Levels}.",
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
            try
            {
                base.OnUpdateFrame(e);

                updateFrames++;
                if (updateFrames <= TraceFrames || updateFrames % 120 == 0)
                {
                    Log.Information(
                        "OnUpdateFrame #{UpdateFrame}: begin (read {FramesRead}, processed {FramesProcessed}).",
                        updateFrames,
                        framesRead,
                        framesProcessed);
                }

                if (KeyboardState.IsKeyDown(Keys.Escape))
                {
                    Close();
                    return;
                }

                // --- FREE VIEW MOUSE CONTROL (whole window) ---
                var mouse = MouseState;

                if (mouse.IsButtonDown(MouseButton.Left))
                {
                    renderer.Orbit(mouse.Delta.X, mouse.Delta.Y);
                }

                float scroll = mouse.ScrollDelta.Y;
                if (scroll != 0)
                {
                    renderer.Zoom(scroll);
                }

                controls.Update(renderer.Options);

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

                Title = $"HySlam | Inliers: {slam.InlierCount} | +3D: {slam.TriangulatedCount} | Total: {map.Points.Count}";

                if (updateFrames <= TraceFrames)
                    Log.Information("OnUpdateFrame #{UpdateFrame}: end.", updateFrames);
            }
            catch (Exception ex)
            {
                Log.Fatal(ex, "HySlam update failed.");
                Close();
            }
        }

        private void ProcessVideoFrame()
        {
            bool traceFrame = framesRead < TraceFrames * FrameSkip;

            if (traceFrame)
                Log.Information("ProcessVideoFrame: reading source frame #{FramesRead}.", framesRead + 1);

            if (!capture.ReadFrame())
            {
                EndVideo("Video finished.");
                return;
            }

            framesRead++;
            frameIndex++;

            // Show the source video with the active-lane overlay. This runs for
            // every decoded frame so the preview stays smooth even when SLAM only
            // processes every third frame.
            if (options.ShowVideo)
            {
                using var annotated = capture.Frame.Clone();
                slam.DrawLaneOverlay(annotated);
                Cv2.ImShow("HySlam Lane", annotated);
                Cv2.WaitKey(1);
            }

            if (frameIndex % FrameSkip != 0)
            {
                FinishIfFrameLimitReached();
                return;
            }

            if (traceFrame)
            {
                Log.Information(
                    "ProcessVideoFrame: decoded frame #{FramesRead} ({Width}x{Height}, {Channels} channels).",
                    framesRead,
                    capture.Frame.Width,
                    capture.Frame.Height,
                    capture.Frame.Channels());
            }

            bool processed = slam.ProcessFrame(capture.Frame, out var kfFiltered, out var currFiltered);
            framesProcessed++;

            if (traceFrame)
                Log.Information("ProcessVideoFrame: SLAM processed={Processed}.", processed);

            if (recorder.Enabled)
                recorder.Record(framesRead, slam, map, keyframes.CurrentId);

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
                videoPath,
                framesRead,
                framesProcessed,
                options.MaxFrames,
                options.Deterministic,
                frameRate);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            try
            {
                base.OnRenderFrame(e);

                renderFrames++;
                if (renderFrames <= TraceFrames || renderFrames % 120 == 0)
                {
                    Log.Information(
                        "OnRenderFrame #{RenderFrame}: begin (points {Points}, trajectory {Trajectory}).",
                        renderFrames,
                        map.Points.Count,
                        map.Trajectory.Count);
                }

                renderer.Render(Size, map, slam.Pose);

                if (renderFrames <= TraceFrames)
                    Log.Information("OnRenderFrame #{RenderFrame}: scene drawn.", renderFrames);

                SwapBuffers();

                if (renderFrames <= TraceFrames)
                    Log.Information("OnRenderFrame #{RenderFrame}: buffers swapped.", renderFrames);
            }
            catch (Exception ex)
            {
                Log.Fatal(ex, "HySlam render failed.");
                Close();
            }
        }


        protected override void OnResize(ResizeEventArgs e)
        {
            Log.Information("OnResize: {Width}x{Height}.", e.Width, e.Height);
            base.OnResize(e);
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            Log.Information(
                "OnClosing: videoEnded={VideoEnded}, frames read={FramesRead}, processed={FramesProcessed}.",
                videoEnded,
                framesRead,
                framesProcessed);

            base.OnClosing(e);
        }

        protected override void OnUnload()
        {
            Log.Information("OnUnload: begin.");
            base.OnUnload();

            FinishRecording();
            Log.Information("OnUnload: recording finished.");

            controls?.Dispose();
            capture?.Dispose();
            keyframes?.Dispose();
            slam?.Dispose();
            recorder?.Dispose();
            Log.Information("OnUnload: resources disposed.");

            try { Cv2.DestroyAllWindows(); } catch { }

            renderer?.Dispose();
            Log.Information("OnUnload: end.");
        }
    }
}
