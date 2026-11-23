using HySlam.Core.Data;
using HySlam.Core.Diagnostics;
using HySlam.Core.Rendering;
using HySlam.Core.Slam;
using HySlam.UI;
using OpenCvSharp;
using Raylib_cs;
using Serilog;

namespace HySlam.Core
{
    /// <summary>
    /// Interactive viewer: reads the video, runs the SLAM pipeline and draws the
    /// live map with raylib. The class owns a native raylib window, so
    /// <see cref="Run"/> must be called from the main thread.
    /// </summary>
    internal sealed class WindowManager : IDisposable
    {
        private const int WindowWidth = 1280;
        private const int WindowHeight = 720;
        private const int TargetFps = 60;

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
        private bool closeRequested;
        private bool disposed;

        // Diagnostics: log every step of the first few frames so a native crash can
        // be located from the last line that reached the log sink.
        private const int TraceFrames = 3;
        private int updateFrames;
        private int renderFrames;

        private bool laneWindowPositioned;
        private const int FrameSkip = 3;

        private double frameInterval = 1.0 / 30.0;
        private double frameRate = 30.0;
        private double accumulatedTime;

        private int lastWindowWidth;
        private int lastWindowHeight;

        public WindowManager(RunOptions options)
        {
            this.options = options;
            videoPath = options.VideoPath
                ?? throw new InvalidOperationException("The interactive viewer requires a video path.");
        }

        /// <summary>
        /// Creates the raylib window, runs the render loop until the user closes it
        /// and releases every resource on the way out.
        /// </summary>
        public void Run()
        {
            Raylib.SetConfigFlags(ConfigFlags.Msaa4xHint | ConfigFlags.ResizableWindow);
            Raylib.SetTraceLogLevel(TraceLogLevel.Warning);
            Raylib.InitWindow(WindowWidth, WindowHeight, "HySlam Window + " + videoPath);
            Raylib.SetTargetFPS(TargetFps);

            lastWindowWidth = Raylib.GetScreenWidth();
            lastWindowHeight = Raylib.GetScreenHeight();

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

                    OnUpdateFrame(Raylib.GetFrameTime());

                    if (closeRequested)
                        break;

                    OnRenderFrame();
                }

                OnClosing();
            }
            finally
            {
                OnUnload();

                if (Raylib.IsWindowReady())
                    Raylib.CloseWindow();
            }
        }

        private void OnLoad()
        {
            try
            {
                Log.Information("OnLoad: begin.");

                Log.Information(
                    "raylib {RaylibVersion} on OpenGL profile {GlVersion}.",
                    Raylib.RAYLIB_VERSION,
                    Rlgl.GetVersion());

                InitializeRenderer();
                InitializeSLAM();
                controls = new ViewerControls(renderer.Options);

                Log.Information("HySlam initialized -> {VideoPath}", videoPath);
                Log.Information("OnLoad: end.");
            }
            catch (Exception ex)
            {
                Log.Fatal(ex, "HySlam failed to initialize.");
                closeRequested = true;
            }
        }

        private void InitializeRenderer()
        {
            renderer = new MapRenderer();
            renderer.Options.ColorMode = options.ColorMode;
            renderer.Options.Theme = options.Theme;
            renderer.Options.ShowGrid = options.ShowGrid;
            Log.Information("Map Renderer initialized.");
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
        }

        private void OnUpdateFrame(float deltaTime)
        {
            try
            {
                updateFrames++;
                if (updateFrames <= TraceFrames || updateFrames % 120 == 0)
                {
                    Log.Information(
                        "OnUpdateFrame #{UpdateFrame}: begin (read {FramesRead}, processed {FramesProcessed}).",
                        updateFrames,
                        framesRead,
                        framesProcessed);
                }

                LogWindowSizeChanges();

                // --- FREE VIEW MOUSE CONTROL (whole window) ---
                if (Raylib.IsMouseButtonDown(MouseButton.Left))
                {
                    var delta = Raylib.GetMouseDelta();
                    renderer.Orbit(delta.X, delta.Y);
                }

                float scroll = Raylib.GetMouseWheelMove();
                if (scroll != 0f)
                    renderer.Zoom(scroll);

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
                        accumulatedTime += deltaTime;
                        const double maxAccum = 0.5;
                        if (accumulatedTime > maxAccum) accumulatedTime = maxAccum;

                        while (accumulatedTime >= frameInterval && !videoEnded)
                        {
                            ProcessVideoFrame();
                            accumulatedTime -= frameInterval;
                        }
                    }
                }

                Raylib.SetWindowTitle(
                    $"HySlam | Inliers: {slam.InlierCount} | +3D: {slam.TriangulatedCount} | Total: {map.Points.Count}");

                if (updateFrames <= TraceFrames)
                    Log.Information("OnUpdateFrame #{UpdateFrame}: end.", updateFrames);
            }
            catch (Exception ex)
            {
                Log.Fatal(ex, "HySlam update failed.");
                closeRequested = true;
            }
        }

        private void LogWindowSizeChanges()
        {
            int width = Raylib.GetScreenWidth();
            int height = Raylib.GetScreenHeight();
            if (width == lastWindowWidth && height == lastWindowHeight)
                return;

            lastWindowWidth = width;
            lastWindowHeight = height;
            Log.Information("OnResize: {Width}x{Height}.", width, height);
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
                if (!laneWindowPositioned)
                {
                    Cv2.MoveWindow("HySlam Lane", 20, 40);
                    Cv2.SetWindowProperty("HySlam Lane", WindowPropertyFlags.Topmost, 1);
                    laneWindowPositioned = true;
                }
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

        private void OnRenderFrame()
        {
            try
            {
                renderFrames++;
                if (renderFrames <= TraceFrames || renderFrames % 120 == 0)
                {
                    Log.Information(
                        "OnRenderFrame #{RenderFrame}: begin (points {Points}, trajectory {Trajectory}).",
                        renderFrames,
                        map.Points.Count,
                        map.Trajectory.Count);
                }

                Raylib.BeginDrawing();
                renderer.Render(map, slam.Pose);
                Raylib.EndDrawing();

                if (renderFrames <= TraceFrames)
                    Log.Information("OnRenderFrame #{RenderFrame}: scene drawn.", renderFrames);
            }
            catch (Exception ex)
            {
                Log.Fatal(ex, "HySlam render failed.");
                closeRequested = true;
            }
        }

        private void OnClosing()
        {
            Log.Information(
                "OnClosing: videoEnded={VideoEnded}, frames read={FramesRead}, processed={FramesProcessed}.",
                videoEnded,
                framesRead,
                framesProcessed);
        }

        private void OnUnload()
        {
            Log.Information("OnUnload: begin.");

            FinishRecording();
            Log.Information("OnUnload: recording finished.");

            DisposeResources();
            Log.Information("OnUnload: resources disposed.");
            Log.Information("OnUnload: end.");
        }

        private void DisposeResources()
        {
            controls?.Dispose();
            capture?.Dispose();
            keyframes?.Dispose();
            slam?.Dispose();
            recorder?.Dispose();

            try { Cv2.DestroyAllWindows(); } catch { }

            renderer?.Dispose();
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
