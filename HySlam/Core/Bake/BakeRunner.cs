using System.Diagnostics;
using System.Globalization;
using System.Text;
using HySlam.Core.Data;
using HySlam.Core.Diagnostics;
using HySlam.Core.Slam;
using HySlam.UI;
using Serilog;

namespace HySlam.Core.Bake
{
    /// <summary>
    /// Offline multi-pass bake.
    ///
    /// The runner processes the whole clip several times without opening an OpenGL
    /// window. Each pass can use the previous pass's trajectory as a motion prior,
    /// widen tracking for weak segments, or triangulate structure with a longer
    /// keyframe history. Every pass is scored, and the best result is exported.
    /// </summary>
    internal sealed class BakeRunner
    {
        // Same sampling as the interactive viewer so presets stay valid.
        private const int FrameSkip = 3;

        private const double MinConfirmedFraction = 0.30;

        private readonly RunOptions options;
        private readonly BakeOptions bake;
        private readonly string videoPath;
        private readonly List<BakePassResult> passes = new();

        public BakeRunner(RunOptions options)
        {
            this.options = options;
            bake = options.Bake ?? throw new InvalidOperationException("Bake options are missing.");
            videoPath = options.VideoPath ?? throw new InvalidOperationException("Bake requires a video path.");
        }

        public int Run()
        {
            Directory.CreateDirectory(bake.OutputDirectory);

            Log.Information(
                "Bake: {Passes} passes over {VideoPath}, output {OutputDirectory}, max keyframes {MaxKeyframes}.",
                bake.Passes,
                videoPath,
                bake.OutputDirectory,
                bake.MaxKeyframes);

            long framesPerPass = EstimateProcessedFrames();
            long totalWork = Math.Max(1, framesPerPass * bake.Passes + 1);
            var bar = new ProgressBar(totalWork);

            BakePassSettings settings = BakePassSettings.From(options);
            IReadOnlyList<PosePriorStep>? prior = null;

            for (int index = 0; index < bake.Passes; index++)
            {
                BakePassResult? best = BestPass();
                string name = PassName(index);

                if (index == 0)
                {
                    settings = BakePassSettings.From(options);
                    prior = null;
                }
                else if (index == 1)
                {
                    settings = Adapt(BakePassSettings.From(options), best!);
                    prior = best!.Steps;
                }
                else if (index == 2)
                {
                    settings = Guide(best!.Settings);
                    prior = best.Steps;
                }
                else if (index == 3)
                {
                    settings = Complete(best!.Settings);
                    prior = best.Steps;
                }
                else
                {
                    // The cleanup repeats the best pass. Apply the structure settings
                    // once when the best pass predates completion; do not halve the
                    // keyframe interval again on every cleanup pass.
                    settings = best!.Index < 3 ? Complete(best.Settings) : best.Settings;
                    if (index > 4)
                        settings = Explore(settings, index - 4);
                    prior = best.Steps;
                }

                BakePassResult result = RunPass(
                    index,
                    name,
                    settings,
                    prior,
                    bar,
                    framesPerPass,
                    framesPerPass * index);

                // Only the cleanup pass removes weakly observed points. The map is
                // kept intact when too few points were confirmed, otherwise the
                // cleanup would delete most of a road-only cloud.
                if (index >= 4)
                    Prune(result);

                result.Score = BakeScorer.Score(result);
                passes.Add(result);

                LogControl.Normal();
                bar.Report((index + 1) * framesPerPass, $"pass {index + 1}/{bake.Passes} done");
                bar.BeforeLog();
                Log.Information(
                    "Pass {Pass}/{Passes} ({Name}) done: score {Score:0.000} (inliers {Inlier:0.00}, coverage {Coverage:0.00}, density {Density:0.00}, smoothness {Smoothness:0.00}), points {Points}, path {Path:0.0} m, {Seconds:0.0} s.",
                    index + 1,
                    bake.Passes,
                    name,
                    result.Score.Total,
                    result.Score.InlierRatio,
                    result.Score.Coverage,
                    result.Score.Density,
                    result.Score.Smoothness,
                    result.Map.Points.Count,
                    result.PathMeters,
                    result.Elapsed.TotalSeconds);
            }

            BakePassResult selected = BestPass()!;
            bar.Report(totalWork, "writing outputs");
            Export(selected);
            bar.Finish();

            if (selected.FramesProcessed == 0)
            {
                Log.Error("Bake finished without processing any frame; the video may be too short or unreadable.");
                return 1;
            }

            Log.Information(
                "Bake complete: selected pass {Pass}/{Passes} ({Name}) with score {Score:0.000}, {Points} points and {Path:0.0} m of trajectory -> {OutputDirectory}.",
                selected.Index + 1,
                bake.Passes,
                selected.Name,
                selected.Score.Total,
                selected.Map.Points.Count,
                selected.PathMeters,
                bake.OutputDirectory);

            return 0;
        }

        private BakePassResult RunPass(
            int index,
            string name,
            BakePassSettings settings,
            IReadOnlyList<PosePriorStep>? prior,
            ProgressBar bar,
            long passUnits,
            long completedBase)
        {
            LogControl.Normal();
            bar.BeforeLog();
            Log.Information(
                "Pass {Pass}/{Passes} ({Name}): LK {Window}/{Levels}/{Error:0.#}, step {Step:0.#} m, keyframe {KeyframeFrames}, structure keyframes {StructureKeyframes}, structure points {StructurePoints}, prior steps {PriorSteps}.",
                index + 1,
                bake.Passes,
                name,
                settings.LkWindow,
                settings.LkLevels,
                settings.LkError,
                settings.MaxStep,
                settings.KeyframeFrames,
                settings.StructureKeyframes,
                settings.StructurePoints,
                prior?.Count ?? 0);

            var map = new MapData();
            var steps = new List<PosePriorStep>();
            var frames = new List<FrameMetric>();
            var clock = Stopwatch.StartNew();

            int framesRead = 0;
            int processed = 0;
            double fps = 30.0;

            using (var capture = new CameraFeedWindow(videoPath))
            {
                fps = capture.Fps;
                if (fps <= 0.1 || fps > 240.0)
                    fps = 30.0;

                int historyLimit = Math.Max(bake.MaxKeyframes, Math.Max(8, settings.StructureKeyframes));
                using var keyframes = new KeyframeManager(historyLimit);
                using var slam = new SlamEngine(map, keyframes);

                slam.Initialize(
                    capture.FrameWidth,
                    capture.FrameHeight,
                    options.Calibration,
                    options.CameraHeight,
                    options.PitchDegrees,
                    options.HorizontalFovDegrees,
                    options.MinGroundDepth,
                    options.MaxGroundDepth,
                    options.MaxStructureDepth,
                    options.GroundBandTop,
                    options.GroundBandBottom,
                    settings.KeyframeFrames,
                    settings.MaxStep,
                    settings.LkWindow,
                    settings.LkLevels,
                    settings.LkError,
                    settings.StructureKeyframes,
                    settings.StructurePoints);

                slam.ApplyPosePrior(prior);

                int frameIndex = FrameSkip - 1;

                // Engine trace lines would collide with the progress line.
                LogControl.Quiet();

                while (capture.ReadFrame())
                {
                    framesRead++;
                    frameIndex++;

                    if (frameIndex % FrameSkip == 0)
                    {
                        bool accepted = slam.ProcessFrame(capture.Frame, out _, out _);
                        if (accepted)
                        {
                            processed++;
                            steps.Add(slam.LastStep);

                            double stepMeters = Math.Sqrt(
                                slam.LastStep.StepX * slam.LastStep.StepX +
                                slam.LastStep.StepZ * slam.LastStep.StepZ);

                            frames.Add(new FrameMetric(
                                framesRead,
                                processed,
                                slam.MatchCount,
                                slam.InlierCount,
                                slam.TriangulatedCount,
                                map.Points.Count,
                                slam.LastMotionValid,
                                stepMeters,
                                slam.LastStep.YawStep * 180.0 / Math.PI));

                            bar.Report(
                                completedBase + processed,
                                $"pass {index + 1}/{bake.Passes}  frame {processed}/{passUnits}  {name}");
                        }
                    }

                    if (options.MaxFrames > 0 && framesRead >= options.MaxFrames)
                        break;
                }

                clock.Stop();
                LogControl.Normal();
            }

            return new BakePassResult
            {
                Index = index,
                Name = name,
                Map = map,
                Steps = steps,
                Frames = frames,
                Settings = settings,
                FramesRead = framesRead,
                FramesProcessed = processed,
                Fps = fps,
                Elapsed = clock.Elapsed
            };
        }

        /// <summary>
        /// Number of sampled frames the bar uses as the work unit for one pass.
        /// Uses the container metadata when available and falls back to counting the
        /// video when the metadata does not report a length.
        /// </summary>
        private long EstimateProcessedFrames()
        {
            try
            {
                using var capture = new CameraFeedWindow(videoPath);
                long total = capture.FrameCount;

                if (total <= 0)
                {
                    Log.Information("Bake: the container does not report a frame count; counting the video once for the progress bar.");
                    total = 0;
                    while (capture.ReadFrame())
                        total++;
                }

                if (options.MaxFrames > 0)
                    total = Math.Min(total, options.MaxFrames);

                // The first sampled frame only initializes the tracker and is not
                // counted as a processed frame, so the estimate is (total - 1) / skip.
                return Math.Max(1, (total - 1) / FrameSkip);
            }
            catch (Exception exception)
            {
                Log.Warning(exception, "Bake: video length could not be estimated; progress is approximate.");
                return 1;
            }
        }

        private BakePassResult? BestPass()
        {
            BakePassResult? best = null;

            foreach (var pass in passes)
            {
                if (best is null || pass.Score.Total > best.Score.Total)
                {
                    best = pass;
                    continue;
                }

                // A cleaned map with the same score is the better final result.
                if (Math.Abs(pass.Score.Total - best.Score.Total) < 1e-9 &&
                    ((pass.Pruned && !best.Pruned) ||
                     (pass.Pruned == best.Pruned && pass.Map.Points.Count > best.Map.Points.Count)))
                {
                    best = pass;
                }
            }

            return best;
        }

        private static string PassName(int index) => index switch
        {
            0 => "baseline",
            1 => "adaptive",
            2 => "guided",
            3 => "structure",
            _ => "cleanup"
        };

        /// <summary>
        /// Widens tracking when the baseline lost too many frames. The pass keeps the
        /// preset's scene geometry; it only changes motion-tracking parameters.
        /// </summary>
        private static BakePassSettings Adapt(BakePassSettings baseSettings, BakePassResult baseline)
        {
            BakeScore score = baseline.Score;
            bool weakCoverage = score.Coverage < 0.92;
            bool weakInliers = score.InlierRatio < 0.55;

            if (!weakCoverage && !weakInliers)
                return baseSettings;

            int window = baseSettings.LkWindow +
                (score.InlierRatio < 0.40 || score.Coverage < 0.75 ? 8 : 4);
            window = Math.Clamp(window, 5, 61);
            if (window % 2 == 0)
                window--;

            return baseSettings with
            {
                LkWindow = window,
                LkLevels = Math.Min(6, baseSettings.LkLevels + 1),
                LkError = Math.Min(60.0, baseSettings.LkError * 1.25),
                MaxStep = baseSettings.MaxStep * 1.2
            };
        }

        /// <summary>
        /// Structure-completion settings: shorter keyframe spacing creates more
        /// baselines, a longer structure window triangulates from more keyframes and
        /// a larger point budget fills more holes in the cloud.
        /// </summary>
        private static BakePassSettings Complete(BakePassSettings settings) => settings with
        {
            KeyframeFrames = Math.Max(4, settings.KeyframeFrames / 2),
            StructureKeyframes = Math.Clamp(Math.Max(settings.StructureKeyframes, 5), 1, 8),
            StructurePoints = Math.Max(settings.StructurePoints, 400)
        };

        /// <summary>
        /// Runs with the best trajectory as a per-frame prior. A slightly stricter LK
        /// error keeps only high-confidence matches; the prior carries the frame when
        /// the stricter gate rejects the fit.
        /// </summary>
        private static BakePassSettings Guide(BakePassSettings settings)
        {
            int window = Math.Clamp(settings.LkWindow + 2, 5, 61);
            if (window % 2 == 0)
                window--;

            return settings with
            {
                LkWindow = window,
                LkError = Math.Max(5.0, settings.LkError * 0.90)
            };
        }

        /// <summary>Adds diversity to passes beyond the fifth so extra runs explore new settings.</summary>
        private static BakePassSettings Explore(BakePassSettings settings, int extra)
        {
            int window = Math.Clamp(settings.LkWindow + 2 * extra, 5, 61);
            if (window % 2 == 0)
                window--;

            return settings with
            {
                LkWindow = window,
                LkError = Math.Min(60.0, settings.LkError * (1.0 + 0.10 * extra))
            };
        }

        private static void Prune(BakePassResult pass)
        {
            var points = pass.Map.Points;
            if (points.Count == 0)
                return;

            int confirmed = pass.ConfirmedPoints;
            if (confirmed < points.Count * MinConfirmedFraction)
            {
                Log.Information(
                    "Pass {Pass}: cleanup skipped, only {Confirmed}/{Total} points were observed twice.",
                    pass.Index + 1,
                    confirmed,
                    points.Count);
                return;
            }

            int removed = points.RemoveAll(point => point.Observations < 2);
            pass.Pruned = removed > 0;

            Log.Information(
                "Pass {Pass}: cleanup removed {Removed} weakly observed points, {Kept} remain.",
                pass.Index + 1,
                removed,
                points.Count);
        }

        private void Export(BakePassResult selected)
        {
            Directory.CreateDirectory(bake.OutputDirectory);

            WritePoints(selected);
            WriteTrajectory(selected);
            WriteMetrics(selected);
            WritePasses(selected);
            WriteSummary(selected);
        }

        private void WritePoints(BakePassResult selected)
        {
            using var writer = NewWriter("points.csv");
            writer.WriteLine("x,y,z,r,g,b,normal_x,normal_y,normal_z,confidence,semantic_label");

            foreach (var point in selected.Map.Points)
            {
                var position = point.Position;
                var normal = point.Normal;
                writer.WriteLine(string.Join(',',
                    F(position.X),
                    F(position.Y),
                    F(position.Z),
                    F(point.Color.X),
                    F(point.Color.Y),
                    F(point.Color.Z),
                    F(normal.X),
                    F(normal.Y),
                    F(normal.Z),
                    F(point.Confidence),
                    CsvEscape(point.SemanticLabel)));
            }
        }

        private void WriteTrajectory(BakePassResult selected)
        {
            using var writer = NewWriter("trajectory.csv");
            writer.WriteLine("x,y,z");

            foreach (var position in selected.Map.Trajectory)
                writer.WriteLine(string.Join(',', F(position.X), F(position.Y), F(position.Z)));
        }

        private void WriteMetrics(BakePassResult selected)
        {
            using var writer = NewWriter("metrics.csv");
            writer.WriteLine("frame,processed,matches,inliers,inlier_ratio,triangulated,total_points,motion_valid,step_m,yaw_deg");

            foreach (var frame in selected.Frames)
            {
                writer.WriteLine(string.Join(',',
                    frame.Frame.ToString(CultureInfo.InvariantCulture),
                    frame.Processed.ToString(CultureInfo.InvariantCulture),
                    frame.Matches.ToString(CultureInfo.InvariantCulture),
                    frame.Inliers.ToString(CultureInfo.InvariantCulture),
                    F(frame.InlierRatio),
                    frame.Triangulated.ToString(CultureInfo.InvariantCulture),
                    frame.TotalPoints.ToString(CultureInfo.InvariantCulture),
                    frame.MotionValid ? "true" : "false",
                    F(frame.StepMeters),
                    F(frame.YawDegrees)));
            }
        }

        private void WritePasses(BakePassResult selected)
        {
            using var writer = NewWriter("passes.csv");
            writer.WriteLine("pass,name,score,inlier_ratio,coverage,density,smoothness,points,confirmed_points,frames,path_m,seconds,pruned,selected");

            foreach (var pass in passes)
            {
                writer.WriteLine(string.Join(',',
                    (pass.Index + 1).ToString(CultureInfo.InvariantCulture),
                    pass.Name,
                    F(pass.Score.Total),
                    F(pass.Score.InlierRatio),
                    F(pass.Score.Coverage),
                    F(pass.Score.Density),
                    F(pass.Score.Smoothness),
                    pass.Map.Points.Count.ToString(CultureInfo.InvariantCulture),
                    pass.ConfirmedPoints.ToString(CultureInfo.InvariantCulture),
                    pass.FramesProcessed.ToString(CultureInfo.InvariantCulture),
                    F(pass.PathMeters),
                    F(pass.Elapsed.TotalSeconds),
                    pass.Pruned ? "true" : "false",
                    ReferenceEquals(pass, selected) ? "true" : "false"));
            }
        }

        private void WriteSummary(BakePassResult selected)
        {
            using var writer = NewWriter("summary.txt");

            writer.WriteLine($"video={videoPath}");
            writer.WriteLine($"preset={options.ScenePreset}");
            writer.WriteLine("bake=true");
            writer.WriteLine($"bake_passes={bake.Passes}");
            writer.WriteLine($"bake_max_keyframes={bake.MaxKeyframes}");
            writer.WriteLine($"bake_output={bake.OutputDirectory}");
            writer.WriteLine($"selected_pass={selected.Index + 1}");
            writer.WriteLine($"selected_name={selected.Name}");
            writer.WriteLine($"selected_score={F(selected.Score.Total)}");
            writer.WriteLine($"score_inlier_ratio={F(selected.Score.InlierRatio)}");
            writer.WriteLine($"score_coverage={F(selected.Score.Coverage)}");
            writer.WriteLine($"score_density={F(selected.Score.Density)}");
            writer.WriteLine($"score_smoothness={F(selected.Score.Smoothness)}");
            writer.WriteLine($"frames_read={selected.FramesRead}");
            writer.WriteLine($"frames_processed={selected.FramesProcessed}");
            writer.WriteLine($"max_frames={options.MaxFrames}");
            writer.WriteLine("deterministic=true");
            writer.WriteLine($"fps={F(selected.Fps)}");
            writer.WriteLine($"points={selected.Map.Points.Count}");
            writer.WriteLine($"points_confirmed={selected.ConfirmedPoints}");
            writer.WriteLine($"trajectory_points={selected.Map.Trajectory.Count}");
            writer.WriteLine($"pruned={(selected.Pruned ? "true" : "false")}");

            writer.WriteLine($"lk_window={selected.Settings.LkWindow}");
            writer.WriteLine($"lk_levels={selected.Settings.LkLevels}");
            writer.WriteLine($"lk_error={F(selected.Settings.LkError)}");
            writer.WriteLine($"max_step_m={F(selected.Settings.MaxStep)}");
            writer.WriteLine($"keyframe_frames={selected.Settings.KeyframeFrames}");
            writer.WriteLine($"structure_keyframes={selected.Settings.StructureKeyframes}");
            writer.WriteLine($"structure_points={selected.Settings.StructurePoints}");

            double duration = selected.Fps > 0.0 ? selected.FramesRead / selected.Fps : 0.0;
            double span = selected.Map.Trajectory.Count > 1
                ? (selected.Map.Trajectory[^1] - selected.Map.Trajectory[0]).Length
                : 0.0;

            writer.WriteLine($"path_meters={F(selected.PathMeters)}");
            writer.WriteLine($"span_meters={F(span)}");
            writer.WriteLine($"duration_seconds={F(duration)}");
            writer.WriteLine($"average_speed_kmh={(duration > 0.0 ? F(selected.PathMeters / duration * 3.6) : "0.0000")}");
            writer.WriteLine($"pass_seconds={F(selected.Elapsed.TotalSeconds)}");
        }

        private StreamWriter NewWriter(string fileName) => new(
            Path.Combine(bake.OutputDirectory, fileName),
            append: false,
            new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));

        private static string F(double value) => value.ToString("F4", CultureInfo.InvariantCulture);

        private static string CsvEscape(string value)
        {
            if (string.IsNullOrEmpty(value))
                return string.Empty;
            if (value.Contains(',') || value.Contains('"') || value.Contains('\n'))
                return $"\"{value.Replace("\"", "\"\"")}\"";
            return value;
        }
    }
}
