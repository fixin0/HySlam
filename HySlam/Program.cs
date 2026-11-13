using System.Globalization;
using HySlam.Core;
using HySlam.Core.Bake;
using HySlam.Core.Data;
using HySlam.Core.Diagnostics;
using HySlam.Core.Slam;
using Serilog;
using Serilog.Events;

namespace HySlam
{
    internal static class Program
    {
        private enum ParseResult
        {
            Run,
            Help,
            Error
        }

        /// <summary>
        /// One row of the scene preset table. A preset only changes numbers that are
        /// scene dependent; explicit CLI options always win over the preset values.
        /// </summary>
        private readonly record struct SceneSettings(
            string Name,
            double MinDepth,
            double MaxDepth,
            double StructureDepth,
            double BandTop,
            double BandBottom,
            int KeyframeFrames,
            double MaxStep,
            double? PitchDegrees,
            int LkWindow,
            int LkLevels,
            double LkError,
            string Description);

        // The road preset keeps the original dashcam tuning, so existing runs stay
        // byte-identical. The others differ only in the numbers that actually change
        // between scenes: how far the ground plane is usable, which image band holds
        // the ground, how much baseline a keyframe may accumulate and how fast a
        // single frame is allowed to move.
        private static readonly Dictionary<string, SceneSettings> Presets = new(StringComparer.OrdinalIgnoreCase)
        {
            ["road"] = new("road", 6.0, 26.0, 45.0, 0.70, 0.99, 30, 3.0, null, 21, 3, 20.0,
                "dashcam on a normal road, rigid mount"),
            ["highway"] = new("highway", 10.0, 55.0, 120.0, 0.68, 0.86, 12, 10.0, null, 27, 4, 30.0,
                "motorway at cruise speed: far field, large steps"),
            ["fast"] = new("fast", 12.0, 70.0, 120.0, 0.65, 0.82, 6, 15.0, null, 31, 4, 35.0,
                "very high speed / low frame rate: wider LK search, capped step"),
            ["mountain"] = new("mountain", 5.0, 30.0, 70.0, 0.70, 0.99, 25, 4.0, null, 21, 3, 25.0,
                "winding mountain road, trees and rock, no lane markings"),
            ["city"] = new("city", 3.0, 18.0, 40.0, 0.72, 0.98, 10, 3.0, null, 21, 3, 20.0,
                "dense urban street: tighter step, more keyframes, wider band"),
            ["office"] = new("office", 1.5, 9.0, 18.0, 0.72, 0.96, 8, 0.8, 5.0, 21, 3, 20.0,
                "office room, hand-held, desk height"),
            ["home"] = new("home", 1.2, 7.0, 12.0, 0.78, 0.97, 8, 0.6, 5.0, 21, 3, 20.0,
                "home / narrow room, hand-held walking")
        };

        // Accepted English aliases.
        private static readonly Dictionary<string, string> PresetAliases = new(StringComparer.OrdinalIgnoreCase)
        {
            ["indoor"] = "home",
            ["house"] = "home",
            ["room"] = "home",
            ["motorway"] = "highway",
            ["expressway"] = "highway",
            ["high-speed"] = "fast",
            ["highspeed"] = "fast",
            ["speed"] = "fast",
            ["urban"] = "city",
            ["mountain-road"] = "mountain"
        };

        private static int Main(string[] args)
        {
            // Configure Serilog before parsing so argument errors are logged through
            // the same pipeline as the rest of the application. Errors go to stderr,
            // regular information stays on stdout; --help output is printed directly.
            Log.Logger = new LoggerConfiguration()
                .MinimumLevel.ControlledBy(LogControl.LevelSwitch)
                .WriteTo.Console(standardErrorFromLevel: LogEventLevel.Error)
                .CreateLogger();

            AppDomain.CurrentDomain.UnhandledException += (_, eventArgs) =>
                Log.Fatal(eventArgs.ExceptionObject as Exception, "Unhandled exception in the application domain.");
            AppDomain.CurrentDomain.ProcessExit += (_, _) => Log.Information("Process exit requested.");
            TaskScheduler.UnobservedTaskException += (_, eventArgs) =>
                Log.Error(eventArgs.Exception, "Unobserved task exception.");

            try
            {
                Log.Information(
                    "Starting HySlam: {Arguments}",
                    args.Length == 0 ? "(no arguments)" : string.Join(' ', args));
                Log.Information(
                    "Runtime {Framework} on {OS} ({Architecture}).",
                    Environment.Version,
                    Environment.OSVersion,
                    System.Runtime.InteropServices.RuntimeInformation.ProcessArchitecture);

                ParseResult result = ParseArguments(args, out RunOptions? options);

                if (result == ParseResult.Help)
                    return 0;

                if (result == ParseResult.Error)
                    return 1;

                if (options!.VerifyDirectory is not null)
                {
                    Log.Information("Verifying the bake output; no window is opened.");
                    return BakeVerifier.Run(options.VerifyDirectory);
                }

                if (options.LoadDirectory is not null)
                {
                    Log.Information("Loading the map; no video is decoded.");

                    MapData map;
                    try
                    {
                        map = MapFileReader.Read(options.LoadDirectory);
                    }
                    catch (Exception exception) when (exception is IOException or InvalidDataException or UnauthorizedAccessException)
                    {
                        Log.Error("Error: could not read the map: {Message}", exception.Message);
                        return 1;
                    }

                    Log.Information(
                        "Loaded {Points} points and {Trajectory} trajectory samples from {Directory}.",
                        map.Points.Count,
                        map.Trajectory.Count,
                        options.LoadDirectory);

                    using var mapWindow = new LoadedMapWindow(options.LoadDirectory, map);
                    Log.Information("Entering the map viewer.");
                    mapWindow.Run();
                    Log.Information("Map viewer closed normally.");
                    return 0;
                }

                if (options.Bake is not null)
                {
                    Log.Information("Starting the offline bake; no window is opened.");
                    var bakeRunner = new BakeRunner(options);
                    return bakeRunner.Run();
                }

                Log.Information("Creating the game window.");
                using var application = new WindowManager(options!);
                Log.Information("Entering the main loop.");
                application.Run();
                Log.Information("Main loop finished normally.");
                return 0;
            }
            catch (Exception exception)
            {
                Log.Fatal(exception, "HySlam terminated unexpectedly.");
                return 1;
            }
            finally
            {
                Log.CloseAndFlush();
            }
        }

        /// <summary>
        /// Parses the command line and prints usage or error messages when needed.
        /// Supports --source:video, --path:file, --calib:file, --dump:directory,
        /// --load:directory, --verify:directory, --max-frames:N, --deterministic, the
        /// scene and tracking options, and the offline bake options (--bake,
        /// --bake-passes, --bake-output, --bake-max-keyframes) in both the joined and
        /// the separated forms.
        /// </summary>
        private static ParseResult ParseArguments(string[] args, out RunOptions? options)
        {
            options = null;

            string? videoPath = null;
            string? calibrationPath = null;
            string? dumpDirectory = null;
            string? loadDirectory = null;
            string? verifyDirectory = null;
            string source = "video";
            int maxFrames = 0;
            bool deterministic = false;
            double cameraHeight = 1.3;
            double? pitchDegrees = null;
            double horizontalFovDegrees = 0.0;
            double? minGroundDepth = null;
            double? maxGroundDepth = null;
            double? maxStructureDepth = null;
            double? groundBandTop = null;
            double? groundBandBottom = null;
            int? keyframeFrames = null;
            double? maxTranslationStep = null;
            int? lkWindow = null;
            int? lkLevels = null;
            double? lkError = null;
            string scenePreset = "road";
            bool loopClosure = false;
            bool bake = false;
            bool bakeOptionUsed = false;
            int bakePasses = 5;
            string? bakeOutput = null;
            int bakeMaxKeyframes = 512;

            for (int i = 0; i < args.Length; i++)
            {
                string argument = args[i];

                if (argument is "--help" or "-h")
                {
                    PrintUsage();
                    return ParseResult.Help;
                }

                if (TrySplitOption(argument, out string name, out string? inlineValue))
                {
                    switch (name)
                    {
                        case "--source":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? sourceValue))
                                return ParseResult.Error;

                            source = sourceValue.ToLowerInvariant();
                            break;

                        case "--path":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? pathValue))
                                return ParseResult.Error;

                            videoPath = Path.GetFullPath(pathValue);
                            break;

                        case "--calib":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? calibrationValue))
                                return ParseResult.Error;

                            calibrationPath = Path.GetFullPath(calibrationValue);
                            break;

                        case "--dump":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? dumpValue))
                                return ParseResult.Error;

                            dumpDirectory = Path.GetFullPath(dumpValue);
                            break;

                        case "--load":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? loadValue))
                                return ParseResult.Error;

                            loadDirectory = Path.GetFullPath(loadValue);
                            break;

                        case "--verify":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? verifyValue))
                                return ParseResult.Error;

                            verifyDirectory = Path.GetFullPath(verifyValue);
                            break;

                        case "--max-frames":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? maxFramesValue))
                                return ParseResult.Error;

                            if (!int.TryParse(maxFramesValue, NumberStyles.Integer, CultureInfo.InvariantCulture, out maxFrames) ||
                                maxFrames <= 0)
                            {
                                Log.Error(
                                    "Error: '--max-frames' must be a positive integer, got '{Value}'.",
                                    maxFramesValue);
                                return ParseResult.Error;
                            }

                            break;

                        case "--deterministic":
                            if (!string.IsNullOrEmpty(inlineValue))
                            {
                                Log.Error("Error: '--deterministic' does not take a value.");
                                return ParseResult.Error;
                            }

                            deterministic = true;
                            break;

                        case "--loop-closure":
                            if (!string.IsNullOrEmpty(inlineValue))
                            {
                                Log.Error("Error: '--loop-closure' does not take a value.");
                                return ParseResult.Error;
                            }

                            loopClosure = true;
                            break;

                        case "--camera-height":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? cameraHeightValue) ||
                                !TryParsePositiveDouble(cameraHeightValue, name, out cameraHeight))
                                return ParseResult.Error;

                            break;

                        case "--pitch":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? pitchValue) ||
                                !TryParseDouble(pitchValue, name, out double pitch))
                                return ParseResult.Error;

                            if (pitch < -45 || pitch > 45)
                            {
                                Log.Error("Error: '--pitch' must be between -45 and 45 degrees.");
                                return ParseResult.Error;
                            }

                            pitchDegrees = pitch;
                            break;

                        case "--fov":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? fovValue) ||
                                !TryParsePositiveDouble(fovValue, name, out horizontalFovDegrees))
                                return ParseResult.Error;

                            if (horizontalFovDegrees >= 180)
                            {
                                Log.Error("Error: '--fov' must be less than 180 degrees.");
                                return ParseResult.Error;
                            }

                            break;

                        case "--preset":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? presetValue))
                                return ParseResult.Error;

                            scenePreset = presetValue.ToLowerInvariant();
                            break;

                        case "--min-depth":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? minDepthValue) ||
                                !TryParsePositiveDouble(minDepthValue, name, out double minDepthParsed))
                                return ParseResult.Error;

                            minGroundDepth = minDepthParsed;
                            break;

                        case "--max-depth":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? maxDepthValue) ||
                                !TryParsePositiveDouble(maxDepthValue, name, out double maxDepthParsed))
                                return ParseResult.Error;

                            maxGroundDepth = maxDepthParsed;
                            break;

                        case "--structure-depth":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? structureDepthValue) ||
                                !TryParsePositiveDouble(structureDepthValue, name, out double structureDepthParsed))
                                return ParseResult.Error;

                            maxStructureDepth = structureDepthParsed;
                            break;

                        case "--band-top":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? bandTopValue) ||
                                !TryParseUnitFraction(bandTopValue, name, out double bandTopParsed))
                                return ParseResult.Error;

                            groundBandTop = bandTopParsed;
                            break;

                        case "--band-bottom":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? bandBottomValue) ||
                                !TryParseUnitFraction(bandBottomValue, name, out double bandBottomParsed))
                                return ParseResult.Error;

                            groundBandBottom = bandBottomParsed;
                            break;

                        case "--keyframe-frames":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? keyframeValue) ||
                                !int.TryParse(keyframeValue, NumberStyles.Integer, CultureInfo.InvariantCulture, out int keyframeParsed) ||
                                keyframeParsed <= 0)
                            {
                                Log.Error(
                                    $"Error: '--keyframe-frames' must be a positive integer, got '{keyframeValue}'.");
                                return ParseResult.Error;
                            }

                            keyframeFrames = keyframeParsed;
                            break;

                        case "--max-step":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? stepValue) ||
                                !TryParsePositiveDouble(stepValue, name, out double stepParsed))
                                return ParseResult.Error;

                            maxTranslationStep = stepParsed;
                            break;

                        case "--lk-window":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? lkWindowValue) ||
                                !TryParsePositiveInt(lkWindowValue, name, out int lkWindowParsed))
                                return ParseResult.Error;

                            lkWindow = lkWindowParsed;
                            break;

                        case "--lk-levels":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? lkLevelsValue) ||
                                !int.TryParse(lkLevelsValue, NumberStyles.Integer, CultureInfo.InvariantCulture, out int lkLevelsParsed) ||
                                lkLevelsParsed < 0)
                            {
                                Log.Error(
                                    $"Error: '--lk-levels' must be zero or a positive integer, got '{lkLevelsValue}'.");
                                return ParseResult.Error;
                            }

                            lkLevels = lkLevelsParsed;
                            break;

                        case "--lk-error":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? lkErrorValue) ||
                                !TryParsePositiveDouble(lkErrorValue, name, out double lkErrorParsed))
                                return ParseResult.Error;

                            lkError = lkErrorParsed;
                            break;

                        case "--bake":
                            if (!string.IsNullOrEmpty(inlineValue))
                            {
                                Log.Error("Error: '--bake' does not take a value.");
                                return ParseResult.Error;
                            }

                            bake = true;
                            bakeOptionUsed = true;
                            break;

                        case "--bake-passes":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? bakePassesValue) ||
                                !int.TryParse(bakePassesValue, NumberStyles.Integer, CultureInfo.InvariantCulture, out bakePasses) ||
                                bakePasses < 1 || bakePasses > 10)
                            {
                                Log.Error(
                                    "Error: '--bake-passes' must be an integer between 1 and 10, got '{Value}'.",
                                    bakePassesValue);
                                return ParseResult.Error;
                            }

                            bakeOptionUsed = true;
                            break;

                        case "--bake-output":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? bakeOutputValue))
                                return ParseResult.Error;

                            bakeOutput = Path.GetFullPath(bakeOutputValue);
                            bakeOptionUsed = true;
                            break;

                        case "--bake-max-keyframes":
                            if (!TryReadValue(args, ref i, inlineValue, name, out string? bakeKeyframesValue) ||
                                !int.TryParse(bakeKeyframesValue, NumberStyles.Integer, CultureInfo.InvariantCulture, out bakeMaxKeyframes) ||
                                bakeMaxKeyframes < 8 || bakeMaxKeyframes > 8192)
                            {
                                Log.Error(
                                    "Error: '--bake-max-keyframes' must be an integer between 8 and 8192, got '{Value}'.",
                                    bakeKeyframesValue);
                                return ParseResult.Error;
                            }

                            bakeOptionUsed = true;
                            break;

                        default:
                            Log.Error("Error: unknown argument '{Argument}'.", name);
                            PrintUsage();
                            return ParseResult.Error;
                    }

                    continue;
                }

                Log.Error("Error: unknown argument '{Argument}'.", argument);
                PrintUsage();
                return ParseResult.Error;
            }

            if (!string.Equals(source, "video", StringComparison.Ordinal))
            {
                Log.Error("Error: unsupported source '{Source}'. Supported sources: video.", source);
                return ParseResult.Error;
            }

            if (loadDirectory is not null && verifyDirectory is not null)
            {
                Log.Error("Error: '--load' and '--verify' cannot be used together.");
                return ParseResult.Error;
            }

            bool readBack = loadDirectory is not null || verifyDirectory is not null;

            if (readBack)
            {
                if (bake || bakeOptionUsed)
                {
                    Log.Error(
                        "Error: '--bake', '--bake-passes', '--bake-output' and '--bake-max-keyframes' " +
                        "cannot be combined with '--load' or '--verify'.");
                    return ParseResult.Error;
                }

                string directory = (loadDirectory ?? verifyDirectory)!;
                if (!Directory.Exists(directory))
                {
                    Log.Error("Error: directory not found: {Directory}", directory);
                    return ParseResult.Error;
                }

                if (!string.IsNullOrWhiteSpace(videoPath))
                    Log.Warning("Read-back mode ignores '--path'; no video is decoded.");
                if (dumpDirectory is not null)
                    Log.Warning("Read-back mode ignores '--dump'.");

                options = new RunOptions
                {
                    VideoPath = null,
                    LoadDirectory = loadDirectory,
                    VerifyDirectory = verifyDirectory
                };

                return ParseResult.Run;
            }

            if (string.IsNullOrWhiteSpace(videoPath))
            {
                Log.Error("Error: '--path' is required unless '--load' or '--verify' is used.");
                PrintUsage();
                return ParseResult.Error;
            }

            if (!File.Exists(videoPath))
            {
                Log.Error("Error: video file not found: {VideoPath}", videoPath);
                return ParseResult.Error;
            }

            // Resolve the preset name (including aliases) and apply its values only to
            // options that were not set explicitly on the command line.
            string presetKey = PresetAliases.TryGetValue(scenePreset, out string? alias) ? alias : scenePreset;
            if (!Presets.TryGetValue(presetKey, out SceneSettings scene))
            {
                Log.Error(
                    "Error: unknown preset '{Preset}'. Available: {AvailablePresets}.",
                    scenePreset,
                    string.Join(", ", Presets.Keys));
                return ParseResult.Error;
            }

            // An indoor camera is normally tilted down towards the floor; the dashcam
            // horizon (0.665 of the frame) would be far below the real horizon here.
            if (scene.PitchDegrees.HasValue)
                pitchDegrees ??= scene.PitchDegrees.Value;

            double effectiveMinDepth = minGroundDepth ?? scene.MinDepth;
            double effectiveMaxDepth = maxGroundDepth ?? scene.MaxDepth;
            double effectiveStructureDepth = maxStructureDepth ?? scene.StructureDepth;
            double effectiveBandTop = groundBandTop ?? scene.BandTop;
            double effectiveBandBottom = groundBandBottom ?? scene.BandBottom;
            int effectiveKeyframeFrames = keyframeFrames ?? scene.KeyframeFrames;
            double effectiveMaxStep = maxTranslationStep ?? scene.MaxStep;
            int effectiveLkWindow = lkWindow ?? scene.LkWindow;
            int effectiveLkLevels = lkLevels ?? scene.LkLevels;
            double effectiveLkError = lkError ?? scene.LkError;

            if (effectiveMaxDepth <= effectiveMinDepth)
            {
                Log.Error(
                    "Error: '--max-depth' ({MaxDepth}) must be greater than '--min-depth' ({MinDepth}).",
                    effectiveMaxDepth,
                    effectiveMinDepth);
                return ParseResult.Error;
            }

            if (effectiveBandTop < 0.0 || effectiveBandBottom > 1.0 || effectiveBandBottom - effectiveBandTop < 0.05)
            {
                Log.Error(
                    $"Error: ground band must satisfy 0 <= --band-top < --band-bottom <= 1 and span at least 0.05.");
                return ParseResult.Error;
            }

            if (effectiveLkWindow < 5 || effectiveLkWindow > 61 || effectiveLkWindow % 2 == 0)
            {
                Log.Error("Error: '--lk-window' must be an odd number between 5 and 61.");
                return ParseResult.Error;
            }

            if (effectiveLkLevels < 0 || effectiveLkLevels > 6)
            {
                Log.Error("Error: '--lk-levels' must be between 0 and 6.");
                return ParseResult.Error;
            }

            if (!bake && bakeOptionUsed)
            {
                Log.Error("Error: '--bake-passes', '--bake-output' and '--bake-max-keyframes' require '--bake'.");
                return ParseResult.Error;
            }

            BakeOptions? bakeOptions = null;
            if (bake)
            {
                string outputDirectory = bakeOutput ?? Path.GetFullPath(
                    Path.Combine("runs", $"bake_{DateTime.Now:yyyyMMdd_HHmmss}"));

                bakeOptions = new BakeOptions
                {
                    Passes = bakePasses,
                    OutputDirectory = outputDirectory,
                    MaxKeyframes = bakeMaxKeyframes
                };

                if (dumpDirectory is not null)
                {
                    Log.Warning(
                        "Bake ignores '--dump'; the selected result is written to {OutputDirectory}.",
                        outputDirectory);
                }

                // A bake must be reproducible: the same clip and options always have
                // to produce the same passes and the same selected result.
                deterministic = true;
            }

            CameraCalibration? calibration = null;
            if (calibrationPath is not null)
            {
                try
                {
                    calibration = CameraCalibration.FromFile(calibrationPath);
                }
                catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or FormatException)
                {
                    Log.Error("Error: could not load calibration: {Message}", ex.Message);
                    return ParseResult.Error;
                }
            }

            options = new RunOptions
            {
                VideoPath = videoPath,
                LoadDirectory = loadDirectory,
                VerifyDirectory = verifyDirectory,
                Calibration = calibration,
                DumpDirectory = bakeOptions is null ? dumpDirectory : null,
                MaxFrames = maxFrames,
                Deterministic = deterministic,
                EnableLoopClosure = loopClosure,
                CameraHeight = cameraHeight,
                PitchDegrees = pitchDegrees,
                HorizontalFovDegrees = horizontalFovDegrees,
                MinGroundDepth = effectiveMinDepth,
                MaxGroundDepth = effectiveMaxDepth,
                MaxStructureDepth = effectiveStructureDepth,
                GroundBandTop = effectiveBandTop,
                GroundBandBottom = effectiveBandBottom,
                KeyframeFrames = effectiveKeyframeFrames,
                MaxTranslationStep = effectiveMaxStep,
                ScenePreset = scene.Name,
                LkWindowSize = effectiveLkWindow,
                LkLevels = effectiveLkLevels,
                LkError = effectiveLkError,
                Bake = bakeOptions
            };

            return ParseResult.Run;
        }

        private static bool TrySplitOption(string argument, out string name, out string? value)
        {
            name = argument;
            value = null;

            if (!argument.StartsWith("--", StringComparison.Ordinal))
                return false;

            int separator = argument.IndexOfAny([':', '=']);
            if (separator > 0)
            {
                name = argument[..separator];
                value = argument[(separator + 1)..];
            }

            return true;
        }

        private static bool TryReadValue(
            string[] args,
            ref int index,
            string? inlineValue,
            string option,
            out string value)
        {
            if (!string.IsNullOrEmpty(inlineValue))
            {
                value = inlineValue;
                return true;
            }

            if (index + 1 >= args.Length)
            {
                Log.Error("Error: '{Option}' requires a value.", option);
                PrintUsage();
                value = string.Empty;
                return false;
            }

            value = args[++index];
            return true;
        }

        private static bool TryParseDouble(string text, string option, out double value)
        {
            if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            {
                Log.Error("Error: '{Option}' requires a number, got '{Value}'.", option, text);
                return false;
            }

            return true;
        }

        private static bool TryParsePositiveDouble(string text, string option, out double value)
        {
            if (!TryParseDouble(text, option, out value))
                return false;

            if (value <= 0)
            {
                Log.Error("Error: '{Option}' must be greater than zero.", option);
                return false;
            }

            return true;
        }

        private static bool TryParsePositiveInt(string text, string option, out int value)
        {
            if (!int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out value) || value <= 0)
            {
                Log.Error("Error: '{Option}' must be a positive integer, got '{Value}'.", option, text);
                return false;
            }

            return true;
        }

        private static bool TryParseUnitFraction(string text, string option, out double value)
        {
            if (!TryParseDouble(text, option, out value))
                return false;

            if (value < 0.0 || value > 1.0)
            {
                Log.Error(
                    "Error: '{Option}' must be between 0 and 1 (a fraction of the frame height).",
                    option);
                return false;
            }

            return true;
        }

        /// <summary>
        /// Prints the contents of usage.txt that is copied next to the executable.
        /// </summary>
        private static void PrintUsage()
        {
            string usagePath = Path.Combine(AppContext.BaseDirectory, "usage.txt");

            if (!File.Exists(usagePath))
            {
                Log.Error("Error: usage information not found: {UsagePath}", usagePath);
                return;
            }

            Console.Write(File.ReadAllText(usagePath));
        }
    }
}
