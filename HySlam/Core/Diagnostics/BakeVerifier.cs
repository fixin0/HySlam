using System.Globalization;
using Serilog;

namespace HySlam.Core.Diagnostics
{
    /// <summary>
    /// Headless validation of a bake output directory.
    ///
    /// The verifier only reads files. It checks that the CSVs parse, then
    /// cross-checks the file contents against summary.txt: map counts, path and
    /// span, per-frame metric rows, the selected pass row and the reported score.
    /// The process exit code is 0 when every check passes and 1 otherwise.
    /// </summary>
    internal static class BakeVerifier
    {
        private static readonly string[] RequiredFiles =
        [
            "points.csv",
            "trajectory.csv",
            "metrics.csv",
            "passes.csv",
            "summary.txt"
        ];

        private static readonly string[] MetricColumns =
        [
            "frame",
            "processed",
            "matches",
            "inliers",
            "inlier_ratio",
            "triangulated",
            "total_points",
            "motion_valid",
            "step_m",
            "yaw_deg"
        ];

        // Lengths are printed with four decimals; allow a small rounding and
        // storage tolerance without hiding a real mismatch.
        private const double LengthToleranceRatio = 0.002;
        private const double LengthToleranceAbsolute = 0.05;
        private const double ScoreTolerance = 0.001;

        public static int Run(string directory)
        {
            Log.Information("Verifying bake output: {Directory}", directory);

            if (!Directory.Exists(directory))
            {
                Log.Error("Verification failed: directory not found: {Directory}", directory);
                return 1;
            }

            var missing = RequiredFiles
                .Where(file => !File.Exists(Path.Combine(directory, file)))
                .ToList();

            if (missing.Count > 0)
            {
                foreach (string file in missing)
                    Log.Error("Missing required file: {File}", file);

                Log.Error("Verification failed: {Count} required file(s) are missing.", missing.Count);
                return 1;
            }

            int checks = 0;
            int failures = 0;
            int warnings = 0;

            void Pass(string message)
            {
                checks++;
                Log.Information("  [ok]   {Message}", message);
            }

            void Fail(string message)
            {
                checks++;
                failures++;
                Log.Information("  [fail] {Message}", message);
            }

            void Check(string message, bool condition)
            {
                if (condition)
                    Pass(message);
                else
                    Fail(message);
            }

            void Warn(string message)
            {
                warnings++;
                Log.Warning("  [warn] {Message}", message);
            }

            Log.Information("  files: {Files}", string.Join(", ", RequiredFiles));

            MapFileInfo info;
            try
            {
                MapFileReader.Read(directory, out info);
            }
            catch (Exception exception) when (exception is IOException or InvalidDataException or UnauthorizedAccessException)
            {
                Fail($"map CSV files could not be parsed: {exception.Message}");
                Log.Error("Verification failed: the map files are not readable.");
                return 1;
            }

            Pass(
                $"map loaded: {info.PointCount} points, {info.TrajectoryCount} trajectory samples, " +
                $"path {info.PathMeters:0.00} m, span {info.SpanMeters:0.00} m.");

            Dictionary<string, string> summary;
            try
            {
                summary = ReadSummary(Path.Combine(directory, "summary.txt"));
            }
            catch (Exception exception) when (exception is IOException or InvalidDataException or UnauthorizedAccessException)
            {
                Fail($"summary.txt could not be parsed: {exception.Message}");
                Log.Error("Verification failed: the summary is not readable.");
                return 1;
            }

            Pass($"summary.txt parsed: {summary.Count} keys.");

            if (info.PointCount == 0)
                Warn("points.csv is empty.");
            if (info.TrajectoryCount < 2)
                Warn("trajectory.csv has fewer than two samples; path and span are zero.");

            int? points = SummaryInt(summary, "points");
            if (points is int summaryPoints)
                Check($"points.csv count {info.PointCount} matches summary.txt ({summaryPoints}).", info.PointCount == summaryPoints);
            else
                Fail("summary.txt has no valid 'points' value.");

            int? trajectoryPoints = SummaryInt(summary, "trajectory_points");
            if (trajectoryPoints is int summaryTrajectoryPoints)
            {
                Check(
                    $"trajectory.csv count {info.TrajectoryCount} matches summary.txt ({summaryTrajectoryPoints}).",
                    info.TrajectoryCount == summaryTrajectoryPoints);
            }
            else
            {
                Fail("summary.txt has no valid 'trajectory_points' value.");
            }

            double? pathMeters = SummaryDouble(summary, "path_meters");
            if (pathMeters is double summaryPath)
            {
                double tolerance = Math.Max(LengthToleranceAbsolute, Math.Abs(summaryPath) * LengthToleranceRatio);
                Check(
                    $"trajectory path {info.PathMeters:0.00} m matches summary.txt ({summaryPath:0.00} m).",
                    Math.Abs(info.PathMeters - summaryPath) <= tolerance);
            }
            else
            {
                Fail("summary.txt has no valid 'path_meters' value.");
            }

            double? spanMeters = SummaryDouble(summary, "span_meters");
            if (spanMeters is double summarySpan)
            {
                double tolerance = Math.Max(LengthToleranceAbsolute, Math.Abs(summarySpan) * LengthToleranceRatio);
                Check(
                    $"trajectory span {info.SpanMeters:0.00} m matches summary.txt ({summarySpan:0.00} m).",
                    Math.Abs(info.SpanMeters - summarySpan) <= tolerance);
            }
            else
            {
                Fail("summary.txt has no valid 'span_meters' value.");
            }

            int? confirmedPoints = SummaryInt(summary, "points_confirmed");
            if (info.ConfirmedPoints is int confirmed && confirmedPoints is int summaryConfirmed)
            {
                Check(
                    $"confirmed points {confirmed} match summary.txt ({summaryConfirmed}).",
                    confirmed == summaryConfirmed);
            }
            else if (info.ConfirmedPoints is null)
            {
                Warn("points.csv has no 'observations' column; confirmed points were not cross-checked.");
            }
            else
            {
                Fail("summary.txt has no valid 'points_confirmed' value.");
            }

            VerifyMetrics(directory, summary, Check, Fail);

            VerifyPasses(directory, summary, info, Check, Fail);

            if (failures == 0)
            {
                Log.Information("Verification passed: {Checks} checks, {Warnings} warnings.", checks, warnings);
                return 0;
            }

            Log.Error(
                "Verification failed: {Failures} of {Checks} checks failed, {Warnings} warnings.",
                failures,
                checks,
                warnings);
            return 1;
        }

        private static void VerifyMetrics(
            string directory,
            IReadOnlyDictionary<string, string> summary,
            Action<string, bool> check,
            Action<string> fail)
        {
            try
            {
                var (header, rows) = ReadCsv(Path.Combine(directory, "metrics.csv"));

                bool headerOk = MetricColumns.All(
                    column => header.Contains(column, StringComparer.OrdinalIgnoreCase));
                check("metrics.csv has the expected columns.", headerOk);

                int? framesProcessed = SummaryInt(summary, "frames_processed");
                if (framesProcessed is int expected)
                {
                    check(
                        $"metrics.csv has {rows.Count} rows, matching frames_processed ({expected}).",
                        rows.Count == expected);
                }
                else
                {
                    fail("summary.txt has no valid 'frames_processed' value.");
                }
            }
            catch (Exception exception) when (exception is IOException or InvalidDataException or UnauthorizedAccessException)
            {
                fail($"metrics.csv could not be parsed: {exception.Message}");
            }
        }

        private static void VerifyPasses(
            string directory,
            IReadOnlyDictionary<string, string> summary,
            MapFileInfo info,
            Action<string, bool> check,
            Action<string> fail)
        {
            try
            {
                var (header, rows) = ReadCsv(Path.Combine(directory, "passes.csv"));

                int passIndex = IndexOf(header, "pass");
                int nameIndex = IndexOf(header, "name");
                int scoreIndex = IndexOf(header, "score");
                int pointsIndex = IndexOf(header, "points");
                int selectedIndex = IndexOf(header, "selected");

                bool headerOk = passIndex >= 0 &&
                                nameIndex >= 0 &&
                                scoreIndex >= 0 &&
                                pointsIndex >= 0 &&
                                selectedIndex >= 0;
                check("passes.csv has the expected columns.", headerOk);
                check("passes.csv contains at least one pass.", rows.Count > 0);

                int? bakePasses = SummaryInt(summary, "bake_passes");
                if (bakePasses is int expectedPasses)
                    check($"passes.csv has {rows.Count} rows, matching bake_passes ({expectedPasses}).", rows.Count == expectedPasses);
                else
                    fail("summary.txt has no valid 'bake_passes' value.");

                if (!headerOk)
                    return;

                var selectedRows = rows
                    .Where(row => Cell(row, selectedIndex).Equals("true", StringComparison.OrdinalIgnoreCase))
                    .ToList();
                check("passes.csv marks exactly one selected pass.", selectedRows.Count == 1);

                if (selectedRows.Count != 1)
                    return;

                string[] selected = selectedRows[0];

                int? selectedPass = SummaryInt(summary, "selected_pass");
                bool passOk = selectedPass is int expectedPass &&
                              int.TryParse(Cell(selected, passIndex), NumberStyles.Integer, CultureInfo.InvariantCulture, out int passNumber) &&
                              passNumber == expectedPass;
                check("passes.csv selected pass matches summary.txt.", passOk);

                string selectedName = Cell(selected, nameIndex);
                bool nameOk = summary.TryGetValue("selected_name", out string? summaryName) &&
                              string.Equals(selectedName, summaryName?.Trim(), StringComparison.OrdinalIgnoreCase);
                check($"passes.csv selected name '{selectedName}' matches summary.txt.", nameOk);

                int? points = SummaryInt(summary, "points");
                bool pointsOk = points is int expectedPoints &&
                                int.TryParse(Cell(selected, pointsIndex), NumberStyles.Integer, CultureInfo.InvariantCulture, out int passPoints) &&
                                passPoints == expectedPoints &&
                                info.PointCount == expectedPoints;
                check("passes.csv selected points match the exported map and summary.txt.", pointsOk);

                double? selectedScore = SummaryDouble(summary, "selected_score");
                bool scoreOk = selectedScore is double expectedScore &&
                               double.TryParse(Cell(selected, scoreIndex), NumberStyles.Float, CultureInfo.InvariantCulture, out double passScore) &&
                               Math.Abs(passScore - expectedScore) <= ScoreTolerance;
                check("passes.csv selected score matches summary.txt.", scoreOk);
            }
            catch (Exception exception) when (exception is IOException or InvalidDataException or UnauthorizedAccessException)
            {
                fail($"passes.csv could not be parsed: {exception.Message}");
            }
        }

        private static Dictionary<string, string> ReadSummary(string path)
        {
            var summary = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

            int lineNumber = 0;
            foreach (string rawLine in File.ReadLines(path))
            {
                lineNumber++;
                string line = rawLine.Trim();
                if (line.Length == 0 || line.StartsWith('#'))
                    continue;

                int separator = line.IndexOf('=');
                if (separator <= 0)
                {
                    throw new InvalidDataException(
                        $"summary.txt line {lineNumber} is not a key=value pair.");
                }

                string key = line[..separator].Trim();
                string value = line[(separator + 1)..].Trim();
                if (key.Length == 0)
                    throw new InvalidDataException($"summary.txt line {lineNumber} has an empty key.");

                summary[key] = value;
            }

            if (summary.Count == 0)
                throw new InvalidDataException("summary.txt contains no key=value pairs.");

            return summary;
        }

        private static (string[] Header, List<string[]> Rows) ReadCsv(string path)
        {
            using var reader = new StreamReader(path);
            string? headerLine = ReadNonEmptyLine(reader);
            if (headerLine is null)
                throw new InvalidDataException($"{Path.GetFileName(path)} is empty.");

            string[] header = headerLine
                .Split(',')
                .Select((cell, index) =>
                {
                    string value = cell.Trim();
                    return (index == 0 ? value.TrimStart('\uFEFF') : value).ToLowerInvariant();
                })
                .ToArray();

            var rows = new List<string[]>();
            int lineNumber = 1;
            string? line;
            while ((line = reader.ReadLine()) is not null)
            {
                lineNumber++;
                if (string.IsNullOrWhiteSpace(line))
                    continue;

                string[] cells = line.Split(',');
                if (cells.Length != header.Length)
                {
                    throw new InvalidDataException(
                        $"{Path.GetFileName(path)} line {lineNumber} has {cells.Length} columns, expected {header.Length}.");
                }

                rows.Add(cells);
            }

            return (header, rows);
        }

        private static string? ReadNonEmptyLine(StreamReader reader)
        {
            string? line;
            while ((line = reader.ReadLine()) is not null)
            {
                if (!string.IsNullOrWhiteSpace(line))
                    return line;
            }

            return null;
        }

        private static int IndexOf(string[] header, string name)
        {
            return Array.FindIndex(header, cell => cell == name);
        }

        private static string Cell(string[] row, int index)
        {
            return index >= 0 && index < row.Length ? row[index].Trim() : string.Empty;
        }

        private static int? SummaryInt(IReadOnlyDictionary<string, string> summary, string key)
        {
            if (!summary.TryGetValue(key, out string? text))
                return null;

            return int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int value)
                ? value
                : null;
        }

        private static double? SummaryDouble(IReadOnlyDictionary<string, string> summary, string key)
        {
            if (!summary.TryGetValue(key, out string? text))
                return null;

            return double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double value) &&
                   double.IsFinite(value)
                ? value
                : null;
        }
    }
}
