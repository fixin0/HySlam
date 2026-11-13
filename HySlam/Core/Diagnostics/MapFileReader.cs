using System.Globalization;
using HySlam.Core.Data;
using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Diagnostics
{
    /// <summary>
    /// Counts and lengths measured from a loaded map directory.
    /// <see cref="ConfirmedPoints"/> is null when points.csv has no observations
    /// column, because the information cannot be recovered from the file.
    /// </summary>
    internal sealed class MapFileInfo
    {
        public required int PointCount { get; init; }
        public required int? ConfirmedPoints { get; init; }
        public required int TrajectoryCount { get; init; }
        public required double PathMeters { get; init; }
        public required double SpanMeters { get; init; }
    }

    /// <summary>
    /// Reads points.csv and trajectory.csv back into a <see cref="MapData"/>.
    ///
    /// The files written by --dump and --bake are accepted as-is: a point row is
    /// x,y,z with optional r,g,b and an optional observations column. Missing
    /// colors default to white. Every value must use the invariant culture and be
    /// finite; a malformed row reports the file name and line number.
    /// </summary>
    internal static class MapFileReader
    {
        public static MapData Read(string directory) => Read(directory, out _);

        public static MapData Read(string directory, out MapFileInfo info)
        {
            if (string.IsNullOrWhiteSpace(directory))
                throw new InvalidDataException("A map directory is required.");

            directory = Path.GetFullPath(directory);
            if (!Directory.Exists(directory))
                throw new InvalidDataException($"Map directory not found: {directory}");

            string pointsPath = Path.Combine(directory, "points.csv");
            string trajectoryPath = Path.Combine(directory, "trajectory.csv");

            if (!File.Exists(pointsPath))
                throw new InvalidDataException($"points.csv is missing in {directory}.");
            if (!File.Exists(trajectoryPath))
                throw new InvalidDataException($"trajectory.csv is missing in {directory}.");

            var map = new MapData();
            bool hasObservations = ReadPoints(pointsPath, map);
            ReadTrajectory(trajectoryPath, map);

            int? confirmed = hasObservations
                ? map.Points.Count(point => point.Observations >= 2)
                : null;

            double pathMeters = 0.0;
            for (int i = 1; i < map.Trajectory.Count; i++)
                pathMeters += (map.Trajectory[i] - map.Trajectory[i - 1]).Length;

            double spanMeters = map.Trajectory.Count > 1
                ? (map.Trajectory[^1] - map.Trajectory[0]).Length
                : 0.0;

            info = new MapFileInfo
            {
                PointCount = map.Points.Count,
                ConfirmedPoints = confirmed,
                TrajectoryCount = map.Trajectory.Count,
                PathMeters = pathMeters,
                SpanMeters = spanMeters
            };

            return map;
        }

        private static bool ReadPoints(string path, MapData map)
        {
            using var reader = new StreamReader(path);
            string? headerLine = ReadNonEmptyLine(reader);
            if (headerLine is null)
                throw new InvalidDataException($"{Path.GetFileName(path)} is empty.");

            string[] header = ParseHeader(headerLine);
            int x = RequiredColumn(header, "x", path);
            int y = RequiredColumn(header, "y", path);
            int z = RequiredColumn(header, "z", path);
            int r = OptionalColumn(header, "r");
            int g = OptionalColumn(header, "g");
            int b = OptionalColumn(header, "b");
            int observations = OptionalColumn(header, "observations");
            int normalX = OptionalColumn(header, "normal_x");
            int normalY = OptionalColumn(header, "normal_y");
            int normalZ = OptionalColumn(header, "normal_z");
            int confidence = OptionalColumn(header, "confidence");
            int semanticLabel = OptionalColumn(header, "semantic_label");

            bool hasColor = r >= 0 && g >= 0 && b >= 0;
            bool hasObservations = observations >= 0;
            bool hasNormal = normalX >= 0 && normalY >= 0 && normalZ >= 0;
            bool hasConfidence = confidence >= 0;

            int lineNumber = 1;
            string? line;
            while ((line = reader.ReadLine()) is not null)
            {
                lineNumber++;
                if (string.IsNullOrWhiteSpace(line))
                    continue;

                string[] cells = line.Split(',');
                var position = new GLVector3(
                    ParseFloat(cells, x, path, lineNumber, "x"),
                    ParseFloat(cells, y, path, lineNumber, "y"),
                    ParseFloat(cells, z, path, lineNumber, "z"));

                var color = hasColor
                    ? new GLVector3(
                        ParseFloat(cells, r, path, lineNumber, "r"),
                        ParseFloat(cells, g, path, lineNumber, "g"),
                        ParseFloat(cells, b, path, lineNumber, "b"))
                    : new GLVector3(1f, 1f, 1f);

                var normal = hasNormal
                    ? new GLVector3(
                        ParseFloat(cells, normalX, path, lineNumber, "normal_x"),
                        ParseFloat(cells, normalY, path, lineNumber, "normal_y"),
                        ParseFloat(cells, normalZ, path, lineNumber, "normal_z"))
                    : GLVector3.Zero;

                float conf = hasConfidence
                    ? ParseConfidence(cells, confidence, path, lineNumber)
                    : 0.0f;

                string label = semanticLabel >= 0 && semanticLabel < cells.Length
                    ? cells[semanticLabel].Trim()
                    : string.Empty;

                int count = hasObservations
                    ? ParseObservationCount(cells, observations, path, lineNumber)
                    : 1;

                map.Points.Add(new MapPoint
                {
                    Position = position,
                    Color = color,
                    Observations = count,
                    Normal = normal,
                    Confidence = conf,
                    SemanticLabel = label,
                });
            }

            return hasObservations;
        }

        private static void ReadTrajectory(string path, MapData map)
        {
            using var reader = new StreamReader(path);
            string? headerLine = ReadNonEmptyLine(reader);
            if (headerLine is null)
                throw new InvalidDataException($"{Path.GetFileName(path)} is empty.");

            string[] header = ParseHeader(headerLine);
            int x = RequiredColumn(header, "x", path);
            int y = RequiredColumn(header, "y", path);
            int z = RequiredColumn(header, "z", path);

            int lineNumber = 1;
            string? line;
            while ((line = reader.ReadLine()) is not null)
            {
                lineNumber++;
                if (string.IsNullOrWhiteSpace(line))
                    continue;

                string[] cells = line.Split(',');
                map.Trajectory.Add(new GLVector3(
                    ParseFloat(cells, x, path, lineNumber, "x"),
                    ParseFloat(cells, y, path, lineNumber, "y"),
                    ParseFloat(cells, z, path, lineNumber, "z")));
            }
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

        private static string[] ParseHeader(string line)
        {
            string[] cells = line.Split(',');
            for (int i = 0; i < cells.Length; i++)
            {
                string cell = cells[i].Trim();
                if (i == 0)
                    cell = cell.TrimStart('\uFEFF');

                cells[i] = cell.ToLowerInvariant();
            }

            return cells;
        }

        private static int RequiredColumn(string[] header, string name, string path)
        {
            int index = Array.FindIndex(header, cell => cell == name);
            if (index < 0)
            {
                throw new InvalidDataException(
                    $"{Path.GetFileName(path)} is missing the required '{name}' column; " +
                    $"found: {string.Join(", ", header)}.");
            }

            return index;
        }

        private static int OptionalColumn(string[] header, string name)
        {
            return Array.FindIndex(header, cell => cell == name);
        }

        private static float ParseFloat(
            string[] cells,
            int index,
            string path,
            int lineNumber,
            string column)
        {
            if (index >= cells.Length)
            {
                throw new InvalidDataException(
                    $"{Path.GetFileName(path)} line {lineNumber} is missing the '{column}' value.");
            }

            string text = cells[index].Trim();
            if (!float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out float value) ||
                !float.IsFinite(value))
            {
                throw new InvalidDataException(
                    $"{Path.GetFileName(path)} line {lineNumber}: '{column}' is not a finite number ('{text}').");
            }

            return value;
        }

        private static int ParseObservationCount(
            string[] cells,
            int index,
            string path,
            int lineNumber)
        {
            if (index >= cells.Length)
            {
                throw new InvalidDataException(
                    $"{Path.GetFileName(path)} line {lineNumber} is missing the 'observations' value.");
            }

            string text = cells[index].Trim();
            if (!int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int value) ||
                value < 0)
            {
                throw new InvalidDataException(
                    $"{Path.GetFileName(path)} line {lineNumber}: 'observations' is not a non-negative integer ('{text}').");
            }

            return value;
        }

        private static float ParseConfidence(
            string[] cells,
            int index,
            string path,
            int lineNumber)
        {
            if (index >= cells.Length)
            {
                throw new InvalidDataException(
                    $"{Path.GetFileName(path)} line {lineNumber} is missing the 'confidence' value.");
            }

            string text = cells[index].Trim();
            if (!float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out float value) ||
                !float.IsFinite(value) || value < 0.0f || value > 1.0f)
            {
                throw new InvalidDataException(
                    $"{Path.GetFileName(path)} line {lineNumber}: 'confidence' must be in [0, 1] ('{text}').");
            }

            return value;
        }
    }
}
