using System.Globalization;

namespace HySlam.Core.Slam
{
    /// <summary>
    /// Pinhole camera intrinsics and distortion coefficients. Values are loaded from a
    /// simple key=value text file so the pipeline can run with real calibration data.
    /// </summary>
    internal sealed class CameraCalibration
    {
        public double Fx { get; private set; }
        public double Fy { get; private set; }
        public double Cx { get; private set; }
        public double Cy { get; private set; }

        // OpenCV order: k1, k2, p1, p2, k3.
        public double[] Distortion { get; } = new double[5];

        // Resolution the intrinsics were calibrated for. Zero means "same as the video".
        public double SourceWidth { get; private set; }
        public double SourceHeight { get; private set; }

        public bool HasDistortion
        {
            get
            {
                foreach (double value in Distortion)
                {
                    if (Math.Abs(value) > 1e-12) return true;
                }

                return false;
            }
        }

        /// <summary>
        /// Fallback intrinsics used when no calibration file is provided. Without an
        /// explicit field of view the focal length is estimated from the frame width,
        /// matching the original pipeline. A positive <paramref name="horizontalFovDegrees"/>
        /// derives the focal length from the requested horizontal field of view instead.
        /// </summary>
        public static CameraCalibration Estimate(double width, double height, double horizontalFovDegrees = 0.0)
        {
            double focal = width * 1.1;

            if (horizontalFovDegrees > 0.0 && horizontalFovDegrees < 180.0)
            {
                double halfAngle = horizontalFovDegrees * Math.PI / 360.0;
                focal = width / (2.0 * Math.Tan(halfAngle));
            }

            return new CameraCalibration
            {
                Fx = focal,
                Fy = focal,
                Cx = width * 0.5,
                Cy = height * 0.5,
                SourceWidth = width,
                SourceHeight = height
            };
        }

        /// <summary>
        /// Loads a calibration file. Expected keys: fx, fy, cx, cy (required) and
        /// width, height, k1, k2, p1, p2, k3 (optional). Lines starting with # or // are
        /// ignored.
        /// </summary>
        public static CameraCalibration FromFile(string path)
        {
            if (!File.Exists(path))
                throw new FileNotFoundException($"Calibration file not found: {path}", path);

            var values = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
            int lineNumber = 0;

            foreach (string rawLine in File.ReadLines(path))
            {
                lineNumber++;
                string line = rawLine.Trim();

                if (line.Length == 0 || line.StartsWith('#') || line.StartsWith("//"))
                    continue;

                int separator = line.IndexOf('=');
                if (separator <= 0)
                    throw new FormatException($"Calibration line {lineNumber} is not key=value: {rawLine}");

                string key = line[..separator].Trim();
                string value = line[(separator + 1)..].Trim();

                if (key.Length == 0)
                    throw new FormatException($"Calibration line {lineNumber} has an empty key.");

                if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out double parsed))
                    throw new FormatException($"Calibration line {lineNumber} has an invalid value: {rawLine}");

                if (values.ContainsKey(key))
                    throw new FormatException($"Calibration key '{key}' is defined more than once.");

                values[key] = parsed;
            }

            double Get(string key)
            {
                if (!values.TryGetValue(key, out double value))
                    throw new FormatException($"Calibration file is missing required key '{key}'.");

                return value;
            }

            var calibration = new CameraCalibration
            {
                Fx = Get("fx"),
                Fy = Get("fy"),
                Cx = Get("cx"),
                Cy = Get("cy")
            };

            if (calibration.Fx <= 0 || calibration.Fy <= 0)
                throw new FormatException("Calibration focal lengths (fx, fy) must be positive.");

            if (values.TryGetValue("width", out double width))
                calibration.SourceWidth = width;

            if (values.TryGetValue("height", out double height))
                calibration.SourceHeight = height;

            string[] distortionKeys = { "k1", "k2", "p1", "p2", "k3" };
            for (int i = 0; i < distortionKeys.Length; i++)
            {
                if (values.TryGetValue(distortionKeys[i], out double value))
                    calibration.Distortion[i] = value;
            }

            return calibration;
        }

        /// <summary>
        /// Returns a copy scaled to the actual video resolution. Intrinsics without a
        /// source resolution are assumed to already match the video.
        /// </summary>
        public CameraCalibration ScaledTo(double width, double height)
        {
            if (SourceWidth <= 0 || SourceHeight <= 0)
                return Clone();

            double sx = width / SourceWidth;
            double sy = height / SourceHeight;

            var scaled = Clone();
            scaled.Fx = Fx * sx;
            scaled.Fy = Fy * sy;
            scaled.Cx = Cx * sx;
            scaled.Cy = Cy * sy;
            scaled.SourceWidth = width;
            scaled.SourceHeight = height;
            return scaled;
        }

        public CameraCalibration Clone()
        {
            var clone = new CameraCalibration
            {
                Fx = Fx,
                Fy = Fy,
                Cx = Cx,
                Cy = Cy,
                SourceWidth = SourceWidth,
                SourceHeight = SourceHeight
            };

            Array.Copy(Distortion, clone.Distortion, Distortion.Length);
            return clone;
        }
    }
}
