using OpenCvSharp;

namespace HySlam.Core.Slam
{
    /// <summary>
    /// End-to-end lane tracker for the video overlay.
    ///
    /// Detects painted left and right lane markings in the road band with Canny
    /// edges and probabilistic Hough lines, fits a quadratic curve x(y) to each
    /// boundary, smooths the coefficients across frames and draws the corridor
    /// from the bottom of the frame up to the road horizon.
    ///
    /// Returns false when painted markings are missing and no recent track is
    /// still valid, so the caller can fall back to the pose-projected corridor.
    /// </summary>
    internal sealed class LaneTracker
    {
        private const double HorizonFraction = 0.67;
        private const double MinSlope = 0.30;
        private const double MaxSlope = 4.0;
        private const double Smoothing = 0.22;
        private const int MaxHoldFrames = 24;

        private readonly double[] leftCoeffs = new double[3];
        private readonly double[] rightCoeffs = new double[3];
        private bool hasTrack;
        private int holdFrames;

        public void Reset()
        {
            Array.Clear(leftCoeffs);
            Array.Clear(rightCoeffs);
            hasTrack = false;
            holdFrames = 0;
        }

        public bool TryDraw(Mat frame)
        {
            if (frame.Empty() || frame.Channels() < 3)
                return false;

            int width = frame.Cols;
            int height = frame.Rows;
            int horizonY = Math.Clamp((int)(height * HorizonFraction), 20, height - 40);
            int bottomY = height - 1;

            bool detected = DetectBoundaries(frame, horizonY, bottomY, out var leftCandidate, out var rightCandidate);

            if (detected)
            {
                if (!hasTrack)
                {
                    Array.Copy(leftCandidate, leftCoeffs, 3);
                    Array.Copy(rightCandidate, rightCoeffs, 3);
                    hasTrack = true;
                }
                else
                {
                    for (int i = 0; i < 3; i++)
                    {
                        leftCoeffs[i] += Smoothing * (leftCandidate[i] - leftCoeffs[i]);
                        rightCoeffs[i] += Smoothing * (rightCandidate[i] - rightCoeffs[i]);
                    }
                }

                holdFrames = 0;
            }
            else if (hasTrack && holdFrames < MaxHoldFrames)
            {
                holdFrames++;
            }
            else
            {
                return false;
            }

            RenderCorridor(frame, horizonY, bottomY);
            return true;
        }

        private static bool DetectBoundaries(
            Mat frame,
            int horizonY,
            int bottomY,
            out double[] left,
            out double[] right)
        {
            left = new double[3];
            right = new double[3];

            int width = frame.Cols;
            int height = frame.Rows;
            int roiTop = Math.Max(0, horizonY - 8);
            int roiHeight = height - roiTop;
            if (roiHeight < 30)
                return false;

            using var roi = new Mat(frame, new Rect(0, roiTop, width, roiHeight));
            using var gray = new Mat();
            using var blurred = new Mat();
            using var edges = new Mat();

            Cv2.CvtColor(roi, gray, ColorConversionCodes.BGR2GRAY);
            Cv2.GaussianBlur(gray, blurred, new Size(5, 5), 1.2);
            Cv2.Canny(blurred, edges, 50, 150);

            LineSegmentPoint[] segments = Cv2.HoughLinesP(
                edges,
                1.0,
                Math.PI / 180.0,
                threshold: 22,
                minLineLength: Math.Max(18.0, height * 0.04),
                maxLineGap: Math.Max(20.0, height * 0.06));

            if (segments.Length == 0)
                return false;

            var leftSamples = new List<(double y, double x)>();
            var rightSamples = new List<(double y, double x)>();
            double midX = width * 0.5;

            foreach (var seg in segments)
            {
                double x1 = seg.P1.X;
                double y1 = seg.P1.Y + roiTop;
                double x2 = seg.P2.X;
                double y2 = seg.P2.Y + roiTop;

                double dx = x2 - x1;
                double dy = y2 - y1;
                if (Math.Abs(dx) < 2.0 || Math.Abs(dy) < 4.0)
                    continue;

                double slope = dy / dx;
                double absSlope = Math.Abs(slope);
                if (absSlope < MinSlope || absSlope > MaxSlope)
                    continue;

                // Extrapolate the segment to the bottom of the frame so we can keep
                // only the two boundaries closest to the ego lane.
                double xBottom = x1 + (bottomY - y1) / slope;

                // Left boundary slopes up-and-right in image coordinates (dy/dx < 0)
                // and meets the bottom left of centre; right boundary is the mirror.
                if (slope < 0 && xBottom > width * 0.02 && xBottom < midX - width * 0.03)
                {
                    AddSegmentSamples(leftSamples, x1, y1, x2, y2);
                }
                else if (slope > 0 && xBottom > midX + width * 0.03 && xBottom < width * 0.98)
                {
                    AddSegmentSamples(rightSamples, x1, y1, x2, y2);
                }
            }

            bool hasLeft = TryFitQuadratic(leftSamples, horizonY, bottomY, out left);
            bool hasRight = TryFitQuadratic(rightSamples, horizonY, bottomY, out right);

            if (!hasLeft && !hasRight)
                return false;

            // Synthesize the missing boundary from a typical lane width when only one
            // side has visible paint (for example a solid outer line and a faded divider).
            if (hasLeft && !hasRight)
                SynthesizeOtherSide(left, right, horizonY, bottomY, width, +1.0);
            else if (hasRight && !hasLeft)
                SynthesizeOtherSide(right, left, horizonY, bottomY, width, -1.0);

            double leftBottom = Eval(left, bottomY, horizonY, bottomY);
            double rightBottom = Eval(right, bottomY, horizonY, bottomY);
            double spanBottom = rightBottom - leftBottom;
            if (spanBottom < width * 0.22 || spanBottom > width * 0.95)
                return false;

            double leftHorizon = Eval(left, horizonY, horizonY, bottomY);
            double rightHorizon = Eval(right, horizonY, horizonY, bottomY);
            double spanHorizon = rightHorizon - leftHorizon;
            if (spanHorizon < -width * 0.04 || spanHorizon > spanBottom * 0.85)
                return false;

            return true;
        }

        private static void AddSegmentSamples(
            List<(double y, double x)> samples,
            double x1,
            double y1,
            double x2,
            double y2)
        {
            double length = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            int steps = Math.Clamp((int)(length / 8.0), 2, 12);

            for (int i = 0; i <= steps; i++)
            {
                double t = i / (double)steps;
                samples.Add((y1 + (y2 - y1) * t, x1 + (x2 - x1) * t));
            }
        }

        /// <summary>
        /// Fits x(y) = c0 + c1*u + c2*u^2 with u in [0, 1] from horizon to bottom.
        /// Uses a linear fit when the vertical span is too small to constrain c2.
        /// </summary>
        private static bool TryFitQuadratic(
            List<(double y, double x)> samples,
            int horizonY,
            int bottomY,
            out double[] coeffs)
        {
            coeffs = new double[3];
            if (samples.Count < 4)
                return false;

            double span = Math.Max(1.0, bottomY - horizonY);
            double minU = 1.0, maxU = 0.0;

            double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
            double b0 = 0, b1 = 0, b2 = 0;

            foreach (var (y, x) in samples)
            {
                double u = Math.Clamp((y - horizonY) / span, 0.0, 1.0);
                minU = Math.Min(minU, u);
                maxU = Math.Max(maxU, u);

                double u2 = u * u;
                s0 += 1.0;
                s1 += u;
                s2 += u2;
                s3 += u2 * u;
                s4 += u2 * u2;

                b0 += x;
                b1 += x * u;
                b2 += x * u2;
            }

            // Linear fallback when samples cover too little of the vertical range.
            if (maxU - minU < 0.35 || samples.Count < 8)
            {
                double det2 = s0 * s2 - s1 * s1;
                if (Math.Abs(det2) < 1e-6)
                    return false;

                coeffs[0] = (b0 * s2 - b1 * s1) / det2;
                coeffs[1] = (s0 * b1 - s1 * b0) / det2;
                coeffs[2] = 0.0;
                return double.IsFinite(coeffs[0]) && double.IsFinite(coeffs[1]);
            }

            // Small ridge term on c2 so noisy far-field edges cannot bend the lane wildly.
            s4 += 0.25;

            if (!Solve3x3(
                    s0, s1, s2, b0,
                    s1, s2, s3, b1,
                    s2, s3, s4, b2,
                    out coeffs[0], out coeffs[1], out coeffs[2]))
            {
                return false;
            }

            coeffs[2] = Math.Clamp(coeffs[2], -120.0, 120.0);
            return double.IsFinite(coeffs[0]) &&
                   double.IsFinite(coeffs[1]) &&
                   double.IsFinite(coeffs[2]);
        }

        private static void SynthesizeOtherSide(
            double[] known,
            double[] target,
            int horizonY,
            int bottomY,
            int width,
            double direction)
        {
            double nearWidth = width * 0.56 * direction;
            double farWidth = width * 0.06 * direction;

            // Linear width taper u -> farWidth + (nearWidth - farWidth) * u.
            target[0] = known[0] + farWidth;
            target[1] = known[1] + (nearWidth - farWidth);
            target[2] = known[2];
        }

        private void RenderCorridor(Mat frame, int horizonY, int bottomY)
        {
            var left = new List<Point>();
            var right = new List<Point>();
            var centre = new List<Point>();

            const int samples = 36;
            for (int i = 0; i <= samples; i++)
            {
                double u = i / (double)samples;
                int y = (int)Math.Round(horizonY + (bottomY - horizonY) * u);

                double xl = Eval(leftCoeffs, y, horizonY, bottomY);
                double xr = Eval(rightCoeffs, y, horizonY, bottomY);
                if (xr <= xl + 4.0)
                    continue;

                int xLeft = (int)Math.Round(xl);
                int xRight = (int)Math.Round(xr);
                int xCentre = (int)Math.Round((xl + xr) * 0.5);

                left.Add(new Point(xLeft, y));
                right.Add(new Point(xRight, y));
                centre.Add(new Point(xCentre, y));
            }

            if (centre.Count < 2)
                return;

            var green = new Scalar(0, 255, 0);
            var darkGreen = new Scalar(0, 190, 0);

            var polygon = new List<Point>(left.Count + right.Count);
            polygon.AddRange(left);
            for (int i = right.Count - 1; i >= 0; i--)
                polygon.Add(right[i]);

            using (var overlay = frame.Clone())
            {
                Cv2.FillPoly(overlay, new[] { polygon.ToArray() }, green);
                Cv2.AddWeighted(overlay, 0.18, frame, 0.82, 0, frame);
            }

            Cv2.Polylines(frame, new[] { left.ToArray() }, false, darkGreen, 2);
            Cv2.Polylines(frame, new[] { right.ToArray() }, false, darkGreen, 2);
            Cv2.Polylines(frame, new[] { centre.ToArray() }, false, green, 3);

            var anchor = centre[Math.Min(centre.Count - 1, (int)(centre.Count * 0.65))];
            var origin = new Point(
                Math.Clamp(anchor.X - 60, 4, frame.Cols - 140),
                Math.Clamp(anchor.Y - 16, 20, frame.Rows - 12));
            Cv2.PutText(frame, "ACTIVE LANE", origin, HersheyFonts.HersheySimplex, 0.6, green, 2);
        }

        private static double Eval(double[] coeffs, double y, int horizonY = 0, int bottomY = 1)
        {
            double span = Math.Max(1.0, bottomY - horizonY);
            double u = (y - horizonY) / span;
            return coeffs[0] + coeffs[1] * u + coeffs[2] * u * u;
        }

        private static bool Solve3x3(
            double a00, double a01, double a02, double b0,
            double a10, double a11, double a12, double b1,
            double a20, double a21, double a22, double b2,
            out double x0, out double x1, out double x2)
        {
            x0 = x1 = x2 = 0.0;

            double det = a00 * (a11 * a22 - a12 * a21)
                       - a01 * (a10 * a22 - a12 * a20)
                       + a02 * (a10 * a21 - a11 * a20);
            if (Math.Abs(det) < 1e-8)
                return false;

            double det0 = b0 * (a11 * a22 - a12 * a21)
                        - a01 * (b1 * a22 - a12 * b2)
                        + a02 * (b1 * a21 - a11 * b2);
            double det1 = a00 * (b1 * a22 - a12 * b2)
                        - b0 * (a10 * a22 - a12 * a20)
                        + a02 * (a10 * b2 - b1 * a20);
            double det2 = a00 * (a11 * b2 - b1 * a21)
                        - a01 * (a10 * b2 - b1 * a20)
                        + b0 * (a10 * a21 - a11 * a20);

            x0 = det0 / det;
            x1 = det1 / det;
            x2 = det2 / det;
            return true;
        }
    }
}
