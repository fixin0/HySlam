using System.Collections.Generic;

namespace HySlam.Core.Slam
{
    /// <summary>
    /// Constant-velocity motion predictor used to seed optical-flow tracking and to
    /// validate planar-motion estimates on fast footage.
    ///
    /// The predictor keeps a short history of yaw and translation steps. On each
    /// processed frame it forecasts the next camera displacement. Frames whose
    /// estimated motion deviates too far from the forecast are treated as outliers
    /// instead of corrupting the trajectory.
    /// </summary>
    internal sealed class MotionPredictor
    {
        private const int HistoryLength = 8;

        private readonly Queue<double> yawHistory = new();
        private readonly Queue<double> stepXHistory = new();
        private readonly Queue<double> stepZHistory = new();

        private double predictedYaw;
        private double predictedStepX;
        private double predictedStepZ;
        private bool hasPrediction;

        public double PredictedYaw => predictedYaw;
        public double PredictedStepX => predictedStepX;
        public double PredictedStepZ => predictedStepZ;
        public bool HasPrediction => hasPrediction;

        /// <summary>
        /// Returns the forecast for the next frame before any new observation is added.
        /// </summary>
        public (double yaw, double stepX, double stepZ) Forecast()
        {
            return (predictedYaw, predictedStepX, predictedStepZ);
        }

        /// <summary>
        /// Records an observed motion step and updates the forecast. Returns true when
        /// the observation was close enough to the previous forecast to be trusted.
        /// </summary>
        public bool Observe(double yawStep, double stepX, double stepZ, double maxDeviationMeters)
        {
            bool trusted = true;

            if (hasPrediction)
            {
                double deviation = Math.Sqrt(
                    (stepX - predictedStepX) * (stepX - predictedStepX) +
                    (stepZ - predictedStepZ) * (stepZ - predictedStepZ));

                double yawDeviation = Math.Abs(WrapAngle(yawStep - predictedYaw));
                const double maxYawDeviationDeg = 20.0;

                if (deviation > maxDeviationMeters || yawDeviation > maxYawDeviationDeg)
                    trusted = false;
            }

            Push(yawHistory, yawStep);
            Push(stepXHistory, stepX);
            Push(stepZHistory, stepZ);

            predictedYaw = Average(yawHistory);
            predictedStepX = Average(stepXHistory);
            predictedStepZ = Average(stepZHistory);
            hasPrediction = true;

            return trusted;
        }

        public void Reset()
        {
            yawHistory.Clear();
            stepXHistory.Clear();
            stepZHistory.Clear();
            predictedYaw = 0.0;
            predictedStepX = 0.0;
            predictedStepZ = 0.0;
            hasPrediction = false;
        }

        private static void Push(Queue<double> queue, double value)
        {
            queue.Enqueue(value);
            if (queue.Count > HistoryLength)
                queue.Dequeue();
        }

        private static double Average(Queue<double> queue)
        {
            double sum = 0.0;
            foreach (double value in queue)
                sum += value;
            return sum / queue.Count;
        }

        /// <summary>
        /// Wraps an angle to the [-pi, pi] interval.
        /// </summary>
        private static double WrapAngle(double radians)
        {
            while (radians > Math.PI) radians -= 2.0 * Math.PI;
            while (radians < -Math.PI) radians += 2.0 * Math.PI;
            return radians;
        }
    }
}
