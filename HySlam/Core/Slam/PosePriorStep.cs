namespace HySlam.Core.Slam
{
    /// <summary>
    /// One expected motion step recorded by a previous bake pass. The values use the
    /// same units as the planar motion estimate: yaw in radians, translation in meters
    /// in the camera frame.
    /// </summary>
    internal readonly struct PosePriorStep
    {
        public PosePriorStep(double yawStep, double stepX, double stepZ)
        {
            YawStep = yawStep;
            StepX = stepX;
            StepZ = stepZ;
        }

        public double YawStep { get; }
        public double StepX { get; }
        public double StepZ { get; }
    }
}
