using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Slam
{
    internal sealed class CameraPose
    {
        // Camera-to-world rotation (the stored form used by triangulation and rendering).
        public double[,] Rotation = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        // Camera center in the world frame.
        public double[] Translation = { 0, 0, 0 };

        public CameraPose Clone()
        {
            return new CameraPose
            {
                Rotation = (double[,])Rotation.Clone(),
                Translation = (double[])Translation.Clone()
            };
        }

        public GLVector3 GetCameraPositionGL()
        {
            return new GLVector3((float)Translation[0], (float)-Translation[1], (float)-Translation[2]);
        }

        /// <summary>
        /// Composes the current pose with a relative motion (X_rel = R_rel * X_this + t_rel).
        /// The resulting rotation stays in camera-to-world form so triangulation and rendering
        /// can consume it directly.
        /// </summary>
        public void UpdateFromRelative(CameraPose keyframePose, double[,] R_rel, double[] t_rel)
        {
            var R_kf = keyframePose.Rotation;
            var t_kf = keyframePose.Translation;

            var newR = new double[3, 3];
            var newT = new double[3];

            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    for (int k = 0; k < 3; k++)
                        newR[r, c] += R_kf[r, k] * R_rel[c, k];

            for (int r = 0; r < 3; r++)
            {
                newT[r] = t_kf[r];
                for (int k = 0; k < 3; k++)
                    newT[r] -= newR[r, k] * t_rel[k];
            }

            Rotation = newR;
            Translation = newT;
        }
    }
}
