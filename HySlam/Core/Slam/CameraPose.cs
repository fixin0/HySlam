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

        /// <summary>
        /// Extracts the camera pitch (rotation around the camera X axis) from the
        /// rotation matrix. Positive pitch means the optical axis points down.
        /// Returns the angle in radians.
        /// </summary>
        public double ExtractPitchRadians()
        {
            // The rotation maps camera coordinates to world coordinates.
            // The camera's forward direction in world space is the third column
            // (or row, depending on convention) of the rotation matrix. Pitch is
            // derived from how much that forward vector tilts away from horizontal.
            var fwdZ = new GLVector3((float)Rotation[0, 2], (float)Rotation[1, 2], (float)Rotation[2, 2]);
            var horizontal = Math.Sqrt(fwdZ.X * fwdZ.X + fwdZ.Z * fwdZ.Z);
            if (horizontal < 1e-6) return 0.0;
            return Math.Atan2(fwdZ.Y, horizontal);
        }

        /// <summary>
        /// Extracts the camera roll (rotation around the camera Z axis / forward
        /// direction) from the rotation matrix. Returns the angle in radians.
        /// </summary>
        public double ExtractRollRadians()
        {
            // Roll is approximated from the rotation of the camera's up vector
            // around the forward axis.
            var up = new GLVector3((float)-Rotation[0, 1], (float)Rotation[1, 1], (float)-Rotation[2, 1]);
            var fwd = new GLVector3((float)Rotation[0, 2], (float)Rotation[1, 2], (float)Rotation[2, 2]);
            var right = GLVector3.Cross(fwd, up);
            if (right.LengthSquared < 1e-6) return 0.0;
            right = GLVector3.Normalize(right);
            var worldUp = GLVector3.UnitY;
            double dot = Math.Clamp(GLVector3.Dot(right, worldUp), -1.0, 1.0);
            return Math.Acos(dot);
        }

        /// <summary>
        /// Extracts the camera heading (yaw around the world Y axis) from the
        /// rotation matrix. Returns the angle in radians.
        /// </summary>
        public double ExtractYawRadians()
        {
            // Heading is the rotation of the camera forward vector around the
            // world Y axis. The forward vector's X and Z components give the yaw.
            var fwd = new GLVector3((float)Rotation[0, 2], (float)Rotation[1, 2], (float)Rotation[2, 2]);
            return Math.Atan2(fwd.X, fwd.Z);
        }
    }
}
