using System.Collections.Generic;
using OpenCvSharp;
using HySlam.Core.Data;
using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Slam
{
    /// <summary>
    /// Lightweight local bundle adjustment.
    ///
    /// Given a sliding window of recent keyframes, this alternates between:
    /// 1. Refining each keyframe's camera pose with solvePnP against observed 3D map
    ///    points (pose refinement).
    /// 2. Refining each 3D point's position by multi-view triangulation from the
    ///    updated keyframe poses (point refinement).
    ///
    /// This reduces drift inside the window and makes the map more consistent without
    /// requiring a full sparse linear solver.
    /// </summary>
    internal sealed class LocalBundleAdjuster
    {
        private const int MaxWindowKeyframes = 8;
        private const int MaxIterations = 3;
        private const double MaxReprojErrPx = 2.0;
        private const double MinPointDepth = 0.5;
        private const double MaxPointDepth = 80.0;
        private const double MaxProjSearchPx = 10.0;

        private readonly SlamEngine engine;
        private readonly int imageWidth;
        private readonly int imageHeight;
        private readonly Mat cameraMatrix = new(3, 3, MatType.CV_64FC1);
        private readonly Mat distCoeffs = new(1, 5, MatType.CV_64FC1, Scalar.All(0));

        private readonly double fx, fy, cx, cy;

        public LocalBundleAdjuster(SlamEngine engine, int imageWidth, int imageHeight,
            double fx, double fy, double cx, double cy)
        {
            this.engine = engine;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            cameraMatrix.Set(0, 0, fx);
            cameraMatrix.Set(1, 1, fy);
            cameraMatrix.Set(0, 2, cx);
            cameraMatrix.Set(1, 2, cy);
            cameraMatrix.Set(2, 2, 1.0);
        }

        /// <summary>
        /// Runs local bundle adjustment over the most recent keyframes.
        /// Returns the number of keyframes whose pose was refined.
        /// </summary>
        public int Adjust(IReadOnlyList<Keyframe> window, MapData map)
        {
            if (window.Count < 2 || map.Points.Count == 0)
                return 0;

            int refinedKeyframes = 0;

            for (int iteration = 0; iteration < MaxIterations; iteration++)
            {
                // 1. Refine each keyframe pose using solvePnP against observed 3D points.
                foreach (var keyframe in window)
                {
                    if (RefineKeyframePose(keyframe, map))
                        refinedKeyframes++;
                }

                // 2. Refine 3D point positions by re-triangulating from keyframe pairs.
                RefineMapPoints(window, map);
            }

            return refinedKeyframes;
        }

        private bool RefineKeyframePose(Keyframe keyframe, MapData map)
        {
            var objectPoints = new List<OpenCvSharp.Point3f>();
            var imagePoints = new List<OpenCvSharp.Point2f>();

            var kfCenter = keyframe.Pose.GetCameraPositionGL();

            foreach (var point in map.Points)
            {
                // Only consider points reasonably close to this keyframe.
                var d = point.Position - kfCenter;
                if (d.LengthSquared > MaxPointDepth * MaxPointDepth)
                    continue;

                // Project the 3D point into the keyframe image.
                var projected = ProjectPoint(point.Position, keyframe.Pose);
                if (!projected.HasValue)
                    continue;

                var proj = projected.Value;
                if (proj.X < 5 || proj.Y < 5 || proj.X >= imageWidth - 5 || proj.Y >= imageHeight - 5)
                    continue;

                // Find nearest keyframe feature.
                int nearest = FindNearestFeature(keyframe.Points2D, proj);
                if (nearest < 0)
                    continue;

                objectPoints.Add(new OpenCvSharp.Point3f((float)point.Position.X, (float)point.Position.Y, (float)point.Position.Z));
                imagePoints.Add(keyframe.Points2D[nearest]);
            }

            if (objectPoints.Count < 8)
                return false;

            using var objMat = Mat.FromArray(objectPoints.ToArray());
            using var imgMat = Mat.FromArray(imagePoints.ToArray());
            using var rvec = new Mat();
            using var tvec = new Mat();

            // SolvePnP returns void in OpenCvSharp; a valid tvec indicates success.
            Cv2.SolvePnP(
                objMat, imgMat, cameraMatrix, distCoeffs, rvec, tvec,
                false, SolvePnPMethod.Iterative);

            if (tvec.Empty() || tvec.Rows < 1)
                return false;

            // Convert rotation vector to rotation matrix.
            using var rotMat = new Mat();
            Cv2.Rodrigues(rvec, rotMat);

            var newPose = new CameraPose
            {
                Rotation = new double[,]
                {
                    { rotMat.Get<double>(0, 0), rotMat.Get<double>(0, 1), rotMat.Get<double>(0, 2) },
                    { rotMat.Get<double>(1, 0), rotMat.Get<double>(1, 1), rotMat.Get<double>(1, 2) },
                    { rotMat.Get<double>(2, 0), rotMat.Get<double>(2, 1), rotMat.Get<double>(2, 2) },
                },
                Translation = new[] { tvec.Get<double>(0, 0), tvec.Get<double>(1, 0), tvec.Get<double>(2, 0) },
            };

            keyframe.Pose = newPose;
            return true;
        }

        private void RefineMapPoints(IReadOnlyList<Keyframe> window, MapData map)
        {
            // For each map point observed by at least 2 keyframes in the window,
            // re-triangulate from the best baseline pair.
            foreach (var point in map.Points)
            {
                if (point.Observations < 2)
                    continue;

                // Find keyframes in the window that could observe this point.
                var observers = new List<(Keyframe kf, OpenCvSharp.Point2f pt)>();
                foreach (var kf in window)
                {
                    var projected = ProjectPoint(point.Position, kf.Pose);
                    if (!projected.HasValue)
                        continue;

                    var proj = projected.Value;
                    int nearest = FindNearestFeature(kf.Points2D, proj);
                    if (nearest >= 0)
                        observers.Add((kf, kf.Points2D[nearest]));
                }

                if (observers.Count < 2)
                    continue;

                // Use the pair with the largest baseline.
                var best = FindBestBaseline(observers);
                if (best is null)
                    continue;

                var (kf1, kf2, pt1, pt2) = best.Value;

                var triangulated = TriangulatePoint(kf1.Pose, pt1, kf2.Pose, pt2);
                if (!triangulated.HasValue)
                    continue;

                var p3d = triangulated.Value;
                if (!double.IsFinite(p3d.X) || !double.IsFinite(p3d.Y) || !double.IsFinite(p3d.Z))
                    continue;
                if (p3d.Z < MinPointDepth || p3d.Z > MaxPointDepth)
                    continue;

                // Update the point position (average with current to avoid oscillation).
                point.Position = new GLVector3(
                    (float)((point.Position.X + p3d.X) * 0.5),
                    (float)((point.Position.Y + p3d.Y) * 0.5),
                    (float)((point.Position.Z + p3d.Z) * 0.5));
            }
        }

        private OpenCvSharp.Point2f? ProjectPoint(
            GLVector3 point,
            CameraPose pose)
        {
            // Transform point to camera coordinates: X_cam = R^T * (X_world - t)
            var R = pose.Rotation;
            var t = pose.Translation;

            double xc = R[0, 0] * (point.X - t[0]) + R[1, 0] * (point.Y - t[1]) + R[2, 0] * (point.Z - t[2]);
            double yc = R[0, 1] * (point.X - t[0]) + R[1, 1] * (point.Y - t[1]) + R[2, 1] * (point.Z - t[2]);
            double zc = R[0, 2] * (point.X - t[0]) + R[1, 2] * (point.Y - t[1]) + R[2, 2] * (point.Z - t[2]);

            if (zc <= 1e-6)
                return null;

            double u = cameraMatrix.Get<double>(0, 0) * xc / zc + cameraMatrix.Get<double>(0, 2);
            double v = cameraMatrix.Get<double>(1, 1) * yc / zc + cameraMatrix.Get<double>(1, 2);

            return new OpenCvSharp.Point2f((float)u, (float)v);
        }

        private static int FindNearestFeature(List<OpenCvSharp.Point2f> features, OpenCvSharp.Point2f projection)
        {
            int nearest = -1;
            double bestDistSq = double.MaxValue;

            for (int i = 0; i < features.Count; i++)
            {
                double dx = features[i].X - projection.X;
                double dy = features[i].Y - projection.Y;
                double distSq = dx * dx + dy * dy;
                if (distSq < bestDistSq)
                {
                    bestDistSq = distSq;
                    nearest = i;
                }
            }

            // Only accept features within MaxProjSearchPx pixels of the projection.
            return bestDistSq <= MaxProjSearchPx * MaxProjSearchPx ? nearest : -1;
        }

        private static (Keyframe kf1, Keyframe kf2, OpenCvSharp.Point2f pt1, OpenCvSharp.Point2f pt2)?
            FindBestBaseline(List<(Keyframe kf, OpenCvSharp.Point2f pt)> observers)
        {
            (Keyframe, Keyframe, OpenCvSharp.Point2f, OpenCvSharp.Point2f)? best = null;
            double bestBaseline = 0.0;

            for (int i = 0; i < observers.Count; i++)
            {
                for (int j = i + 1; j < observers.Count; j++)
                {
                    var c1 = observers[i].kf.Pose.GetCameraPositionGL();
                    var c2 = observers[j].kf.Pose.GetCameraPositionGL();
                    var d = c2 - c1;
                    double baseline = d.LengthSquared;
                    if (baseline > bestBaseline)
                    {
                        bestBaseline = baseline;
                        best = (observers[i].kf, observers[j].kf, observers[i].pt, observers[j].pt);
                    }
                }
            }

            return best;
        }

        private (double X, double Y, double Z)? TriangulatePoint(
            CameraPose pose1, OpenCvSharp.Point2f pt1,
            CameraPose pose2, OpenCvSharp.Point2f pt2)
        {
            var proj1 = BuildProjectionMatrix(pose1, fx, fy, cx, cy);
            var proj2 = BuildProjectionMatrix(pose2, fx, fy, cx, cy);

            using var p1Mat = Mat.FromArray(new[] { pt1 });
            using var p2Mat = Mat.FromArray(new[] { pt2 });
            using var points4D = new Mat();

            Cv2.TriangulatePoints(proj1, proj2, p1Mat, p2Mat, points4D);

            if (points4D.Rows < 4 || points4D.Cols < 1)
                return null;

            float w = points4D.At<float>(3, 0);
            if (Math.Abs(w) < 1e-5f)
                return null;

            double x = points4D.At<float>(0, 0) / w;
            double y = points4D.At<float>(1, 0) / w;
            double z = points4D.At<float>(2, 0) / w;

            return (x, y, z);
        }

        private static Mat BuildProjectionMatrix(CameraPose pose, double fx, double fy, double cx, double cy)
        {
            // World-to-camera rotation = transpose of camera-to-world rotation.
            // X_cam = R^T * X_world - R^T * t
            var R = pose.Rotation;
            var t = pose.Translation;

            // R^T elements: R^T[i,j] = R[j,i]
            double r00 = R[0, 0], r01 = R[1, 0], r02 = R[2, 0];
            double r10 = R[0, 1], r11 = R[1, 1], r12 = R[2, 1];
            double r20 = R[0, 2], r21 = R[1, 2], r22 = R[2, 2];

            // t' = -R^T * t
            double tx = -(r00 * t[0] + r01 * t[1] + r02 * t[2]);
            double ty = -(r10 * t[0] + r11 * t[1] + r12 * t[2]);
            double tz = -(r20 * t[0] + r21 * t[1] + r22 * t[2]);

            var proj = new double[3, 4]
            {
                { fx * r00 + cx * r20, fx * r10 + cx * r21, fx * r20 + cx * r22, fx * tx + cx * tz },
                { fy * r01 + cy * r20, fy * r11 + cy * r21, fy * r21 + cy * r22, fy * ty + cy * tz },
                { r02,                   r12,                   r22,                   tz },
            };

            var mat = new Mat(3, 4, MatType.CV_64FC1);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 4; j++)
                    mat.Set(i, j, proj[i, j]);

            return mat;
        }
    }
}
