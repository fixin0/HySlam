using System.Globalization;
using System.Text;
using HySlam.Core.Data;
using HySlam.Core.Slam;
using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Diagnostics
{
    /// <summary>
    /// Writes per-frame tracking metrics and the final map/trajectory as CSV files.
    /// When no output directory is configured the recorder is a no-op.
    /// </summary>
    internal sealed class RunRecorder : IDisposable
    {
        private readonly string? directory;
        private StreamWriter? metrics;
        private bool disposed;

        public RunRecorder(string? directory)
        {
            if (string.IsNullOrWhiteSpace(directory))
                return;

            this.directory = directory;
            Directory.CreateDirectory(directory);

            metrics = new StreamWriter(
                Path.Combine(directory, "metrics.csv"),
                append: false,
                new UTF8Encoding(encoderShouldEmitUTF8Identifier: false))
            {
                AutoFlush = true
            };

            metrics.WriteLine("frame,matches,inliers,inlier_ratio,triangulated,total_points,keyframe_id,x,y,z");
        }

        public bool Enabled => metrics is not null;

        public void Record(int frame, SlamEngine slam, MapData map, int keyframeId)
        {
            if (metrics is null)
                return;

            var position = slam.Pose.GetCameraPositionGL();
            double inlierRatio = slam.MatchCount > 0 ? slam.InlierCount / (double)slam.MatchCount : 0.0;

            metrics.WriteLine(string.Join(',',
                frame.ToString(CultureInfo.InvariantCulture),
                slam.MatchCount.ToString(CultureInfo.InvariantCulture),
                slam.InlierCount.ToString(CultureInfo.InvariantCulture),
                inlierRatio.ToString("F4", CultureInfo.InvariantCulture),
                slam.TriangulatedCount.ToString(CultureInfo.InvariantCulture),
                map.Points.Count.ToString(CultureInfo.InvariantCulture),
                keyframeId.ToString(CultureInfo.InvariantCulture),
                F(position.X),
                F(position.Y),
                F(position.Z)));
        }

        public void Finish(
            IReadOnlyList<MapPoint> points,
            IReadOnlyList<GLVector3> trajectory,
            string videoPath,
            int framesRead,
            int framesProcessed,
            int maxFrames,
            bool deterministic,
            double fps)
        {
            if (directory is null || disposed)
                return;

            metrics?.Dispose();
            metrics = null;

            using (var writer = new StreamWriter(
                       Path.Combine(directory, "points.csv"),
                       append: false,
                       new UTF8Encoding(encoderShouldEmitUTF8Identifier: false)))
            {
                writer.WriteLine("x,y,z,r,g,b");
                foreach (var point in points)
                {
                    var p = point.Position;
                    writer.WriteLine(string.Join(',',
                        F(p.X), F(p.Y), F(p.Z),
                        F(point.Color.X), F(point.Color.Y), F(point.Color.Z)));
                }
            }

            using (var writer = new StreamWriter(
                       Path.Combine(directory, "trajectory.csv"),
                       append: false,
                       new UTF8Encoding(encoderShouldEmitUTF8Identifier: false)))
            {
                writer.WriteLine("x,y,z");
                foreach (var position in trajectory)
                    writer.WriteLine(string.Join(',', F(position.X), F(position.Y), F(position.Z)));
            }

            using (var writer = new StreamWriter(
                       Path.Combine(directory, "summary.txt"),
                       append: false,
                       new UTF8Encoding(encoderShouldEmitUTF8Identifier: false)))
            {
                writer.WriteLine($"video={videoPath}");
                writer.WriteLine($"frames_read={framesRead}");
                writer.WriteLine($"frames_processed={framesProcessed}");
                writer.WriteLine($"max_frames={maxFrames}");
                writer.WriteLine($"deterministic={deterministic}");
                writer.WriteLine($"points={points.Count}");
                writer.WriteLine($"trajectory_points={trajectory.Count}");

                double path = 0.0;
                for (int i = 1; i < trajectory.Count; i++)
                    path += (trajectory[i] - trajectory[i - 1]).Length;

                double span = trajectory.Count > 1
                    ? (trajectory[^1] - trajectory[0]).Length
                    : 0.0;

                double duration = fps > 0 ? framesRead / fps : 0.0;

                writer.WriteLine($"path_meters={F((float)path)}");
                writer.WriteLine($"span_meters={F((float)span)}");
                writer.WriteLine($"duration_seconds={F((float)duration)}");
                writer.WriteLine($"average_speed_kmh={(duration > 0 ? F((float)(path / duration * 3.6)) : "0.0000")}");
            }

            disposed = true;
        }

        public void Dispose()
        {
            metrics?.Dispose();
            metrics = null;
            disposed = true;
        }

        private static string F(float value) => value.ToString("F4", CultureInfo.InvariantCulture);
    }
}
