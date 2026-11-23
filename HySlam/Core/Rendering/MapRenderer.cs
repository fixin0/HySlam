using System.Numerics;
using HySlam.Core.Data;
using HySlam.Core.Slam;
using Raylib_cs;
using GLVector3 = System.Numerics.Vector3;

namespace HySlam.Core.Rendering
{
    /// <summary>How map points are coloured.</summary>
    public enum RenderColorMode
    {
        /// <summary>Use the colour sampled from the source video.</summary>
        Sampled,

        /// <summary>Colour by semantic label (ground, building, ...).</summary>
        Semantic,

        /// <summary>Colour by height using a Turbo/LiDAR-style palette.</summary>
        Height,

        /// <summary>Colour by point confidence using the same palette.</summary>
        Confidence,
    }

    /// <summary>Viewer colour theme.</summary>
    public enum RenderTheme
    {
        Dark,
        Light,
    }

    /// <summary>Which map points are visible.</summary>
    public enum VisibilityFilter
    {
        All,
        RoadsOnly,
        ObjectsOnly,
        BuildingsOnly,
    }

    /// <summary>
    /// Rendering options for the map viewer.
    /// </summary>
    public sealed class RenderOptions
    {
        /// <summary>
        /// Only points whose semantic label is in this set are rendered.
        /// Null means render all labels.
        /// </summary>
        public HashSet<string>? SemanticFilter { get; set; }

        /// <summary>
        /// Points with confidence below this value are not rendered.
        /// </summary>
        public float MinConfidence { get; set; }

        /// <summary>Point colour mode. Defaults to the LiDAR-style height palette.</summary>
        public RenderColorMode ColorMode { get; set; } = RenderColorMode.Height;

        /// <summary>Viewer background and grid theme.</summary>
        public RenderTheme Theme { get; set; } = RenderTheme.Dark;

        /// <summary>Which point classes are visible.</summary>
        public VisibilityFilter Visibility { get; set; } = VisibilityFilter.All;

        /// <summary>Point size in pixels.</summary>
        public float PointSize { get; set; } = 5.0f;

        /// <summary>Draw the ground grid and height axis.</summary>
        public bool ShowGrid { get; set; }

        /// <summary>Draw the trajectory as a continuous line.</summary>
        public bool ShowTrajectory { get; set; } = true;

        /// <summary>Draw the lane/road boundaries as continuous lines.</summary>
        public bool ShowLaneLines { get; set; } = true;

        /// <summary>Lane width in meters used for the boundary offset.</summary>
        public double LaneWidthMeters { get; set; } = 3.5;

        /// <summary>
        /// When true, points outside the camera frustum are not uploaded.
        /// </summary>
        public bool EnableFrustumCulling { get; set; }
    }

    /// <summary>
    /// Free-camera point-cloud viewer backed by raylib.
    ///
    /// raylib's batch renderer has no point primitive, so each map point is drawn
    /// as a small camera-facing quad. The quad size is derived from the configured
    /// pixel size, the field of view and the point distance so the cloud keeps the
    /// same on-screen density regardless of the zoom level.
    /// </summary>
    internal sealed class MapRenderer : IDisposable
    {
        private const float Deg2Rad = MathF.PI / 180f;
        private const float CameraFovY = 60f;
        private const float CameraNear = 0.01f;
        private const float CameraFar = 1000f;

        // Free camera (orbit) variables
        private float freeCamYaw = 45f;
        private float freeCamPitch = 30f;
        private float freeCamDistance = 40f;
        private GLVector3 freeCamTarget = GLVector3.Zero;

        // Interactive runs follow the newest trajectory sample. A loaded map is
        // static, so the camera is framed once and then left to the user.
        private bool followTrajectory = true;

        // Rendering options.
        private readonly RenderOptions options = new();

        /// <summary>Gets the mutable render options.</summary>
        public RenderOptions Options => options;

        public void Orbit(float deltaX, float deltaY)
        {
            freeCamYaw -= deltaX * 0.3f;
            freeCamPitch += deltaY * 0.3f;
            freeCamPitch = Math.Clamp(freeCamPitch, -89f, 89f);
        }

        public void Zoom(float scroll)
        {
            freeCamDistance -= scroll * 3f;
            freeCamDistance = Math.Clamp(freeCamDistance, 5f, 500f);
        }

        public void Render(MapData map, CameraPose? pose)
        {
            if (options.Theme == RenderTheme.Light)
                Raylib.ClearBackground(new Color(235, 235, 235, 255));
            else
                Raylib.ClearBackground(new Color(0, 0, 0, 255));

            GetFreeCamera(map, out var eye, out var target, out var up);

            var camera = new Camera3D
            {
                Position = eye,
                Target = target,
                Up = up,
                FovY = CameraFovY,
                Projection = CameraProjection.Perspective
            };

            Raylib.BeginMode3D(camera);
            RenderScene(map, pose, camera);
            Raylib.EndMode3D();
        }

        /// <summary>
        /// Frames the complete map once. Used for a loaded map, which has no camera
        /// pose to follow, so the user starts with the whole point cloud in view.
        /// </summary>
        public void FrameMap(MapData map)
        {
            if (map.Points.Count == 0 && map.Trajectory.Count == 0)
                return;

            var min = new GLVector3(float.MaxValue, float.MaxValue, float.MaxValue);
            var max = new GLVector3(float.MinValue, float.MinValue, float.MinValue);

            foreach (var position in map.Trajectory)
            {
                min = GLVector3.Min(min, position);
                max = GLVector3.Max(max, position);
            }

            foreach (var point in map.Points)
            {
                min = GLVector3.Min(min, point.Position);
                max = GLVector3.Max(max, point.Position);
            }

            freeCamTarget = (min + max) * 0.5f;
            freeCamDistance = Math.Clamp((max - min).Length() * 0.8f, 5f, 500f);
            freeCamYaw = 45f;
            freeCamPitch = 30f;
            followTrajectory = false;
        }

        private void RenderScene(MapData map, CameraPose? pose, Camera3D camera)
        {
            var lines = new List<float>();

            // Grid and height axis are optional so the view can match a clean
            // LiDAR-style point cloud.
            if (options.ShowGrid)
            {
                AddGroundGrid(lines);
                AddHeightAxis(lines);
            }

            if (pose is not null)
                AddCameraFrustum(lines, pose);

            AddTrajectoryLines(lines, map);

            if (lines.Count > 0)
                DrawLines(lines);

            // Height range for the LiDAR-style height palette.
            float minHeight = 0f, maxHeight = 1f;
            if (options.ColorMode == RenderColorMode.Height && map.Points.Count > 0)
            {
                minHeight = float.MaxValue;
                maxHeight = float.MinValue;
                foreach (var p in map.Points)
                {
                    if (p.Position.Y < minHeight) minHeight = p.Position.Y;
                    if (p.Position.Y > maxHeight) maxHeight = p.Position.Y;
                }

                if (maxHeight - minHeight < 1e-3f)
                    maxHeight = minHeight + 1f;
            }

            // Build frustum planes for culling when a pose is available.
            Matrix4x4? frustumMVP = null;
            if (pose is not null && options.EnableFrustumCulling)
            {
                float aspect = (float)Math.Max(1, Raylib.GetScreenWidth()) / Math.Max(1, Raylib.GetScreenHeight());
                var projection = Matrix4x4.CreatePerspectiveFieldOfView(
                    CameraFovY * Deg2Rad, aspect, CameraNear, CameraFar);
                var view = Matrix4x4.CreateLookAt(camera.Position, camera.Target, camera.Up);
                frustumMVP = view * projection;
            }

            DrawPointCloud(map, camera, minHeight, maxHeight, frustumMVP);
        }

        private static void DrawLines(List<float> lines)
        {
            Rlgl.Begin(DrawMode.Lines);
            for (int i = 0; i < lines.Count; i += 6)
            {
                Rlgl.Color4ub(ToByte(lines[i + 3]), ToByte(lines[i + 4]), ToByte(lines[i + 5]), 255);
                Rlgl.Vertex3f(lines[i], lines[i + 1], lines[i + 2]);
            }
            Rlgl.End();
        }

        private void DrawPointCloud(
            MapData map,
            Camera3D camera,
            float minHeight,
            float maxHeight,
            Matrix4x4? frustumMVP)
        {
            if (map.Points.Count == 0)
                return;

            // Camera-facing basis so every point is a screen-aligned quad.
            var forward = GLVector3.Normalize(camera.Target - camera.Position);
            var right = GLVector3.Normalize(GLVector3.Cross(forward, camera.Up));
            var up = GLVector3.Cross(right, forward);

            float screenHeight = Math.Max(1, Raylib.GetScreenHeight());
            float tanHalfFov = MathF.Tan(CameraFovY * Deg2Rad * 0.5f);
            float basePointSize = Math.Max(1f, options.PointSize);

            Rlgl.Begin(DrawMode.Quads);

            foreach (var p in map.Points)
            {
                // Semantic filter.
                if (options.SemanticFilter is not null &&
                    !options.SemanticFilter.Contains(p.SemanticLabel))
                    continue;

                // Confidence threshold.
                if (p.Confidence < options.MinConfidence)
                    continue;

                // Road / object / building visibility filter.
                if (!PassesVisibilityFilter(p))
                    continue;

                // Frustum culling.
                if (frustumMVP.HasValue && !IsPointInFrustum(p.Position, frustumMVP.Value))
                    continue;

                var color = ColorForPoint(p, minHeight, maxHeight);

                float distance = GLVector3.Distance(p.Position, camera.Position);
                float pixelSize = basePointSize * Math.Clamp(15f / (distance + 1f), 0.5f, 2f);
                float worldPerPixel = 2f * tanHalfFov * Math.Max(distance, 0.01f) / screenHeight;
                float half = Math.Max(pixelSize * worldPerPixel * 0.5f, 0.001f);

                Rlgl.Color4ub(ToByte(color.X), ToByte(color.Y), ToByte(color.Z), 255);

                var pos = p.Position;
                Rlgl.Vertex3f(
                    pos.X - right.X * half - up.X * half,
                    pos.Y - right.Y * half - up.Y * half,
                    pos.Z - right.Z * half - up.Z * half);
                Rlgl.Vertex3f(
                    pos.X + right.X * half - up.X * half,
                    pos.Y + right.Y * half - up.Y * half,
                    pos.Z + right.Z * half - up.Z * half);
                Rlgl.Vertex3f(
                    pos.X + right.X * half + up.X * half,
                    pos.Y + right.Y * half + up.Y * half,
                    pos.Z + right.Z * half + up.Z * half);
                Rlgl.Vertex3f(
                    pos.X - right.X * half + up.X * half,
                    pos.Y - right.Y * half + up.Y * half,
                    pos.Z - right.Z * half + up.Z * half);
            }

            Rlgl.End();
        }

        private static bool IsPointInFrustum(GLVector3 point, Matrix4x4 mvp)
        {
            var clip = Vector4.Transform(new Vector4(point, 1f), mvp);

            // Perspective divide.
            if (MathF.Abs(clip.W) < 1e-6f)
                return false;

            // Normalised device coordinates are in [-1, 1] for points inside the frustum.
            return clip.X >= -clip.W && clip.X <= clip.W &&
                   clip.Y >= -clip.W && clip.Y <= clip.W &&
                   clip.Z >= -clip.W && clip.Z <= clip.W;
        }

        private bool PassesVisibilityFilter(MapPoint point)
        {
            switch (options.Visibility)
            {
                case VisibilityFilter.RoadsOnly:
                    // Prefer semantic labels when the map has them; otherwise use
                    // the road-plane height as a fallback for older exports.
                    if (!string.IsNullOrEmpty(point.SemanticLabel))
                        return point.SemanticLabel == "ground";
                    return Math.Abs(point.Position.Y) < 0.5f;

                case VisibilityFilter.ObjectsOnly:
                    if (!string.IsNullOrEmpty(point.SemanticLabel))
                        return point.SemanticLabel != "ground" && point.SemanticLabel != "sky";
                    return Math.Abs(point.Position.Y) >= 0.5f;

                case VisibilityFilter.BuildingsOnly:
                    return point.SemanticLabel == "building";

                default:
                    return true;
            }
        }

        private GLVector3 ColorForPoint(MapPoint point, float minHeight, float maxHeight)
        {
            switch (options.ColorMode)
            {
                case RenderColorMode.Semantic:
                    return SemanticColor(point.SemanticLabel);

                case RenderColorMode.Height:
                {
                    float t = (point.Position.Y - minHeight) / (maxHeight - minHeight);
                    return Turbo(Math.Clamp(t, 0f, 1f));
                }

                case RenderColorMode.Confidence:
                    return Turbo(Math.Clamp(point.Confidence, 0f, 1f));

                default:
                    return point.Color;
            }
        }

        /// <summary>
        /// Google's Turbo colormap: a high-contrast blue-cyan-green-yellow-red
        /// palette similar to LiDAR height/intensity colouring.
        /// </summary>
        private static GLVector3 Turbo(double t)
        {
            t = Math.Clamp(t, 0.0, 1.0);
            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t3 * t;
            double t5 = t4 * t;

            double r = 0.13572138 + 4.61539260 * t - 42.66032258 * t2
                     + 132.13108234 * t3 - 152.94239396 * t4 + 59.28637943 * t5;
            double g = 0.09140261 + 2.19418839 * t + 4.84296658 * t2
                     - 14.18503333 * t3 + 4.27729857 * t4 + 2.82956604 * t5;
            double b = 0.10667330 + 12.64194608 * t - 60.58204836 * t2
                     + 110.36276771 * t3 - 89.90310912 * t4 + 27.34824973 * t5;

            return new GLVector3(
                (float)Math.Clamp(r, 0.0, 1.0),
                (float)Math.Clamp(g, 0.0, 1.0),
                (float)Math.Clamp(b, 0.0, 1.0));
        }

        private static GLVector3 SemanticColor(string label)
        {
            return label switch
            {
                "ground" => new GLVector3(0.3f, 0.8f, 0.3f),      // green
                "vegetation" => new GLVector3(0.5f, 1.0f, 0.5f), // bright green
                "building" => new GLVector3(0.8f, 0.8f, 0.8f),   // gray
                "sky" => new GLVector3(0.3f, 0.5f, 1.0f),        // blue
                _ => new GLVector3(1.0f, 1.0f, 1.0f),            // white
            };
        }

        private void GetFreeCamera(MapData map, out GLVector3 eye, out GLVector3 target, out GLVector3 up)
        {
            if (followTrajectory && map.Trajectory.Count > 0)
                freeCamTarget = map.Trajectory[^1];

            float yawRad = freeCamYaw * Deg2Rad;
            float pitchRad = freeCamPitch * Deg2Rad;

            var offset = new GLVector3(
                freeCamDistance * (float)Math.Cos(pitchRad) * (float)Math.Cos(yawRad),
                freeCamDistance * (float)Math.Sin(pitchRad),
                freeCamDistance * (float)Math.Cos(pitchRad) * (float)Math.Sin(yawRad)
            );

            eye = freeCamTarget + offset;
            target = freeCamTarget;
            up = GLVector3.UnitY;
        }

        /// <summary>
        /// Draws the camera trajectory as a continuous line and, when enabled, two
        /// lane/road boundary lines offset from the path. The boundaries are placed
        /// on the road plane (GL y = 0) rather than at camera height.
        /// </summary>
        private void AddTrajectoryLines(List<float> lines, MapData map)
        {
            if (!options.ShowTrajectory || map.Trajectory.Count < 2)
                return;

            // Camera path in cyan so it stands apart from the green road boundaries.
            var trajectoryColor = new GLVector3(0.1f, 0.9f, 1.0f);
            for (int i = 0; i < map.Trajectory.Count - 1; i++)
                AddLine(lines, map.Trajectory[i], map.Trajectory[i + 1], trajectoryColor);

            if (!options.ShowLaneLines)
                return;

            var laneColor = new GLVector3(0.2f, 1.0f, 0.3f);
            float halfWidth = (float)(options.LaneWidthMeters * 0.5);

            for (int i = 0; i < map.Trajectory.Count - 1; i++)
            {
                var a = map.Trajectory[i];
                var b = map.Trajectory[i + 1];

                var direction = b - a;
                direction.Y = 0f;
                if (direction.LengthSquared() < 1e-6f)
                    continue;

                direction = GLVector3.Normalize(direction);
                var right = GLVector3.Cross(direction, GLVector3.UnitY);
                if (right.LengthSquared() < 1e-6f)
                    continue;

                right = GLVector3.Normalize(right);
                var offset = right * halfWidth;

                // Place the boundaries on the road plane, not at camera height.
                var aGround = new GLVector3(a.X, 0f, a.Z);
                var bGround = new GLVector3(b.X, 0f, b.Z);

                AddLine(lines, aGround - offset, bGround - offset, laneColor);
                AddLine(lines, aGround + offset, bGround + offset, laneColor);
            }
        }

        private void AddGroundGrid(List<float> data)
        {
            const float extent = 100f;
            const float step = 5f;
            const float majorStep = 25f;
            const float y = 0.0f;

            bool light = options.Theme == RenderTheme.Light;

            var minorColor = light
                ? new GLVector3(0.78f, 0.80f, 0.83f)
                : new GLVector3(0.16f, 0.18f, 0.22f);
            var majorColor = light
                ? new GLVector3(0.55f, 0.58f, 0.62f)
                : new GLVector3(0.32f, 0.35f, 0.40f);
            var axisX = light
                ? new GLVector3(0.75f, 0.25f, 0.25f)
                : new GLVector3(0.55f, 0.18f, 0.18f);
            var axisZ = light
                ? new GLVector3(0.25f, 0.40f, 0.75f)
                : new GLVector3(0.18f, 0.30f, 0.60f);

            for (float x = -extent; x <= extent; x += step)
            {
                var c = Math.Abs(x) < 0.001f
                    ? axisZ
                    : IsMajorLine(x, majorStep) ? majorColor : minorColor;
                AddLine(data, new GLVector3(x, y, -extent), new GLVector3(x, y, extent), c);
            }
            for (float z = -extent; z <= extent; z += step)
            {
                var c = Math.Abs(z) < 0.001f
                    ? axisX
                    : IsMajorLine(z, majorStep) ? majorColor : minorColor;
                AddLine(data, new GLVector3(-extent, y, z), new GLVector3(extent, y, z), c);
            }
        }

        private static bool IsMajorLine(float coordinate, float step)
        {
            return Math.Abs(coordinate % step) < 0.001f;
        }

        /// <summary>
        /// Vertical Y axis with 5-unit ticks, so height differences can be read directly.
        /// </summary>
        private void AddHeightAxis(List<float> data)
        {
            const float height = 30f;
            const float tickStep = 5f;
            const float tick = 2.5f;

            var color = options.Theme == RenderTheme.Light
                ? new GLVector3(0.35f, 0.45f, 0.50f)
                : new GLVector3(0.30f, 0.50f, 0.55f);

            AddLine(data, new GLVector3(0f, 0f, 0f), new GLVector3(0f, height, 0f), color);

            for (float y = tickStep; y <= height; y += tickStep)
            {
                AddLine(data, new GLVector3(-tick, y, 0f), new GLVector3(tick, y, 0f), color);
                AddLine(data, new GLVector3(0f, y, -tick), new GLVector3(0f, y, tick), color);
            }
        }

        // Green frustum showing where the camera is looking
        private void AddCameraFrustum(List<float> data, CameraPose pose)
        {
            var center = pose.GetCameraPositionGL();

            var fwd = new GLVector3((float)pose.Rotation[0, 2], (float)-pose.Rotation[1, 2], (float)-pose.Rotation[2, 2]);
            var up = new GLVector3((float)-pose.Rotation[0, 1], (float)pose.Rotation[1, 1], (float)pose.Rotation[2, 1]);

            if (fwd.LengthSquared() < 1e-6f) return;
            fwd = GLVector3.Normalize(fwd);
            up = GLVector3.Normalize(up);

            var right = GLVector3.Normalize(GLVector3.Cross(fwd, up));
            up = GLVector3.Normalize(GLVector3.Cross(right, fwd));

            float fovY = 70f * Deg2Rad;
            float length = 4.0f;
            float h = (float)Math.Tan(fovY / 2) * length;
            float w = h * 1.5f; // Wide-angle aspect ratio

            var p1 = center + fwd * length - right * w + up * h;
            var p2 = center + fwd * length + right * w + up * h;
            var p3 = center + fwd * length + right * w - up * h;
            var p4 = center + fwd * length - right * w - up * h;

            var c = new GLVector3(0.2f, 1.0f, 0.2f); // Bright green

            AddLine(data, center, p1, c);
            AddLine(data, center, p2, c);
            AddLine(data, center, p3, c);
            AddLine(data, center, p4, c);

            AddLine(data, p1, p2, c);
            AddLine(data, p2, p3, c);
            AddLine(data, p3, p4, c);
            AddLine(data, p4, p1, c);
        }

        private static byte ToByte(float value)
        {
            return (byte)Math.Clamp((int)MathF.Round(value * 255f), 0, 255);
        }

        private static void AddVertex(List<float> data, float x, float y, float z, float r, float g, float b)
        {
            data.Add(x); data.Add(y); data.Add(z);
            data.Add(r); data.Add(g); data.Add(b);
        }

        private static void AddLine(List<float> data, GLVector3 a, GLVector3 b, GLVector3 color)
        {
            AddVertex(data, a.X, a.Y, a.Z, color.X, color.Y, color.Z);
            AddVertex(data, b.X, b.Y, b.Z, color.X, color.Y, color.Z);
        }

        public void Dispose()
        {
            // raylib owns the rendering resources; nothing to release here.
        }
    }
}
