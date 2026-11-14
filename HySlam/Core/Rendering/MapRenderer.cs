using HySlam.Core.Data;
using HySlam.Core.Slam;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using GLVector3 = OpenTK.Mathematics.Vector3;

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

    internal sealed class MapRenderer : IDisposable
    {
        private readonly Shader shader;
        private readonly int vao, vbo;

        // Cached buffer upload: reallocate only when the total float count changes.
        private int bufferFloats;

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

        public MapRenderer()
        {
            string shaderPath = Path.Combine(AppContext.BaseDirectory, "shaders", "Shaders.glsl");
            shader = Shader.FromGlslFile(shaderPath);

            vao = GL.GenVertexArray();
            vbo = GL.GenBuffer();

            GL.BindVertexArray(vao);
            GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);

            const int stride = 6 * sizeof(float);

            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, IntPtr.Zero);

            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, new IntPtr(3 * sizeof(float)));

            GL.BindVertexArray(0);
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        }

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

        public void Render(Vector2i size, MapData map, CameraPose? pose)
        {
            // Single full-window free-camera view.
            GL.Viewport(0, 0, size.X, size.Y);
            GL.Scissor(0, 0, size.X, size.Y);
            if (options.Theme == RenderTheme.Light)
                GL.ClearColor(0.92f, 0.92f, 0.92f, 1f);
            else
                GL.ClearColor(0.0f, 0.0f, 0.0f, 1f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            RenderScene(size, map, pose);
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
                min = GLVector3.ComponentMin(min, position);
                max = GLVector3.ComponentMax(max, position);
            }

            foreach (var point in map.Points)
            {
                min = GLVector3.ComponentMin(min, point.Position);
                max = GLVector3.ComponentMax(max, point.Position);
            }

            freeCamTarget = (min + max) * 0.5f;
            freeCamDistance = Math.Clamp((max - min).Length * 0.8f, 5f, 500f);
            freeCamYaw = 45f;
            freeCamPitch = 30f;
            followTrajectory = false;
        }

        private void RenderScene(Vector2i size, MapData map, CameraPose? pose)
        {
            var lines = new List<float>();
            var points = new List<float>();

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

            int lineStart = 0;
            int lineCount = lines.Count / 6;

            // Build frustum planes for culling when a pose is available.
            Matrix4? frustumMVP = null;
            if (pose is not null && options.EnableFrustumCulling)
            {
                GLVector3 fcEye, fcTarget, fcUp;
                GetFreeCamera(map, out fcEye, out fcTarget, out fcUp);
                float fcAspect = (float)size.X / size.Y;
                var fcProjection = Matrix4.CreatePerspectiveFieldOfView(
                    MathHelper.DegreesToRadians(60f), fcAspect, 0.1f, 2000f);
                var fcView = Matrix4.LookAt(fcEye, fcTarget, fcUp);
                frustumMVP = fcView * fcProjection;
            }

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

            // Point cloud with optional semantic filtering, confidence threshold
            // and frustum culling.
            int pointStart = lineCount;
            int renderedPoints = 0;

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

                AddVertex(points, p.Position.X, p.Position.Y, p.Position.Z,
                    color.X, color.Y, color.Z);
                renderedPoints++;
            }

            int pointCount = renderedPoints;

            // The point cloud is the expensive part of the upload; geometry lines are
            // small. Reallocate the buffer only when its total size changes, otherwise
            // update just the line range.
            int totalFloats = lines.Count + points.Count;
            if (totalFloats > 0)
            {
                GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);

                if (totalFloats != bufferFloats)
                {
                    GL.BufferData(BufferTarget.ArrayBuffer, totalFloats * sizeof(float),
                        CombineLists(lines, points), BufferUsageHint.DynamicDraw);
                    bufferFloats = totalFloats;
                }
                else if (lines.Count > 0)
                {
                    float[] lineData = lines.ToArray();
                    GL.BufferSubData(BufferTarget.ArrayBuffer, IntPtr.Zero,
                        lineData.Length * sizeof(float), lineData);
                }
            }

            GLVector3 eye, target, up;
            Matrix4 projection;

            float aspect = (float)size.X / size.Y;

            GetFreeCamera(map, out eye, out target, out up);
            projection = Matrix4.CreatePerspectiveFieldOfView(MathHelper.DegreesToRadians(60f), aspect, 0.1f, 2000f);

            Matrix4 view = Matrix4.LookAt(eye, target, up);
            Matrix4 mvp = view * projection;

            shader.Use();
            shader.SetMatrix4("uMVP", ref mvp);

            // Distance fading stays disabled so the cloud remains bright when the free
            // camera is zoomed out.
            shader.SetVector3("uCameraPos", eye);
            shader.SetFloat("uNearFade", 0.0f);
            shader.SetFloat("uFarFade", 0.0f);

            GL.BindVertexArray(vao);

            if (lineCount > 0)
            {
                // A Core profile only guarantees a line width of 1.0; asking for more
                // raises GL_INVALID_VALUE and can upset some drivers.
                shader.SetInt("uRoundPoints", 0);
                GL.DrawArrays(PrimitiveType.Lines, lineStart, lineCount);
            }

            if (pointCount > 0)
            {
                shader.SetInt("uRoundPoints", 1);
                // Point size is controlled from the viewer menu.
                shader.SetFloat("uPointSize", options.PointSize);
                GL.DrawArrays(PrimitiveType.Points, pointStart, pointCount);
            }

            GL.BindVertexArray(0);
            GL.UseProgram(0);
        }

        private static bool IsPointInFrustum(GLVector3 point, Matrix4 mvp)
        {
            // Transform point to clip space: v_clip = MVP * v_model (manual mul to avoid
            // Vector4.Transform overload ambiguity with Quaternion).
            float x = mvp.Row0.X * point.X + mvp.Row0.Y * point.Y + mvp.Row0.Z * point.Z + mvp.Row0.W;
            float y = mvp.Row1.X * point.X + mvp.Row1.Y * point.Y + mvp.Row1.Z * point.Z + mvp.Row1.W;
            float z = mvp.Row2.X * point.X + mvp.Row2.Y * point.Y + mvp.Row2.Z * point.Z + mvp.Row2.W;
            float w = mvp.Row3.X * point.X + mvp.Row3.Y * point.Y + mvp.Row3.Z * point.Z + mvp.Row3.W;

            // Perspective divide.
            if (Math.Abs(w) < 1e-6f)
                return false;

            // Normalised device coordinates are in [-1, 1] for points inside the
            // frustum (with the OpenGL convention used by OpenTK).
            return x >= -w && x <= w &&
                   y >= -w && y <= w &&
                   z >= -w && z <= w;
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

            float yawRad = MathHelper.DegreesToRadians(freeCamYaw);
            float pitchRad = MathHelper.DegreesToRadians(freeCamPitch);

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
                if (direction.LengthSquared < 1e-6f)
                    continue;

                direction = GLVector3.Normalize(direction);
                var right = GLVector3.Cross(direction, GLVector3.UnitY);
                if (right.LengthSquared < 1e-6f)
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

            if (fwd.LengthSquared < 1e-6f) return;
            fwd = GLVector3.Normalize(fwd);
            up = GLVector3.Normalize(up);

            var right = GLVector3.Normalize(GLVector3.Cross(fwd, up));
            up = GLVector3.Normalize(GLVector3.Cross(right, fwd));

            float fovY = MathHelper.DegreesToRadians(70f);
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

        private static float[] CombineLists(List<float> a, List<float> b)
        {
            float[] result = new float[a.Count + b.Count];
            a.CopyTo(result);
            b.CopyTo(result, a.Count);
            return result;
        }
        public void Dispose()
        {
            GL.DeleteBuffer(vbo);
            GL.DeleteVertexArray(vao);
            shader.Dispose();
        }
    }
}
