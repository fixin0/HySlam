using HySlam.Core.Data;
using HySlam.Core.Slam;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Rendering
{
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

        /// <summary>
        /// When true, points are coloured by semantic label instead of their
        /// sampled colour.
        /// </summary>
        public bool ColorBySemantic { get; set; }

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
            GL.ClearColor(0.0f, 0.0f, 0.0f, 1f); // Pure black background
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

            // Draw the ground grid, height axis and, for a live run, the camera frustum.
            AddGroundGrid(lines);
            AddHeightAxis(lines);
            if (pose is not null)
                AddCameraFrustum(lines, pose);

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

                // Frustum culling.
                if (frustumMVP.HasValue && !IsPointInFrustum(p.Position, frustumMVP.Value))
                    continue;

                var color = options.ColorBySemantic
                    ? SemanticColor(p.SemanticLabel)
                    : p.Color;

                AddVertex(points, p.Position.X, p.Position.Y, p.Position.Z,
                    color.X, color.Y, color.Z);
                renderedPoints++;
            }

            int pointCount = renderedPoints;

            // Trajectory (bright green points)
            int trajStart = pointStart + pointCount;
            foreach (var c in map.Trajectory)
            {
                AddVertex(points, c.X, c.Y, c.Z, 0.1f, 1.0f, 0.3f);
            }
            int trajCount = map.Trajectory.Count;

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
                // Point size varies with confidence: confident points are larger.
                shader.SetFloat("uPointSize", 5.0f);
                GL.DrawArrays(PrimitiveType.Points, pointStart, pointCount);
            }

            if (trajCount > 1)
            {
                shader.SetInt("uRoundPoints", 1);
                shader.SetFloat("uPointSize", 7.0f);
                GL.DrawArrays(PrimitiveType.Points, trajStart, trajCount);
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

        private void AddGroundGrid(List<float> data)
        {
            const float extent = 100f;
            const float step = 5f;
            const float majorStep = 25f;
            const float y = 0.0f;

            var minorColor = new GLVector3(0.16f, 0.18f, 0.22f);
            var majorColor = new GLVector3(0.32f, 0.35f, 0.40f);
            var axisX = new GLVector3(0.55f, 0.18f, 0.18f);
            var axisZ = new GLVector3(0.18f, 0.30f, 0.60f);

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
        private static void AddHeightAxis(List<float> data)
        {
            const float height = 30f;
            const float tickStep = 5f;
            const float tick = 2.5f;

            var color = new GLVector3(0.30f, 0.50f, 0.55f);

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
