using HySlam.Core.Data;
using HySlam.Core.Slam;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Rendering
{
    internal sealed class MapRenderer : IDisposable
    {
        private readonly Shader shader;
        private readonly int vao, vbo;

        // Cached buffer upload: skip GL.BufferData when nothing changed since last frame.
        private int lastPointCount;
        private int lastTrajCount;

        // Free camera (orbit) variables
        private float freeCamYaw = 45f;
        private float freeCamPitch = 30f;
        private float freeCamDistance = 40f;
        private GLVector3 freeCamTarget = GLVector3.Zero;

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

        public void Render(Vector2i size, MapData map, CameraPose pose)
        {
            int w = size.X;
            int h = size.Y;
            int halfW = w / 2;

            // 1. Top-Down (left side)
            GL.Viewport(0, 0, halfW, h);
            GL.Scissor(0, 0, halfW, h);
            GL.ClearColor(0.0f, 0.0f, 0.0f, 1f); // Pure black background
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            RenderScene(size, map, pose, topDown: true);

            // 2. Free View (right side)
            GL.Viewport(halfW, 0, halfW, h);
            GL.Scissor(halfW, 0, halfW, h);
            GL.ClearColor(0.0f, 0.0f, 0.0f, 1f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            RenderScene(size, map, pose, topDown: false);
        }

        private void RenderScene(Vector2i size, MapData map, CameraPose pose, bool topDown)
        {
            var lines = new List<float>();
            var points = new List<float>();

            // Draw the grid and frustum in both views
            if (!topDown)
            {
                AddGroundGrid(lines); // Draw the grid only in the free view
                AddHeightAxis(lines);
            }
            AddCameraFrustum(lines, pose);

            int lineStart = 0;
            int lineCount = lines.Count / 6;

            // Point cloud
            int pointStart = lineCount;
            foreach (var p in map.Points)
            {
                AddVertex(points, p.Position.X, p.Position.Y, p.Position.Z, p.Color.X, p.Color.Y, p.Color.Z);
            }
            int pointCount = map.Points.Count;

            // Trajectory (bright green points)
            int trajStart = pointStart + pointCount;
            foreach (var c in map.Trajectory)
            {
                AddVertex(points, c.X, c.Y, c.Z, 0.1f, 1.0f, 0.3f);
            }
            int trajCount = map.Trajectory.Count;

            // Only re-upload the big point cloud when it actually changed. The line
            // geometry (grid + frustum) is small and re-uploaded every frame.
            bool pointsChanged = pointCount != lastPointCount || trajCount != lastTrajCount;
            if (pointsChanged)
            {
                GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);
                GL.BufferData(BufferTarget.ArrayBuffer, (lines.Count + points.Count) * sizeof(float),
                    CombineLists(lines, points), BufferUsageHint.DynamicDraw);
                lastPointCount = pointCount;
                lastTrajCount = trajCount;
            }

            GLVector3 eye, target, up;
            Matrix4 projection;

            float aspect = (size.X / 2.0f) / size.Y;

            if (topDown)
            {
                GetTopDownCamera(map, out eye, out target, out up);
                float radius = (target - eye).Length;
                projection = Matrix4.CreateOrthographicOffCenter(-radius * aspect, radius * aspect, -radius, radius, 0.1f, 5000f);
            }
            else
            {
                GetFreeCamera(map, out eye, out target, out up);
                projection = Matrix4.CreatePerspectiveFieldOfView(MathHelper.DegreesToRadians(60f), aspect, 0.1f, 2000f);
            }

            Matrix4 view = Matrix4.LookAt(eye, target, up);
            Matrix4 mvp = view * projection;

            shader.Use();
            shader.SetMatrix4("uMVP", ref mvp);

            // Disable distance fading in both views so the cloud stays bright when
            // the free camera is zoomed out.
            shader.SetVector3("uCameraPos", topDown ? GLVector3.Zero : eye);
            shader.SetFloat("uNearFade", 0.0f);
            shader.SetFloat("uFarFade", 0.0f);

            GL.BindVertexArray(vao);

            if (lineCount > 0)
            {
                GL.LineWidth(1.5f);
                GL.DrawArrays(PrimitiveType.Lines, lineStart, lineCount);
            }

            if (pointCount > 0)
            {
                shader.SetFloat("uPointSize", topDown ? 4.0f : 5.0f);
                GL.DrawArrays(PrimitiveType.Points, pointStart, pointCount);
            }

            if (trajCount > 1)
            {
                shader.SetFloat("uPointSize", 7.0f);
                GL.DrawArrays(PrimitiveType.Points, trajStart, trajCount);
            }

            GL.BindVertexArray(0);
            GL.UseProgram(0);
        }

        private void GetTopDownCamera(MapData map, out GLVector3 eye, out GLVector3 target, out GLVector3 up)
        {
            if (map.Points.Count == 0)
            {
                eye = new GLVector3(0, 100, 0);
                target = GLVector3.Zero;
                up = -GLVector3.UnitZ;
                return;
            }

            var min = map.Points[0].Position;
            var max = map.Points[0].Position;
            foreach (var p in map.Points)
            {
                min = GLVector3.ComponentMin(min, p.Position);
                max = GLVector3.ComponentMax(max, p.Position);
            }

            var center = (min + max) * 0.5f;
            float radius = (max - min).Length * 0.5f;
            radius = Math.Clamp(radius, 10f, 500f);

            eye = center + new GLVector3(0, radius * 2.0f, 0);
            target = center;
            up = -GLVector3.UnitZ;
        }

        private void GetFreeCamera(MapData map, out GLVector3 eye, out GLVector3 target, out GLVector3 up)
        {
            if (map.Trajectory.Count > 0)
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
