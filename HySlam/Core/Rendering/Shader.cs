using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Rendering
{
    internal sealed class Shader : IDisposable
    {
        private readonly int handle;
        private readonly Dictionary<string, int> uniformLocations = new();

        public int Handle => handle;

        public Shader(string vertexSource, string fragmentSource)
        {
            int vertex = CompileShader(ShaderType.VertexShader, vertexSource);
            int fragment = CompileShader(ShaderType.FragmentShader, fragmentSource);

            handle = GL.CreateProgram();
            GL.AttachShader(handle, vertex);
            GL.AttachShader(handle, fragment);
            GL.LinkProgram(handle);

            GL.GetProgram(handle, GetProgramParameterName.LinkStatus, out int linkStatus);
            if (linkStatus == 0)
                throw new Exception("Shader link error: " + GL.GetProgramInfoLog(handle));

            GL.DeleteShader(vertex);
            GL.DeleteShader(fragment);
        }

        public static Shader FromGlslFile(string path)
        {
            string text = File.ReadAllText(path);

            int fragmentIndex = text.IndexOf("#fragment", StringComparison.Ordinal);
            if (fragmentIndex < 0)
                throw new Exception("Shader file is missing the '#fragment' section: " + path);

            string vertexSource = text[..fragmentIndex];
            string fragmentSource = text[(fragmentIndex + "#fragment".Length)..];

            int vertexIndex = vertexSource.IndexOf("#vertex", StringComparison.Ordinal);
            if (vertexIndex >= 0)
                vertexSource = vertexSource[(vertexIndex + "#vertex".Length)..];

            return new Shader(vertexSource, fragmentSource);
        }

        public void Use()
        {
            GL.UseProgram(handle);
        }

        public int GetUniformLocation(string name)
        {
            if (uniformLocations.TryGetValue(name, out int location))
                return location;

            location = GL.GetUniformLocation(handle, name);
            uniformLocations[name] = location;
            return location;
        }

        public void SetMatrix4(string name, ref Matrix4 value)
        {
            GL.UniformMatrix4(GetUniformLocation(name), false, ref value);
        }

        public void SetVector3(string name, GLVector3 value)
        {
            GL.Uniform3(GetUniformLocation(name), value.X, value.Y, value.Z);
        }

        public void SetFloat(string name, float value)
        {
            GL.Uniform1(GetUniformLocation(name), value);
        }

        public void SetInt(string name, int value)
        {
            GL.Uniform1(GetUniformLocation(name), value);
        }

        private static int CompileShader(ShaderType type, string source)
        {
            int shaderId = GL.CreateShader(type);
            GL.ShaderSource(shaderId, source);
            GL.CompileShader(shaderId);

            GL.GetShader(shaderId, ShaderParameter.CompileStatus, out int status);
            if (status == 0)
                throw new Exception(type + " compile error: " + GL.GetShaderInfoLog(shaderId));

            return shaderId;
        }

        public void Dispose()
        {
            GL.DeleteProgram(handle);
        }
    }
}
