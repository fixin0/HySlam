using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Data
{
    /// <summary>
    /// A single 3D map point.
    ///
    /// In addition to position and colour, a point now carries:
    /// - <see cref="Normal"/> - estimated surface normal (unit length, may be zero
    ///   when unknown).
    /// - <see cref="Confidence"/> - quality estimate in [0, 1]; higher means the
    ///   point has been observed across multiple keyframes and passes.
    /// - <see cref="SemanticLabel"/> - optional class such as "ground", "building",
    ///   "vegetation" or "sky". Empty when no classifier is active.
    /// </summary>
    internal sealed class MapPoint
    {
        public GLVector3 Position;
        public GLVector3 Color;
        public int Observations;

        /// <summary>Estimated surface normal. Zero vector when not available.</summary>
        public GLVector3 Normal = GLVector3.Zero;

        /// <summary>Quality score in [0, 1]. Zero means untrusted / unknown.</summary>
        public float Confidence;

        /// <summary>Optional semantic class label. Empty when unknown.</summary>
        public string SemanticLabel = string.Empty;
    }
}
