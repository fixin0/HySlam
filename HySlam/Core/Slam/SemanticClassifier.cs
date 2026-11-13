using HySlam.Core.Data;
using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Slam
{
    /// <summary>
    /// Minimal semantic classifier for map points.
    ///
    /// Without a trained deep learning model, points are classified using simple
    /// rules based on colour, surface normal and height:
    /// - ground: near y = 0 with upward-facing normal
    /// - vegetation: greenish colour
    /// - building: vertical normal with neutral/gray colour
    /// - sky: high above ground with blueish colour
    /// - unknown: everything else
    ///
    /// Labels are stored on <see cref="MapPoint.SemanticLabel"/> and written to
    /// points.csv by <see cref="Core.Bake.BakeRunner"/>.
    /// </summary>
    internal static class SemanticClassifier
    {
        public static string Classify(
            GLVector3 position,
            GLVector3 color,
            GLVector3 normal)
        {
            // Vegetation: clearly greenish.
            if (color.Y > color.X + 0.15f && color.Y > color.Z + 0.15f && color.Y > 0.3f)
                return "vegetation";

            // Sky: high above ground and blueish.
            if (position.Y > 10.0f && color.Z > color.X + 0.15f && color.Z > color.Y + 0.15f)
                return "sky";

            // Ground: on or very near the road plane with upward normal.
            if (Math.Abs(position.Y) < 0.5f && normal.Y < -0.7f)
                return "ground";

            // Building: vertical-ish normal (wall-like) with neutral colour.
            double normalHorizontal = Math.Sqrt(normal.X * normal.X + normal.Z * normal.Z);
            if (normalHorizontal > 0.7f &&
                Math.Abs(color.X - color.Y) < 0.1f &&
                Math.Abs(color.Y - color.Z) < 0.1f)
            {
                return "building";
            }

            return "unknown";
        }
    }
}
