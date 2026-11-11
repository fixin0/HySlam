using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Data
{
    internal sealed class MapData
    {
        public readonly List<MapPoint> Points = new();
        public readonly List<GLVector3> Trajectory = new();
    }
}
