using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Data
{
    internal sealed class MapPoint
    {
        public GLVector3 Position;
        public GLVector3 Color;
        public int Observations;
    }
}
