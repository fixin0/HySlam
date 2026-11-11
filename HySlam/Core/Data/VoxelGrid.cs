namespace HySlam.Core.Data
{
    internal sealed class VoxelGrid
    {
        // Minimum spacing between map points. Duplicate observations inside the same
        // neighbourhood are rejected so the cloud stays clean.
        private const float VoxelSize = 0.3f;

        private readonly Dictionary<(int, int, int), List<int>> spatialIndex = new();

        public bool IsTooClose(IReadOnlyList<MapPoint> points, float x, float y, float z)
        {
            int vx = (int)Math.Floor(x / VoxelSize), vy = (int)Math.Floor(y / VoxelSize), vz = (int)Math.Floor(z / VoxelSize);
            float min2 = VoxelSize * VoxelSize * 0.49f;

            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (!spatialIndex.TryGetValue((vx + dx, vy + dy, vz + dz), out var list)) continue;
                        foreach (var idx in list)
                        {
                            var p = points[idx].Position;
                            float ddx = p.X - x, ddy = p.Y - y, ddz = p.Z - z;
                            if (ddx * ddx + ddy * ddy + ddz * ddz < min2) return true;
                        }
                    }
            return false;
        }

        public void IndexPoint(IReadOnlyList<MapPoint> points, int index)
        {
            var p = points[index].Position;
            var key = ((int)Math.Floor(p.X / VoxelSize), (int)Math.Floor(p.Y / VoxelSize), (int)Math.Floor(p.Z / VoxelSize));
            if (!spatialIndex.TryGetValue(key, out var list))
            {
                list = new List<int>();
                spatialIndex[key] = list;
            }
            list.Add(index);
        }

        public void Rebuild(IReadOnlyList<MapPoint> points)
        {
            spatialIndex.Clear();
            for (int i = 0; i < points.Count; i++) IndexPoint(points, i);
        }
    }
}
