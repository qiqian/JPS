using System.Collections.Generic;

namespace JPS.Data
{
    public sealed class PointData
    {
        public int X { get; set; }
        public int Y { get; set; }
    }

    public sealed class MapData
    {
        public int Width { get; set; }
        public int Height { get; set; }
        public PointData? Start { get; set; }
        public PointData? End { get; set; }
        public List<PointData> Obstacles { get; set; } = new List<PointData>();
    }
}
