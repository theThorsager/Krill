using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public interface IBoundaryCondition2d
    {
        Curve curve { get; set; }

    }
    public class BoundaryConditionNuemann2d : IBoundaryCondition2d
    {
        public Curve curve { get; set; } = null;
        public bool normal { get; set; } = true;
        public Vector3d load { get; set; } = Vector3d.YAxis;
    }

    public class BoundaryConditionDirechlet2d : IBoundaryCondition2d
    {
        public Curve curve { get; set; } = null;
        public bool normal { get; set; } = true;
        public Vector3d displacement { get; set; } = Vector3d.Zero;
        public bool lockX { get; set; } = true;
        public bool lockY { get; set; } = true;
        public Plane Coordinate { get; set; } = Plane.WorldXY;
        public int tag { get; set; } = -1;
    }
}
