using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public interface IDiscreteBoundaryCondition
    {
        Point3d pts { get; set; }

    }
    public class DiscreteBoundaryConditionNuemann : IDiscreteBoundaryCondition
    {
        public Point3d pts { get; set; }
        public Vector3d load { get; set; }
    }

    public class DiscreteBoundaryConditionDirechlet : IDiscreteBoundaryCondition
    {
        public Point3d pts { get; set; }
        public Vector3d displacement { get; set; }
        public bool lockX { get; set; } = true;
        public bool lockY { get; set; } = true;
        public bool lockZ { get; set; } = true;
    }
}
