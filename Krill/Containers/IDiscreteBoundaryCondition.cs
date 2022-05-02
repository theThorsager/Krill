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
        //Line line { get; set; }

    }
    public class DiscreteBoundaryConditionNuemann : IDiscreteBoundaryCondition
    {
        public Line line { get; set; }
        public double load { get; set; }
    }

    public class DiscreteBoundaryConditionDirechlet : IDiscreteBoundaryCondition
    {
        public Line line { get; set; }
        public bool Fixed { get; set; }
        //public Vector3d displacement { get; set; }
        //public bool lockX { get; set; } = true;
        //public bool lockY { get; set; } = true;
        //public bool lockZ { get; set; } = true;
    }
    public class DiscreteBoundaryConditionVariables : IDiscreteBoundaryCondition
    {
        public Point3d point { get; set; }
        public bool lockX { get; set; } = true;
        public bool lockY { get; set; } = true;
        public bool lockZ { get; set; } = true;
    }
}
