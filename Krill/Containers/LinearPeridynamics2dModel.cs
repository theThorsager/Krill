using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public class LinearPeridynamics2dModel
    {
        public Voxels2d<int> mask { get; set; } = null;
        public List<IBoundaryCondition2d> boundaryConditions { get; set; } = null;
        public Settings settings { get; set; } = null;
        public int tag;
    }
}
