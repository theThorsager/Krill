using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public class LinearPeridynamicsModel
    {
        public Voxels<int> mask { get; set; } = null;
        public List<IBoundaryCondition> boundaryConditions { get; set; } = null;
        public Settings settings { get; set; } = null;
        public int tag;
    }
}
