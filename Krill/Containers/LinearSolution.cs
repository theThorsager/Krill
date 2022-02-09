using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public class LinearSolution
    {
        public Voxels<int> mask { get; set; } = null;
        public Voxels<Vector3d> displacments { get; set; } = null;
        public List<BoundaryConditionNuemann> boundaryConditions { get; set; } = null;
    }
}
