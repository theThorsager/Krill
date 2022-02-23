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
        public double peridelta { get; set; } = 0;
        public Voxels<Vector3d[]> principalDirections { get; set; } = null;
        public double elasticModulus { get; set; } = 0;
        public int[] nList { get; set; } = null;
        public double bondStiffness { get; set; } = 0;
        public Voxels<Vector3d> springs { get; set; } = null;
        public Voxels<Vector3d> bodyload { get; set; } = null;
    }
}
