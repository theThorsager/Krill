using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public class LinearSolution2d
    {
        public Voxels2d<int> mask { get; set; } = null;
        public Voxels2d<Vector2d> displacments { get; set; } = null;
        public List<BoundaryConditionNuemann2d> boundaryConditions { get; set; } = null;
        public double peridelta { get; set; } = 0;
        public Voxels2d<Vector3d[]> principalDirections { get; set; } = null;
        public double elasticModulus { get; set; } = 0;
        public int[] nList { get; set; } = null;
        public double bondStiffness { get; set; } = 0;
        public Voxels2d<Vector2d> springs { get; set; } = null;
        public Voxels2d<Vector2d> bodyload { get; set; } = null;
        public Voxels2d<double> utilization { get; set; } = null;
        public Voxels2d<double> weighting { get; set; } = null;

        public bool relaxTension = false;
        public double oldcuttoff = 0.0;
    }
}
