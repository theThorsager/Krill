using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public class PostProcessingResults2d
    {
        public Voxels2d<int> mask { get; set; } = null;
        public Voxels2d<double> strainXX { get; set; } = null;
        public Voxels2d<double> strainYY { get; set; } = null;
        public Voxels2d<double> strainXY { get; set; } = null;
        public Voxels2d<double> stressXX { get; set; } = null;
        public Voxels2d<double> stressYY { get; set; } = null;
        public Voxels2d<double> stressXY { get; set; } = null;
        public Voxels2d<double> vonMises { get; set; } = null;
        public Voxels2d<Vector3d> princpStress { get; set; } = null;
        public Voxels2d<Vector3d[]> princpDir { get; set; } = null;
        public Voxels2d<Matrix> stressTensor { get; set; } = null;

    }
}
