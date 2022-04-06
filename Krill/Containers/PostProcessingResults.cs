using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public class PostProcessingResults
    {
        public Voxels<int> mask { get; set; } = null;

        public Voxels<double> strainXX { get; set; } = null;
        public Voxels<double> strainYY { get; set; } = null;
        public Voxels<double> strainZZ { get; set; } = null;
        public Voxels<double> strainXY { get; set; } = null; // epsilon values, not gamma (gamma = 2*epsilon)
        public Voxels<double> strainXZ { get; set; } = null; // epsilon values, not gamma (gamma = 2*epsilon)
        public Voxels<double> strainYZ { get; set; } = null; // epsilon values, not gamma (gamma = 2*epsilon)

        public Voxels<double> stressXX { get; set; } = null;
        public Voxels<double> stressYY { get; set; } = null;
        public Voxels<double> stressZZ { get; set; } = null;
        public Voxels<double> stressXY { get; set; } = null;
        public Voxels<double> stressXZ { get; set; } = null;
        public Voxels<double> stressYZ { get; set; } = null;

        public Voxels<double> vonMises { get; set; } = null;
        public Voxels<Vector3d> princpStress { get; set; } = null;
        public Voxels<Vector3d[]> princpDir { get; set; } = null;
        public Voxels<Matrix> stressTensor { get; set; } = null;

        public Voxels<double> lengthDivStress { get; set; } = null;
        public Voxels<Vector3d> divOfStress { get; set; } = null;

    }
}
