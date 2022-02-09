using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Krill.Containers
{
    public class Settings
    {
        // Side length of the voxel cells
        public double Delta { get; set; } = 0.5;
        // Number of voxels the horisions radii is
        public double delta { get; set; } = 2.5;
        // number of timesteps to compute for
        public int n_timesteps { get; set; } = 10000;
        // BondStiffness
        public double bond_stiffness { get; set; } = 100000;
        // Youngs
        public double E { get; set; } = 1000;
    }
}
