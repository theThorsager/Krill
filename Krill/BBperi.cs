using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class BBperi
    {
        public double bond_stiffness;
        public Voxels<int> startVoxels;
        public int[] nlist;    // Neighbour list
        public double vol;   // Volume of the particles (same for all)
        public double dt;   // Time step

        public Voxels<Vector3d> forceVoxels;
        public Voxels<Vector3d> dispVoxels;

        private int noVoxels;

        public BBperi(Voxels<int> orgVoxels, double Bond_stiffness, int[] neighbour_list,
            double volume, double time_step)
        {
            startVoxels = orgVoxels;
            bond_stiffness = Bond_stiffness;
            nlist = neighbour_list;
            vol = volume;
            dt = time_step;

            noVoxels = startVoxels.n;
            forceVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            dispVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);         

        }

        public void UpdateForce()
        {

            double[] local_cache_x = new double[noVoxels];
            double[] local_cache_y = new double[noVoxels];
            double[] local_cache_z = new double[noVoxels];

            int noBonds = nlist.Length;

            // Assuming the bonds never break, otherwise need a new if-statement
            for (int i = 0; i < noVoxels*noVoxels*noVoxels; i++)
            {
                if (startVoxels.cellValues[i] == 0)
                {
                    continue;
                }

                forceVoxels.cellValues[i] = Vector3d.Zero;

                for (int a = 0; a < noBonds; a++)
                {

                    int j = i + nlist[a];

                    if (startVoxels.cellValues[j] == 0)
                    {
                        continue;
                    }

                    Vector3d xi_vec = startVoxels.IndexToPoint(j)-startVoxels.IndexToPoint(i);

                    Vector3d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

                    double xi = xi_vec.Length;
                    double y = xi_eta_vec.Length;
                    double s = (y - xi) / xi;

                    double f = s * bond_stiffness * vol;

                    forceVoxels.cellValues[i] += f * xi_eta_vec / y;

                }

            }
        }

        public void UpdateDisp()
        {

            for (int i = 0; i < noVoxels; i++)
            {
                if (startVoxels.cellValues[i] == 0)
                {
                    continue;
                }

                dispVoxels.cellValues[i] += dt * forceVoxels.cellValues[i];
            }

        }


    }
}
