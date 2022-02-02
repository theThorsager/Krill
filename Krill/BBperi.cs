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
        public double density;      // Density, probably change to list later on
        public double dt;   // Time step
        public double damping;  // Damping constant

        public Voxels<Vector3d> forceVoxels;
        public Voxels<Vector3d> dispVoxels;
        public Voxels<Vector3d> velVoxels;
        public Voxels<Vector3d> accVoxels;

        private int noVoxels;

        public BBperi(Voxels<int> orgVoxels, double Bond_stiffness, int[] neighbour_list,
            double volume, double time_step, double density, double damping)
        {
            startVoxels = orgVoxels;
            bond_stiffness = Bond_stiffness;
            nlist = neighbour_list;
            vol = volume;
            dt = time_step;
            this.density = density;
            this.damping = damping;

            noVoxels = startVoxels.n;
            forceVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            dispVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            velVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            accVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);

        }

        public void UpdateForce()
        {
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

                    if (startVoxels.cellValues[j] != 0)
                    {
                        CalcBondForce(i, j);
                    }

                    j = i - nlist[a];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        CalcBondForce(i, j);
                    }

                }

            }
        }

        void CalcBondForce(int i, int j)
        {
            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);

            Vector3d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

            double xi = xi_vec.Length;
            double y = xi_eta_vec.Length;
            double s = (y - xi) / xi;

            double f = s * bond_stiffness * vol;

            forceVoxels.cellValues[i] += f * xi_eta_vec / y;
        }

        public void UpdateDisp()
        {
            // Based on velocity_verlet from Peripy

            for (int i = 0; i < noVoxels; i++)
            {
                if (startVoxels.cellValues[i] == 0)
                {
                    continue;
                }

                Vector3d velHalf = velVoxels.cellValues[i] + (dt/2)*accVoxels.cellValues[i];

                accVoxels.cellValues[i] = (forceVoxels.cellValues[i] - damping * velHalf) / density;

                velVoxels.cellValues[i] = velHalf + (dt/2)*accVoxels.cellValues[i];

                dispVoxels.cellValues[i] = dispVoxels.cellValues[i] + dt * (velVoxels.cellValues[i] + (dt / 2) * accVoxels.cellValues[i]);

            }

        }


    }
}
