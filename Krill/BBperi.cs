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

        public Voxels<Vector3d> forceVoxels;
        public Voxels<Vector3d> oldforceVoxels;
        public Voxels<Vector3d> dispVoxels;
        public Voxels<Vector3d> velVoxels;
        public Voxels<Vector3d> accVoxels;
        public Voxels<Vector3d> densities;

        private int noVoxels;

        public BBperi(Voxels<int> orgVoxels, double Bond_stiffness, int[] neighbour_list,
            double volume)
        {
            startVoxels = orgVoxels;
            bond_stiffness = Bond_stiffness;
            nlist = neighbour_list;
            vol = volume;

            noVoxels = startVoxels.n;
            forceVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            oldforceVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            dispVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            velVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            accVoxels = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            densities = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);

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

                oldforceVoxels.cellValues[i] = forceVoxels.cellValues[i];
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

                forceVoxels.cellValues[i] += Vector3d.ZAxis;

            }
        }

        public double CalculateDampening()
        {
            double denominator = 0;
            double nominator = 0; 

            // Assuming the bonds never break, otherwise need a new if-statement
            for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
            {
                if (startVoxels.cellValues[i] == 0)
                    continue;

                Vector3d K = - (forceVoxels.cellValues[i] - oldforceVoxels.cellValues[i]);

                K.X /= (velVoxels.cellValues[i].X * densities.cellValues[i].X);
                K.Y /= (velVoxels.cellValues[i].Y * densities.cellValues[i].Y);
                K.Z /= (velVoxels.cellValues[i].Z * densities.cellValues[i].Z);

                Vector3d disp = dispVoxels.cellValues[i];

                nominator += disp.X * disp.X * Math.Abs(K.X) + disp.Y * disp.Y * Math.Abs(K.Y) + disp.Z * disp.Z * Math.Abs(K.Z);
                denominator += disp * disp;
            }

            double c = 2 * Math.Sqrt(nominator / denominator);

            return Double.IsNaN(c) ? 0.5 : c;
        }

        public void SetDensities(double delta)
        {
            int noBonds = nlist.Length;

            // Assuming the bonds never break, otherwise need a new if-statement
            for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
            {
                if (startVoxels.cellValues[i] == 0)
                    continue;

                for (int a = 0; a < noBonds; a++)
                {
                    int j = i + nlist[a];

                    if (startVoxels.cellValues[j] != 0)
                        CalcPartialDensity(i, j, delta);

                    j = i - nlist[a];
                    if (startVoxels.cellValues[j] != 0)
                        CalcPartialDensity(i, j, delta);
                }

                densities.cellValues[i] *= 1; // 1/4;
            }
        }

        void CalcPartialDensity(int i, int j, double delta)
        {
            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);

            double d = 9 / (4 * Math.PI * delta * delta * delta * delta);
            double a = bond_stiffness * 10; // Helt Dumt

            Vector3d temp = xi_vec * 4 * delta / xi_vec.SquareLength;
            
            temp.X = temp.X > 0 ? xi_vec.X : -temp.X;
            temp.Y = temp.Y > 0 ? xi_vec.Y : -temp.Y;
            temp.Z = temp.Z > 0 ? xi_vec.Z : -temp.Z;

            densities.cellValues[i] += temp * 0.5 * a * d * d * delta / xi_vec.Length * 2 * vol;
            densities.cellValues[i] += new Vector3d(0,0,temp.Z);

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

        public void UpdateDisp(double c)
        {
            // Based on velocity_verlet from Peripy

            for (int i = 0; i < noVoxels*noVoxels*noVoxels; i++)
            {
                if (startVoxels.cellValues[i] == 0)
                {
                    continue;
                }
                velVoxels.cellValues[i] *= (1-c);

                Vector3d velHalf = velVoxels.cellValues[i] + 0.5*accVoxels.cellValues[i];

                accVoxels.cellValues[i] = forceVoxels.cellValues[i]; // - c * velHalf;

                accVoxels.cellValues[i].X /= densities.cellValues[i].X;
                accVoxels.cellValues[i].Y /= densities.cellValues[i].Y;
                accVoxels.cellValues[i].Z /= densities.cellValues[i].Z;

                velVoxels.cellValues[i] = velHalf + 0.5*accVoxels.cellValues[i];

                dispVoxels.cellValues[i] = dispVoxels.cellValues[i] + (velVoxels.cellValues[i] + 0.5 * accVoxels.cellValues[i]);

            }

        }


    }
}
