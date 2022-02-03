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
        public int padding;

        public Voxels<Vector3d> forceVoxels;
        public Voxels<Vector3d> oldforceVoxels;
        public Voxels<Vector3d> dispVoxels;
        public Voxels<Vector3d> velVoxels;
        public Voxels<Vector3d> accVoxels;
        public Voxels<Vector3d> densities;

        private int noVoxels;

        public BBperi(Voxels<int> orgVoxels, double Bond_stiffness, int[] neighbour_list,
            double volume, double delta)
        {
            startVoxels = orgVoxels;
            bond_stiffness = Bond_stiffness;
            nlist = neighbour_list;
            vol = volume;
            this.padding = (int)Math.Floor(delta);

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
            int nBonds = nlist.Length;

            // Assuming the bonds never break, otherwise need a new if-statement

            for (int i = padding; i < noVoxels - padding; i++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int k = padding; k < noVoxels - padding; k++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if (startVoxels.cellValues[I] == 0)
                            continue;

                        updateForce(I, nBonds);

                        if (startVoxels.cellValues[I] == 3)
                            applyDirechlet(I, nBonds);
                    }
                }
            }
        }

        private void applyDirechlet(int i, int nBonds)
        {
            for (int a = 0; a < nBonds; a++)
            {
                int j = i + nlist[a];

                if (startVoxels.cellValues[j] == 0)
                    CalcBondForce(i, j);

                j = i - nlist[a];
                if (startVoxels.cellValues[j] == 0)
                    CalcBondForce(i, j);
            }
        }

        private void updateForce(int i, int nBonds)
        {
            oldforceVoxels.cellValues[i] = forceVoxels.cellValues[i];
            forceVoxels.cellValues[i] = Vector3d.Zero;

            for (int a = 0; a < nBonds; a++)
            {
                int j = i + nlist[a];

                if (startVoxels.cellValues[j] != 0)
                    CalcBondForce(i, j);

                j = i - nlist[a];
                if (startVoxels.cellValues[j] != 0)
                    CalcBondForce(i, j);
            }

            // Temporary
            forceVoxels.cellValues[i] += Vector3d.ZAxis;
        }

        public double CalculateDampening()
        {
            double denominator = 0;
            double nominator = 0;

            for (int i = padding; i < noVoxels - padding; i++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int k = padding; k < noVoxels - padding; k++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if (startVoxels.cellValues[I] == 0)
                            continue;

                        Vector3d K = -(forceVoxels.cellValues[I] - oldforceVoxels.cellValues[I]);

                        K.X /= Utility.NotZero(velVoxels.cellValues[I].X * densities.cellValues[I].X);
                        K.Y /= Utility.NotZero(velVoxels.cellValues[I].Y * densities.cellValues[I].Y);
                        K.Z /= Utility.NotZero(velVoxels.cellValues[I].Z * densities.cellValues[I].Z);

                        Vector3d disp = dispVoxels.cellValues[I];

                        nominator += disp.X * disp.X * Math.Abs(K.X) + disp.Y * disp.Y * Math.Abs(K.Y) + disp.Z * disp.Z * Math.Abs(K.Z);
                        denominator += disp * disp;
                    }
                }
            }
            denominator = Utility.NotZero(denominator);
            nominator = nominator > 0 ? nominator : -nominator;
            double c = 2.0 * Math.Sqrt(nominator / denominator);

            return c;
        }

        public void SetDensities(double delta)
        {
            int noBonds = nlist.Length;

            for (int i = padding; i < noVoxels - padding; i++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int k = padding; k < noVoxels - padding; k++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if (startVoxels.cellValues[I] == 0)
                            continue;

                        for (int a = 0; a < noBonds; a++)
                        {
                            int J = I + nlist[a];

                            if (startVoxels.cellValues[J] != 0)
                                CalcPartialDensity(I, J, delta);

                            J = I - nlist[a];
                            if (startVoxels.cellValues[J] != 0)
                                CalcPartialDensity(I, J, delta);
                        }

                        densities.cellValues[I] *= 2; // 1/4;
                    }
                }
            }
        }

        void CalcPartialDensity(int i, int j, double delta)
        {
            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            double l = xi_vec.Length;
            
            xi_vec.X *= xi_vec.X;
            xi_vec.Y *= xi_vec.Y;
            xi_vec.Z *= xi_vec.Z;
            xi_vec *= bond_stiffness * vol / (l*l*l);

            densities.cellValues[i] += xi_vec;
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

            for (int i = padding; i < noVoxels - padding; i++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int k = padding; k < noVoxels - padding; k++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if (startVoxels.cellValues[I] == 0)
                            continue;

                        //velVoxels.cellValues[I] *= (1 - c);

                        Vector3d velHalf = velVoxels.cellValues[I] + 0.5 * accVoxels.cellValues[i];

                        accVoxels.cellValues[I] = forceVoxels.cellValues[I];

                        accVoxels.cellValues[I].X /= densities.cellValues[I].X;
                        accVoxels.cellValues[I].Y /= densities.cellValues[I].Y;
                        accVoxels.cellValues[I].Z /= densities.cellValues[I].Z;

                        accVoxels.cellValues[I] -= c * velHalf;

                        velVoxels.cellValues[I] = velHalf + 0.5 * accVoxels.cellValues[I];

                        dispVoxels.cellValues[I] = dispVoxels.cellValues[I] + (velVoxels.cellValues[I] + 0.5 * accVoxels.cellValues[I]);

                    }

                }
            }
        }


    }
}
