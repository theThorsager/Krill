﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using Krill.Containers;

namespace Krill
{
    internal class BBperi
    {
        const int maskbit = 0x000000FF;

        public double bond_stiffness;
        public Voxels<int> startVoxels;
        public int[] nlist;    // Neighbour list
        public double[] kernel;
        public double vol;   // Volume of the particles (same for all)
        public int padding;

        public Voxels<Vector3d> forceVoxels;
        public Voxels<Vector3d> oldforceVoxels;
        public Voxels<Vector3d> dispVoxels;
        public Voxels<Vector3d> velVoxels;
        public Voxels<Vector3d> accVoxels;
        public Voxels<Vector3d> densities;

        public Voxels<Vector3d> spring;
        public Voxels<Vector3d> bodyload;

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

            spring = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
            bodyload = new Voxels<Vector3d>(startVoxels.origin, startVoxels.delta, noVoxels);
        }

        public void UpdateForce(double factor = 1)
        {
            int nBonds = nlist.Length;

            // Assuming the bonds never break, otherwise need a new if-statement

            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & maskbit) == 0)
                            continue;

                        updateForce(I, nBonds, factor);
                    }
                }
            }
        }


        private void updateForce(int i, int nBonds, double factor)
        {
            oldforceVoxels.cellValues[i] = forceVoxels.cellValues[i];
            forceVoxels.cellValues[i] = Vector3d.Zero;

            for (int a = 0; a < nBonds; a++)
            {
                int jp = i + nlist[a];
                int jm = i - nlist[a];
                double surfCorr = 1;
                //surfCorr = kernel[a];
                bool p = startVoxels.cellValues[jp] != 0;
                bool m = startVoxels.cellValues[jm] != 0;
                
                if (p)
                    CalcBondForce(i, jp, surfCorr);

                if (m)
                    CalcBondForce(i, jm, surfCorr);
            }

            forceVoxels.cellValues[i] += bodyload.cellValues[i] * factor;
            forceVoxels.cellValues[i].X += spring.cellValues[i].X * dispVoxels.cellValues[i].X;
            forceVoxels.cellValues[i].Y += spring.cellValues[i].Y * dispVoxels.cellValues[i].Y;
            forceVoxels.cellValues[i].Z += spring.cellValues[i].Z * dispVoxels.cellValues[i].Z;

        }

        public double CalculateDampening()
        {
            double denominator = 0;
            double nominator = 0;

            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & maskbit) == 0)
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

            return Math.Min(c, 1.0);
        }

        public void SetDensities(double delta, double factor = 24)
        {
            int noBonds = nlist.Length;

            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & maskbit) == 0)
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

                        densities.cellValues[I] *= 0.25 * factor; // 1/4;
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

        public void RemoveUnstressedVoxels(double factor, double E)
        {
            OutputResults vonMises = new OutputResults(startVoxels, nlist, E, 0.25);

            vonMises.UpdateStrains(dispVoxels);
            vonMises.UpdateStresses();
            vonMises.UpdateVonMises();

            double maxVon = vonMises.vonMises.cellValues.Max();
            double cutoff = maxVon * factor;

            for (int i = 0; i < noVoxels*noVoxels*noVoxels; i++)
            {
                if (vonMises.vonMises.cellValues[i] < cutoff)
                    startVoxels.cellValues[i] = 0;
            }

        }

        void CalcBondForce(int i, int j, double factor)
        {
            int vols = startVoxels.cellValues[i] >> 24;
            vols += startVoxels.cellValues[j] >> 24;
            factor *= (double)(nlist.Length * 4.0 + 2.0) / (double)vols;

            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);

            Vector3d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

            double xi = xi_vec.Length;
            double y = xi_eta_vec.Length;
            double s = (y - xi) / xi;      // Engineering strain
            double s2 = Math.Log(y / xi);   // True strain
            double s3 = (y * y - xi * xi) / (2 * xi * xi);   // Green Lagrange strain

            double f = s * bond_stiffness * vol;
            forceVoxels.cellValues[i] += factor * f * xi_eta_vec / y;
        }

        public void UpdateDisp(double c)
        {
            // Based on velocity_verlet from Peripy

            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & maskbit) == 0)
                            continue;

                        Vector3d velHalf = velVoxels.cellValues[I] + 0.5 * accVoxels.cellValues[I];

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

        public double ComputeResidual()
        {
            // residual of our equations
            double R = forceVoxels.cellValues.Sum(x =>
            {
                double res = x.X > 0 ? x.X : -x.X;
                res += x.Y > 0 ? x.Y : -x.Y;
                res += x.Z > 0 ? x.Z : -x.Z;
                return res;
            }
            );

            // number of relevant points
            int n = 0;

            // scale with "force" in the system
            // Average displacement * stiffness ? :)
            double F = 0;
            for (int i = 0; i < noVoxels*noVoxels*noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                F += dispVoxels.cellValues[i].Length;
                ++n;
            }
            F *= bond_stiffness / n;
            
            F = Math.Max(0.001, F);

            // average residual per point
            R /= (double)n;

            return R / F;
        }

        public void SetNuemann(BoundaryConditionNuemann bc, int tag)
        {
            int tot = startVoxels.cellValues.Count(x => (x & tag) != 0);

            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & tag) == 0)
                            continue;

                        Vector3d load = bc.load;
                        // divide by number of cells
                        load /= tot;

                        // reorient to normal direction
                        if (bc.normal)
                        {
                            bc.area.ClosestPoint(startVoxels.IndexToPoint(I), out Point3d trash, out Vector3d normal, startVoxels.delta * 0.87 /* sqrt(3)/2 */);
                            load = normal * load.Z;
                        }

                        bodyload.cellValues[I] += load;
                    }
                }
            }
        }

        public void SetDirechlets(BoundaryConditionDirechlet bc)
        {
            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & maskbit) == 0)
                            continue;

                        SetDirechlet(I, bc);
                    }
                }
            }
        }

        void SetDirechlet(int i, BoundaryConditionDirechlet bc)
        {
            var displacment = bc.displacement;

            var resBodyLoad = new Vector3d();
            var resSpringConstant = new Vector3d();

            int sum = startVoxels.cellValues[i] >> 24;
            int localsum = 0;
            bc.area.ClosestPoint(startVoxels.IndexToPoint(i), out Point3d trash, out Vector3d normal, padding * startVoxels.delta * 0.87 * 2 /* sqrt(3)/2 */);
            for (int j = 0; j < this.nlist.Length; j++)
            {
                // Find all spring constants of springs that are locked by Direchlet
                int J = i + this.nlist[j];
                if (startVoxels.cellValues[J] == bc.tag)
                    resSpringConstant += setDirechlet(i, J, bc, ref localsum);

                J = i - this.nlist[j];
                if (startVoxels.cellValues[J] == bc.tag)
                    resSpringConstant += setDirechlet(i, J, bc, ref localsum);
                // sum all different constant into one for each direction
                // Sum for springs in parallel

            }
            startVoxels.cellValues[i] &= ~(0xFFF << 24);
            startVoxels.cellValues[i] |= ((sum + localsum) << 24);

            resSpringConstant.X *= bc.lockX ? 1 : 0;
            resSpringConstant.Y *= bc.lockY ? 1 : 0;
            resSpringConstant.Z *= bc.lockZ ? 1 : 0;


            Vector3d disp = bc.normal ? normal * bc.displacement.Z : bc.displacement;

            spring.cellValues[i] += resSpringConstant;
            // Calculate a bodyload based on enforced displacment and the spring constant
            resBodyLoad.X = -disp.X * resSpringConstant.X;
            resBodyLoad.Y = -disp.Y * resSpringConstant.Y;
            resBodyLoad.Z = -disp.Z * resSpringConstant.Z;
            bodyload.cellValues[i] += resBodyLoad;
        }

        private Vector3d setDirechlet(int i, int j, BoundaryConditionDirechlet bc, ref int sum)
        {
            // Find how much of this vector that is in deformable volume
            // TODO!!
            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            double t = Rhino.Geometry.Intersect.Intersection.MeshRay(bc.area, new Ray3d(startVoxels.IndexToPoint(i), xi_vec));
            if (t < 0)
                return Vector3d.Zero;

            sum++;
            xi_vec *= t;    // new start length

            double length = xi_vec.Length;
            double h = 1e-6;

            double sx = ((xi_vec + h * Vector3d.XAxis).Length - length) / length;
            Vector3d sx_vec = sx * xi_vec / length;
            sx_vec *= bond_stiffness * vol / h;
            double sy = ((xi_vec + h * Vector3d.YAxis).Length - length) / length;
            Vector3d sy_vec = sy * xi_vec / length;
            sy_vec *= bond_stiffness * vol / h;
            double sz = ((xi_vec + h * Vector3d.ZAxis).Length - length) / length;
            Vector3d sz_vec = sz * xi_vec / length;
            sz_vec *= bond_stiffness * vol / h;

            // split up for all cardinal directions
            double lx = sx_vec.X < 0 ? -sx_vec.X : sx_vec.X;
            double fx = -lx;
            double ly = sy_vec.Y < 0 ? -sy_vec.Y : sy_vec.Y;
            double fy = -ly;
            double lz = sz_vec.Z < 0 ? -sz_vec.Z : sz_vec.Z;
            double fz = -lz;

            return new Vector3d(fx, fy, fz);
        }

        public void SetVolumes()
        {
            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I]) == 0)
                            continue;

                        int sum = 1;
                        for (int jj = 0; jj < nlist.Length; jj++)
                        {
                            int J = I + nlist[jj];
                            if ((startVoxels.cellValues[J]) != 0)
                                ++sum;
                            J = I - nlist[jj];
                            if ((startVoxels.cellValues[J]) != 0)
                                ++sum;
                        }

                        startVoxels.cellValues[I] |= sum << 24;
                    }
                }
            }
        }
    }
}
