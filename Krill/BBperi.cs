using System;
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
            this.padding = 0; // (int)Math.Floor(delta);

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
                        if ((startVoxels.cellValues[I] & maskbit) == 0)
                            continue;

                        updateForce(I, nBonds);
                    }
                }
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

            forceVoxels.cellValues[i] += bodyload.cellValues[i];
            forceVoxels.cellValues[i].X += spring.cellValues[i].X * dispVoxels.cellValues[i].X;
            forceVoxels.cellValues[i].Y += spring.cellValues[i].Y * dispVoxels.cellValues[i].Y;
            forceVoxels.cellValues[i].Z += spring.cellValues[i].Z * dispVoxels.cellValues[i].Z;

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

                        densities.cellValues[I] *= 6; // 1/4;
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
                        if ((startVoxels.cellValues[I] & maskbit) == 0)
                            continue;

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
            
            F = Math.Max(1, F);

            // average residual per point
            R /= (double)n;

            return R / F;
        }

        public void SetDirechlets(BoundaryConditionDirechlet bc)
        {
            for (int i = padding; i < noVoxels - padding; i++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int k = padding; k < noVoxels - padding; k++)
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

            for (int j = 0; j < this.nlist.Length; j++)
            {
                // Find all spring constants of springs that are locked by Direchlet
                int J = i + this.nlist[j];
                if (startVoxels.cellValues[J] == bc.tag)
                    resSpringConstant += setDirechlet(i, J, bc);

                J = i - this.nlist[j];
                if (startVoxels.cellValues[J] == bc.tag)
                    resSpringConstant += setDirechlet(i, J, bc);
                // sum all different constant into one for each direction
                // Sum for springs in parallel

            }

            resSpringConstant.X *= bc.lockX ? 1 : 0;
            resSpringConstant.Y *= bc.lockY ? 1 : 0;
            resSpringConstant.Z *= bc.lockZ ? 1 : 0;

            spring.cellValues[i] += resSpringConstant;
            // Calculate a bodyload based on enforced displacment and the spring constant
            resBodyLoad.X = -displacment.X * resSpringConstant.X;
            resBodyLoad.Y = -displacment.Y * resSpringConstant.Y;
            resBodyLoad.Z = -displacment.Z * resSpringConstant.Z;
            bodyload.cellValues[i] += resBodyLoad;
        }

        private Vector3d setDirechlet(int i, int j, BoundaryConditionDirechlet bc)
        {
            // Find how much of this vector that is in deformable volume
            // TODO!!
            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            double t = Rhino.Geometry.Intersect.Intersection.MeshRay(bc.area, new Ray3d(startVoxels.IndexToPoint(i), xi_vec));
            if (t < 0)
                return Vector3d.Zero;

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

    }
}
