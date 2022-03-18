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
        public double[] kernel;

        public Vector3d[] nlist_xi;
        public double[] nlist_xi_length;

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

        public Voxels<double> stiffnessMod;

        public Voxels<double> weighting;
        int N;
        public Voxels<double> utilization;

        private int noVoxels;

        public bool relaxTension = false;
        double oldcuttoff = 0;
        double newcuttoff = 0;

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
            
            stiffnessMod = new Voxels<double>(startVoxels.origin, startVoxels.delta, noVoxels);
            weighting = new Voxels<double>(startVoxels.origin, startVoxels.delta, noVoxels);
            utilization = new Voxels<double>(startVoxels.origin, startVoxels.delta, noVoxels);

            N = 0;
            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & maskbit) != 0)
                        {
                            weighting.cellValues[I] = 1.0;
                            ++N;
                        }
                    }
                }
            }

            PreComputeXi();
        }

        public void PreComputeXi()
        {
            nlist_xi = new Vector3d[nlist.Length];
            nlist_xi_length = new double[nlist.Length];
            int i = noVoxels * noVoxels * noVoxels / 2;
            for (int aa = 0; aa < nlist.Length; aa++)
            {
                int j = i + nlist[aa];
                Vector3d xi = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
                double xi_length = xi.Length;
                nlist_xi[aa] = xi;
                nlist_xi_length[aa] = xi_length;
            }
        }

        public void UpdateForce(double factor = 1.0)
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
            utilization.cellValues[i] = 0;

            oldcuttoff = newcuttoff;

            for (int a = 0; a < nBonds; a++)
            {
                int jp = i + nlist[a];
                int jm = i - nlist[a];
                double surfCorr = 1;
                //surfCorr = kernel[a];
                bool p = startVoxels.cellValues[jp] != 0;
                bool m = startVoxels.cellValues[jm] != 0;
                
                if (p)
                    CalcBondForce(i, jp, surfCorr, a);

                if (m)
                    CalcBondForce(i, jm, surfCorr, a);
            }

            forceVoxels.cellValues[i] += bodyload.cellValues[i] * factor;
            forceVoxels.cellValues[i].X += spring.cellValues[i].X * dispVoxels.cellValues[i].X;
            forceVoxels.cellValues[i].Y += spring.cellValues[i].Y * dispVoxels.cellValues[i].Y;
            forceVoxels.cellValues[i].Z += spring.cellValues[i].Z * dispVoxels.cellValues[i].Z;
            utilization.cellValues[i] +=
                Math.Sqrt(Math.Abs(factor * bodyload.cellValues[i].X + spring.cellValues[i].X * dispVoxels.cellValues[i].X)) +
                Math.Sqrt(Math.Abs(factor * bodyload.cellValues[i].Y + spring.cellValues[i].Y * dispVoxels.cellValues[i].Y)) +
                Math.Sqrt(Math.Abs(factor * bodyload.cellValues[i].Z + spring.cellValues[i].Z * dispVoxels.cellValues[i].Z));

            utilization.cellValues[i] = Math.Sqrt(utilization.cellValues[i]);
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

                        nominator += disp.X * disp.X * (K.X) + disp.Y * disp.Y * (K.Y) + disp.Z * disp.Z * (K.Z);
                        denominator += disp * disp;
                    }
                }
            }
            denominator = Utility.NotZero(denominator);
            nominator = nominator > 0 ? nominator : -nominator;
            double c = 2.0 * Math.Sqrt(nominator / denominator);

            return Double.IsNaN(c) ? 0.01 : Math.Min(c, 1.0);
        }

        public void SetDensities(double delta, double factor = 20)
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

                        densities.cellValues[I] -= spring.cellValues[I];

                        //densities.cellValues[I].X += Math.Abs(bodyload.cellValues[I].X);
                        //densities.cellValues[I].Y += Math.Abs(bodyload.cellValues[I].Y);
                        //densities.cellValues[I].Z += Math.Abs(bodyload.cellValues[I].Z);

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
            int vols = startVoxels.cellValues[i] >> 20;
            vols += startVoxels.cellValues[j] >> 20;
            double factor = (double)(nlist.Length * 4.0 + 2.0) / (double)vols;
            xi_vec *= bond_stiffness * vol / (l*l*l);

            densities.cellValues[i] += xi_vec;
        }

        public void RemoveUnstressedVoxels(double factor, double E)
        {

            OutputResults vonMises = new OutputResults(new Containers.LinearSolution() { mask = startVoxels,
                                                                                         bodyload = this.bodyload,
                                                                                         nList = this.nlist,
                                                                                         elasticModulus = E,
                                                                                         bondStiffness = bond_stiffness,
                                                                                         springs = this.spring } );

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

        public double TotalDisplacement()
        {
            // deal with rigid body motion?
            double displacement = 0;
            for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
            {
                Vector3d disp = dispVoxels.cellValues[i];
                displacement += Math.Abs(disp.X) + Math.Abs(disp.Y) + Math.Abs(disp.Z);
            }
            return displacement;
        }

        public void ModifyWeightsUti()
        {
            double aveUti = 0;
            double maxUti = 0;
            double minUti = double.MaxValue;
            int count = 0;
            for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
            {
                if (weighting.cellValues[i] > 1e-6)
                {
                    double uti = utilization.cellValues[i];
                    aveUti += uti;
                    ++count;
                    if (uti > maxUti)
                        maxUti = uti;
                    if (uti < minUti)
                        minUti = uti;
                }
            }
            aveUti /= count;

            double gotyckligt = (maxUti - minUti) * 0.2 + minUti;

            Func<double, double> sigsmoid = x => x / Math.Sqrt(1 + x * x);
            // construct direction vector
            double[] dir = new double[noVoxels*noVoxels*noVoxels];
            int countPos = 0;
            int countNeg = 0;
            for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
            {
                //if ((startVoxels.cellValues[i] & maskbit) != 0)
                if (weighting.cellValues[i] > 1e-6)
                {
                    double uti = utilization.cellValues[i];
                    double x = (uti - gotyckligt) / maxUti * 10;
                    if (uti < gotyckligt)
                    { 
                        dir[i] = -Math.Min(-0.5 * sigsmoid(x), weighting.cellValues[i]); 
                        countNeg++; 
                    }
                    else
                    {
                        dir[i] = 0.00005 * sigsmoid(x);
                        countPos++; 
                    }

                    //double uti = utilization.cellValues[i];
                    //double x = (uti - gotyckligt) / maxUti / 10;
                    //if (uti < gotyckligt)
                    //{ dir[i] = 0.25 * sigsmoid(x); countNeg++; }
                    //else
                    //{ dir[i] = 0.25 * sigsmoid(x); countPos++; }

                }
            }

            // project to hyperplane
            // calculate the difference from zero
            double diff = dir.Sum();
            double factor = 1;

            // "decrease" the others to make it zero
            if (diff < 0)
            {
                double current = dir.Where(x => x > 0).Sum();
                double factorP = -diff / current + 1;

                //diff /= countPos;
                for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
                {
                    if (weighting.cellValues[i] > 1e-6 && dir[i] >= 0)
                    //{ dir[i] -= diff; }
                    { dir[i] *= factorP; }
                }
            }
            else if (diff > 0)
            {
                diff /= countNeg;
                for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
                {
                    if (weighting.cellValues[i] > 1e-6 && dir[i] <= 0)
                    { dir[i] -= diff; }
                }

                // find outlier
                for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
                {
                    //if ((startVoxels.cellValues[i] & maskbit) != 0 && influence.cellValues[i] > 1e-6)
                    if (weighting.cellValues[i] > 1e-6)
                    {
                        //double newval = dir[i] + influence.cellValues[i];
                        double temp = -weighting.cellValues[i] / dir[i];
                        if (temp > 0 && temp < factor)
                        {
                            factor = temp;
                        }
                    }
                }
            }

            // Apply change
            for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
                weighting.cellValues[i] += dir[i] * factor;

        }

        void CalcBondForce(int i, int j, double factor, int a)
        {
            int vols = startVoxels.cellValues[i] >> 20;
            vols += startVoxels.cellValues[j] >> 20;
            factor *= (double)(nlist.Length * 4.0 + 2.0) / (double)vols;

            //factor *= (1.0 / stiffnessMod.cellValues[i] + 1.0 / stiffnessMod.cellValues[j]) * 0.5;

            factor *= (weighting.cellValues[i] + weighting.cellValues[j]) * 0.5;

            //Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            Vector3d xi_vec = i < j ? nlist_xi[a] : -nlist_xi[a];

            Vector3d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

            double xi = nlist_xi_length[a];
            double y = xi_eta_vec.Length;
            double s = (y - xi) / xi;      // Engineering strain

            double f = s * bond_stiffness * vol;

            // Make the stress strain curve stranger
            const double low = 0.05;
            const double high = 0.5;
            const double fade = 0.1;

            if (relaxTension && f > low * oldcuttoff)
            {
                double fa = f / oldcuttoff;
                if (fa < low + fade)
                    f *= -(fa - (low + fade)) / fade;
                else if (fa < high - fade)
                    f = 0;
                else if (fa < high)
                    f *= (fa - (high - fade)) / fade;
                else if (f > newcuttoff)
                    newcuttoff = f;
            }

            forceVoxels.cellValues[i] += f * xi_eta_vec / y;
            utilization.cellValues[i] += Math.Sqrt(Math.Abs(f));
        }
        
        public void UpdateDisp(double c)
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

                        // Based on velocity_verlet from Peripy

                        //Vector3d velHalf = velVoxels.cellValues[I] + 0.5 * accVoxels.cellValues[I];

                        //accVoxels.cellValues[I] = forceVoxels.cellValues[I];

                        //accVoxels.cellValues[I].X /= densities.cellValues[I].X;
                        //accVoxels.cellValues[I].Y /= densities.cellValues[I].Y;
                        //accVoxels.cellValues[I].Z /= densities.cellValues[I].Z;

                        //accVoxels.cellValues[I] -= c * velHalf;

                        //velVoxels.cellValues[I] = velHalf + 0.5 * accVoxels.cellValues[I];

                        //dispVoxels.cellValues[I] = dispVoxels.cellValues[I] + (velVoxels.cellValues[I] + 0.5 * accVoxels.cellValues[I]);

                        ///////////////////////////////////////////////////////////////////
                        // Based on Wikipedias Velocity Verlet with added dampening

                        accVoxels.cellValues[I] = forceVoxels.cellValues[I];

                        accVoxels.cellValues[I].X /= densities.cellValues[I].X;
                        accVoxels.cellValues[I].Y /= densities.cellValues[I].Y;
                        accVoxels.cellValues[I].Z /= densities.cellValues[I].Z;

                        accVoxels.cellValues[I] -= c * velVoxels.cellValues[I];
                        velVoxels.cellValues[I] += 0.5 * accVoxels.cellValues[I];

                        dispVoxels.cellValues[I] += velVoxels.cellValues[I] + 0.5 * accVoxels.cellValues[I];

                        velVoxels.cellValues[I] += 0.5 * accVoxels.cellValues[I];
                    }
                }
            }
        }

        public double ComputeResidual(double F)
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
            for (int i = 0; i < noVoxels*noVoxels*noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                ++n;
            }
            //F *= bond_stiffness / n;
            
            F = Math.Max(0.001, F);

            // average residual per point
            R /= (double)n;

            return R / F;
        }

        public double ComputeF(List<IBoundaryCondition> bcs, double E)
        {
            double CharesteristicStiffness = E;

            Vector3d totalVec = new Vector3d();
            double absoluteForce = 0;

            foreach (IBoundaryCondition bc in bcs)
            {
                if (bc is BoundaryConditionDirechlet bcD)
                {
                    var areae = AreaMassProperties.Compute(bcD.area);
                    absoluteForce += bcD.displacement.Length * CharesteristicStiffness * areae.Area;
                    if (bcD.normal)
                    {
                        int count = bcD.area.FaceNormals.Count;
                        for (int i = 0; i < count; ++i)
                        {
                            totalVec += (Vector3d)bcD.area.FaceNormals[i] * bcD.displacement.Z * CharesteristicStiffness * areae.Area / count;
                        }
                    }
                    else
                    {
                        totalVec += bcD.displacement * CharesteristicStiffness * areae.Area;
                    }
                }
                else if (bc is BoundaryConditionNuemann bcN)
                {
                    absoluteForce += bcN.load.Length;
                    if (bcN.normal)
                    {
                        int count = bcN.area.FaceNormals.Count; 
                        for (int i = 0; i < count; ++i)
                        {
                            totalVec += (Vector3d)bcN.area.FaceNormals[i] * bcN.load.Z / count;
                        }
                    }
                    else
                        totalVec += bcN.load;
                }
            }

            return absoluteForce - totalVec.Length;
        }

        public void SetNuemann(BoundaryConditionNuemann bc, int tag)
        {
            int tot = startVoxels.cellValues.Count(x => (x & tag) != 0 && (x & 3) != 0);

            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        if ((startVoxels.cellValues[I] & tag) == 0 || (startVoxels.cellValues[I] & 3) == 0)
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
                        if ((startVoxels.cellValues[I] & maskbit) == 0 || (startVoxels.cellValues[I] & bc.tag) == 0)
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

            int sum = startVoxels.cellValues[i] >> 20;
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
            startVoxels.cellValues[i] &= ~(0xFFFFF << 20);
            startVoxels.cellValues[i] |= ((sum + localsum) << 20);

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
            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            double t = Rhino.Geometry.Intersect.Intersection.MeshRay(bc.area, new Ray3d(startVoxels.IndexToPoint(i), xi_vec));
            if (t < 0)
                return Vector3d.Zero;

            sum++;
            xi_vec *= t;

            double l = xi_vec.Length;
            stiffnessMod.cellValues[i] += l;

            xi_vec.X *= xi_vec.X;
            xi_vec.Y *= xi_vec.Y;
            xi_vec.Z *= xi_vec.Z;
            int vols = startVoxels.cellValues[i] >> 20;
            vols += startVoxels.cellValues[j] >> 20;
            double factor = (double)(nlist.Length * 4.0 + 2.0) / (double)vols;

            xi_vec *= bond_stiffness * vol / (l * l * l);

            return -xi_vec;
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

                        startVoxels.cellValues[I] |= sum << 20;
                    }
                }
            }
        }

        public void SetVolumesStiffness()
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

                        double sum = 0;
                        for (int jj = 0; jj < nlist.Length; jj++)
                        {
                            int J = I + nlist[jj];
                            if ((startVoxels.cellValues[J]) != 0)
                                sum += (startVoxels.IndexToPoint(J) - startVoxels.IndexToPoint(I)).Length;
                            J = I - nlist[jj];
                            if ((startVoxels.cellValues[J]) != 0)
                                sum += (startVoxels.IndexToPoint(J) - startVoxels.IndexToPoint(I)).Length;
                        }

                        stiffnessMod.cellValues[I] += sum;
                    }
                }
            }
        }

        public double KineticEnergy()
        {
            // 0.5 * m * v^2

            double kineticEnergy = 0;
            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        kineticEnergy += velVoxels.cellValues[I].SquareLength;
                    }
                }
            }

            return 0.5 * vol * kineticEnergy;
        }

        public void ZeroVelocities()
        {
            for (int k = padding; k < noVoxels - padding; k++)
            {
                for (int j = padding; j < noVoxels - padding; j++)
                {
                    for (int i = padding; i < noVoxels - padding; i++)
                    {
                        int I = startVoxels.ToLinearIndex(i, j, k);
                        velVoxels.cellValues[I] = Vector3d.Zero;
                    }
                }
            }
        }
    }
}
