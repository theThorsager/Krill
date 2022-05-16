using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using Krill.Containers;

namespace Krill
{
    internal class BBperi2dState
    {
        public Voxels2d<Vector2d> forceVoxels;
        public Voxels2d<Vector2d> dispVoxels;
        public Voxels2d<Vector2d> velVoxels;

        public bool relaxTension = false;
        public double oldcuttoff = 0;

        public int i = -1;

        //public object handle = new object();
        public BBperi2dState(BBperi2d bperi)
        {
            forceVoxels = bperi.forceVoxels;
            dispVoxels = bperi.dispVoxels;
            velVoxels = bperi.velVoxels;
        }

        public void LastUpdate(Voxels2d<int> mask, int[] nlist, BBperi2d bperi, int i)
        {
            Voxels2d<int>.SoftenNearMask(mask, bperi.forceVoxels, nlist);
            Voxels2d<int>.SoftenNearMask(mask, bperi.dispVoxels, nlist);
            Voxels2d<int>.SoftenNearMask(mask, bperi.velVoxels, nlist);

            relaxTension = bperi.relaxTension;
            oldcuttoff = bperi.oldcuttoff;

            this.i = i;
        }
    }

    internal class BBperi2d
    {
        const int maskbit = 0x000000FF;

        public double bond_stiffness;
        public Voxels2d<int> startVoxels;
        public int[] nlist;    // Neighbour list
        public double[] kernel;
        public Vector2d[] nlist_xi;
        public double[] nlist_xi_length;
        public double vol;   // Volume of the particles (same for all)
        public int padding;

        public Voxels2d<Vector2d> forceVoxels;
        public Voxels2d<Vector2d> oldforceVoxels;
        public Voxels2d<Vector2d> dispVoxels;
        public Voxels2d<Vector2d> velVoxels;
        public Voxels2d<Vector2d> accVoxels;
        public Voxels2d<Vector2d> densities;

        public Voxels2d<Vector2d> spring;
        public Voxels2d<Vector2d> bodyload;

        public Voxels2d<double> stiffnessMod;

        public Voxels2d<double> weighting;
        int N;
        public Voxels2d<double> utilization;

        private int noVoxels;

        public bool relaxTension = false;
        public double oldcuttoff = 0;
        double newcuttoff = 0;

        public BBperi2d(Voxels2d<int> orgVoxels, double Bond_stiffness, int[] neighbour_list,
            double volume, double delta)
        {
            startVoxels = orgVoxels;
            bond_stiffness = Bond_stiffness;
            nlist = neighbour_list;
            vol = volume;
            this.padding = (int)Math.Floor(delta);

            noVoxels = startVoxels.n;
            forceVoxels = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);
            oldforceVoxels = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);
            dispVoxels = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);
            velVoxels = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);
            accVoxels = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);
            densities = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);

            spring = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);
            bodyload = new Voxels2d<Vector2d>(startVoxels.origin, startVoxels.delta, noVoxels);

            stiffnessMod = new Voxels2d<double>(startVoxels.origin, startVoxels.delta, noVoxels);
            weighting = new Voxels2d<double>(startVoxels.origin, startVoxels.delta, noVoxels);
            utilization = new Voxels2d<double>(startVoxels.origin, startVoxels.delta, noVoxels);

            N = 0;
            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
                    if ((startVoxels.cellValues[I] & maskbit) != 0)
                    {
                        weighting.cellValues[I] = 1.0;
                        ++N;
                    }
                }
            }

            PreComputeXi();
        }
        public void ApplyState(BBperi2dState bbState)
        {
            if (bbState == null)
                return;

            Voxels2d<int>.LERPFrom(forceVoxels, bbState.forceVoxels);
            Voxels2d<int>.LERPFrom(dispVoxels, bbState.dispVoxels);
            Voxels2d<int>.LERPFrom(velVoxels, bbState.velVoxels);

            relaxTension = bbState.relaxTension;
            oldcuttoff = bbState.oldcuttoff;
        }
        public void PreComputeXi()
        {
            nlist_xi = new Vector2d[nlist.Length];
            nlist_xi_length = new double[nlist.Length];
            int i = startVoxels.ToLinearIndex(startVoxels.n / 2, startVoxels.n / 2);
            for (int aa = 0; aa < nlist.Length; aa++)
            {
                int j = i + nlist[aa];
                Vector2d xi = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
                double xi_length = xi.Length;
                nlist_xi[aa] = xi;
                nlist_xi_length[aa] = xi_length;
            }
        }

        public void UpdateForce(double factor = 1.0)
        {
            int nBonds = nlist.Length;

            // Assuming the bonds never break, otherwise need a new if-statement

            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
                    if ((startVoxels.cellValues[I] & maskbit) == 0)
                        continue;

                    updateForce(I, nBonds, factor);
                }
            }
        }


        private void updateForce(int i, int nBonds, double factor)
        {
            oldforceVoxels.cellValues[i] = forceVoxels.cellValues[i];
            forceVoxels.cellValues[i] = Vector2d.Zero;
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

            forceVoxels.cellValues[i] += bodyload.cellValues[i] * factor / vol;
            forceVoxels.cellValues[i].X += spring.cellValues[i].X * dispVoxels.cellValues[i].X;
            forceVoxels.cellValues[i].Y += spring.cellValues[i].Y * dispVoxels.cellValues[i].Y;
            utilization.cellValues[i] +=
                Math.Sqrt(Math.Abs(factor * bodyload.cellValues[i].X + spring.cellValues[i].X * dispVoxels.cellValues[i].X)) +
                Math.Sqrt(Math.Abs(factor * bodyload.cellValues[i].Y + spring.cellValues[i].Y * dispVoxels.cellValues[i].Y));

            utilization.cellValues[i] = Math.Sqrt(utilization.cellValues[i]);
        }

        public double CalculateDampening()
        {
            double denominator = 0;
            double nominator = 0;

            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
                    if ((startVoxels.cellValues[I] & maskbit) == 0)
                        continue;

                    Vector2d K = -(forceVoxels.cellValues[I] - oldforceVoxels.cellValues[I]);

                    K.X /= Utility.NotZero(velVoxels.cellValues[I].X * densities.cellValues[I].X);
                    K.Y /= Utility.NotZero(velVoxels.cellValues[I].Y * densities.cellValues[I].Y);

                    Vector2d disp = dispVoxels.cellValues[I];

                    nominator += disp.X * disp.X * (K.X) + disp.Y * disp.Y * (K.Y);
                    denominator += disp * disp;
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

            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
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

        void CalcPartialDensity(int i, int j, double delta)
        {
            Vector2d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            double l = xi_vec.Length;
            
            xi_vec.X *= xi_vec.X;
            xi_vec.Y *= xi_vec.Y;
            int vols = startVoxels.cellValues[i] >> 20;
            vols += startVoxels.cellValues[j] >> 20;
            double factor = (double)(nlist.Length * 4.0 + 2.0) / (double)vols;
            xi_vec *= factor * bond_stiffness * vol / (l*l*l);

            densities.cellValues[i] += xi_vec;
        }

        public double TotalDisplacement()
        {
            // deal with rigid body motion?
            double displacement = 0;
            for (int i = 0; i < noVoxels * noVoxels * noVoxels; i++)
            {
                Vector2d disp = dispVoxels.cellValues[i];
                displacement += Math.Abs(disp.X) + Math.Abs(disp.Y);
            }
            return displacement;
        }

        public void ModifyWeightsUti()
        {
            double aveUti = 0;
            double maxUti = 0;
            double minUti = double.MaxValue;
            int count = 0;
            for (int i = 0; i < noVoxels * noVoxels; i++)
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
            double[] dir = new double[noVoxels*noVoxels];
            int countPos = 0;
            int countNeg = 0;
            for (int i = 0; i < noVoxels * noVoxels; i++)
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
                for (int i = 0; i < noVoxels * noVoxels; i++)
                {
                    if (weighting.cellValues[i] > 1e-6 && dir[i] >= 0)
                    //{ dir[i] -= diff; }
                    { dir[i] *= factorP; }
                }
            }
            else if (diff > 0)
            {
                diff /= countNeg;
                for (int i = 0; i < noVoxels * noVoxels; i++)
                {
                    if (weighting.cellValues[i] > 1e-6 && dir[i] <= 0)
                    { dir[i] -= diff; }
                }

                // find outlier
                for (int i = 0; i < noVoxels * noVoxels; i++)
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
            for (int i = 0; i < noVoxels * noVoxels; i++)
                weighting.cellValues[i] += dir[i] * factor;

        }

        void CalcBondForce(int i, int j, double factor, int a)
        {
            int vols = startVoxels.cellValues[i] >> 20;
            vols += startVoxels.cellValues[j] >> 20;
            factor *= (double)(nlist.Length * 4.0 + 2.0) / (double)vols;

            //factor *= (1.0 / stiffnessMod.cellValues[i] + 1.0 / stiffnessMod.cellValues[j]) * 0.5;

            //factor *= (weighting.cellValues[i] + weighting.cellValues[j]) * 0.5;

            //Vector2d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            Vector2d xi_vec = i < j ? nlist_xi[a] : -nlist_xi[a];

            Vector2d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

            double xi = nlist_xi_length[a];
            double y = xi_eta_vec.Length;
            double s = (y - xi) / xi;      // Engineering strain

            double f = s * bond_stiffness * vol * factor;

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

            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
                    if ((startVoxels.cellValues[I] & maskbit) == 0)
                        continue;

                    // Based on velocity_verlet from Peripy

                    //Vector2d velHalf = velVoxels.cellValues[I] + 0.5 * accVoxels.cellValues[I];

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

                    accVoxels.cellValues[I] -= c * velVoxels.cellValues[I];
                    velVoxels.cellValues[I] += accVoxels.cellValues[I];

                    dispVoxels.cellValues[I] += velVoxels.cellValues[I];
                }
            }
        }

        public double ComputeResidual(double F)
        {
            // residual of our equations
            double R = 0;

            // number of relevant points
            int n = 0;

            // scale with "force" in the system
            // Average displacement * stiffness ? :)
            for (int i = 0; i < noVoxels*noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                var x = forceVoxels.cellValues[i];
                double res = x.X > 0 ? x.X : -x.X;
                res += x.Y > 0 ? x.Y : -x.Y;
                R += res;
                ++n;
            }
            //F *= bond_stiffness / n;
            
            F = Math.Max(0.001, F);

            // average residual per point
            R /= (double)n;

            return R / F;
        }
        
        public double ComputeF(List<IBoundaryCondition2d> bcs, double E)
        {
            double CharesteristicStiffness = E;

            Vector2d totalVec = new Vector2d();
            double absoluteForce = 0;

            foreach (IBoundaryCondition2d bc in bcs)
            {
                if (bc is BoundaryConditionDirechlet2d bcD)
                {
                    double length = bc.curve.GetLength();
                    absoluteForce += bcD.displacement.Length * CharesteristicStiffness * length;
                    if (bcD.normal)
                    {
                        int count = 10;
                        for (int i = 0; i < count; ++i)
                        {
                            var normal = Vector3d.CrossProduct(bcD.curve.TangentAt(i / count), Vector3d.ZAxis);
                            totalVec += new Vector2d(normal.X, normal.Y) * bcD.displacement.Y * CharesteristicStiffness * length / count;
                        }
                    }
                    else
                    {
                        totalVec += new Vector2d(bcD.displacement.X, bcD.displacement.Y) * CharesteristicStiffness * length;
                    }
                }
                else if (bc is BoundaryConditionNuemann2d bcN)
                {
                    absoluteForce += bcN.load.Length;
                    if (bcN.normal)
                    {
                        int count = 10; 
                        for (int i = 0; i < count; ++i)
                        {
                            var normal = Vector3d.CrossProduct(bcN.curve.TangentAt(i / count), Vector3d.ZAxis);
                            totalVec += new Vector2d(normal.X, normal.Y) * bcN.load.Y / count;
                        }
                    }
                    else
                        totalVec += new Vector2d(bcN.load.X, bcN.load.Y);
                }
            }

            return absoluteForce / vol;
        }


        public void SetNuemann(BoundaryConditionNuemann2d bc, int tag)
        {
            // Find on how many voxels the load will be placed, such that the load per exterior voxel can be set

            // Redistribute those onto the interior voxels through the bond stiffnesses
            if (bc.load.SquareLength < 1e-12)
                return;

            var xis = nlist_xi.Zip(nlist_xi_length, (xi, l) => xi / l).ToArray();

            int count = 0;
            List<int> indices = new List<int>();
            var loads = new List<Vector3d>();
            var dummyNormals = new List<Vector3d>();
            for (int i = 0; i < noVoxels * noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & tag) == 0 || (startVoxels.cellValues[i] & 3) == 0)
                    continue;

                // Find load direction
                Vector3d localLoad = bc.load;
                if (bc.normal)
                {
                    var pt = startVoxels.IndexToPoint(i);
                    bc.curve.ClosestPoint(new Point3d(pt.X, pt.Y, 0), out var t);
                    var normal3d = Vector3d.CrossProduct(bc.curve.TangentAt(t), Vector3d.ZAxis);
                    var surfaceNormal = new Vector2d(normal3d.X, normal3d.Y);
                    localLoad = normal3d * bc.load.Y;
                }
                localLoad.Unitize();

                Vector3d localLoadFactor = new Vector3d();
                // 

                bool connects = false;
                for (int a = 0; a < nlist.Length; a++)
                {
                    int J = i + nlist[a];
                    if ((startVoxels.cellValues[J] & tag) != 0 && (startVoxels.cellValues[J] & 3) == 0)
                    {
                        connects = true;

                        Vector2d xi = xis[a];
                        localLoadFactor += new Vector3d(
                            xi.X * xi.X * localLoad.X + xi.X * xi.Y * localLoad.Y,
                            xi.Y * xi.X * localLoad.X + xi.Y * xi.Y * localLoad.Y,
                            0);
                    }

                    J = i - nlist[a];
                    if ((startVoxels.cellValues[J] & tag) != 0 && (startVoxels.cellValues[J] & 3) == 0)
                    {
                        connects = true;

                        Vector2d xi = xis[a];
                        localLoadFactor += new Vector3d(
                            xi.X * xi.X * localLoad.X + xi.X * xi.Y * localLoad.Y,
                            xi.Y * xi.X * localLoad.X + xi.Y * xi.Y * localLoad.Y,
                            0);
                    }
                }
                if (connects)
                {
                    count++;

                    double volumeFactor = (double)(nlist.Length * 2.0 + 1.0) / (double)(startVoxels.cellValues[i] >> 20);

                    localLoadFactor = localLoad * localLoadFactor.Length * volumeFactor;

                    loads.Add(localLoadFactor);
                    indices.Add(i);
                }
            }

            Vector3d totalFactor = new Vector3d();
            foreach (var v in loads)
                totalFactor += v;

            double factor = bc.load.Length / totalFactor.Length;

            for (int ii = 0; ii < indices.Count; ii++)
            {
                int i = indices[ii];
                var load2d = new Vector2d(loads[ii].X, loads[ii].Y);
                bodyload.cellValues[i] += load2d * factor;
            }
        }
        public void SetDirechlets(BoundaryConditionDirechlet2d bc)
        {
            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
                    if ((startVoxels.cellValues[I] & maskbit) == 0 || (startVoxels.cellValues[I] & bc.tag) == 0)
                        continue;

                    SetDirechlet(I, bc);
                }
            }
        }

        void SetDirechlet(int i, BoundaryConditionDirechlet2d bc)
        {
            var displacment = bc.displacement;

            var resBodyLoad = new Vector2d();
            var resSpringConstant = new Vector2d();

            int sum = startVoxels.cellValues[i] >> 20;
            int localsum = 0;

            var pt = startVoxels.IndexToPoint(i);
            bc.curve.ClosestPoint(new Point3d(pt.X, pt.Y, 0), out var t);
            var normal = Vector3d.CrossProduct(bc.curve.TangentAt(t), Vector3d.ZAxis);
            Vector2d normal2d = new Vector2d(normal.X, normal.Y);

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

            Vector2d disp = bc.normal ? normal2d * bc.displacement.Y : new Vector2d(bc.displacement.X, bc.displacement.Y);

            double factor = (double)(nlist.Length * 2.0 + 1.0) / (double)(sum + localsum);
            //factor = 1;
            resSpringConstant *= factor;
            spring.cellValues[i] += resSpringConstant;
            // Calculate a bodyload based on enforced displacement and the spring constant
            resBodyLoad.X = -disp.X * resSpringConstant.X * vol;
            resBodyLoad.Y = -disp.Y * resSpringConstant.Y * vol;
            bodyload.cellValues[i] += resBodyLoad;
        }

        private Vector2d setDirechlet(int i, int j, BoundaryConditionDirechlet2d bc, ref int sum)
        {
            // Find how much of this vector that is in deformable volume
            Vector2d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            Point2d point = startVoxels.IndexToPoint(i);

            Line line = new Line(new Point3d(point.X, point.Y, 0),
                                    new Vector3d(xi_vec.X, xi_vec.Y, 0));

            var intersection = Rhino.Geometry.Intersect.Intersection.
                CurveCurve(bc.curve, line.ToNurbsCurve(), 1e-6, 1e-6);
            double? ti = intersection.FirstOrDefault(x => x.ParameterB > 0)?.ParameterB;
            if (ti is null || ti <= 0)
                return Vector2d.Zero;

            double t = ti.Value;

            sum++;
            xi_vec *= t / xi_vec.Length;

            double l = xi_vec.Length;
            stiffnessMod.cellValues[i] += l;

            xi_vec.X *= xi_vec.X;
            xi_vec.Y *= xi_vec.Y;
            xi_vec *= bond_stiffness * vol / (l * l * l);

            return -xi_vec;
        }

        public void SetVolumes()
        {
            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
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

                    startVoxels.cellValues[I] &= ~(0xFFFFF << 20);
                    startVoxels.cellValues[I] |= sum << 20;
                }
            }
        }

        public void SetVolumesStiffness()
        {
            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
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

        public double KineticEnergy()
        {
            // 0.5 * m * v^2

            double kineticEnergy = 0;
            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
                    kineticEnergy += velVoxels.cellValues[I].SquareLength;
                }
            }

            return 0.5 * vol * kineticEnergy;
        }

        public void ZeroVelocities()
        {
            for (int j = padding; j < noVoxels - padding; j++)
            {
                for (int i = padding; i < noVoxels - padding; i++)
                {
                    int I = startVoxels.ToLinearIndex(i, j);
                    velVoxels.cellValues[I] = Vector2d.Zero;
                }
            }
        }
    }
}
