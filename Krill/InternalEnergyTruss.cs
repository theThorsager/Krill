using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using CSparse;
using CSparse.Double;
using CSparse.Double.Factorization;
using CSparse.Storage;
using Rhino.Geometry;

namespace Krill
{
    internal class InternalEnergyTruss
    {
        Tuple<int, int>[] connections = null;
        double[] As = null;
        double[] ls = null;

        public double[] xs = null;

        public double[] eps = null;
        double[] dAs = null;
        double[] dls = null;

        public double[] us = null;
        double[] f = null;
        double[] fb = null;

        double E;
        double Esteel;
        double fyd;

        SparseLDL LDL = null;

        public int nElements;
        public int nVariables;

        public int nlockedDOF;
        int[] ailising;
        bool[] lockedDOF = null;
        bool[] fixedVariable = null;
        double[] fixedDirections = null;

        public int nExtraElements = 0;
        double[] AsE = null;
        double[] lsE = null;
        List<Tuple<Point3d, int, double>> ExtraElements = new List<Tuple<Point3d, int, double>>();

        public bool mechanisim { get; private set; } = false;

        double objectiveValue = double.MaxValue;
        double[] realGradient = null;
        double[] realGradientA = null;

        public BoxSDF SDF = null;
        public double[] areaFactor = null;

        public Tuple<double, double>[] endAreas = null;
        public double[] capacityReduction = null;
        public List<int>[] neighbours = null;

        public List<double> Forces()
        {
            return eps.Zip(Areas(), (e, a) => e * a * E).ToList();
        }

        public List<double> Areas()
        {
            return areaFactor.Zip(As, (e, a) => e * a).ToList();
        }

        public InternalEnergyTruss()
        { }

        public void Init(Containers.TrussGeometry truss, double fyd = 1, double E = 1)
        {
            xs = truss.Nodes.SelectMany(x => new double[] { x.X, x.Y, x.Z }).ToArray();
            connections = truss.Connections.ToArray();

            As = truss.Areas.ToArray();

            nVariables = xs.Length;
            nElements = connections.Length;

            f = new double[nVariables];

            this.fyd = fyd*0.6;
            this.E = E;
            this.Esteel = E * 5;    // Double check

            ls = new double[nElements];
            dAs = new double[nElements];
            dls = new double[nElements];
            eps = new double[nElements];

            us = new double[nVariables];

            nlockedDOF = 0;
            ailising = Enumerable.Range(0, nVariables).ToArray();
            lockedDOF = new bool[nVariables];
            fixedVariable = new bool[nVariables];
            fixedDirections = new double[nVariables];

            // Working memory Matrices
            T = new DenseMatrix(2, 6);
            dT = new DenseMatrix(2, 6);
            Kelocal = new DenseMatrix(2, 2);
            dKelocal = new DenseMatrix(2, 2);
            Tt = new DenseMatrix(6, 2);
            dTt = new DenseMatrix(6, 2);

            TtK1 = new DenseMatrix(6, 2);
            TtK2 = new DenseMatrix(6, 2);
            TtK3 = new DenseMatrix(6, 2);
            TtKT1 = new DenseMatrix(6, 6);

            realGradient = new double[nVariables];
            realGradientA = new double[nElements];
            areaFactor = Vector.Create(nElements, 1.0);

            endAreas = new Tuple<double, double>[nElements];
            capacityReduction = Vector.Create(nVariables / 3, 1);

            SetNeighbourList();
        }

        public double ArmijoStep(double[] gradient, double[] gradientA, ref double a, out double stepLength, double gamma)
        {
            double c1 = 0.001;

            double initialValue = objectiveValue;
            double dot = Vector.DotProduct(nVariables, gradient, realGradient);
            dot += Vector.DotProduct(nElements, gradientA, realGradientA);

            double newValue, expectedValue;
            a *= 2.2;
            double lastA = 0;
            do
            {
                a *= 0.5;
                this.ApplyGradient(gradient, a - lastA);
                this.ApplyGradientA(gradientA, a - lastA);
                this.SetData(null);
                lastA = a;
                newValue = ComputeValue();

                expectedValue = initialValue - c1 * a * dot;

                stepLength = a * dot;

            } while (newValue > expectedValue && stepLength > 1e-12);

            return newValue;
        }

        public void LockZ()
        {
            for (int i = 0; i < nVariables; i += 3)
            {
                lockedDOF[i + 2] = true;
                fixedVariable[i + 2] = true;
            }
        }
        public void LockElement(Point3d from, Point3d to, bool first)
        {
            int i = FindPoint(from);
            int j = FindPoint(to);
            fixedVariable[i * 3 + 0] = true;
            fixedVariable[i * 3 + 1] = true;
            fixedVariable[i * 3 + 2] = true;
            lockedDOF[i * 3 + 0] = true;
            lockedDOF[i * 3 + 1] = true;
            lockedDOF[i * 3 + 2] = true;

            SetFixedDir(j, from - to, first);
        }

        public void SetExtraElement(Point3d extra, Point3d to, double f)
        {
            int i = FindPoint(to);
            SetExtraElement(extra, i, f);
        }
        public void SetExtraElement(Point3d extra, int to, double load)
        {
            nExtraElements++;
            ExtraElements.Add(new Tuple<Point3d, int, double>(extra, to, load));
            Point3d pt = new Point3d(xs[to * 3], xs[to * 3 + 1], xs[to * 3 + 2]);
            var dir = pt - extra;
            f[to * 3] += dir.X * load;
            f[to * 3 + 1] += dir.Y * load;
            f[to * 3 + 2] += dir.Z * load;
            SetFixedDir(to, dir, false);
        }
        private int FindPoint(Point3d pt)
        {
            const double tol = 1e-1;
            const double sqtol = tol * tol;
            // Find corresponding point
            for (int i = 0; i < nVariables; i += 3)
            {
                double x = xs[i] - pt.X;
                double y = xs[i + 1] - pt.Y;
                double z = xs[i + 2] - pt.Z;
                double sq = x * x + y * y + z * z;
                if (sq < sqtol)
                {
                    return i / 3;
                }
            }
            return -1;
        }

        public void SetFixedDir(Point3d pt, Vector3d dir, bool first)
        {
            int i  = FindPoint(pt);
            if (i != -1)
                SetFixedDir(i, dir, first);
        }
        public void SetFixedDir(int i, Vector3d dir, bool first)
        {
            dir.Unitize();
            fixedDirections[i*3+0] = dir.X;
            fixedDirections[i*3+1] = dir.Y;
            fixedDirections[i*3+2] = dir.Z;

            if (first)
            {
                // lock in all directions except ours

                if (Math.Abs(dir.X) > Math.Abs(dir.Y) && Math.Abs(dir.X) > Math.Abs(dir.Z))
                {
                    lockedDOF[i * 3 + 1] = true;
                    lockedDOF[i * 3 + 2] = true;
                }
                else if (Math.Abs(dir.Y) > Math.Abs(dir.Z))
                {
                    lockedDOF[i * 3 + 0] = true;
                    lockedDOF[i * 3 + 2] = true;
                }
                else
                {
                    lockedDOF[i * 3 + 0] = true;
                    lockedDOF[i * 3 + 1] = true;
                }
            }
        }
        public void ConstrainGradient(double[] gradient)
        {
            Vector.Copy(gradient, realGradient);
            for (int i = 0; i < nVariables; i += 3)
            {
                double x = fixedDirections[i];
                double y = fixedDirections[i+1];
                double z = fixedDirections[i+2];
                double sq = x * x + y * y + z * z;
                if (sq > 0.1)
                {
                    double gx = gradient[i];
                    double gy = gradient[i+1];
                    double gz = gradient[i+2];
                    double l = gx * x + gy * y + gz * z;
                    gradient[i] = l * x;
                    gradient[i+1] = l * y;
                    gradient[i+2] = l * z;
                }
            }
        }
        public void ConstrainGradientA(double[] gradient)
        {
            Vector.Copy(gradient, realGradientA);
            // Gradient projection for box support
            for (int i = 0; i < nElements; i++)
            {
                double val = areaFactor[i];
                double grad = gradient[i];

                grad = Math.Min(1.0 - val,
                            Math.Max(0.001 - val, 
                                grad));

                gradient[i] = grad;
            }
        }
        //public void ApplyBC(Containers.DiscreteBoundaryConditionDirechlet bcD)
        //{
        //    int i = FindPoint(bcD.pts);
        //    if (i != -1)
        //    {
        //        // Apply disp
        //        if (bcD.lockX)
        //            us[i * 3] = bcD.displacement.X;

        //        if (bcD.lockY)
        //            us[i * 3 + 1] = bcD.displacement.Y;

        //        if (bcD.lockZ)
        //            us[i * 3 + 2] = bcD.displacement.Z;

        //        lockedDOF[i * 3] |= bcD.lockX;
        //        lockedDOF[i * 3 + 1] |= bcD.lockY;
        //        lockedDOF[i * 3 + 2] |= bcD.lockZ;
        //        //fixedVariable[i] |= true;
        //        //fixedVariable[i + 1] |= true;
        //        //fixedVariable[i + 2] |= true;
        //    }
        //}

        public void LockDOFs(bool[] locked)
        {
            int countPos = 0;
            int countNeg = -1;
            for (int i = 0; i < nVariables; i++)
            {
                lockedDOF[i] = locked[i];
                if (locked[i])
                {
                    ailising[i] = countNeg--;
                }
                else
                {
                    ailising[i] = countPos++;
                }
            }
            nlockedDOF = -(countNeg + 1);
        }

        public void BCPost()
        {
            int countPos = 0;
            int countNeg = -1;
            for (int i = 0; i < nVariables; i++)
            {
                if (lockedDOF[i])
                {
                    ailising[i] = countNeg--;
                }
                else
                {
                    ailising[i] = countPos++;
                }
            }
            nlockedDOF = -(countNeg + 1);

            double[] fa = new double[nlockedDOF];
            fb = new double[nVariables - nlockedDOF];
            Split(f, fb, fa);

            lsE = new double[nExtraElements];
            AsE = new double[nExtraElements];
        }

        public void SetData(double[] As)
        {
            if (!(As is null))
            {
                this.As = As;
            }
            // Set new lengths
            for (int i = 0; i < nElements; i++)
            {
                double x = xs[connections[i].Item2 * 3] - xs[connections[i].Item1 * 3];
                double y = xs[connections[i].Item2 * 3 + 1] - xs[connections[i].Item1 * 3 + 1];
                double z = xs[connections[i].Item2 * 3 + 2] - xs[connections[i].Item1 * 3 + 2];
                double l = Math.Sqrt(x * x + y * y + z * z);
                ls[i] = l;

                this.As[i] = Area(i);
            }

            for (int i = 0; i < nExtraElements; i++)
            {
                var curr = ExtraElements[i];
                double x = xs[curr.Item2 * 3] - curr.Item1.X;
                double y = xs[curr.Item2 * 3 + 1] - curr.Item1.Y;
                double z = xs[curr.Item2 * 3 + 2] - curr.Item1.Z;
                double l = Math.Sqrt(x * x + y * y + z * z);
                lsE[i] = l;
                AsE[i] = 1;
            }
        }

        public void ApplyGradient(double[] gradient, double factor = 1)
        {
            Vector.Add(nVariables, factor, gradient, xs, xs);
        }
        public void ApplyGradientA(double[] gradientA, double factor = 1)
        {
            Vector.Add(nElements, Math.Min(1, factor), gradientA, areaFactor, areaFactor);
        }
        public double ComputeValue()
        {
            if (nVariables == nlockedDOF)
            {
                return 0;
            }
            us = Displacments();
            for (int i = 0; i < nElements; i++)
            {
                double strain = Strain(i);
                eps[i] = strain;
            }
            SetReductions();

            return Compute();
        }

        double strainFactorMain = 1;

        double strainFactor = 5;
        double maxStressT = 1;
        double maxStressC = 1;

        double utilizationFactor = 0;
        double utilizationMargin = 0.1;

        public double penaltyFactor = 0;

        public void ComputeGradient(ref double[] gradient)
        {
            if (nVariables == nlockedDOF)
            {
                Vector.Clear(gradient);
                return;
            }

            for (int i = 0; i < nVariables; i++)
            {
                if (!fixedVariable[i])
                    gradient[i] = ComputeDerivative(i);
            }
        }
        public void ComputeGradientA(ref double[] gradient)
        {
            if (nVariables == nlockedDOF)
            {
                gradient = new double[nElements];
                return;
            }

            for (int i = 0; i < nElements; i++)
            {
                gradient[i] = ComputeDerivativeA(i);
            }
        }
        private void SetNeighbourList()
        {
            neighbours = new List<int>[nVariables / 3];
            for (int i = 0; i < nVariables / 3; i++)
            {
                var curr = new List<int>();
                neighbours[i] = curr;

                for (int j = 0; j < nElements; j++)
                {
                    if (connections[j].Item1 == i || connections[j].Item2 == i)
                    {
                        curr.Add(j);
                    }
                }
            }
        }
        private void SetReductions()
        {
            for (int i = 0; i < nVariables / 3; i++)
            {
                int countT = 0;
                for (int j = 0; j < neighbours[i].Count; j++)
                {
                    int index = neighbours[i][j];
                    double strain = eps[index];

                    if (strain > 0 && areaFactor[index] > 0.01)
                        countT++;
                }
                if (countT >= 2)
                {
                    capacityReduction[i] = 0.75;
                }
                else if (countT >= 1)
                {
                    capacityReduction[i] = 0.85;
                }
                else
                {
                    capacityReduction[i] = 1;
                }
            }
        }

        private double Compute()
        {
            double strainEnergy = 0.0;
            double utilization = 0.0;
            double penalty = 0.0;

            for (int i = 0; i < nElements; i++)
            {
                double ep = eps[i];

                // Strain Energy
                double factor = ep < 0 ? 1 : strainFactor;
                strainEnergy += factor * E * areaFactor[i] * As[i] * ls[i] * ep * ep;

                // Utilization
                double temp = E * As[i] * ep;
                Func<double, double> Square = x => x * x;
                utilization += Square(Math.Max(0, temp / endAreas[i].Item1 / maxStressT - 1 + utilizationMargin));
                utilization -= Square(Math.Min(0, 1 - utilizationMargin + temp / endAreas[i].Item1 / (maxStressC * capacityReduction[connections[i].Item1])));
                utilization += Square(Math.Max(0, temp / endAreas[i].Item2 / maxStressT - 1 + utilizationMargin));
                utilization -= Square(Math.Min(0, 1 - utilizationMargin + temp / endAreas[i].Item2 / (maxStressC * capacityReduction[connections[i].Item2])));

                // Penalty
                if (ep > 0)
                {
                    double t = Esteel * ep - fyd;
                    penalty += t * t;

                    //////////////
                    //double t2 = Esteel * ep * As[i];
                    //penalty += t2 * t2;
                }
            }

            for (int i = 0; i < nExtraElements; i++)
            {
                double ep = StrainE(i);

                // Strain Energy
                double factor = ep < 0 ? 1 : strainFactor;
                strainEnergy += factor * E * AsE[i] * lsE[i] * ep * ep;

                // Utilization
                double cap = capacityReduction[ExtraElements[i].Item2];

                utilization += Math.Max(0, E * ep / maxStressT - 1 + utilizationMargin);
                utilization -= Math.Min(0, 1 - utilizationMargin + E * ep / (maxStressC * cap));

                // Penalty
                if (ep > 0)
                {
                    double temp = Esteel * ep - fyd;
                    penalty += temp * temp;
                }
            }

            double result = strainFactorMain * strainEnergy + utilization * utilizationFactor + penalty * penaltyFactor;

            objectiveValue = result;
            return result;
        }

        private double ComputeDerivative(int dindex)
        {
            double strainEnergy = 0.0;
            double utilization = 0.0;
            double penalty = 0.0;

            for (int i = 0; i < nElements; i++)
            {
                double dl = Dlength(i, dindex);
                dls[i] = dl;

                double dA = dArea(i, dindex);
                dAs[i] = dA;
            }

            double[] dus = Ddisplacments(dindex);

            for (int i = 0; i < nElements; i++)
            {
                double A = As[i];
                double l = ls[i];
                double ep = eps[i];

                double dl = dls[i];
                double deps = Dstrain(i, dindex, dl, dus);
                double dA = dAs[i];


                // Strain Energy
                double factor = ep < 0 ? 1 : strainFactor;
                strainEnergy += factor *
                    areaFactor[i] * E * (2 * A * l * deps * ep + 
                    (dA * l + A * dl) * ep * ep);

                // Utilization
                double Astart = endAreas[i].Item1;
                double Aend = endAreas[i].Item2;

                double dAstart, dAend;
                dAreaSE(i, dindex, out dAstart, out dAend);

                double temp = E * A * ep;
                double startValT = temp / Astart / maxStressT - 1 + utilizationMargin;
                double startValC = temp / Astart / (maxStressC * capacityReduction[connections[i].Item1]) + 1 - utilizationMargin;
                double endValT = temp / Aend / maxStressT - 1 + utilizationMargin;
                double endValC = temp / Aend / (maxStressC * capacityReduction[connections[i].Item2]) + 1 - utilizationMargin;

                if (startValT > 0)
                    utilization += 2 * startValT * E * (deps * A * Astart + ep * dA * Astart - ep * A * dAstart) / (Astart * Astart * maxStressT);
                else if (startValC < 0)
                    utilization -= 2 * startValC * E * (deps * A * Astart + ep * dA * Astart - ep * A * dAstart) / (Astart * Astart * maxStressC);

                if (endValT > 0)
                    utilization += 2 * endValT * E * (deps * A * Aend + ep * dA * Aend - ep * A * dAend) / (Aend * Aend * maxStressT);
                else if (endValC < 0)
                    utilization -= 2 * endValC * E * (deps * A * Aend + ep * dA * Aend - ep * A * dAend) / (Aend * Aend * maxStressC);

                // Penalty
                if (ep > 0)
                {
                    penalty += 2 * (Esteel * ep - fyd) * Esteel * deps;


                    //penalty += 2 * (Esteel * ep * A) * Esteel * (deps * A + ep * dA);
                }
            }

            for (int i = 0; i < nExtraElements; i++)
            {
                double A = AsE[i];      // Will these areas change??
                double l = lsE[i];
                double ep = StrainE(i);

                double dl = DlengthE(i, dindex);
                double dA = 0; // E * A * deps / fyd;
                double deps = DstrainE(i, dindex, dl, dus, dA);


                // Strain Energy
                double factor = ep < 0 ? 1 : strainFactor;
                strainEnergy -= factor *
                    E * (2 * A * l * deps * ep +
                    (dA * l + A * dl) * ep * ep);

                // Utilization
                double valT = E * ep / maxStressT - 1 + utilizationMargin;
                double valC = E * ep / (maxStressC * capacityReduction[ExtraElements[i].Item2]) + 1 - utilizationMargin;

                if (valT > 0)
                    utilization -= 2 * valT * E * deps / maxStressT;
                else if (valC < 0)
                    utilization += 2 * valC * E * deps / (maxStressC * capacityReduction[ExtraElements[i].Item2]);

                // Penalty
                if (ep > 0)
                {
                    penalty += 2 * (Esteel * ep - fyd) * Esteel * deps;
                }
            }

            double result = strainFactorMain * strainEnergy + utilization * utilizationFactor + penalty * penaltyFactor;

            return result;
        }
        private double ComputeDerivativeA(int dindex)
        {
            double strainEnergy = 0.0;
            double utilization = 0.0;
            double penalty = 0.0;

            double[] dus = DdisplacmentsA(dindex);

            for (int i = 0; i < nElements; i++)
            {
                double A = As[i];
                double l = ls[i];
                double ep = eps[i];

                double deps = DstrainA(i, dus);

                // Strain Energy
                double factor = ep < 0 ? 1 : strainFactor;
                strainEnergy += factor *
                    E * A * l * ep * (ep + 2 * areaFactor[i] * deps);

                // Utilization
                double Astart = endAreas[i].Item1;
                double Aend = endAreas[i].Item2;

                double temp = E * A * ep;
                double startValT = temp / Astart / maxStressT - 1 + utilizationMargin;
                double startValC = temp / Astart / (maxStressC * capacityReduction[connections[i].Item1]) + 1 - utilizationMargin;
                double endValT = temp / Aend / maxStressT - 1 + utilizationMargin;
                double endValC = temp / Aend / (maxStressC * capacityReduction[connections[i].Item2]) + 1 - utilizationMargin;

                if (startValT > 0)
                    utilization += 2 * startValT * E * deps * A / (endAreas[i].Item1 * maxStressT);
                else if (startValC < 0)
                    utilization -= 2 * startValC * E * deps * A / (endAreas[i].Item1 * maxStressC);

                if (endValT > 0)
                    utilization += 2 * endValT * E * deps * A / (endAreas[i].Item2 * maxStressT);
                else if (endValC < 0)
                    utilization -= 2 * endValC * E * deps * A / (endAreas[i].Item2 * maxStressC);

                // Penalty
                if (ep > 0)
                {
                    penalty += 2 * (Esteel * ep - fyd) * Esteel * deps;

                    //penalty += 2 * (Esteel * ep * A) * Esteel * deps * A;
                }
            }

            double result = strainFactorMain * strainEnergy + utilization * utilizationFactor + penalty * penaltyFactor;

            return result;
        }

        private double Area(int i)
        {
            // Use average area of the different nodes

            if (SDF is null)
            {
                endAreas[i] = new Tuple<double, double>(As[i], As[i]);
                return As[i];
            }
            int s = connections[i].Item1 * 3;
            int e = connections[i].Item2 * 3;

            var start = new Point3d(xs[s], xs[s + 1], xs[s + 2]);
            var end = new Point3d(xs[e], xs[e + 1], xs[e + 2]);

            var boxS = SDF.BoxValueAt(start);
            var boxE = SDF.BoxValueAt(end);

            var n = end - start;
            n.Unitize();
            n.X = Math.Abs(n.X);
            n.Y = Math.Abs(n.Y);
            n.Z = Math.Abs(n.Z);

            double areaS = n.X * boxS.Y * boxS.Z + n.Y * boxS.X * boxS.Z + n.Z * boxS.X * boxS.Y;
            double areaE = n.X * boxE.Y * boxE.Z + n.Y * boxE.X * boxE.Z + n.Z * boxE.X * boxE.Y;

            endAreas[i] = new Tuple<double, double>(areaS, areaE);
            return (areaS + areaE) * 0.5;
        }

        private double dArea(int i, int dindex)
        {
            // Use average area of the different nodes
            if (SDF is null)
                return 0.0;

            if (!ElementConnectsToVariable(i, dindex))
                return 0.0;

            int s = connections[i].Item1 * 3;
            int e = connections[i].Item2 * 3;

            var start = new Point3d(xs[s], xs[s + 1], xs[s + 2]);
            var end = new Point3d(xs[e], xs[e + 1], xs[e + 2]);

            var boxS = SDF.BoxValueAt(start);
            var boxE = SDF.BoxValueAt(end);

            SDF.BoxGradientAt(start, out var dxs, out var dys, out var dzs);
            SDF.BoxGradientAt(end, out var dxe, out var dye, out var dze);

            var box = NodeConnectsToVariable(connections[i].Item1, dindex) ? boxS : boxE;

            var n = end - start;
            n.Unitize();

            var dn = new Vector3d();
            Vector3d dbox = Vector3d.Unset;

            int off = dindex - connections[i].Item1 * 3;
            switch (off)
            {
                case 0:
                    dbox = dxs;
                    dn.X = -(Math.Sign(n.X) - n.X * Math.Abs(n.X));
                    dn.Y = n.X * Math.Abs(n.Y);
                    dn.Z = n.X * Math.Abs(n.Z);
                    break;
                case 1:
                    dbox = dys;
                    dn.X = n.Y * Math.Abs(n.X);
                    dn.Y = -(Math.Sign(n.Y) - n.Y * Math.Abs(n.Y));
                    dn.Z = n.Y * Math.Abs(n.Z);
                    break;
                case 2:
                    dbox = dzs;
                    dn.X = n.Z * Math.Abs(n.X);
                    dn.Y = n.Z * Math.Abs(n.Y);
                    dn.Z = -(Math.Sign(n.Z) - n.Z * Math.Abs(n.Z));
                    break;
                default:
                    off = dindex - connections[i].Item2 * 3;
                    switch (off)
                    {
                        case 0:
                            dbox = dxe;
                            dn.X = (Math.Sign(n.X) - n.X * Math.Abs(n.X));
                            dn.Y = -n.X * Math.Abs(n.Y);
                            dn.Z = -n.X * Math.Abs(n.Z);
                            break;
                        case 1:
                            dbox = dye;
                            dn.X = -n.Y * Math.Abs(n.X);
                            dn.Y = (Math.Sign(n.Y) - n.Y * Math.Abs(n.Y));
                            dn.Z = -n.Y * Math.Abs(n.Z);
                            break;
                        case 2:
                            dbox = dze;
                            dn.X = -n.Z * Math.Abs(n.X);
                            dn.Y = -n.Z * Math.Abs(n.Y);
                            dn.Z = (Math.Sign(n.Z) - n.Z * Math.Abs(n.Z));
                            break;
                        default:
                            break;
                    }
                    break;
            }

            n.X = Math.Abs(n.X);
            n.Y = Math.Abs(n.Y);
            n.Z = Math.Abs(n.Z);

            double darea = dn.X * box.Y * box.Z + n.X * dbox.Y * box.Z + n.X * box.Y * dbox.Z +
                           dn.Y * box.X * box.Z + n.Y * dbox.X * box.Z + n.Y * box.X * dbox.Z +
                           dn.Z * box.X * box.Y + n.Z * dbox.X * box.Y + n.Z * box.X * dbox.Y;

            return darea * 0.5;
        }

        private void dAreaSE(int i, int dindex, out double dAs, out double dAe)
        {
            // Use average area of the different nodes
            if (SDF is null || !ElementConnectsToVariable(i, dindex))
            {
                dAs = 0.0;
                dAe = 0.0;
                return;
            }

            int s = connections[i].Item1 * 3;
            int e = connections[i].Item2 * 3;

            var start = new Point3d(xs[s], xs[s + 1], xs[s + 2]);
            var end = new Point3d(xs[e], xs[e + 1], xs[e + 2]);

            var boxS = SDF.BoxValueAt(start);
            var boxE = SDF.BoxValueAt(end);

            SDF.BoxGradientAt(start, out var dxs, out var dys, out var dzs);
            SDF.BoxGradientAt(end, out var dxe, out var dye, out var dze);

            bool isstart = NodeConnectsToVariable(connections[i].Item1, dindex);

            var box = isstart ? boxS : boxE;

            var n = end - start;
            n.Unitize();

            var dn = new Vector3d();
            Vector3d dbox = Vector3d.Unset;

            int off = dindex - connections[i].Item1 * 3;
            switch (off)
            {
                case 0:
                    dbox = dxs;
                    dn.X = -(Math.Sign(n.X) - n.X * Math.Abs(n.X));
                    dn.Y = n.X * Math.Abs(n.Y);
                    dn.Z = n.X * Math.Abs(n.Z);
                    break;
                case 1:
                    dbox = dys;
                    dn.X = n.Y * Math.Abs(n.X);
                    dn.Y = -(Math.Sign(n.Y) - n.Y * Math.Abs(n.Y));
                    dn.Z = n.Y * Math.Abs(n.Z);
                    break;
                case 2:
                    dbox = dzs;
                    dn.X = n.Z * Math.Abs(n.X);
                    dn.Y = n.Z * Math.Abs(n.Y);
                    dn.Z = -(Math.Sign(n.Z) - n.Z * Math.Abs(n.Z));
                    break;
                default:
                    off = dindex - connections[i].Item2 * 3;
                    switch (off)
                    {
                        case 0:
                            dbox = dxe;
                            dn.X = (Math.Sign(n.X) - n.X * Math.Abs(n.X));
                            dn.Y = -n.X * Math.Abs(n.Y);
                            dn.Z = -n.X * Math.Abs(n.Z);
                            break;
                        case 1:
                            dbox = dye;
                            dn.X = -n.Y * Math.Abs(n.X);
                            dn.Y = (Math.Sign(n.Y) - n.Y * Math.Abs(n.Y));
                            dn.Z = -n.Y * Math.Abs(n.Z);
                            break;
                        case 2:
                            dbox = dze;
                            dn.X = -n.Z * Math.Abs(n.X);
                            dn.Y = -n.Z * Math.Abs(n.Y);
                            dn.Z = (Math.Sign(n.Z) - n.Z * Math.Abs(n.Z));
                            break;
                        default:
                            break;
                    }
                    break;
            }

            n.X = Math.Abs(n.X);
            n.Y = Math.Abs(n.Y);
            n.Z = Math.Abs(n.Z);

            double darea = dn.X * box.Y * box.Z + n.X * dbox.Y * box.Z + n.X * box.Y * dbox.Z +
                           dn.Y * box.X * box.Z + n.Y * dbox.X * box.Z + n.Y * box.X * dbox.Z +
                           dn.Z * box.X * box.Y + n.Z * dbox.X * box.Y + n.Z * box.X * dbox.Y;

            if (isstart)
            {
                dAs = darea;
                dAe = 0;
            }
            else
            {
                dAs = 0;
                dAe = darea;
            }
        }

        private double[] Displacments()
        {
            // Compute derivative of stiffness matrix
            var Kcoord = new CoordinateStorage<double>(nVariables - nlockedDOF, nVariables - nlockedDOF, 6 * 6 * nElements);
            var Kcoords = new CoordinateStorage<double>(nlockedDOF, nVariables - nlockedDOF, 6 * 6 * nElements);

            for (int i = 0; i < nElements; i++)
                Assemble(Kcoord, Kcoords, ElementStiffness(i), i);

            //ApplyFakeSprings(Kcoord, 0);

            SparseMatrix K = (SparseMatrix)SparseMatrix.OfIndexed(Kcoord);

            // Solve LDL decomposition
            var ordering = ColumnOrdering.MinimumDegreeAtPlusA;
            try
            {
                LDL = SparseLDL.Create(K, ordering);
            }
            catch
            {
                mechanisim = true;
                return us;
            }


            double[] usa = new double[nlockedDOF];
            double[] usb = new double[nVariables - nlockedDOF];
            Split(us, usb, usa);

            LDL.Solve(fb, usb);

            Merge(usb, usa, us);

            return us;
        }

        string MatrixForm(SparseMatrix A)
        {
            string res = "";
            for (int i = 0; i < A.RowCount; i++)
            {
                for (int j = 0; j < A.ColumnCount; j++)
                {
                    res += A.At(i, j).ToString("0.00");
                    res += " ";
                }
                res += "\n";
            }
            return res;
        }

        private double[] Ddisplacments(int dindex)
        {
            // Compute derivative of stiffness matrix
            double[] res = new double[nVariables];
            for (int i = 0; i < nElements; i++)
                if (ElementConnectsToVariable(i, dindex))
                    ImplicitAssembleMultiply(DElementStiffness(dindex, i), i, us, res);

            // Solve with precomputed LDL decomposition
            double[] usa = new double[nlockedDOF];
            double[] usb = new double[nVariables - nlockedDOF];
            Split(res, usb, usa);

            double[] res1 = new double[nVariables - nlockedDOF];
            LDL.Solve(usb, res1);

            double[] res2 = new double[nVariables];
            Merge(res1, usa, res2);
            return res2;
        }

        private double[] DdisplacmentsA(int dindex)
        {
            // Compute derivative of stiffness matrix
            double[] res = new double[nVariables];
            
            ImplicitAssembleMultiply(DElementStiffnessA(dindex), dindex, us, res);

            // Solve with precomputed LDL decomposition
            double[] usa = new double[nlockedDOF];
            double[] usb = new double[nVariables - nlockedDOF];
            Split(res, usb, usa);

            double[] res1 = new double[nVariables - nlockedDOF];
            LDL.Solve(usb, res1);

            double[] res2 = new double[nVariables];
            Merge(res1, usa, res2);
            return res2;
        }

        private void Split<T>(T[] input, T[] a, T[] b)
        {
            for (int i = 0; i < input.Length; i++)
            {
                int j = ailising[i];
                if (j >= 0)
                {
                    a[j] = input[i];
                }
                else
                {
                    j = -(j + 1);
                    b[j] = input[i];
                }
            }
        }

        private void Merge<T>(T[] a, T[] b, T[] output)
        {
            for (int i = 0; i < output.Length; i++)
            {
                int j = ailising[i];
                if (j >= 0)
                {
                    output[i] = a[j];
                }
                else
                {
                    j = -(j + 1);
                    output[i] = b[j];
                }
            }
        }

        private void ApplyFakeSprings(CoordinateStorage<double> A, double k)
        {
            for (int i = 0; i < A.ColumnCount; i++)
                A.At(i, i, k);
        }
        private void ImplicitAssembleMultiply(DenseMatrix Ae, int eIndex, double[] u, double[] res)
        {
            int start = connections[eIndex].Item1 * 3;
            int end = connections[eIndex].Item2 * 3;

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    res[start + i] += Ae[i, j] * u[start + j];
                    res[start + i] += Ae[i, j + 3] * u[end + j];
                    res[end + i] += Ae[i + 3, j] * u[start + j];
                    res[end + i] += Ae[i + 3, j + 3] * u[end + j];
                }
            }
        }
        private void Assemble(CoordinateStorage<double> A, CoordinateStorage<double> As, DenseMatrix Ae, int elementIndex)
        {
            int ix = ailising[connections[elementIndex].Item1 * 3];
            int iy = ailising[connections[elementIndex].Item1 * 3 + 1];
            int iz = ailising[connections[elementIndex].Item1 * 3 + 2];
            int jx = ailising[connections[elementIndex].Item2 * 3];
            int jy = ailising[connections[elementIndex].Item2 * 3 + 1];
            int jz = ailising[connections[elementIndex].Item2 * 3 + 2];

            int[] arr = new int[6] { ix, iy, iz, jx, jy, jz };

            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    if (arr[i] >= 0 && arr[j] >= 0)
                    {
                        A.At(arr[i], arr[j], Ae[i, j]);
                    }
                }
            }
        }
        DenseMatrix T;
        DenseMatrix Kelocal;
        DenseMatrix Tt;

        private DenseMatrix ElementStiffness(int elementIndex)
        {
            TransformationMatrix(elementIndex);
            LocalElementStiffness(elementIndex);
            T.Transpose(Tt);

            TtK1.Clear();
            TtKT1.Clear();

            Tt.Multiply(Kelocal, TtK1);
            TtK1.Multiply(T, TtKT1);
            return TtKT1;
        }
        DenseMatrix TtK1;
        DenseMatrix TtK2;
        DenseMatrix TtK3;

        DenseMatrix TtKT1;

        DenseMatrix dT;
        DenseMatrix dKelocal;
        DenseMatrix dTt;
        private DenseMatrix DElementStiffness(int dindex, int elementIndex)
        {
            TransformationMatrix(elementIndex);
            DTransformationMatrix(dindex, elementIndex);
            LocalElementStiffness(elementIndex);
            DLocalElementStiffness(dindex, elementIndex);

            T.Transpose(Tt);
            dT.Transpose(dTt);

            TtK1.Clear();
            TtK2.Clear();
            TtK3.Clear();
            TtKT1.Clear();

            dTt.Multiply(Kelocal, TtK1);
            TtK1.Multiply(T, TtKT1);

            Tt.Multiply(dKelocal, TtK2);
            TtK2.Multiply(T, TtKT1);

            Tt.Multiply(Kelocal, TtK3);
            TtK3.Multiply(dT, TtKT1);

            return TtKT1;
        }

        private DenseMatrix DElementStiffnessA(int elementIndex)
        {
            TransformationMatrix(elementIndex);
            DLocalElementStiffnessA(elementIndex);

            T.Transpose(Tt);

            TtK2.Clear();
            TtKT1.Clear();

            Tt.Multiply(dKelocal, TtK2);
            TtK2.Multiply(T, TtKT1);

            return TtKT1;
        }

        private void DLocalElementStiffness(int dindex, int elementIndex)
        {
            int i = elementIndex;
            double temp = areaFactor[i] * (E * ls[i] * dAs[i] - dls[i] * E * As[i]) / (ls[i] * ls[i]);
            dKelocal[0, 0] = temp;
            dKelocal[0, 1] = -temp;
            dKelocal[1, 0] = -temp;
            dKelocal[1, 1] = temp;
        }
        private void DLocalElementStiffnessA(int elementIndex)
        {
            int i = elementIndex;
            double temp = E * As[i] / ls[i];
            dKelocal[0, 0] = temp;
            dKelocal[0, 1] = -temp;
            dKelocal[1, 0] = -temp;
            dKelocal[1, 1] = temp;
        }
        private void LocalElementStiffness(int elementIndex)
        {
            int i = elementIndex;
            double temp = areaFactor[i] * E * As[i] / ls[i];
            Kelocal[0, 0] = temp;
            Kelocal[0, 1] = -temp;
            Kelocal[1, 0] = -temp;
            Kelocal[1, 1] = temp;
        }
        private void TransformationMatrix(int elementIndex)
        {
            int i = elementIndex;
            double nx = (xs[connections[i].Item2 * 3] - xs[connections[i].Item1 * 3]) / ls[i];
            double ny = (xs[connections[i].Item2 * 3 + 1] - xs[connections[i].Item1 * 3 + 1]) / ls[i];
            double nz = (xs[connections[i].Item2 * 3 + 2] - xs[connections[i].Item1 * 3 + 2]) / ls[i];
            T[0, 0] = nx;
            T[0, 1] = ny;
            T[0, 2] = nz;
            T[1, 3] = nx;
            T[1, 4] = ny;
            T[1, 5] = nz;
        }
        private void DTransformationMatrix(int dindex, int elementIndex)
        {
            int i = elementIndex;
            var factor = connections[i].Item1 * 3 == dindex ? -1 : 0;
            factor = connections[i].Item2 * 3 == dindex ? 1 : factor;
            double nx = (factor * ls[i] - (xs[connections[i].Item2 * 3] - xs[connections[i].Item1 * 3]) * dls[i]) / (ls[i] * ls[i]);

            factor = connections[i].Item1 * 3 + 1 == dindex ? -1 : 0;
            factor = connections[i].Item2 * 3 + 1 == dindex ? 1 : factor;
            double ny = (factor * ls[i] - (xs[connections[i].Item2 * 3 + 1] - xs[connections[i].Item1 * 3 + 1]) * dls[i]) / (ls[i] * ls[i]);

            factor = connections[i].Item1 * 3 + 2 == dindex ? -1 : 0;
            factor = connections[i].Item2 * 3 + 2 == dindex ? 1 : factor;
            double nz = (factor * ls[i] - (xs[connections[i].Item2 * 3 + 2] - xs[connections[i].Item1 * 3 + 2]) * dls[i]) / (ls[i] * ls[i]);

            dT[0, 0] = nx;
            dT[0, 1] = ny;
            dT[0, 2] = nz;
            dT[1, 3] = nx;
            dT[1, 4] = ny;
            dT[1, 5] = nz;
        }

        private double Strain(int i)
        {
            var con = connections[i];
            double l = ls[i];

            Vector3d t = new Vector3d()
            {
                X = (xs[con.Item2 * 3] - xs[con.Item1 * 3]) / l,
                Y = (xs[con.Item2 * 3 + 1] - xs[con.Item1 * 3 + 1]) / l,
                Z = (xs[con.Item2 * 3 + 2] - xs[con.Item1 * 3 + 2]) / l
            };
            
            Vector3d ustart = new Vector3d(
                us[con.Item1 * 3 + 0],
                us[con.Item1 * 3 + 1],
                us[con.Item1 * 3 + 2]);
            Vector3d uend = new Vector3d(
                us[con.Item2 * 3 + 0],
                us[con.Item2 * 3 + 1],
                us[con.Item2 * 3 + 2]);

            double res = t * (uend - ustart);
            return res / l;
        }

        private double StrainE(int i)
        {
            var con = ExtraElements[i];

            double F = con.Item3;
            return -F / (AsE[i] * E);
        }

        private double Dstrain(int i, int dindex, double dlength, double[] dus)
        {
            var con = connections[i];
            double l = ls[i];

            Vector3d t = new Vector3d()
            {
                X = (xs[con.Item2 * 3] - xs[con.Item1 * 3]) / l,
                Y = (xs[con.Item2 * 3 + 1] - xs[con.Item1 * 3 + 1]) / l,
                Z = (xs[con.Item2 * 3 + 2] - xs[con.Item1 * 3 + 2]) / l
            };
            Vector3d dt = new Vector3d()
            {
                X = -(xs[con.Item2 * 3] - xs[con.Item1 * 3]) * dlength,
                Y = -(xs[con.Item2 * 3 + 1] - xs[con.Item1 * 3 + 1]) * dlength,
                Z = -(xs[con.Item2 * 3 + 2] - xs[con.Item1 * 3 + 2]) * dlength

            };
            int index = dindex - con.Item1 * 3;
            if (index >= 0 && index < 3)
                dt[index] += -l;
            else
            {
                index = dindex - con.Item2 * 3;
                if (index >= 0 && index < 3)
                    dt[index] += l;
            }
            dt /= l * l;

            Vector3d ustart = new Vector3d(
                us[con.Item1 * 3 + 0],
                us[con.Item1 * 3 + 1],
                us[con.Item1 * 3 + 2]);
            Vector3d dustart = new Vector3d(
                dus[con.Item1 * 3 + 0],
                dus[con.Item1 * 3 + 1],
                dus[con.Item1 * 3 + 2]);
            Vector3d uend = new Vector3d(
                us[con.Item2 * 3 + 0],
                us[con.Item2 * 3 + 1],
                us[con.Item2 * 3 + 2]);
            Vector3d duend = new Vector3d(
                dus[con.Item2 * 3 + 0],
                dus[con.Item2 * 3 + 1],
                dus[con.Item2 * 3 + 2]);

            double res = (dt * (uend- ustart) + t * (duend - dustart)) * l - t * (uend - ustart) * dlength;
            return res / (l * l);
        }
        private double DstrainA(int i, double[] dus)
        {
            var con = connections[i];
            double l = ls[i];

            Vector3d t = new Vector3d()
            {
                X = (xs[con.Item2 * 3] - xs[con.Item1 * 3]) / l,
                Y = (xs[con.Item2 * 3 + 1] - xs[con.Item1 * 3 + 1]) / l,
                Z = (xs[con.Item2 * 3 + 2] - xs[con.Item1 * 3 + 2]) / l
            };

            Vector3d dustart = new Vector3d(
                dus[con.Item1 * 3 + 0],
                dus[con.Item1 * 3 + 1],
                dus[con.Item1 * 3 + 2]);
            Vector3d duend = new Vector3d(
                dus[con.Item2 * 3 + 0],
                dus[con.Item2 * 3 + 1],
                dus[con.Item2 * 3 + 2]);

            double res = t * (duend - dustart);
            return res / l;
        }
        private double DstrainE(int i, int dindex, double dlength, double[] dus, double dA)
        {
            var curr = ExtraElements[i];

            double F = curr.Item3;
            double res = -F * dA / (E * AsE[i] * AsE[i]);
            return res;
        }

        private double Dlength(int i, int dindex)
        {
            var con = connections[i];

            if (NodeConnectsToVariable(con.Item1, dindex))
            {
                double diff = xs[con.Item2 * 3 + (dindex % 3)] - xs[dindex];
                return - diff / ls[i];
            }
            else if (NodeConnectsToVariable(con.Item2, dindex))
            {
                double diff = xs[dindex] - xs[con.Item1 * 3 + (dindex % 3)];
                return diff / ls[i];
            }
            return 0.0;
        }

        private double DlengthE(int i, int dindex)
        {
            var curr = ExtraElements[i];

            if (NodeConnectsToVariable(curr.Item2, dindex))
            {
                double diff = xs[dindex] - curr.Item1[dindex % 3];
                return diff / lsE[i];
            }
            return 0.0;
        }

        private bool ElementConnectsToVariable(int eIndex, int vIndex)
        {
            return NodeConnectsToVariable(connections[eIndex].Item1, vIndex) ||
                   NodeConnectsToVariable(connections[eIndex].Item2, vIndex);
        }

        private bool NodeConnectsToVariable(int nIndex, int vIndex)
        {
            int i = vIndex - nIndex * 3;
            return i >= 0 && i < 3;
        }
    }
}
