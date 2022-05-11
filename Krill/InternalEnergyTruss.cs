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

        public double stepTol = 1e-16;
        public double ArmijoStep(double[] gradient, ref double a, out double stepLength, double gamma)
        {
            double c1 = 0.001;

            double initialValue = objectiveValue;
            double dot = Vector.DotProduct(nVariables, gradient, realGradient);
            double sqdot = Math.Sqrt(dot);
            double manhattan = gradient.Sum(x => Math.Abs(x));

            double newValue, expectedValue;
            a *= 2.2;
            double lastA = 0;
            do
            {
                a *= 0.5;
                this.ApplyGradient(gradient, a - lastA);
                this.SetData(null);
                lastA = a;
                newValue = ComputeValue();

                expectedValue = initialValue - c1 * a * dot;

                stepLength = a * manhattan;

            } while (newValue > expectedValue && stepLength > stepTol);
            if (stepLength <= stepTol)
            {
                this.ApplyGradient(gradient, -a);
                this.SetData(null);
                lastA = a;
                newValue = ComputeValue();
                stepLength = 0;
            }


            return newValue;
        }

        public double ArmijoStepA(double[] gradientA, ref double a, out double stepLength, double gamma)
        {
            double c1 = 0.001;

            double initialValue = objectiveValue;
            double dot = Vector.DotProduct(nElements, gradientA, gradientA);
            double sqdot = Math.Sqrt(dot);
            double manhattan = gradientA.Sum(x => Math.Abs(x));

            double newValue, expectedValue;
            a *= 2.2;
            double lastA = 0;
            do
            {
                a *= 0.5;
                this.ApplyGradientA(gradientA, a - lastA);
                this.SetData(null);
                lastA = a;
                newValue = ComputeValue();

                expectedValue = initialValue - c1 * a * dot;

                stepLength = a * manhattan;

            } while (newValue > expectedValue && stepLength > stepTol);
            if (stepLength <= stepTol)
            {
                this.ApplyGradientA(gradientA, -a);
                this.SetData(null);
                lastA = a;
                newValue = ComputeValue();
                stepLength = 0;
            }

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
        public void LockElement(Point3d from, Point3d to, bool Fixed)
        {
            int i = FindPoint(from);
            int j = FindPoint(to);
            fixedVariable[i * 3 + 0] = true;
            fixedVariable[i * 3 + 1] = true;
            fixedVariable[i * 3 + 2] = true;
            lockedDOF[i * 3 + 0] = true;
            lockedDOF[i * 3 + 1] = true;
            lockedDOF[i * 3 + 2] = true;

            if (!Fixed)
                SetFixedDir(j, from - to);
        }

        public void LockVariable(Point3d pt, bool x, bool y, bool z)
        {
            int i = FindPoint(pt);
            lockedDOF[i * 3 + 0] |= x;
            lockedDOF[i * 3 + 1] |= y;
            lockedDOF[i * 3 + 2] |= z;
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
            SetFixedDir(to, dir);
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

        public void SetFixedDir(Point3d pt, Vector3d dir)
        {
            int i  = FindPoint(pt);
            if (i != -1)
                SetFixedDir(i, dir);
        }
        public void SetFixedDir(int i, Vector3d dir)
        {
            dir.Unitize();
            fixedDirections[i*3+0] = dir.X;
            fixedDirections[i*3+1] = dir.Y;
            fixedDirections[i*3+2] = dir.Z;
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

        public void ConstrainValuesA()
        {
            // Gradient projection for box support
            for (int i = 0; i < nElements; i++)
            {
                double val = areaFactor[i];

                val = Math.Max(Math.Min(1.0, val), 0);

                areaFactor[i] = val;
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
        public void SetPenalties(double factor)
        {
            utilizationFactor = factor;
            penaltyFactor = factor;

            smoothingFunctionScale = 0.01;

            // This isn't a real penalty only a cost function
            // orthogonalityFactor = factor;
        }
        public void ModifyPenalties(double factor)
        {
            utilizationFactor *= factor;
            penaltyFactor *= factor;

            smoothingFunctionScale /= factor * 2;

            // This isn't a real penalty only a cost function
            // orthogonalityFactor *= factor;
        }

        double SmoothingFunction(double x, double scale)
        {
            double u = x / scale;
            if (u <= 0)
                return 0;
            else if (u >= 1)
                return 1;

            return -2.0 * u * u * u + 3.0 * u * u;
        }

        double dSmoothingFunction(double x, double scale)
        {
            double u = x / scale;
            if (u <= 0)
                return 0;
            else if (u >= 1)
                return 0;

            return -6.0 * u * u + 6.0 * u;
        }

        public void ApplyGradient(double[] gradient, double factor = 1)
        {
            Vector.Add(nVariables, factor, gradient, xs, xs);
        }
        public void ApplyGradientA(double[] gradientA, double factor = 1)
        {
            Vector.Add(nElements, factor, gradientA, areaFactor, areaFactor);
            ConstrainValuesA();
        }
        public double ComputeValue()
        {
            if (nVariables == nlockedDOF)
            {
                return 0;
            }
            us = Displacments();

            double maxlength = ls.Max();
            if (mechanisim || us.Any(x => Math.Abs(x) > maxlength))
            {
                this.mechanisim = false;
                return double.MaxValue;
            }

            for (int i = 0; i < nElements; i++)
            {
                double strain = Strain(i);
                eps[i] = strain;
            }
            SetReductions();

            return Compute();
        }

        double reinforcmentFactor = 1;

        double maxStressC = 1;

        double utilizationFactor = 1;
        double utilizationMargin = 0.1;

        double penaltyFactor = 1;
        const bool useWfunction = false;

        double orthogonalityFactor = 0;
        double orthoCutoff = 0.01;

        double smoothingFunctionScale = 0.05;

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
                    gradient[i] = -ComputeDerivative(i);
            }

            for (int i = 0; i < nElements; i++)
            {
                if (eps[i] > 0)
                {
                    SetGradients(i, orthoCutoff, gradient);
                }
            }
            for (int i = 0; i < nExtraElements; i++)
            {
                if (StrainE(i) > 0)
                {
                    SetGradientsE(i, orthoCutoff, gradient);
                }
            }

            ConstrainGradient(gradient);
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
            double reinforcement = 0.0;
            double utilization = 0.0;
            double penalty = 0.0;
            double orthogonality = 0.0;

            for (int i = 0; i < nElements; i++)
            {
                double ep = eps[i];

                // Reinforcement
                if (ep > 0)
                {
                    reinforcement += areaFactor[i] * As[i] * ls[i] * SmoothingFunction(E * ep, smoothingFunctionScale);
                }

                // Utilization
                double sigma = E * As[i] * ep;
                Func<double, double> Square = x => x * x;
                utilization -= Square(Math.Min(0, 1 - utilizationMargin + sigma / endAreas[i].Item1 / (maxStressC * capacityReduction[connections[i].Item1])));
                utilization -= Square(Math.Min(0, 1 - utilizationMargin + sigma / endAreas[i].Item2 / (maxStressC * capacityReduction[connections[i].Item2])));

                // Penalty
                if (ep > 0)
                {
                    double stress = Esteel * ep;
                    double t = stress - fyd;

                    if (useWfunction)
                        penalty += t * t * stress * stress;
                    else
                    {
                        if (t > 0)
                            penalty += t * t;
                    }
                }

                // Ortho
                if (ep > 0)
                {
                    orthogonality += EvaluateElement(i, orthoCutoff);
                }
            }

            for (int i = 0; i < nExtraElements; i++)
            {
                double ep = StrainE(i);

                // Reinforcement
                if (ep > 0)
                {
                    reinforcement += AsE[i] * lsE[i] * SmoothingFunction(E * ep, smoothingFunctionScale);
                }
                // Utilization
                double cap = capacityReduction[ExtraElements[i].Item2];

                utilization -= Math.Min(0, 1 - utilizationMargin + E * ep / (maxStressC * cap));

                // Penalty
                if (ep > 0)
                {
                    double stress = Esteel * ep;
                    double t = stress - fyd;

                    if (useWfunction)
                        penalty += t * t * stress * stress;
                    else
                    {
                        if (t > 0)
                            penalty += t * t;
                    }

                }
                if (ep > 0)
                {
                    orthogonality += EvaluateElementE(i, orthoCutoff);
                }
            }

            double result = 
                reinforcement * reinforcmentFactor + 
                utilization * utilizationFactor + 
                penalty * penaltyFactor +
                orthogonality * orthogonalityFactor;

            objectiveValue = result;
            return result;
        }
        private double ComputeDerivative(int dindex)
        {
            double reinforcement = 0.0;
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

                double smooth = SmoothingFunction(E * ep, smoothingFunctionScale);
                double dsmooth = dSmoothingFunction(E * ep, smoothingFunctionScale) * E * deps;

                // Reinforcement
                if (ep > 0)
                {
                    reinforcement += areaFactor[i] * (
                        dA * l * smooth +
                        A * dl * smooth +
                        A * l * dsmooth);
                }

                // Utilization
                double Astart = endAreas[i].Item1;
                double Aend = endAreas[i].Item2;

                double dAstart, dAend;
                dAreaSE(i, dindex, out dAstart, out dAend);

                double temp = E * A * ep;
                double startValC = temp / Astart / (maxStressC * capacityReduction[connections[i].Item1]) + 1 - utilizationMargin;
                double endValC = temp / Aend / (maxStressC * capacityReduction[connections[i].Item2]) + 1 - utilizationMargin;

                if (startValC < 0)
                    utilization -= 2 * startValC * E * (deps * A * Astart + ep * dA * Astart - ep * A * dAstart) / (Astart * Astart * maxStressC);

                if (endValC < 0)
                    utilization -= 2 * endValC * E * (deps * A * Aend + ep * dA * Aend - ep * A * dAend) / (Aend * Aend * maxStressC);

                // Penalty
                if (ep > 0)
                {
                    if (useWfunction)
                    {
                        double stress = Esteel * ep;
                        penalty += 2 * deps * Esteel * stress * (stress - fyd) * (2 * stress - fyd);
                    }
                    else
                    {
                        double t = ep * Esteel - fyd;
                        if (t > 0)
                        {
                            penalty += 2 * t * deps * Esteel;
                        }
                    }
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

                double smooth = SmoothingFunction(E * ep, smoothingFunctionScale);
                double dsmooth = dSmoothingFunction(E * ep, smoothingFunctionScale) * E * deps;

                // Reinforcement
                if (ep > 0)
                {
                    reinforcement += 
                        dA * l * smooth + 
                        A * dl * smooth +
                        A * l * dsmooth;
                }

                // Utilization
                double valC = E * ep / (maxStressC * capacityReduction[ExtraElements[i].Item2]) + 1 - utilizationMargin;

                if (valC < 0)
                    utilization += 2 * valC * E * deps / (maxStressC * capacityReduction[ExtraElements[i].Item2]);

                // Penalty
                if (ep > 0)
                {
                    if (useWfunction)
                    {
                        double stress = Esteel * ep;
                        penalty += 2 * deps * Esteel * stress * (stress - fyd) * (2 * stress - fyd);
                    }
                    else
                    {
                        double t = ep * Esteel - fyd;
                        if (t > 0)
                        {
                            penalty += 2 * t * deps * Esteel;
                        }
                    }
                }
            }

            double result = 
                reinforcement * reinforcmentFactor + 
                utilization * utilizationFactor + 
                penalty * penaltyFactor;

            return result;
        }
        private double ComputeDerivativeA(int dindex)
        {
            double reinforcement = 0.0;
            double utilization = 0.0;
            double penalty = 0.0;

            double[] dus = DdisplacmentsA(dindex);

            for (int i = 0; i < nElements; i++)
            {
                double A = As[i];
                double l = ls[i];
                double ep = eps[i];

                double deps = DstrainA(i, dus);
                double dfactor = i == dindex ? 1 : 0;

                double smooth = SmoothingFunction(E * ep, smoothingFunctionScale);
                double dsmooth = dSmoothingFunction(E * ep, smoothingFunctionScale) * E * deps;

                // Reinforcement
                if (ep > 0)
                {
                    reinforcement +=
                        dfactor * A * l * smooth +
                        areaFactor[i] * A * l * dsmooth;
                }

                // Utilization
                double Astart = endAreas[i].Item1;
                double Aend = endAreas[i].Item2;

                double temp = E * A * ep;
                double startValC = temp / Astart / (maxStressC * capacityReduction[connections[i].Item1]) + 1 - utilizationMargin;
                double endValC = temp / Aend / (maxStressC * capacityReduction[connections[i].Item2]) + 1 - utilizationMargin;

                if (startValC < 0)
                    utilization -= 2 * startValC * E * deps * A / (endAreas[i].Item1 * maxStressC);

                if (endValC < 0)
                    utilization -= 2 * endValC * E * deps * A / (endAreas[i].Item2 * maxStressC);

                // Penalty
                if (ep > 0)
                {
                    if (useWfunction)
                    {
                        double stress = Esteel * ep;
                        penalty -= 2 * deps * Esteel * stress * (stress - fyd) * (2 * stress - fyd);
                    }
                    else
                    {
                        double t = ep * Esteel - fyd;
                        if (t > 0)
                        {
                            penalty += 2 * t * deps * Esteel;
                        }
                    }
                }
            }

            double result = 
                reinforcement * reinforcmentFactor + 
                utilization * utilizationFactor + 
                penalty * penaltyFactor;

            return -result * 1;
        }

        double EvaluateElement(int eIndex, double cutoff)
        {
            var i = connections[eIndex].Item1 * 3;
            var j = connections[eIndex].Item2 * 3;

            double x = xs[j] - xs[i];
            double y = xs[j + 1] - xs[i + 1];
            double z = xs[j + 2] - xs[i + 2];
            double norm = x * x + y * y + z * z;
            //double l = Math.Sqrt(norm);

            if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(z))
            {
                x = 0;
            }
            else if (Math.Abs(y) > Math.Abs(z))
            {
                y = 0;
            }
            else
            {
                z = 0;
            }

            return Math.Min(cutoff * cutoff * 0.4, (x * x + y * y + z * z) / norm);
        }
        double EvaluateElementE(int eIndex, double cutoff)
        {
            var el = ExtraElements[eIndex];
            var point = el.Item1;
            var j = el.Item2 * 3;

            double x = xs[j] - point.X;
            double y = xs[j + 1] - point.Y;
            double z = xs[j + 2] - point.Z;
            double norm = x * x + y * y + z * z;
            //double l = Math.Sqrt(norm);

            if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(z))
            {
                x = 0;
            }
            else if (Math.Abs(y) > Math.Abs(z))
            {
                y = 0;
            }
            else
            {
                z = 0;
            }

            return Math.Min(cutoff * cutoff * 0.4, (x * x + y * y + z * z) / norm);
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

            if (eps[i] > 0)
            {
                double area = Math.Min(areaS, areaE);
                endAreas[i] = new Tuple<double, double>(area, area);
                return Math.Max(area, 0.0);
            }
            else
            {
                endAreas[i] = new Tuple<double, double>(areaS, areaE);
                return Math.Max((areaS + areaE) * 0.5, 0.0);
            }
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

            var n = end - start;
            double l = n.Length;
            double linv = 1 / (l * l * l);

            var dn = new Vector3d();
            Vector3d dbox = Vector3d.Unset;

            int off = dindex - connections[i].Item1 * 3;
            Vector3d box = boxS;
            Vector3d boxO = boxE;

            Func<double, double> sign = x => x < 0 ? -1 : 1;

            switch (off)
            {
                case 0:
                    dbox = dxs;
                    dn.X = (l*l* sign(n.X) - n.X * Math.Abs(n.X));
                    dn.Y = -n.X * Math.Abs(n.Y);
                    dn.Z = -n.X * Math.Abs(n.Z);
                    break;
                case 1:
                    dbox = dys;
                    dn.X = -n.Y * Math.Abs(n.X);
                    dn.Y = (l*l* sign(n.Y) - n.Y * Math.Abs(n.Y));
                    dn.Z = -n.Y * Math.Abs(n.Z);
                    break;
                case 2:
                    dbox = dzs;
                    dn.X = -n.Z * Math.Abs(n.X);
                    dn.Y = -n.Z * Math.Abs(n.Y);
                    dn.Z = (l*l* sign(n.Z) - n.Z * Math.Abs(n.Z));
                    break;
                default:
                    off = dindex - connections[i].Item2 * 3;
                    box = boxE;
                    boxO = boxS;
                    switch (off)
                    {
                        case 0:
                            dbox = dxe;
                            dn.X = -(l*l* sign(n.X) - n.X * Math.Abs(n.X));
                            dn.Y = n.X * Math.Abs(n.Y);
                            dn.Z = n.X * Math.Abs(n.Z);
                            break;
                        case 1:
                            dbox = dye;
                            dn.X = n.Y * Math.Abs(n.X);
                            dn.Y = -(l*l* sign(n.Y) - n.Y * Math.Abs(n.Y));
                            dn.Z = n.Y * Math.Abs(n.Z);
                            break;
                        case 2:
                            dbox = dze;
                            dn.X = n.Z * Math.Abs(n.X);
                            dn.Y = n.Z * Math.Abs(n.Y);
                            dn.Z = -(l*l* sign(n.Z) - n.Z * Math.Abs(n.Z));
                            break;
                        default:
                            break;
                    }
                    break;
            }
            dn *= -linv;
            n.X = Math.Abs(n.X);
            n.Y = Math.Abs(n.Y);
            n.Z = Math.Abs(n.Z);
            n.Unitize();

            double darea = dn.X * box.Y * box.Z + n.X * dbox.Y * box.Z + n.X * box.Y * dbox.Z +
                           dn.Y * box.X * box.Z + n.Y * dbox.X * box.Z + n.Y * box.X * dbox.Z +
                           dn.Z * box.X * box.Y + n.Z * dbox.X * box.Y + n.Z * box.X * dbox.Y;

            double dareaOther = dn.X * boxO.Y * boxO.Z + dn.Y * boxO.X * boxO.Z + dn.Z * boxO.X * boxO.Y;

            return (darea + dareaOther) * 0.5;
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
            var boxO = !isstart ? boxS : boxE;

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

            double dareaOther = dn.X * boxO.Y * boxO.Z + dn.Y * boxO.X * boxO.Z + dn.Z * boxO.X * boxO.Y;

            if (isstart)
            {
                dAs = darea;
                dAe = dareaOther;
            }
            else
            {
                dAs = dareaOther;
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
            Merge(res1, new double[nlockedDOF], res2);

            Vector.Scale(-1, res2);
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
            Merge(res1, new double[nlockedDOF], res2);

            Vector.Scale(-1, res2);
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
                    res[start + i] += Ae[i + 3, j] * u[end + j];
                    res[end + i] += Ae[i, j + 3] * u[start + j];
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
        // Ortho
        void SetGradients(int eIndex, double f, double[] dxs)
        {
            var i = connections[eIndex].Item1 * 3;
            var j = connections[eIndex].Item2 * 3;

            double x = xs[j] - xs[i];
            double y = xs[j + 1] - xs[i + 1];
            double z = xs[j + 2] - xs[i + 2];
            double norm = x * x + y * y + z * z;
            double l = Math.Sqrt(norm);
            double sqnorm = 1 / (norm * l);
            double rx = 0;
            double ry = 0;
            double rz = 0;
            if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(z))
            {
                rx = -2 * x * (y * y + z * z) * sqnorm;
                ry = 2 * x * x * y * sqnorm;
                rz = 2 * x * x * z * sqnorm;
            }
            else if (Math.Abs(y) > Math.Abs(z))
            {
                rx = 2 * y * y * x * sqnorm;
                ry = -2 * y * (x * x + z * z) * sqnorm;
                rz = 2 * y * y * z * sqnorm;
            }
            else
            {
                rx = 2 * z * z * x * sqnorm;
                ry = 2 * z * z * y * sqnorm;
                rz = -2 * z * (x * x + y * y) * sqnorm;
            }
            SmoothConstrain(ref rx, ref ry, ref rz, f);
            dxs[i] += lockedDOF[i] ? 0 : orthogonalityFactor * rx / (l * 2);
            dxs[i + 1] += lockedDOF[i + 1] ? 0 : orthogonalityFactor * ry / (l * 2);
            dxs[i + 2] += lockedDOF[i + 2] ? 0 : orthogonalityFactor * rz / (l * 2);
            dxs[j] -= lockedDOF[j] ? 0 : orthogonalityFactor * rx / (l * 2);
            dxs[j + 1] -= lockedDOF[j + 1] ? 0 : orthogonalityFactor * ry / (l * 2);
            dxs[j + 2] -= lockedDOF[j + 2] ? 0 : orthogonalityFactor * rz / (l * 2);
        }
        // Ortho
        void SetGradientsE(int eIndex, double f, double[] dxs)
        {
            var el = ExtraElements[eIndex];
            var point = el.Item1;
            var j = connections[eIndex].Item2 * 3;

            double x = xs[j] - point.X;
            double y = xs[j + 1] - point.Y;
            double z = xs[j + 2] - point.Z;
            double norm = x * x + y * y + z * z;
            double l = Math.Sqrt(norm);
            double sqnorm = 1 / (norm * l);
            double rx = 0;
            double ry = 0;
            double rz = 0;
            if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(z))
            {
                rx = -2 * x * (y * y + z * z) * sqnorm;
                ry = 2 * x * x * y * sqnorm;
                rz = 2 * x * x * z * sqnorm;
            }
            else if (Math.Abs(y) > Math.Abs(z))
            {
                rx = 2 * y * y * x * sqnorm;
                ry = -2 * y * (x * x + z * z) * sqnorm;
                rz = 2 * y * y * z * sqnorm;
            }
            else
            {
                rx = 2 * z * z * x * sqnorm;
                ry = 2 * z * z * y * sqnorm;
                rz = -2 * z * (x * x + y * y) * sqnorm;
            }
            SmoothConstrain(ref rx, ref ry, ref rz, f);
            dxs[j] -= lockedDOF[j] ? 0 : orthogonalityFactor * rx / (l * 2);
            dxs[j + 1] -= lockedDOF[j + 1] ? 0 : orthogonalityFactor * ry / (l * 2);
            dxs[j + 2] -= lockedDOF[j + 2] ? 0 : orthogonalityFactor * rz / (l * 2);
        }
        void SmoothConstrain(ref double x, ref double y, ref double z, double f)
        {
            const double lower = 0.01;
            const double fade = 0.15;
            double sqL = x * x + y * y + z * z;
            double lf = f - fade;
            double uf = f + fade;
            if (sqL < lf * lf)
            {
            }
            else if (sqL < uf * uf)
            {
                double l = Math.Sqrt(sqL);
                double t = (l - lf) / (2 * fade);
                double newLength = (t * lower + (1 - t) * (f - fade));
                x *= newLength / l;
                y *= newLength / l;
                z *= newLength / l;
            }
            else
            {
                double l = Math.Sqrt(sqL);
                x *= lower / l;
                y *= lower / l;
                z *= lower / l;
            }
        }
    }
}
