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
        double[] dls = null;

        public double[] us = null;
        double[] f = null;
        double[] fb = null;

        double E;
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

        public List<double> Forces()
        {
            return eps.Zip(As, (e, a) => e * a * E).ToList();
        }

        public InternalEnergyTruss()
        { }

        public void Init(Containers.TrussGeometry truss, double fyd = 1, double E = 1)
        {
            xs = truss.Nodes.SelectMany(x => new double[] { x.X, x.Y, x.Z }).ToArray();
            connections = truss.Connections.ToArray();

            nVariables = xs.Length;
            nElements = connections.Length;

            // Need to apply f somehow
            f = new double[nVariables];
            //for (int i = 0; i < f.Length; i++)
            //{
            //    f[i] = (i + 1) % 3 == 0 ? -1 : 0;
            //}

            this.fyd = fyd;
            this.E = E;

            ls = new double[nElements];
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

            
        }

        public void LockZ()
        {
            for (int i = 0; i < nVariables; i += 3)
            {
                lockedDOF[i + 2] = true;
                fixedVariable[i + 2] = true;
            }
        }
        public void LockElement(Point3d from, Point3d to)
        {
            int i = FindPoint(from);
            int j = FindPoint(to);
            fixedVariable[i * 3 + 0] = true;
            fixedVariable[i * 3 + 1] = true;
            fixedVariable[i * 3 + 2] = true;
            lockedDOF[i * 3 + 0] = true;
            lockedDOF[i * 3 + 1] = true;
            lockedDOF[i * 3 + 2] = true;

            SetFixedDir(j, from - to);
        }

        public void SetExtraElement(Point3d extra, Point3d to, double f)
        {
            int i = FindPoint(to);
            SetExtraElement(extra, i, f);
        }
        public void SetExtraElement(Point3d extra, int to, double f)
        {
            nExtraElements++;
            ExtraElements.Add(new Tuple<Point3d, int, double>(extra, to, f));
            Point3d pt = new Point3d(xs[to * 3], xs[to * 3 + 1], xs[to * 3 + 2]);
            SetFixedDir(to, pt - extra);
        }
        public void ApplyBC(Containers.DiscreteBoundaryConditionNuemann bcN)
        {
            int i = FindPoint(bcN.pts);
            if (i != -1)
            {
                // Apply load
                f[i * 3] += bcN.load.X;
                f[i * 3 + 1] += bcN.load.Y;
                f[i * 3 + 2] += bcN.load.Z;
                //fixedVariable[i] |= true;
                //fixedVariable[i + 1] |= true;
                //fixedVariable[i + 2] |= true;
            }
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
        public void ConstrainToDirections(double[] gradient)
        {
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
        public void ApplyBC(Containers.DiscreteBoundaryConditionDirechlet bcD)
        {
            int i = FindPoint(bcD.pts);
            if (i != -1)
            {
                // Apply disp
                if (bcD.lockX)
                    us[i * 3] = bcD.displacement.X;

                if (bcD.lockY)
                    us[i * 3 + 1] = bcD.displacement.Y;

                if (bcD.lockZ)
                    us[i * 3 + 2] = bcD.displacement.Z;

                lockedDOF[i * 3] |= bcD.lockX;
                lockedDOF[i * 3 + 1] |= bcD.lockY;
                lockedDOF[i * 3 + 2] |= bcD.lockZ;
                //fixedVariable[i] |= true;
                //fixedVariable[i + 1] |= true;
                //fixedVariable[i + 2] |= true;
            }
        }

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

        public void SetData(double[] As, double[] xs = null)
        {
            if (!(xs is null))
                this.xs = xs;

            
            if (!(As is null))
            {
                As[4] = 0.01;
                As[5] = 0.01;

                this.As = As;
            }
            // Set new lengths
            for (int i = 0; i < nElements; i++)
            {
                double x = this.xs[connections[i].Item2 * 3] - this.xs[connections[i].Item1 * 3];
                double y = this.xs[connections[i].Item2 * 3 + 1] - this.xs[connections[i].Item1 * 3 + 1];
                double z = this.xs[connections[i].Item2 * 3 + 2] - this.xs[connections[i].Item1 * 3 + 2];
                double l = Math.Sqrt(x * x + y * y + z * z);
                ls[i] = l;
            }

            for (int i = 0; i < nExtraElements; i++)
            {
                var curr = ExtraElements[i];
                double x = this.xs[curr.Item2 * 3] - curr.Item1.X;
                double y = this.xs[curr.Item2 * 3 + 1] - curr.Item1.Y;
                double z = this.xs[curr.Item2 * 3 + 2] - curr.Item1.Z;
                double l = Math.Sqrt(x * x + y * y + z * z);
                lsE[i] = l;
                AsE[i] = 1;
            }
        }

        public void ApplyGradient(double[] gradient, double factor = 1)
        {
            Vector.Add(nVariables, factor, gradient, xs, xs);
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

            return Compute();
        }

        double strainFactor = 10;

        public double ComputeValueAndGradient(ref double[] gradient)
        {
            if (nVariables == nlockedDOF)
            {
                gradient = new double[nVariables];
                return 0;
            }
            us = Displacments();
            for (int i = 0; i < nElements; i++)
            {
                double strain = Strain(i);
                eps[i] = strain;
            }

            for (int i = 0; i < nVariables; i++)
            {
                if (fixedVariable[i])
                    gradient[i] = 0;
                else
                    gradient[i] = ComputeDerivative(i);
            }

            return Compute(); 
        }

        private double Compute()
        {
            double res = 0.0;

            for (int i = 0; i < nElements; i++)
            {
                double factor = eps[i] < 0 ? 1 : strainFactor;

                res += factor * E * As[i] * ls[i] * eps[i] * eps[i];
            }

            for (int i = 0; i < nExtraElements; i++)
            {
                double ep = StrainE(i);
                double factor = ep < 0 ? 1 : strainFactor;

                res += factor * E * AsE[i] * lsE[i] * ep * ep;
            }

            return res;
        }

        private double ComputeDerivative(int dindex)
        {
            double res = 0.0;
            for (int i = 0; i < nElements; i++)
            {
                double dl = Dlength(i, dindex);
                dls[i] = dl;
            }

            double[] dus = Ddisplacments(dindex);

            for (int i = 0; i < nElements; i++)
            {
                double A = As[i];
                double l = ls[i];
                double ep = eps[i];

                double dl = dls[i];
                double deps = Dstrain(i, dindex, dl, dus);
                double dA = 0; // E * A * deps / fyd;

                double factor = ep < 0 ? 1 : strainFactor;


                res += factor *
                    E * (2 * A * l * deps * ep + 
                    (dA * l + A * dl) * ep * ep);
            }

            for (int i = 0; i < nExtraElements; i++)
            {
                double A = AsE[i];
                double l = lsE[i];
                double ep = StrainE(i);

                double dl = DlengthE(i, dindex);
                double dA = 0; // E * A * deps / fyd;
                double deps = DstrainE(i, dindex, dl, dus, dA);

                double factor = ep < 0 ? 1 : strainFactor;

                res -= factor *
                    E * (2 * A * l * deps * ep +
                    (dA * l + A * dl) * ep * ep);
            }

            return res;
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
            LDL = SparseLDL.Create(K, ordering);

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

        private void DLocalElementStiffness(int dindex, int elementIndex)
        {
            int i = elementIndex;
            double temp = -dls[i] * E * As[i] / (ls[i] * ls[i]); // dAs needs to be added
            dKelocal[0, 0] = temp;
            dKelocal[0, 1] = -temp;
            dKelocal[1, 0] = -temp;
            dKelocal[1, 1] = temp;
        }
        private void LocalElementStiffness(int elementIndex)
        {
            int i = elementIndex;
            double temp = E * As[i] / ls[i];
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
            return F / (AsE[i] * E);
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
