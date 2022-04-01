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

        double E;
        double fyd;

        SparseLDL LDL = null;

        public int nElements;
        public int nVariables;

        public int nlockedDOF;
        int[] ailising;
        bool[] lockedDOF = null;

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
            for (int i = 0; i < f.Length; i++)
            {
                f[i] = (i + 1) % 3 == 0 ? -1 : 0;
            }

            this.fyd = fyd;
            this.E = E;

            ls = new double[nElements];
            dls = new double[nElements];
            eps = new double[nElements];

            us = new double[nVariables];

            nlockedDOF = 0;
            ailising = Enumerable.Range(0, nVariables).ToArray();
            lockedDOF = new bool[nVariables];

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

        public void SetData(double[] As, double[] xs = null)
        {
            if (!(xs is null))
                this.xs = xs;

            this.As = As;

            // Set new lengths
            for (int i = 0; i < nElements; i++)
            {
                double x = this.xs[connections[i].Item2 * 3] - this.xs[connections[i].Item1 * 3];
                double y = this.xs[connections[i].Item2 * 3 + 1] - this.xs[connections[i].Item1 * 3 + 1];
                double z = this.xs[connections[i].Item2 * 3 + 2] - this.xs[connections[i].Item1 * 3 + 2];
                double l = Math.Sqrt(x * x + y * y + z * z);
                ls[i] = l;
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
                if (lockedDOF[i])
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
                res += E * As[i] * ls[i] * eps[i] * eps[i];
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

                res += 
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

            LDL.Solve(f, usb);

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
            var dKcoord = new CoordinateStorage<double>(nVariables - nlockedDOF, nVariables - nlockedDOF, 6 * 6 * nElements);
            var dKcoords = new CoordinateStorage<double>(nlockedDOF, nVariables - nlockedDOF, 6 * 6 * nElements);

            // All of these coordinate storage are filled in at the same indecies
            // There is bound to be some efficiencies to be saved there

            for (int i = 0; i < nElements; i++)
                Assemble(dKcoord, dKcoords, DElementStiffness(dindex, i), i);

            SparseMatrix dK = (SparseMatrix)SparseMatrix.OfIndexed(dKcoord);

            double[] usa = new double[nlockedDOF];
            double[] usb = new double[nVariables - nlockedDOF];
            Split(us, usb, usa);

            // Multiply with displacements
            double[] temp = new double[nVariables - nlockedDOF];
            dK.Multiply(usb, temp);

            // Solve with precomputed LDL decomposition
            double[] res = new double[nVariables - nlockedDOF];
            LDL.Solve(temp, res);

            double[] res2 = new double[nVariables];
            Merge(res, new double[nlockedDOF], res2);
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

        private void Assemble(CoordinateStorage<double> A, CoordinateStorage<double> As, DenseMatrix Ae, int elementIndex)
        {
            int ix = ailising[connections[elementIndex].Item1 * 3];
            int jx = ailising[connections[elementIndex].Item2 * 3];

            if (ix >= 0 && jx >= 0)
            {
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        A.At(ix + i, ix + j, Ae[i, j]);
                        A.At(ix + i, jx + j, Ae[i, j + 3]);
                        A.At(jx + i, ix + j, Ae[i + 3, j]);
                        A.At(jx + i, jx + j, Ae[i + 3, j + 3]);
                    }
                }
            }
            else if (ix < 0 && jx >= 0)
            {
                ix = -(ix + 1);
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        //As.At(ix + i, ix + j, Ae[i, j]);
                        //As.At(ix + i, jx + j, Ae[i, j + 3]);
                        //As.At(jx + i, ix + j, Ae[i + 3, j]);
                        A.At(jx + i, jx + j, Ae[i + 3, j + 3]);
                    }
                }
            }
            else if (ix >= 0 && jx < 0)
            {
                jx = -(jx + 1);
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        A.At(ix + i, ix + j, Ae[i, j]);
                        //As.At(ix + i, jx + j, Ae[i, j + 3]);
                        //As.At(jx + i, ix + j, Ae[i + 3, j]);
                        //As.At(jx + i, jx + j, Ae[i + 3, j + 3]);
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
            double temp = -dls[i] * E * As[i] / (ls[i] * ls[i]);
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
            double nx = ( factor * ls[i] - (xs[connections[i].Item2 * 3] - xs[connections[i].Item1 * 3]) * dls[i]) / (ls[i] * ls[i]);

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

            double res = t * (ustart - uend);
            return res / l;
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

            double res = (dt * (ustart - uend) + t * (dustart - duend)) * l - t * (ustart - uend) * dlength;
            return res / (l * l);
        }
        
        private double Dlength(int i, int dindex)
        {
            var con = connections[i];

            if (Math.Abs(con.Item1 * 3 - dindex + 1) <= 1)
            {
                double diff = xs[con.Item2 * 3 + (dindex % 3)] - xs[dindex];
                return - diff / ls[i];
            }
            else if (Math.Abs(con.Item2 * 3 - dindex + 1) <= 1)
            {
                double diff = xs[dindex] - xs[con.Item1 * 3 + (dindex % 3)];
                return diff / ls[i];
            }
            return 0.0;
        }
    }
}
