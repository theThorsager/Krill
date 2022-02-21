using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class OutputResults
    {
        const int maskbit = 0x000000FF;

        public Voxels<int> startVoxels;
        public int[] nList;

        public double E;
        public double nu;
        public double bond_stiffness;
        public Voxels<Vector3d> springs;

        private int noVoxels;
        private int noBonds;
        private double vol;

        public Voxels<double> strainXX;
        public Voxels<double> strainYY;
        public Voxels<double> strainZZ;
        public Voxels<double> strainXY; // epsilon values, not gamma (gamma = 2*epsilon)
        public Voxels<double> strainXZ; // epsilon values, not gamma (gamma = 2*epsilon)
        public Voxels<double> strainYZ; // epsilon values, not gamma (gamma = 2*epsilon)

        public Voxels<double> stressXX;
        public Voxels<double> stressYY;
        public Voxels<double> stressZZ;
        public Voxels<double> stressXY;
        public Voxels<double> stressXZ;
        public Voxels<double> stressYZ;

        public Voxels<double> vonMises;

        public Voxels<Vector3d> princpStress;   // "magnitude" (only sign is relevant?)
        public Voxels<Vector3d[]> princpDir;

        public OutputResults(Voxels<int> startVoxels, int[] nList, double ElasticModulus, double PoissonsRatio, double bond_stiffness, Voxels<Vector3d> springs)
        {
            this.startVoxels = startVoxels;
            this.nList = nList;

            E = ElasticModulus;
            nu = PoissonsRatio;

            this.bond_stiffness = bond_stiffness;
            this.springs = springs;

            noVoxels = startVoxels.n * startVoxels.n * startVoxels.n;
            noBonds = nList.Length * 2;     // *2 because the way the neigbour list is constructed

            Point3d origin = startVoxels.origin;
            double delta = startVoxels.delta;
            int n = startVoxels.n;

            vol = delta * delta * delta;

            strainXX = new Voxels<double>(origin, delta, n);
            strainYY = new Voxels<double>(origin, delta, n);
            strainZZ = new Voxels<double>(origin, delta, n);
            strainXY = new Voxels<double>(origin, delta, n);
            strainXZ = new Voxels<double>(origin, delta, n);
            strainYZ = new Voxels<double>(origin, delta, n);

            stressXX = new Voxels<double>(origin, delta, n);
            stressYY = new Voxels<double>(origin, delta, n);
            stressZZ = new Voxels<double>(origin, delta, n);
            stressXY = new Voxels<double>(origin, delta, n);
            stressXZ = new Voxels<double>(origin, delta, n);
            stressYZ = new Voxels<double>(origin, delta, n);

            vonMises = new Voxels<double>(origin, delta, n);

            princpStress = new Voxels<Vector3d>(origin, delta, n);
            princpDir = new Voxels<Vector3d[]>(origin, delta, n);
        }

        public void UpdateFakeStress(Voxels<Vector3d> dispVoxels)
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix E;
                Matrix A = new Matrix(noBonds + 3, 6);  // Först utan att tänka på special vid boundary
                Matrix S = new Matrix(noBonds + 3, 1);

                int bondCount = 0;

                for (int ii = 0; ii < noBonds / 2; ii++)
                {
                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount += 1;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector3d n);
                        s *= bond_stiffness*vol;
                        S[ii, 0] = s;

                        A[ii, 0] = n.X * n.X;
                        A[ii, 1] = n.Y * n.Y;
                        A[ii, 2] = n.Z * n.Z;
                        A[ii, 3] = 2 * n.X * n.Y;
                        A[ii, 4] = 2 * n.X * n.Z;
                        A[ii, 5] = 2 * n.Y * n.Z;
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount += 1;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector3d n);
                        s *= bond_stiffness * vol;
                        S[noBonds / 2 + ii, 0] = s;

                        A[noBonds / 2 + ii, 0] = n.X * n.X;
                        A[noBonds / 2 + ii, 1] = n.Y * n.Y;
                        A[noBonds / 2 + ii, 2] = n.Z * n.Z;
                        A[noBonds / 2 + ii, 3] = 2 * n.X * n.Y;
                        A[noBonds / 2 + ii, 4] = 2 * n.X * n.Z;
                        A[noBonds / 2 + ii, 5] = 2 * n.Y * n.Z;
                    }
                }

                Matrix AT = A.Duplicate();
                AT.Transpose();

                Matrix AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                E = (AA * AT) * S;


                Vector3d d = dispVoxels.cellValues[i];
                int noRows = A.RowCount;

                double factor = 1;
                double factorA = 1;

                if (springs.cellValues[i].X != 0)
                {
                    A[noRows - 3, 0] = 1 * factorA;
                    S[noRows - 3, 0] = Math.Abs(springs.cellValues[i].X * d.X / (vol/dispVoxels.delta)) * factor * Math.Sign(E[0, 0]);
                }
                
                if (springs.cellValues[i].Y != 0)
                {
                    A[noRows - 2, 1] = 1 * factorA;
                    //S[noRows - 2, 0] = d.Y;
                    S[noRows - 2, 0] = Math.Abs(springs.cellValues[i].Y * d.Y / (vol / dispVoxels.delta)) * factor * Math.Sign(E[1, 0]);
                }

                if (springs.cellValues[i].Z != 0)
                {
                    A[noRows - 1, 2] = 1 * factorA;
                    //S[noRows - 1, 0] = d.Z;
                    S[noRows - 1, 0] = Math.Abs(springs.cellValues[i].Z * d.Z / (vol / dispVoxels.delta)) * factor * Math.Sign(E[2, 0]);
                }

                AT = A.Duplicate();
                AT.Transpose();

                AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                E = (AA * AT) * S;

                stressXX.cellValues[i] = E[0, 0];
                stressYY.cellValues[i] = E[1, 0];
                stressZZ.cellValues[i] = E[2, 0];
                stressXY.cellValues[i] = E[3, 0];
                stressXZ.cellValues[i] = E[4, 0];
                stressYZ.cellValues[i] = E[5, 0];
            }
        }

        public void UpdateFakeStrains(Voxels<Vector3d> dispVoxels)
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix E;
                Matrix A = new Matrix(noBonds + 3, 6);  // Först utan att tänka på special vid boundary
                Matrix S = new Matrix(noBonds + 3, 1);

                int bondCount = 0;

                for (int ii = 0; ii < noBonds / 2; ii++)
                {
                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount++;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector3d n);
                        S[ii, 0] = s;

                        A[ii, 0] = n.X * n.X;
                        A[ii, 1] = n.Y * n.Y;
                        A[ii, 2] = n.Z * n.Z;
                        A[ii, 3] = 2 * n.X * n.Y;
                        A[ii, 4] = 2 * n.X * n.Z;
                        A[ii, 5] = 2 * n.Y * n.Z;
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount++;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector3d n);
                        S[noBonds / 2 + ii, 0] = s;

                        A[noBonds / 2 + ii, 0] = n.X * n.X;
                        A[noBonds / 2 + ii, 1] = n.Y * n.Y;
                        A[noBonds / 2 + ii, 2] = n.Z * n.Z;
                        A[noBonds / 2 + ii, 3] = 2 * n.X * n.Y;
                        A[noBonds / 2 + ii, 4] = 2 * n.X * n.Z;
                        A[noBonds / 2 + ii, 5] = 2 * n.Y * n.Z;
                    }
                }

                Matrix AT = A.Duplicate();
                AT.Transpose();

                Matrix AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                E = (AA * AT) * S;


                Vector3d d = dispVoxels.cellValues[i];
                int noRows = A.RowCount;

                double factorA = 1;
                double factorS = 1;

                int dirBonds = (startVoxels.cellValues[i] >> 24) - bondCount;

                //factorA = dirBonds;
                //factorS = 1.0 * startVoxels.delta;

                //if (springs.cellValues[i].X != 0)
                //{
                //    A[noRows - 3, 0] = 1 * factorA;
                //    S[noRows - 3, 0] = Math.Abs(springs.cellValues[i].X * d.X / (bond_stiffness)) * factorS * Math.Sign(E[0, 0]);
                //}

                //if (springs.cellValues[i].Y != 0)
                //{
                //    A[noRows - 2, 1] = 1 * factorA;
                //    S[noRows - 2, 0] = Math.Abs(springs.cellValues[i].Y * d.Y / (bond_stiffness)) * factorS * Math.Sign(E[1, 0]);
                //}

                //if (springs.cellValues[i].Z != 0)
                //{
                //    A[noRows - 1, 2] = 1 * factorA;
                //    S[noRows - 1, 0] = Math.Abs(springs.cellValues[i].Z * d.Z / (bond_stiffness)) * factorS * Math.Sign(E[2, 0]);
                //}

                //if (springs.cellValues[i].X != 0)
                //{
                //    A[noRows - 3, 0] = 1 * factorA;
                //    S[noRows - 3, 0] = springs.cellValues[i].X * d.X / (bond_stiffness) * factorS;
                //}

                //if (springs.cellValues[i].Y != 0)
                //{
                //    A[noRows - 2, 1] = 1 * factorA;
                //    S[noRows - 2, 0] = (springs.cellValues[i].Y * d.Y / (bond_stiffness)) * factorS;
                //}

                //if (springs.cellValues[i].Z != 0)
                //{
                //    A[noRows - 1, 2] = 1 * factorA;
                //    S[noRows - 1, 0] = (springs.cellValues[i].Z * d.Z / (bond_stiffness)) * factorS;
                //}

                //AT = A.Duplicate();
                //AT.Transpose();

                //AA = AT * A;
                //AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                //E = (AA * AT) * S;

                strainXX.cellValues[i] = E[0, 0];
                strainYY.cellValues[i] = E[1, 0];
                strainZZ.cellValues[i] = E[2, 0];
                strainXY.cellValues[i] = E[3, 0];
                strainXZ.cellValues[i] = E[4, 0];
                strainYZ.cellValues[i] = E[5, 0];
            }
        }

        void CalcStretchAndNormal(Voxels<Vector3d> dispVoxels, int i, int j, out double s, out Vector3d n)
        {
            Vector3d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            Vector3d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

            double xi = xi_vec.Length;
            double y = xi_eta_vec.Length;

            s = (y - xi) / xi;
            n = xi_eta_vec / y;
        }

        public void UpdateStrains(Voxels<Vector3d> dispVoxels)
        {
            for (int i = 0; i < noVoxels; i++)
            {
                // lägg till statement om vad som händer då partiklen också hanterar Dirichlet boundary
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix F;
                Matrix A = new Matrix(noBonds * 3 + 9, 9);  // Plus 9 because of D-boundary
                A.Zero();
                Matrix b = new Matrix(noBonds * 3 + 9, 1);
                b.Zero();

                for (int ii = 0; ii < noBonds / 2; ii++)
                {
                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        Matrix A_j = Calc_Aj(i, j);
                        A[ii * 3 + 0, 0] = A_j[0, 0];
                        A[ii * 3 + 0, 1] = A_j[0, 1];
                        A[ii * 3 + 0, 2] = A_j[0, 2];

                        A[ii * 3 + 1, 3] = A_j[1, 3];
                        A[ii * 3 + 1, 4] = A_j[1, 4];
                        A[ii * 3 + 1, 5] = A_j[1, 5];

                        A[ii * 3 + 2, 6] = A_j[2, 6];
                        A[ii * 3 + 2, 7] = A_j[2, 7];
                        A[ii * 3 + 2, 8] = A_j[2, 8];

                        Matrix b_j = Calc_bj(i, j, dispVoxels);
                        b[ii * 3 + 0, 0] = b_j[0, 0];
                        b[ii * 3 + 1, 0] = b_j[1, 0];
                        b[ii * 3 + 2, 0] = b_j[2, 0];
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        Matrix A_j = Calc_Aj(i, j);
                        A[noBonds / 2 * 3 + ii * 3 + 0, 0] = A_j[0, 0];
                        A[noBonds / 2 * 3 + ii * 3 + 0, 1] = A_j[0, 1];
                        A[noBonds / 2 * 3 + ii * 3 + 0, 2] = A_j[0, 2];

                        A[noBonds / 2 * 3 + ii * 3 + 1, 3] = A_j[1, 3];
                        A[noBonds / 2 * 3 + ii * 3 + 1, 4] = A_j[1, 4];
                        A[noBonds / 2 * 3 + ii * 3 + 1, 5] = A_j[1, 5];

                        A[noBonds / 2 * 3 + ii * 3 + 2, 6] = A_j[2, 6];
                        A[noBonds / 2 * 3 + ii * 3 + 2, 7] = A_j[2, 7];
                        A[noBonds / 2 * 3 + ii * 3 + 2, 8] = A_j[2, 8];

                        Matrix b_j = Calc_bj(i, j, dispVoxels);
                        b[noBonds / 2 * 3 + ii * 3 + 0, 0] = b_j[0, 0];
                        b[noBonds / 2 * 3 + ii * 3 + 1, 0] = b_j[1, 0];
                        b[noBonds / 2 * 3 + ii * 3 + 2, 0] = b_j[2, 0];
                    }
                }
                //Double[] xi_D = new Double[3];

                //if (springs.cellValues[i].X == 0)
                //    xi_D[0] = 0;
                //else
                //    xi_D[0] = bond_stiffness * vol / springs.cellValues[i].X;

                //if (springs.cellValues[i].Y == 0)
                //    xi_D[1] = 0;
                //else
                //    xi_D[1] = bond_stiffness * vol / springs.cellValues[i].Y;

                //if (springs.cellValues[i].Z == 0)
                //    xi_D[2] = 0;
                //else
                //    xi_D[2] = 0.5 * startVoxels.delta;
                    // xi_D[2] = bond_stiffness * vol / springs.cellValues[i].Z;

                //xi_D[0] = 0;
                //xi_D[1] = 0;
                //xi_D[2] = 0;

                int noRows = A.RowCount;

                //for (int j = 0; j < 3; j++)
                //{
                //    A[noRows - (j * 3 + 8), 0 + j] = xi_D[j];
                //    A[noRows - (j * 3 + 9), 4 + j] = xi_D[j];
                //    A[noRows - (j * 3 - 7), 7 + j] = xi_D[j];
                //}

                //A[noRows - 9, 0] = xi_D[0];
                //A[noRows - 8, 3] = xi_D[0];
                //A[noRows - 7, 6] = xi_D[0];

                //A[noRows - 6, 1] = xi_D[1];
                //A[noRows - 5, 4] = xi_D[1];
                //A[noRows - 4, 7] = xi_D[1];

                //A[noRows - 3, 2] = xi_D[2];
                //A[noRows - 2, 5] = xi_D[2];
                //A[noRows - 1, 8] = xi_D[2];

                //Vector3d eta_D = dispVoxels.cellValues[i];
                //eta_D.X += xi_D[0];
                //eta_D.Y += xi_D[1];
                //eta_D.Z += xi_D[2];

                //if (xi_D[0] == 0)
                //{
                //    b[noRows - 9, 0] = 0;
                //    b[noRows - 8, 0] = 0;
                //    b[noRows - 7, 0] = 0;
                //}
                //else
                //{
                //    b[noRows - 9, 0] = eta_D.X;
                //    b[noRows - 8, 0] = eta_D.Y;
                //    b[noRows - 7, 0] = eta_D.Z;
                //}

                //if (xi_D[1] == 0)
                //{
                //    b[noRows - 6, 0] = 0;
                //    b[noRows - 5, 0] = 0;
                //    b[noRows - 4, 0] = 0;
                //}
                //else
                //{
                //    b[noRows - 6, 0] = eta_D.X;
                //    b[noRows - 5, 0] = eta_D.Y;
                //    b[noRows - 4, 0] = eta_D.Z;
                //}

                //if (xi_D[2] == 0)
                //{
                //    b[noRows - 3, 0] = 0;
                //    b[noRows - 2, 0] = 0;
                //    b[noRows - 1, 0] = 0;
                //}
                //else
                //{
                //    b[noRows - 3, 0] = eta_D.X;
                //    b[noRows - 2, 0] = eta_D.Y;
                //    b[noRows - 1, 0] = eta_D.Z;
                //}
                            
                // F = (A'*A)^(-1)*A'*b
                // Borde kanske introducera weightning matrix W för bättre resultat
                Matrix AT = A.Duplicate();
                AT.Transpose();

                Matrix AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                F = (AA * AT) * b;

                double Fxx = F[0, 0];
                double Fxy = (F[1, 0] + F[3, 0]) * 0.5;
                double Fxz = (F[2, 0] + F[6, 0]) * 0.5;
                double Fyy = F[4, 0];
                double Fyz = (F[5, 0] + F[7, 0]) * 0.5;
                double Fzz = F[8, 0];

                strainXX.cellValues[i] = 0.5 * (Fxx * Fxx + Fxy * Fxy + Fxz * Fxz - 1);
                strainYY.cellValues[i] = 0.5 * (Fxy * Fxy + Fyy * Fyy + Fyz * Fyz - 1);
                strainZZ.cellValues[i] = 0.5 * (Fxz * Fxz + Fyz * Fyz + Fzz * Fzz - 1);
                strainXY.cellValues[i] = 0.5 * (Fxx * Fxy + Fyy * Fxy + Fyz * Fxz);
                strainXZ.cellValues[i] = 0.5 * (Fxz * Fxx + Fyz * Fxy + Fzz * Fxz);
                strainYZ.cellValues[i] = 0.5 * (Fxz * Fxy + Fyz * Fyy + Fzz * Fyz);

            }
        }

        private Matrix Calc_Aj(int i, int j)
        {
            Matrix A_j = new Matrix(3, 9);
            A_j.Zero();
            Vector3d xi = startVoxels.IndexToPoint(i) - startVoxels.IndexToPoint(j);

            A_j[0, 0] = xi.X;
            A_j[0, 1] = xi.Y;
            A_j[0, 2] = xi.Z;

            A_j[1, 3] = xi.X;
            A_j[1, 4] = xi.Y;
            A_j[1, 5] = xi.Z;

            A_j[2, 6] = xi.X;
            A_j[2, 7] = xi.Y;
            A_j[2, 8] = xi.Z;

            return A_j;
        }

        private Matrix Calc_bj(int i, int j, Voxels<Vector3d> dispVoxels)
        {
            Matrix b_j = new Matrix(3, 1);
            b_j.Zero();
            Vector3d eta = (dispVoxels.cellValues[i] + startVoxels.IndexToPoint(i)) - (dispVoxels.cellValues[j] + startVoxels.IndexToPoint(j));

            b_j[0, 0] = eta.X;
            b_j[1, 0] = eta.Y;
            b_j[2, 0] = eta.Z;

            return b_j;
        }

        public void UpdateStresses()
        {
            double C = E / ((1 + nu) * (1 - 2 * nu));

            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                stressXX.cellValues[i] = C * ((1 - nu) * strainXX.cellValues[i] + nu * strainYY.cellValues[i] + nu * strainZZ.cellValues[i]);
                stressYY.cellValues[i] = C * (nu * strainXX.cellValues[i] + (1 - nu) * strainYY.cellValues[i] + nu * strainZZ.cellValues[i]);
                stressZZ.cellValues[i] = C * (nu * strainXX.cellValues[i] + nu * strainYY.cellValues[i] + (1 - nu) * strainZZ.cellValues[i]);
                stressXY.cellValues[i] = C * (1 - 2 * nu) * strainXY.cellValues[i];
                stressXZ.cellValues[i] = C * (1 - 2 * nu) * strainXZ.cellValues[i];
                stressYZ.cellValues[i] = C * (1 - 2 * nu) * strainYZ.cellValues[i];
            }
        }

        public void UpdateVonMises()
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;
                double xx = stressXX.cellValues[i];
                double yy = stressYY.cellValues[i];
                double zz = stressZZ.cellValues[i];
                double xy = stressXY.cellValues[i];
                double xz = stressXZ.cellValues[i];
                double yz = stressYZ.cellValues[i];

                vonMises.cellValues[i] = Math.Sqrt(0.5*((xx-yy)*(xx-yy)+(yy-zz)*(yy-zz)+(zz-xx)*(zz-xx))+3*(xy*xy+xz*xz+yz*yz));
            }
        }

        public void UpdatePrincipalStresses()
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Transform stressTensor = new Transform();

                stressTensor.M00 = stressXX.cellValues[i];
                stressTensor.M01 = stressXY.cellValues[i];
                stressTensor.M02 = stressXZ.cellValues[i];

                stressTensor.M10 = stressXY.cellValues[i];
                stressTensor.M11 = stressYY.cellValues[i];
                stressTensor.M12 = stressYZ.cellValues[i];

                stressTensor.M20 = stressXZ.cellValues[i];
                stressTensor.M21 = stressYZ.cellValues[i];
                stressTensor.M22 = stressZZ.cellValues[i];

                stressTensor.M33 = 1;   // In order to make the stressTensor linear

                bool test = stressTensor.DecomposeSymmetric(out Transform EigenVectors, out Vector3d EigenValues);

                princpStress.cellValues[i] = EigenValues;

                Vector3d[] vecList = new Vector3d[3];
                
                for (int j = 0; j < 3; j++)
                {
                    vecList[j].X = EigenVectors[0, j];
                    vecList[j].Y = EigenVectors[1, j];
                    vecList[j].Z = EigenVectors[2, j];
                }
                princpDir.cellValues[i] = vecList;               

            }
            
        }

    }
}
