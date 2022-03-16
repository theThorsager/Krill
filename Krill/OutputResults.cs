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
        public Voxels<Vector3d> bodyload;

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

        public Voxels<Matrix> strain;
        public Voxels<Matrix> stressTensor;

        public Voxels<double> sumCurl;

        public Voxels<double> sumCurlPstress;

        public Voxels<double> lengthDivStress;

        public Voxels<Vector3d> divOfStress;

        //public OutputResults(Voxels<int> startVoxels, int[] nList, double ElasticModulus, double PoissonsRatio, double bond_stiffness, Voxels<Vector3d> springs, Voxels<Vector3d> bodyload)
        public OutputResults(Containers.LinearSolution linSol, double PoissonsRatio = 0.25)
        {
            this.startVoxels = linSol.mask;
            this.bodyload = linSol.bodyload;
            this.nList = linSol.nList;

            E = linSol.elasticModulus;
            nu = PoissonsRatio;

            this.bond_stiffness = linSol.bondStiffness;
            this.springs = linSol.springs;

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
            strain = new Voxels<Matrix>(origin, delta, n);
            this.stressTensor = new Voxels<Matrix>(origin, delta, n);

            sumCurl = new Voxels<double>(origin, delta, n);

            sumCurlPstress = new Voxels<double>(origin, delta, n);

            lengthDivStress = new Voxels<double>(origin, delta, n);

            divOfStress = new Voxels<Vector3d>(origin, delta, n);
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
                        int vols = startVoxels.cellValues[i] >> 20;
                        vols += startVoxels.cellValues[j] >> 20;
                        double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                        s *= bond_stiffness * vol * factorS;
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
                        int vols = startVoxels.cellValues[i] >> 20;
                        vols += startVoxels.cellValues[j] >> 20;
                        double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                        s *= bond_stiffness * vol * factorS;
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


                //Vector3d d = dispVoxels.cellValues[i];
                //int noRows = A.RowCount;

                //double factor = 1;
                //double factorA = 1;

                //if (springs.cellValues[i].X != 0)
                //{
                //    A[noRows - 3, 0] = 1 * factorA;
                //    S[noRows - 3, 0] = Math.Abs(springs.cellValues[i].X * d.X + bodyload.cellValues[i].X) * factor * Math.Sign(E[0, 0]);
                //}
                
                //if (springs.cellValues[i].Y != 0)
                //{
                //    A[noRows - 2, 1] = 1 * factorA;
                //    //S[noRows - 2, 0] = d.Y;
                //    S[noRows - 2, 0] = Math.Abs(springs.cellValues[i].Y * d.Y + bodyload.cellValues[i].Y) * factor * Math.Sign(E[1, 0]);
                //}

                //if (springs.cellValues[i].Z != 0)
                //{
                //    A[noRows - 1, 2] = 1 * factorA;
                //    //S[noRows - 1, 0] = d.Z;
                //    S[noRows - 1, 0] = Math.Abs(springs.cellValues[i].Z * d.Z + bodyload.cellValues[i].Z) * factor * Math.Sign(E[2, 0]);
                //}

                //AT = A.Duplicate();
                //AT.Transpose();

                //AA = AT * A;
                //AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                //E = (AA * AT) * S;

                stressXX.cellValues[i] = E[0, 0];
                stressYY.cellValues[i] = E[1, 0];
                stressZZ.cellValues[i] = E[2, 0];
                stressXY.cellValues[i] = E[3, 0];
                stressXZ.cellValues[i] = E[4, 0];
                stressYZ.cellValues[i] = E[5, 0];

                Matrix tensor = new Matrix(3, 3);
                tensor[0, 0] = stressXX.cellValues[i];
                tensor[0, 1] = stressXY.cellValues[i];
                tensor[0, 2] = stressXZ.cellValues[i];
                tensor[1, 0] = stressXY.cellValues[i];
                tensor[1, 1] = stressYY.cellValues[i];
                tensor[1, 2] = stressYZ.cellValues[i];
                tensor[2, 0] = stressXZ.cellValues[i];
                tensor[2, 1] = stressYZ.cellValues[i];
                tensor[2, 2] = stressZZ.cellValues[i];

                stressTensor.cellValues[i] = tensor;
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
                int dirBonds = (startVoxels.cellValues[i] >> 20) - bondCount;


                //factorA = dirBonds;
                //factorS = 1.0 * startVoxels.delta;

                if (springs.cellValues[i].X != 0)
                {
                    A[noRows - 3, 0] = 1 * factorA;
                    S[noRows - 3, 0] = Math.Abs((springs.cellValues[i].X * d.X + bodyload.cellValues[i].X) / (bond_stiffness)) * factorS * Math.Sign(E[0, 0]);
                }

                if (springs.cellValues[i].Y != 0)
                {
                    A[noRows - 2, 1] = 1 * factorA;
                    S[noRows - 2, 0] = Math.Abs((springs.cellValues[i].Y * d.Y + bodyload.cellValues[i].Y) / (bond_stiffness)) * factorS * Math.Sign(E[1, 0]);
                }

                if (springs.cellValues[i].Z != 0)
                {
                    A[noRows - 1, 2] = 1 * factorA;
                    S[noRows - 1, 0] = Math.Abs((springs.cellValues[i].Z * d.Z + bodyload.cellValues[i].Z) / (bond_stiffness)) * factorS * Math.Sign(E[2, 0]);
                }

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

                AT = A.Duplicate();
                AT.Transpose();

                AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                E = (AA * AT) * S;

                strainXX.cellValues[i] = E[0, 0];
                strainYY.cellValues[i] = E[1, 0];
                strainZZ.cellValues[i] = E[2, 0];
                strainXY.cellValues[i] = E[3, 0];
                strainXZ.cellValues[i] = E[4, 0];
                strainYZ.cellValues[i] = E[5, 0];
                E[3, 0] *= 2.0;
                E[4, 0] *= 2.0;
                E[5, 0] *= 2.0;
                strain.cellValues[i] = E;
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

                Matrix tensor = new Matrix(3, 3);
                tensor[0, 0] = stressXX.cellValues[i];
                tensor[0, 1] = stressXY.cellValues[i];
                tensor[0, 2] = stressXZ.cellValues[i];
                tensor[1, 0] = stressXY.cellValues[i];
                tensor[1, 1] = stressYY.cellValues[i];
                tensor[1, 2] = stressYZ.cellValues[i];
                tensor[2, 0] = stressXZ.cellValues[i];
                tensor[2, 1] = stressYZ.cellValues[i];
                tensor[2, 2] = stressZZ.cellValues[i];

                stressTensor.cellValues[i] = tensor;
            }
        }

        public void UpdateFakeStresses()
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix C = ConstructFakeMaterial(i);
                Matrix stress = C * strain.cellValues[i];

                stressXX.cellValues[i] = stress[0, 0];
                stressYY.cellValues[i] = stress[1, 0];
                stressZZ.cellValues[i] = stress[2, 0];
                stressXY.cellValues[i] = stress[3, 0];
                stressXZ.cellValues[i] = stress[4, 0];
                stressYZ.cellValues[i] = stress[5, 0];
            }
        }

        public Matrix ConstructFakeMaterial(int i)
        {

            const int numb = 36;
            Matrix matrix = new Matrix(numb, noBonds);
            Matrix c = new Matrix(noBonds, 1);
            for (int j = 0; j < nList.Length; j++)
            {
                int jj = i - nList[j];
                if (startVoxels.cellValues[jj] != 0)
                    continue;

                Vector3d xi = startVoxels.IndexToPoint(jj) - startVoxels.IndexToPoint(i);
                double xiL = xi.Length;

                int vols = startVoxels.cellValues[i] >> 20;
                vols += startVoxels.cellValues[jj] >> 20;
                double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                c[j, 0] = bond_stiffness * factorS;

                for (int i1 = 0; i1 < 6; i1++)
                {
                    double z1 = GetZ(i1, xi);
                    for (int i2 = 0; i2 < 6; i2++)
                    {
                        double z2 = GetZ(i2, xi);

                        double value = vol * 0.5 * z1 * z2 / (xiL * xiL * xiL);
                        matrix[i1 * 6 + i2, j] = value;
                    }
                }
            }
            for (int j = 0; j < nList.Length; j++)
            {
                int jj = i + nList[j];
                if (startVoxels.cellValues[jj] != 0)
                    continue;

                Vector3d xi = startVoxels.IndexToPoint(jj) - startVoxels.IndexToPoint(i);
                double xiL = xi.Length;

                int vols = startVoxels.cellValues[i] >> 20;
                vols += startVoxels.cellValues[jj] >> 20;
                double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                c[nList.Length + j, 0] = bond_stiffness * factorS;

                for (int i1 = 0; i1 < 6; i1++)
                {
                    double z1 = GetZ(i1, xi);
                    for (int i2 = 0; i2 < 6; i2++)
                    {
                        double z2 = GetZ(i2, xi);

                        double value = vol * 0.5 * z1 * z2/ (xiL * xiL * xiL);
                        matrix[i1 * 6 + i2, nList.Length + j] = value;
                    }
                }
            }

            Matrix Voigt = matrix * c;

            Matrix C = new Matrix(6, 6);
            for (int ii = 0; ii < 6; ii++)
            {
                for (int jj = 0; jj < 6; jj++)
                {
                    C[ii, jj] = Voigt[6 * ii + jj, 0];
                }
            }
            Matrix Ct = C.Duplicate();
            Ct.Transpose();
            C = (C + Ct);
            for (int ii = 0; ii < 6; ii++)
            {
                for (int jj = 0; jj < 6; jj++)
                {
                    C[ii, jj] *= 0.5;
                }
            }

            return C;
        }

        private double GetZ(int i1, Vector3d xi)
        {
            //switch (i1)
            //{
            //    case 0:
            //        return xi.X * xi.X;
            //    case 1:
            //        return xi.Y * xi.Y;
            //    case 2:
            //        return xi.Z * xi.Z;
            //    case 3:
            //        return xi.X * xi.Y;
            //    case 4:
            //        return xi.X * xi.Z;
            //    case 5:
            //        return xi.Z * xi.Y;
            //}
            //return 0;

            switch (i1)
            {
                case 0:
                    return xi.X * xi.X;
                case 1:
                    return xi.Y * xi.Y;
                case 2:
                    return xi.Z * xi.Z;
                case 3:
                    return xi.Y * xi.Z;
                case 4:
                    return xi.X * xi.Z;
                case 5:
                    return xi.X * xi.Y;
            }
            return 0;
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

        public void CalcCurlOfStressField()
        {
            NormalizeStressTensor();

            for (int ind = 0; ind < noVoxels; ind++)
            {
                if ((startVoxels.cellValues[ind] & maskbit) == 0)
                    continue;

                Matrix curl = new Matrix(3, 3);
                double curlSum = 0;

                for (int m = 0; m < 3; m++)
                {
                    int k = 0;
                    curl[k, m] = DerStress(ind, m, 2, 1) - DerStress(ind, m, 1, 2);
                    curlSum += curl[k, m] * curl[k, m];

                    k = 1;
                    curl[k, m] = DerStress(ind, m, 0, 2) - DerStress(ind, m, 2, 0);
                    curlSum += curl[k, m] * curl[k, m];

                    k = 2;
                    curl[k, m] = DerStress(ind, m, 1, 0) - DerStress(ind, m, 0, 1);
                    curlSum += curl[k, m] * curl[k, m];
                }

                sumCurl.cellValues[ind] = curlSum;

            }
        }

        public void CalcDivOfStressField()
        {
            for (int ind = 0; ind < noVoxels; ind++)
            {
                if ((startVoxels.cellValues[ind] & maskbit) == 0)
                    continue;

                Vector3d div = new Vector3d();

                for (int k = 0; k < 3; k++)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        div[k] += DerStress(ind, i, k, i);
                    }
                }

                lengthDivStress.cellValues[ind] = div.SquareLength;
                divOfStress.cellValues[ind] = div;
            }
        }

        private void NormalizeStressTensor()
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix tensor = stressTensor.cellValues[i];
                double sum = 0;

                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        sum += tensor[k, j] * tensor[k, j];
                    }
                }

                sum = Math.Sqrt(sum);

                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        tensor[k, j] /= sum;
                    }
                }
            }
        }

        private double DerStress(int ind, int a, int b, int dir)
        {
            double dS;

            int i = ind;

            stressTensor.To3DIndex(ref i, out int j, out int k);

            Vector3d ind3d = new Vector3d(i, j, k);

            double dx = stressTensor.delta * 2;

            ind3d[dir] += -1;

            int n0 = stressTensor.ToLinearIndex((int)ind3d.X, (int)ind3d.Y, (int)ind3d.Z);
            double dS0;
            if (startVoxels.cellValues[n0] != 0)
                dS0 = stressTensor.cellValues[n0][a, b];
            else
            {
                dS0 = stressTensor.cellValues[ind][a, b];
                dx *= 0.5;
            }                

            ind3d[dir] += 2;

            int n2 = stressTensor.ToLinearIndex((int)ind3d.X, (int)ind3d.Y, (int)ind3d.Z);
            double dS2;
            if (startVoxels.cellValues[n2] != 0)
                dS2 = stressTensor.cellValues[n2][a, b];
            else
            {
                dS2 = stressTensor.cellValues[ind][a, b];
                dx *= 0.5;
            }

            dS = (dS2 - dS0) / dx;

            return dS;
        }

        public void CalcCurlOfPrinpStress()
        {
            for (int ind = 0; ind < noVoxels; ind++)
            {
                if ((startVoxels.cellValues[ind] & maskbit) == 0)
                    continue;

                Vector3d[] curlPstress = new Vector3d[3];
                double sumLengthCurl = 0;

                for (int pStress = 0; pStress < 3; pStress++)
                {
                    Vector3d curlP = new Vector3d();

                    int k = 0;
                    curlP[k] = DerPrinStress(ind, 2, 1, pStress) - DerPrinStress(ind, 1, 2, pStress);

                    k = 1;
                    curlP[k] = DerPrinStress(ind, 0, 2, pStress) - DerPrinStress(ind, 2, 0, pStress);

                    k = 2;
                    curlP[k] = DerPrinStress(ind, 1, 0, pStress) - DerPrinStress(ind, 0, 1, pStress);

                    curlPstress[pStress] = curlP;
                    sumLengthCurl += curlP.SquareLength;
                }

                sumCurlPstress.cellValues[ind] = sumLengthCurl;
            }
        }

        private double DerPrinStress(int ind, int a, int derDir, int indPs)
        {
            double dP;
            Vector3d dir = princpDir.cellValues[ind][indPs] * princpStress.cellValues[ind][indPs];

            int i = ind;

            princpDir.To3DIndex(ref i, out int j, out int k);
            Vector3d ind3d = new Vector3d(i, j, k);

            ind3d[derDir] += -1;
            int n0 = stressTensor.ToLinearIndex((int)ind3d.X, (int)ind3d.Y, (int)ind3d.Z);
            ind3d[derDir] += 2;
            int n2 = stressTensor.ToLinearIndex((int)ind3d.X, (int)ind3d.Y, (int)ind3d.Z);

            double dx = princpStress.delta * 2;

            double dv0;
            if (startVoxels.cellValues[n0] != 0)
            {
                Vector3d v0 = CorrectPrincpDir(n0, dir);
                dv0 = v0[a];
            }
            else
            {
                dv0 = dir[a];
                dx *= 0.5;
            }

            double dv2;
            if (startVoxels.cellValues[n2] != 0)
            {
                Vector3d v2 = CorrectPrincpDir(n2, dir);
                dv2 = v2[a];
            }
            else
            {
                dv2 = dir[a];
                dx *= 0.5;
            }
            
            dP = (dv2 - dv0) / dx;

            return dP;
        }

        private Vector3d CorrectPrincpDir(int ind, Vector3d dir)
        {
            Vector3d[] pDir = new Vector3d[6];
            double[] angle = new double[6];
            double minAng = double.MaxValue;
            int minInd = 0;

            for (int n = 0; n < 3; n++)
            {
                pDir[n] = princpDir.cellValues[ind][n] * princpStress.cellValues[ind][n];
                pDir[n + 3] = -pDir[n];

                angle[n] = Vector3d.VectorAngle(dir, pDir[n]);
                angle[n + 3] = Vector3d.VectorAngle(dir, pDir[n + 3]);
            }

            for (int k = 0; k < 6; k++)
            {
                if (angle[k] < minAng)
                {
                    minAng = angle[k];
                    minInd = k;
                }
            }

            return pDir[minInd];
        }
    }
}
