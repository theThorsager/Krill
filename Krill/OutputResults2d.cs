using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class OutputResults2d
    {
        const int maskbit = 0x000000FF;

        public Voxels2d<int> startVoxels;
        public int[] nList;

        double oldcuttoff = 0.0;
        bool relaxTension = false;

        public double E;
        public double nu;
        public double bond_stiffness;
        public Voxels2d<Vector2d> springs;
        public Voxels2d<Vector2d> bodyload;

        private int noVoxels;
        private int noBonds;
        private double vol;

        public Voxels2d<double> strainXX;
        public Voxels2d<double> strainYY;
        public Voxels2d<double> strainXY; // epsilon values, not gamma (gamma = 2*epsilon)

        public Voxels2d<double> stressXX;
        public Voxels2d<double> stressYY;
        public Voxels2d<double> stressXY;

        public Voxels2d<double> vonMises;

        public Voxels2d<Vector3d> princpStress;   // "magnitude" (only sign is relevant?)
        public Voxels2d<Vector3d[]> princpDir;
        public Voxels2d<Matrix> strain;

        public Voxels2d<double> strainDer;

        public Voxels2d<Matrix> stressTensor;

        public Voxels2d<Vector3d> curlVec;
        public Voxels2d<double> curlLsquared;

        //public OutputResults(Voxels<int> startVoxels, int[] nList, double ElasticModulus, double PoissonsRatio, double bond_stiffness, Voxels<Vector3d> springs, Voxels<Vector3d> bodyload)
        public OutputResults2d(Containers.LinearSolution2d linSol, double PoissonsRatio = 0.333333)
        {
            this.oldcuttoff = linSol.oldcuttoff;
            this.relaxTension = linSol.relaxTension;

            this.startVoxels = linSol.mask;
            this.bodyload = linSol.bodyload;
            this.nList = linSol.nList;

            E = linSol.elasticModulus;
            nu = PoissonsRatio;

            this.bond_stiffness = linSol.bondStiffness;
            this.springs = linSol.springs;

            noVoxels = startVoxels.n * startVoxels.n;
            noBonds = nList.Length * 2;     // *2 because the way the neigbour list is constructed

            Point2d origin = startVoxels.origin;
            double delta = startVoxels.delta;
            int n = startVoxels.n;

            vol = delta * delta;

            strainXX = new Voxels2d<double>(origin, delta, n);
            strainYY = new Voxels2d<double>(origin, delta, n);
            strainXY = new Voxels2d<double>(origin, delta, n);

            stressXX = new Voxels2d<double>(origin, delta, n);
            stressYY = new Voxels2d<double>(origin, delta, n);
            stressXY = new Voxels2d<double>(origin, delta, n);

            vonMises = new Voxels2d<double>(origin, delta, n);

            princpStress = new Voxels2d<Vector3d>(origin, delta, n);
            princpDir = new Voxels2d<Vector3d[]>(origin, delta, n);
            strain = new Voxels2d<Matrix>(origin, delta, n);

            strainDer = new Voxels2d<double>(origin, delta, n);

            stressTensor = new Voxels2d<Matrix>(origin, delta, n);

            curlVec = new Voxels2d<Vector3d>(origin, delta, n);
            curlLsquared = new Voxels2d<double>(origin, delta, n);
        }

        public void UpdateFakeStress2(Voxels2d<Vector2d> dispVoxels)
        {
            // Make the stress strain curve stranger
            const double low = 0.05;
            const double high = 0.5;
            const double fade = 0.1;

            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                for (int ii = 0; ii < noBonds / 2; ii++)
                {

                    double counttwice = 1;
                    if (startVoxels.cellValues[i + nList[ii]] != 0 ^ startVoxels.cellValues[i - nList[ii]] != 0)
                        counttwice = 1;

                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        Vector2d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
                        Vector2d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

                        double xi = xi_vec.Length;
                        double y = xi_eta_vec.Length;

                        double s = (y - xi) / xi;

                        int vols = startVoxels.cellValues[i] >> 20;
                        vols += startVoxels.cellValues[j] >> 20;
                        double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                        s *= bond_stiffness * vol * factorS;

                        if (relaxTension && s > low * oldcuttoff)
                        {
                            double fa = s / oldcuttoff;
                            if (fa < low + fade)
                                s *= -(fa - (low + fade)) / fade;
                            else if (fa < high - fade)
                                s = 0;
                            else if (fa < high)
                                s *= (fa - (high - fade)) / fade;
                        }

                        AddDyadicProduct(s * xi_eta_vec / y, xi_vec * counttwice, i);
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        Vector2d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
                        Vector2d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

                        double xi = xi_vec.Length;
                        double y = xi_eta_vec.Length;

                        double s = (y - xi) / xi;

                        int vols = startVoxels.cellValues[i] >> 20;
                        vols += startVoxels.cellValues[j] >> 20;
                        double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                        s *= bond_stiffness * vol * factorS;

                        if (relaxTension && s > low * oldcuttoff)
                        {
                            double fa = s / oldcuttoff;
                            if (fa < low + fade)
                                s *= -(fa - (low + fade)) / fade;
                            else if (fa < high - fade)
                                s = 0;
                            else if (fa < high)
                                s *= (fa - (high - fade)) / fade;
                        }

                        AddDyadicProduct(s * xi_eta_vec / y, xi_vec * counttwice, i);
                    }
                }

                stressXX.cellValues[i] *= 0.5;
                stressXY.cellValues[i] *= 0.5;
                stressYY.cellValues[i] *= 0.5;

                // the BCs force contributions should be added
                double reduction = startVoxels.delta;
                stressXX.cellValues[i] += Math.Abs(bodyload.cellValues[i].X / vol) * reduction;
                stressYY.cellValues[i] += Math.Abs(bodyload.cellValues[i].Y / vol) * reduction;
                stressXX.cellValues[i] += springs.cellValues[i].X * dispVoxels.cellValues[i].X * reduction;
                stressYY.cellValues[i] += springs.cellValues[i].Y * dispVoxels.cellValues[i].Y * reduction;
            }
        }

        void AddDyadicProduct(Vector2d a, Vector2d b, int i)
        {
            stressXX.cellValues[i] += a.X * b.X;
            stressXY.cellValues[i] += a.X * b.Y;
            stressYY.cellValues[i] += a.Y * b.Y;

            double test = a.X * b.Y;
            double test2 = a.Y * b.X;
        }

        public void UpdateFakeStress(Voxels2d<Vector2d> dispVoxels)
        {
            // Make the stress strain curve stranger
            const double low = 0.05;
            const double high = 0.5;
            const double fade = 0.1;

            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix E;
                Matrix A = new Matrix(noBonds + 2, 3);  // Först utan att tänka på special vid boundary
                Matrix S = new Matrix(noBonds + 2, 1);

                int bondCount = 0;

                for (int ii = 0; ii < noBonds / 2; ii++)
                {
                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount += 1;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector2d n);
                        int vols = startVoxels.cellValues[i] >> 20;
                        vols += startVoxels.cellValues[j] >> 20;
                        double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                        s *= bond_stiffness;

                        if (relaxTension && s > low * oldcuttoff)
                        {
                            double fa = s / oldcuttoff;
                            if (fa < low + fade)
                                s *= -(fa - (low + fade)) / fade;
                            else if (fa < high - fade)
                                s = 0;
                            else if (fa < high)
                                s *= (fa - (high - fade)) / fade;
                        }

                        S[ii, 0] = s * factorS;

                        A[ii, 0] = n.X * n.X;
                        A[ii, 1] = n.Y * n.Y;
                        A[ii, 2] = 2 * n.X * n.Y;
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount += 1;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector2d n);
                        int vols = startVoxels.cellValues[i] >> 20;
                        vols += startVoxels.cellValues[j] >> 20;
                        double factorS = (double)(noBonds * 2.0 + 2.0) / (double)vols;
                        s *= bond_stiffness;

                        if (relaxTension && s > low * oldcuttoff)
                        {
                            double fa = s / oldcuttoff;
                            if (fa < low + fade)
                                s *= -(fa - (low + fade)) / fade;
                            else if (fa < high - fade)
                                s = 0;
                            else if (fa < high)
                                s *= (fa - (high - fade)) / fade;
                        }

                        S[noBonds / 2 + ii, 0] = s * factorS;

                        A[noBonds / 2 + ii, 0] = n.X * n.X;
                        A[noBonds / 2 + ii, 1] = n.Y * n.Y;
                        A[noBonds / 2 + ii, 2] = 2 * n.X * n.Y;
                    }
                }

                Matrix AT = A.Duplicate();
                AT.Transpose();

                Matrix AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                E = (AA * AT) * S;


                //Vector2d d = dispVoxels.cellValues[i];
                //int noRows = A.RowCount;

                //double factor = 1;
                //double factorA = 1;

                //if (springs.cellValues[i].X != 0)
                //{
                //    A[noRows - 2, 0] = 1 * factorA;
                //    S[noRows - 2, 0] = Math.Abs(springs.cellValues[i].X * d.X + bodyload.cellValues[i].X) * factor * Math.Sign(E[0, 0]);
                //}

                //if (springs.cellValues[i].Y != 0)
                //{
                //    A[noRows - 1, 1] = 1 * factorA;
                //    S[noRows - 1, 0] = Math.Abs(springs.cellValues[i].Y * d.Y + bodyload.cellValues[i].Y) * factor * Math.Sign(E[1, 0]);
                //}

                //AT = A.Duplicate();
                //AT.Transpose();

                //AA = AT * A;
                //AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                //E = (AA * AT) * S;

                stressXX.cellValues[i] = E[0, 0];
                stressYY.cellValues[i] = E[1, 0];
                stressXY.cellValues[i] = E[2, 0];

                Matrix tensor = new Matrix(2, 2);
                tensor[0, 0] = stressXX.cellValues[i];
                tensor[0, 1] = stressXY.cellValues[i];
                tensor[1, 0] = stressXY.cellValues[i];
                tensor[1, 1] = stressYY.cellValues[i];

                stressTensor.cellValues[i] = tensor;
            }
        }

        public void UpdateFakeStrains(Voxels2d<Vector2d> dispVoxels)
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix E;
                Matrix A = new Matrix(noBonds + 2, 3);  // Först utan att tänka på special vid boundary
                Matrix S = new Matrix(noBonds + 2, 1);

                int bondCount = 0;

                for (int ii = 0; ii < noBonds / 2; ii++)
                {
                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount++;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector2d n);
                        S[ii, 0] = s;

                        A[ii, 0] = n.X * n.X;
                        A[ii, 1] = n.Y * n.Y;
                        A[ii, 2] = 2 * n.X * n.Y;
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount++;

                        CalcStretchAndNormal(dispVoxels, i, j, out double s, out Vector2d n);
                        S[noBonds / 2 + ii, 0] = s;

                        A[noBonds / 2 + ii, 0] = n.X * n.X;
                        A[noBonds / 2 + ii, 1] = n.Y * n.Y;
                        A[noBonds / 2 + ii, 2] = 2 * n.X * n.Y;
                    }
                }

                Matrix AT = A.Duplicate();
                AT.Transpose();

                Matrix AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                E = (AA * AT) * S;


                Vector2d d = dispVoxels.cellValues[i];
                int noRows = A.RowCount;

                int vols = startVoxels.cellValues[i] >> 20;
                double factor = (double)(nList.Length * 2.0 + 1.0) / (double)vols;
                int dirBonds = (startVoxels.cellValues[i] >> 20) - bondCount;

                double factorA = 0;
                double factorS = 1;

                double l = 1;

                if (springs.cellValues[i].X != 0)
                {
                    A[noRows - 2, 0] = 1 * factorA;
                    S[noRows - 2, 0] = Math.Abs((springs.cellValues[i].X * d.X + bodyload.cellValues[i].X) / (springs.cellValues[i].X * l)) * factorS * factorA * Math.Sign(E[0, 0]);
                }

                if (springs.cellValues[i].Y != 0)
                {
                    A[noRows - 1, 1] = 1 * factorA;
                    S[noRows - 1, 0] = Math.Abs((springs.cellValues[i].Y * d.Y + bodyload.cellValues[i].Y) / (springs.cellValues[i].Y * l)) * factorS * factorA * Math.Sign(E[1, 0]);
                }


                //if (springs.cellValues[i].X != 0)
                //{
                //    A[noRows - 3, 0] = 1 * factorA;
                //    S[noRows - 3, 0] = Math.Abs((springs.cellValues[i].X * d.X + bodyload.cellValues[i].X) / (bond_stiffness)) * factorS * factorA * Math.Sign(E[0, 0]);
                //}

                //if (springs.cellValues[i].Y != 0)
                //{
                //    A[noRows - 2, 1] = 1 * factorA;
                //    S[noRows - 2, 0] = Math.Abs((springs.cellValues[i].Y * d.Y + bodyload.cellValues[i].Y) / (bond_stiffness)) * factorS * factorA * Math.Sign(E[1, 0]);
                //}

                AT = A.Duplicate();
                AT.Transpose();

                AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                E = (AA * AT) * S;

                strainXX.cellValues[i] = E[0, 0];
                strainYY.cellValues[i] = E[1, 0];
                strainXY.cellValues[i] = E[2, 0];
                // E[2, 0] *= 2.0;

                Matrix strainTensor = new Matrix(2, 2);
                strainTensor[0, 0] = E[0, 0];
                strainTensor[0, 1] = E[2, 0];
                strainTensor[1, 0] = E[2, 0];
                strainTensor[1, 1] = E[1, 0];

                strain.cellValues[i] = strainTensor;
            }
        }

        public void UpdateDsDx(Voxels2d<Vector2d> dispVoxels)
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                int bondCount = 0;
                double dsdx = 0;

                for (int ii = 0; ii < noBonds / 2; ii++)
                {
                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount++;

                        CalcDxAndNormal(dispVoxels, i, j, out double dx, out Vector2d n);
                        Matrix N = new Matrix(2, 1);
                        N[0, 0] = n.X;
                        N[1, 0] = n.Y;

                        Matrix Nt = N.Duplicate();
                        Nt.Transpose();

                        Matrix dsi = Nt * strain.cellValues[i] * N;
                        Matrix dsj = Nt * strain.cellValues[j] * N;

                        dsdx += Math.Abs(dsi[0, 0] - dsj[0, 0]) / dx;
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        bondCount++;

                        CalcDxAndNormal(dispVoxels, i, j, out double dx, out Vector2d n);
                        Matrix N = new Matrix(2, 1);
                        N[0, 0] = n.X;
                        N[1, 0] = n.Y;

                        Matrix Nt = N.Duplicate();
                        Nt.Transpose();

                        Matrix dsi = Nt * strain.cellValues[i] * N;
                        Matrix dsj = Nt * strain.cellValues[j] * N;

                        dsdx += Math.Abs(dsi[0, 0] - dsj[0, 0]) / dx;
                    }
                }

                strainDer.cellValues[i] = dsdx;
            }

        }

        void CalcDxAndNormal(Voxels2d<Vector2d> dispVoxels, int i, int j, out double dx, out Vector2d n)
        {
            Vector2d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            Vector2d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

            double y = xi_eta_vec.Length;

            dx = y;
            n = xi_eta_vec / y;
        }

        void CalcStretchAndNormal(Voxels2d<Vector2d> dispVoxels, int i, int j, out double s, out Vector2d n)
        {
            Vector2d xi_vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(i);
            Vector2d xi_eta_vec = dispVoxels.cellValues[j] - dispVoxels.cellValues[i] + xi_vec;

            double xi = xi_vec.Length;
            double y = xi_eta_vec.Length;

            s = (y - xi) / xi;
            n = xi_eta_vec / y;
        }

        public void UpdateStrains(Voxels2d<Vector2d> dispVoxels)
        {
            for (int i = 0; i < noVoxels; i++)
            {
                // lägg till statement om vad som händer då partiklen också hanterar Dirichlet boundary
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix F;
                Matrix A = new Matrix(noBonds * 2 + 4, 4);  // Plus 9 because of D-boundary
                A.Zero();
                Matrix b = new Matrix(noBonds * 2 + 4, 1);
                b.Zero();

                for (int ii = 0; ii < noBonds / 2; ii++)
                {
                    int j = i + nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        Matrix A_j = Calc_Aj(i, j);
                        A[ii * 2 + 0, 0] = A_j[0, 0];
                        A[ii * 2 + 0, 1] = A_j[0, 1];

                        A[ii * 2 + 1, 2] = A_j[1, 2];
                        A[ii * 2 + 1, 3] = A_j[1, 3];

                        Matrix b_j = Calc_bj(i, j, dispVoxels);
                        b[ii * 2 + 0, 0] = b_j[0, 0];
                        b[ii * 2 + 1, 0] = b_j[1, 0];
                    }

                    j = i - nList[ii];
                    if (startVoxels.cellValues[j] != 0)
                    {
                        Matrix A_j = Calc_Aj(i, j);
                        A[noBonds / 2 * 2 + ii * 2 + 0, 0] = A_j[0, 0];
                        A[noBonds / 2 * 2 + ii * 2 + 0, 1] = A_j[0, 1];

                        A[noBonds / 2 * 2 + ii * 2 + 1, 2] = A_j[1, 2];
                        A[noBonds / 2 * 2 + ii * 2 + 1, 3] = A_j[1, 3];

                        Matrix b_j = Calc_bj(i, j, dispVoxels);
                        b[noBonds / 2 * 2 + ii * 2 + 0, 0] = b_j[0, 0];
                        b[noBonds / 2 * 2 + ii * 2 + 1, 0] = b_j[1, 0];
                    }
                }

                int noRows = A.RowCount;

                // F = (A'*A)^(-1)*A'*b
                // Borde kanske introducera weightning matrix W för bättre resultat
                Matrix AT = A.Duplicate();
                AT.Transpose();

                Matrix AA = AT * A;
                AA.Invert(1e-6);        // For now chose an arbitrary tolerance

                F = (AA * AT) * b;

                double Fxx = F[0, 0];
                double Fxy = (F[1, 0] + F[2, 0]) * 0.5;
                double Fyy = F[3, 0];

                strainXX.cellValues[i] = 0.5 * (Fxx * Fxx + Fxy * Fxy - 1);
                strainYY.cellValues[i] = 0.5 * (Fxy * Fxy + Fyy * Fyy - 1);
                strainXY.cellValues[i] = 0.5 * (Fxx * Fxy + Fyy * Fxy);

            }
        }

        private Matrix Calc_Aj(int i, int j)
        {
            Matrix A_j = new Matrix(2, 4);
            A_j.Zero();
            Vector2d xi = startVoxels.IndexToPoint(i) - startVoxels.IndexToPoint(j);

            A_j[0, 0] = xi.X;
            A_j[0, 1] = xi.Y;

            A_j[1, 2] = xi.X;
            A_j[1, 3] = xi.Y;

            return A_j;
        }

        private Matrix Calc_bj(int i, int j, Voxels2d<Vector2d> dispVoxels)
        {
            Matrix b_j = new Matrix(2, 1);
            b_j.Zero();
            Vector2d eta = (dispVoxels.cellValues[i] + startVoxels.IndexToPoint(i)) - (dispVoxels.cellValues[j] + startVoxels.IndexToPoint(j));

            b_j[0, 0] = eta.X;
            b_j[1, 0] = eta.Y;

            return b_j;
        }

        public void UpdateStresses()
        {
            double C = E / (1 - nu * nu);

            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                stressXX.cellValues[i] = C * (strainXX.cellValues[i] + nu * strainYY.cellValues[i]);
                stressYY.cellValues[i] = C * (nu * strainXX.cellValues[i] + strainYY.cellValues[i]);
                stressXY.cellValues[i] = C * (1 - nu) * strainXY.cellValues[i];

                Matrix tensor = new Matrix(2, 2);
                tensor[0, 0] = stressXX.cellValues[i];
                tensor[0, 1] = stressXY.cellValues[i];
                tensor[1, 0] = stressXY.cellValues[i];
                tensor[1, 1] = stressYY.cellValues[i];

                stressTensor.cellValues[i] = tensor;
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
                double zz = 0;
                double xy = stressXY.cellValues[i];
                double xz = 0;
                double yz = 0;

                vonMises.cellValues[i] = Math.Sqrt(0.5 * ((xx - yy) * (xx - yy) + (yy - zz) * (yy - zz) + (zz - xx) * (zz - xx)) + 3 * (xy * xy + xz * xz + yz * yz));
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

                stressTensor.M10 = stressXY.cellValues[i];
                stressTensor.M11 = stressYY.cellValues[i];

                stressTensor.M22 = 1;
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

        public void UpdatePrincipalStressesPQ()
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                double sx = stressXX.cellValues[i];
                double sy = stressYY.cellValues[i];
                double txy = stressXY.cellValues[i];

                Vector3d eigenVals = new Vector3d();

                double temp = (sx - sy) * 0.5;

                eigenVals.X = (sx + sy) * 0.5 + Math.Sqrt(temp * temp + txy * txy);
                eigenVals.Y = (sx + sy) * 0.5 - Math.Sqrt(temp * temp + txy * txy);

                princpStress.cellValues[i] = eigenVals;

                double theta = 0.5 * Math.Atan((2 * txy) / (sx - sy));

                Vector3d dir1 = Vector3d.XAxis;

                dir1.Rotate(theta, Vector3d.ZAxis);

                Vector3d dir2 = Vector3d.YAxis;

                dir2.Rotate(theta, Vector3d.ZAxis);

                Vector3d[] vecList = new Vector3d[3];

                vecList[0] = dir1;
                vecList[1] = dir2;

                princpDir.cellValues[i] = vecList;

            }

        }
        public void CalcCurlOfStressField()
        {
            for (int ind = 0; ind < noVoxels; ind++)
            {
                if ((startVoxels.cellValues[ind] & maskbit) == 0)
                    continue;

                Vector3d curl = new Vector3d();

                for (int m = 0; m < 2; m++)
                {
                    curl[m] = DerStress(ind, m, 1, 0) - DerStress(ind, m, 0, 1);
                }

                curlVec.cellValues[ind] = curl;
                curlLsquared.cellValues[ind] = curl.SquareLength;
            }
        }

        private double DerStress(int ind, int a, int b, int dir)
        {
            double dS;

            int i = ind;

            stressTensor.To2DIndex(ref i, out int j);

            Vector3d ind2d = new Vector3d(i, j, 0);

            double dx = stressTensor.delta * 2;

            ind2d[dir] += -1;

            int n0 = stressTensor.ToLinearIndex((int)ind2d.X, (int)ind2d.Y);
            double dS0;
            if (startVoxels.cellValues[n0] != 0)
                dS0 = stressTensor.cellValues[n0][a, b];
            else
            {
                dS0 = stressTensor.cellValues[ind][a, b];
                dx *= 0.5;
            }

            ind2d[dir] += 2;

            int n2 = stressTensor.ToLinearIndex((int)ind2d.X, (int)ind2d.Y);
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

        public void NormalizeStressTensor()
        {
            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix tensor = stressTensor.cellValues[i];
                double sum = 0;

                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        sum += tensor[k, j] * tensor[k, j];
                    }
                }

                sum = Math.Sqrt(sum);

                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        tensor[k, j] /= sum;
                    }
                }
            }

        }
    }
}
