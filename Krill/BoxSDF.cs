using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    public class BoxSDF
    {
        public Voxels<int> mask;
        public Voxels<Vector3d> SDF;

        public BoxSDF(Voxels<int> mask)
        {
            this.mask = mask;
        }
        
        public void ConstructSDF()
        {
            SDF = new Voxels<Vector3d>(mask.origin, mask.delta, mask.n);

            int N = SDF.n * SDF.n * SDF.n;
            for (int i = 0; i < N; i++)
                SDF.cellValues[i] = mask.cellValues[i] == 0 ? new Vector3d(-1, -1, -1) : -Vector3d.Unset;

            for (int k = 1; k < SDF.n - 1; k++)
            {
                for (int j = 1; j < SDF.n - 1; j++)
                {
                    for (int i = 1; i < SDF.n - 1; i++)
                    {
                        int I = SDF.ToLinearIndex(i, j, k);
                        if (SDF.cellValues[I].X == -1)
                            continue;

                        SDF.cellValues[I] = IntersectionBox(i, j, k);
                        UpdateBiggestBox(I);
                    }
                }
            }

            for (int i = 0; i < N; i++)
            {
                SDF.cellValues[i] *= SDF.delta;
            }

        }
        
        Vector3d IntersectionBox(int I, int J, int K)
        {
            Vector3d result = -Vector3d.Unset;
            Vector3d neigh;

            Func<Vector3d, Vector3d, Vector3d> Intersection = (a, b) =>
            {
                return new Vector3d(
                    a.X == -1 ? b.X : Math.Min(a.X, b.X),
                    a.Y == -1 ? b.Y : Math.Min(a.Y, b.Y),
                    a.Z == -1 ? b.Z : Math.Min(a.Z, b.Z));
            };

            neigh = SDF.cellValues[mask.ToLinearIndex(I - 1, J, K)];
            neigh.X += 2;
            result = Intersection(neigh, result);

            neigh = SDF.cellValues[mask.ToLinearIndex(I, J - 1, K)];
            neigh.Y += 2;
            result = Intersection(neigh, result);

            neigh = SDF.cellValues[mask.ToLinearIndex(I, J, K - 1)];
            neigh.Z += 2;
            result = Intersection(neigh, result);

            return result;
        }

        void UpdateBiggestBox(int I)
        {
            // kind want to work outwards until it hits something

            // There's a box centered at pt, and it grows in every step until that side hits an 

            Vector3d init = SDF.cellValues[I];

            int increaseI = 1;
            int increaseJ = 1;
            int increaseK = 1;
            int sizeI = (int)init.X;
            int sizeJ = (int)init.Y;
            int sizeK = (int)init.Z;

            // Find the smallest direction with "open" borders
            int minSize = int.MaxValue;
            // X
            int off = SDF.ToLinearIndex(sizeI / 2, 0, 0);
            if (mask.cellValues[I - off] != 0 && mask.cellValues[I + off] != 0)
            {
                minSize = Math.Min(minSize, sizeI);
            }
            // Y 
            off = SDF.ToLinearIndex(0, sizeJ / 2, 0);
            if (mask.cellValues[I - off] != 0 && mask.cellValues[I + off] != 0)
            {
                minSize = Math.Min(minSize, sizeJ);
            }
            // Z
            off = SDF.ToLinearIndex(0, 0, sizeK / 2);
            if (mask.cellValues[I - off] != 0 && mask.cellValues[I + off] != 0)
            {
                minSize = Math.Min(minSize, sizeK);
            }

            sizeI = Math.Min(minSize, sizeI);
            sizeJ = Math.Min(minSize, sizeJ);
            sizeK = Math.Min(minSize, sizeK);

            // Check each face

            for (int jj = 0; jj < 50; jj++)
            {
                if (increaseI + increaseJ + increaseK == 0)
                    break;

                sizeI += 2 * increaseI;
                sizeJ += 2 * increaseJ;
                sizeK += 2 * increaseK;

                if (increaseI == 1)
                {
                    for (int i = 0; i < sizeI; i += (sizeI - 1))
                    {
                        for (int j = 1; j < sizeJ - 1; j++)
                        {
                            for (int k = 1; k < sizeK - 1; k++)
                            {
                                int J = mask.ToLinearIndex(i - sizeI / 2, j - sizeJ / 2, k - sizeK / 2);
                                int ii = I + J;

                                if (mask.cellValues[ii] == 0)
                                {
                                    // not part of the model
                                    increaseI = 0;
                                    goto endloopI;
                                }
                            }
                        }
                    }
                }
            endloopI:
                if (increaseJ == 1)
                {
                    for (int i = 1; i < sizeI - 1; i++)
                    {
                        for (int j = 0; j < sizeJ; j += (sizeJ - 1))
                        {
                            for (int k = 1; k < sizeK - 1; k++)
                            {
                                int J = mask.ToLinearIndex(i - sizeI / 2, j - sizeJ / 2, k - sizeK / 2);
                                int ii = I + J;

                                if (mask.cellValues[ii] == 0)
                                {
                                    // not part of the model
                                    increaseJ = 0;
                                    goto endloopJ;
                                }
                            }
                        }
                    }
                }
            endloopJ:
                if (increaseK == 1)
                {
                    for (int i = 1; i < sizeI - 1; i++)
                    {
                        for (int j = 1; j < sizeJ - 1; j++)
                        {
                            for (int k = 0; k < sizeK; k += (sizeK - 1))
                            {
                                int J = mask.ToLinearIndex(i - sizeI / 2, j - sizeJ / 2, k - sizeK / 2);
                                int ii = I + J;

                                if (mask.cellValues[ii] == 0)
                                {
                                    // not part of the model
                                    increaseK = 0;
                                    goto endloopK;
                                }
                            }
                        }
                    }
                }
            endloopK:;
            }

            SDF.cellValues[I] = new Vector3d((sizeI - 2), (sizeJ - 2), (sizeK - 2));
        }

        public Vector3d BoxValueAt(Point3d pt)
        {
            // Use lower bound and that the gradient is +-2
            if (!SDF.InsideLerp(pt))
                return Vector3d.Zero;

            pt -= (Vector3d)SDF.origin;
            pt /= SDF.delta;

            double u = pt.X;
            double v = pt.Y;
            double w = pt.Z;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int k0 = (int)Math.Floor(w - 0.5);
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            int INDi0j0k0 = SDF.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = SDF.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = SDF.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = SDF.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = SDF.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = SDF.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = SDF.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = SDF.ToLinearIndex(i1, j1, k1);

            var vecs = new List<Vector3d>()
            {
                SDF.cellValues[INDi0j0k0],
                SDF.cellValues[INDi1j0k0],
                SDF.cellValues[INDi0j1k0],
                SDF.cellValues[INDi1j1k0],
                SDF.cellValues[INDi0j0k1],
                SDF.cellValues[INDi1j0k1],
                SDF.cellValues[INDi0j1k1],
                SDF.cellValues[INDi1j1k1]
            };

            double minX = double.MaxValue;
            double minY = double.MaxValue;
            double minZ = double.MaxValue;

            for (int i = 0; i < 8; i++)
            {
                minX = minX < vecs[i].X ? minX : vecs[i].X == -SDF.delta ? minX : vecs[i].X;
                minY = minY < vecs[i].Y ? minY : vecs[i].Y == -SDF.delta ? minY : vecs[i].Y;
                minZ = minZ < vecs[i].Z ? minZ : vecs[i].Z == -SDF.delta ? minZ : vecs[i].Z;
            }

            double maxX = minX + 2 * SDF.delta;
            double maxY = minY + 2 * SDF.delta;
            double maxZ = minZ + 2 * SDF.delta;

            for (int i = 0; i < 8; i++)
            {
                var vec = vecs[i];
                vec.X = maxX < vec.X ? maxX : vec.X;
                vec.Y = maxY < vec.Y ? maxY : vec.Y;
                vec.Z = maxZ < vec.Z ? maxZ : vec.Z;
                vecs[i] = vec;
            }

            return (1 - a) * (1 - b) * (1 - c) * vecs[0]
                    + a * (1 - b) * (1 - c) * vecs[1]
                    + (1 - a) * b * (1 - c) * vecs[2]
                    + a * b * (1 - c) * vecs[3]
                    + (1 - a) * (1 - b) * c * vecs[4]
                    + a * (1 - b) * c * vecs[5]
                    + (1 - a) * b * c * vecs[6]
                    + a * b * c * vecs[7];
        }

        public Vector3d BoxGradientAt(Point3d pt, double cover, double gamma)
        {
            // Ignores second order effects i.e. dF.x/dy

            // Use lower bound and that the gradient is +-2
            cover *= 2;
            pt -= (Vector3d)SDF.origin;
            pt /= SDF.delta;

            double u = pt.X;
            double v = pt.Y;
            double w = pt.Z;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int k0 = (int)Math.Floor(w - 0.5);
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            int INDi0j0k0 = SDF.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = SDF.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = SDF.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = SDF.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = SDF.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = SDF.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = SDF.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = SDF.ToLinearIndex(i1, j1, k1);

            var vecs = new List<Vector3d>()
            {
                SDF.cellValues[INDi0j0k0],
                SDF.cellValues[INDi1j0k0],
                SDF.cellValues[INDi0j1k0],
                SDF.cellValues[INDi1j1k0],
                SDF.cellValues[INDi0j0k1],
                SDF.cellValues[INDi1j0k1],
                SDF.cellValues[INDi0j1k1],
                SDF.cellValues[INDi1j1k1]
            };

            double minX = double.MaxValue;
            double minY = double.MaxValue;
            double minZ = double.MaxValue;

            for (int i = 0; i < 8; i++)
            {
                minX = minX < vecs[i].X ? minX : vecs[i].X == -SDF.delta ? minX : vecs[i].X;
                minY = minY < vecs[i].Y ? minY : vecs[i].Y == -SDF.delta ? minY : vecs[i].Y;
                minZ = minZ < vecs[i].Z ? minZ : vecs[i].Z == -SDF.delta ? minZ : vecs[i].Z;
            }

            double maxX = minX + 2 * SDF.delta;
            double maxY = minY + 2 * SDF.delta;
            double maxZ = minZ + 2 * SDF.delta;

            for (int i = 0; i < 8; i++)
            {
                var vec = vecs[i];
                vec.X = maxX < vec.X ? maxX : vec.X;
                vec.Y = maxY < vec.Y ? maxY : vec.Y;
                vec.Z = maxZ < vec.Z ? maxZ : vec.Z;
                vecs[i] = vec;
            }
            var value = (1 - a) * (1 - b) * (1 - c) * vecs[0]
                    + a * (1 - b) * (1 - c) * vecs[1]
                    + (1 - a) * b * (1 - c) * vecs[2]
                    + a * b * (1 - c) * vecs[3]
                    + (1 - a) * (1 - b) * c * vecs[4]
                    + a * (1 - b) * c * vecs[5]
                    + (1 - a) * b * c * vecs[6]
                    + a * b * c * vecs[7];
        
            double dx = value.X > cover ? 0.0 :
                - (1 - b) * (1 - c) * vecs[0].X
                    + (1 - b) * (1 - c) * vecs[1].X
                    - b * (1 - c) * vecs[2].X
                    + b * (1 - c) * vecs[3].X
                    - (1 - b) * c * vecs[4].X
                    + (1 - b) * c * vecs[5].X
                    - b * c * vecs[6].X
                    + b * c * vecs[7].X;

            double dy = value.Y > cover ? 0.0 :
                 -(1 - a) * (1 - c) * vecs[0].Y
                    - a * (1 - c) * vecs[1].Y
                    + (1 - a) * (1 - c) * vecs[2].Y
                    + a * (1 - c) * vecs[3].Y
                    - (1 - a) * c * vecs[4].Y
                    - a * c * vecs[5].Y
                    + (1 - a) * c * vecs[6].Y
                    + a * c * vecs[7].Y;

            double dz = value.Z > cover ? 0.0 :
                -(1 - a) * (1 - b) * vecs[0].Z
                    - a * (1 - b) * vecs[1].Z
                    - (1 - a) * b * vecs[2].Z
                    - a * b * vecs[3].Z
                    + (1 - a) * (1 - b) * c * vecs[4].Z
                    + a * (1 - b) * vecs[5].Z
                    + (1 - a) * b * vecs[6].Z
                    + a * b * vecs[7].Z;

            return new Vector3d(
                Math.Sign(dx) * (cover - value.X * 0.5) * 2 * gamma,
                Math.Sign(dy) * (cover - value.Y * 0.5) * 2 * gamma,
                Math.Sign(dz) * (cover - value.Z * 0.5) * 2 * gamma);
        }

        public void BoxGradientAt(Point3d pt, out Vector3d dx, out Vector3d dy, out Vector3d dz)
        {
            // Use lower bound and that the gradient is +-2
            pt -= (Vector3d)SDF.origin;
            pt /= SDF.delta;

            double u = pt.X;
            double v = pt.Y;
            double w = pt.Z;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int k0 = (int)Math.Floor(w - 0.5);
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            int INDi0j0k0 = SDF.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = SDF.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = SDF.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = SDF.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = SDF.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = SDF.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = SDF.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = SDF.ToLinearIndex(i1, j1, k1);

            var vecs = new List<Vector3d>()
            {
                SDF.cellValues[INDi0j0k0],
                SDF.cellValues[INDi1j0k0],
                SDF.cellValues[INDi0j1k0],
                SDF.cellValues[INDi1j1k0],
                SDF.cellValues[INDi0j0k1],
                SDF.cellValues[INDi1j0k1],
                SDF.cellValues[INDi0j1k1],
                SDF.cellValues[INDi1j1k1]
            };

            double minX = double.MaxValue;
            double minY = double.MaxValue;
            double minZ = double.MaxValue;

            for (int i = 0; i < 8; i++)
            {
                minX = minX < vecs[i].X ? minX : vecs[i].X == -SDF.delta ? minX : vecs[i].X;
                minY = minY < vecs[i].Y ? minY : vecs[i].Y == -SDF.delta ? minY : vecs[i].Y;
                minZ = minZ < vecs[i].Z ? minZ : vecs[i].Z == -SDF.delta ? minZ : vecs[i].Z;
            }

            double maxX = minX + 2 * SDF.delta;
            double maxY = minY + 2 * SDF.delta;
            double maxZ = minZ + 2 * SDF.delta;

            for (int i = 0; i < 8; i++)
            {
                var vec = vecs[i];
                vec.X = maxX < vec.X ? maxX : vec.X;
                vec.Y = maxY < vec.Y ? maxY : vec.Y;
                vec.Z = maxZ < vec.Z ? maxZ : vec.Z;
                vecs[i] = vec;
            }
            //var value = (1 - a) * (1 - b) * (1 - c) * vecs[0]
            //        + a * (1 - b) * (1 - c) * vecs[1]
            //        + (1 - a) * b * (1 - c) * vecs[2]
            //        + a * b * (1 - c) * vecs[3]
            //        + (1 - a) * (1 - b) * c * vecs[4]
            //        + a * (1 - b) * c * vecs[5]
            //        + (1 - a) * b * c * vecs[6]
            //        + a * b * c * vecs[7];

            dx = -(1 - b) * (1 - c) * vecs[0]
                    + (1 - b) * (1 - c) * vecs[1]
                    - b * (1 - c) * vecs[2]
                    + b * (1 - c) * vecs[3]
                    - (1 - b) * c * vecs[4]
                    + (1 - b) * c * vecs[5]
                    - b * c * vecs[6]
                    + b * c * vecs[7];

            dy = -(1 - a) * (1 - c) * vecs[0]
                    - a * (1 - c) * vecs[1]
                    + (1 - a) * (1 - c) * vecs[2]
                    + a * (1 - c) * vecs[3]
                    - (1 - a) * c * vecs[4]
                    - a * c * vecs[5]
                    + (1 - a) * c * vecs[6]
                    + a * c * vecs[7];

            dz = -(1 - a) * (1 - b) * vecs[0]
                    - a * (1 - b) * vecs[1]
                    - (1 - a) * b * vecs[2]
                    - a * b * vecs[3]
                    + (1 - a) * (1 - b) * vecs[4]
                    + a * (1 - b) * vecs[5]
                    + (1 - a) * b * vecs[6]
                    + a * b * vecs[7];

            dx /= SDF.delta;
            dy /= SDF.delta;
            dz /= SDF.delta;
        }

    }
}
