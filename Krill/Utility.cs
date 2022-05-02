using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Krill
{
    internal class Utility
    {
        /// <summary>
        /// Gets an array of the positive offsets for neighbour cells in a 3d array of size nxnxn
        /// </summary>
        /// <param name="n">Number of cells per direction in a nxnxn array</param>
        /// <param name="r">Number of neighbours wide sphere to consider as neighbours</param>
        /// <returns>Offset array for neighbours</returns>
        public static int[] GetNeighbourOffsets(int n, double r)
        {
            int rsquared = (int)(r * r);
            int rfloor = (int)r;

            List<int> result = new List<int>();

            for (int k = 0; k <= rfloor; k++)
            {
                for (int j = 0; j <= rfloor; j++)
                {
                    for (int i = 0; i <= rfloor; i++)
                    {
                        if (i * i + j * j + k * k <= rsquared)
                        {
                            result.Add(i + n * j + n * n * k);
                            result.Add(i - n * j + n * n * k);
                            result.Add(i - n * j - n * n * k);
                            result.Add(i + n * j - n * n * k);
                            result.Add(-i + n * j + n * n * k);
                            result.Add(-i - n * j + n * n * k);
                            result.Add(-i - n * j - n * n * k);
                            result.Add(-i + n * j - n * n * k);
                        }
                    }
                }
            }

            return result.Where(x => x > 0).ToHashSet().OrderBy(x => x).ToArray();
        }

        public static int[] GetNeighbourOffsets2d(int n, double r)
        {
            int rsquared = (int)(r * r);
            int rfloor = (int)r;

            List<int> result = new List<int>();

            for (int j = 0; j <= rfloor; j++)
            {
                for (int i = 0; i <= rfloor; i++)
                {
                    if (i * i + j * j <= rsquared)
                    {
                        result.Add(i + n * j);
                        result.Add(i - n * j);
                        result.Add(-i + n * j);
                        result.Add(-i - n * j);
                    }
                }
            }

            return result.Where(x => x > 0).ToHashSet().OrderBy(x => x).ToArray();
        }

        public static double[] getKernelWeights(int n, double r, int[] nlist)
        {
            int rfloor = (int)r;

            int p = rfloor + 1;
            double[] result = new double[nlist.Length];

            for (int ii = 0; ii < nlist.Length; ii++)
            {
                int lin_i = p + n*p + nlist[ii];
                int k = lin_i / (n * n);
                int j = (lin_i - k * n * n) / n;
                int i = lin_i - k * n * n - j * n;
                double val = Math.Sqrt(k * k + (j-p) * (j - p) + (i - p) * (i - p)) / r;
                result[ii] = -dkernel(val);
            }

            return result;
        }

        private static double dkernel(double x)
        {
            if (x > 1)
                return 0;
            return x * (x * x - 1);
        }
        //private static double kernel(double x1)
        //{
        //    double x = Math.Abs(x1);
        //    if (x > 2)
        //        return 0;
        //    else if (x > 1)
        //    {
        //        double temp = 2 - x;
        //        return (1.0 / 6.0) * temp * temp * temp;
        //    }
        //    else
        //        return 2.0 / 3.0 - x * x * (1 - x / 2.0);
        //}
        //private static double dkernel(double x1)
        //{
        //    double x = Math.Abs(x1);
        //    if (x > 2)
        //        return 0;
        //    else if (x > 1)
        //    {
        //        double temp = 2 - x;
        //        return -0.5 * temp * temp;
        //    }
        //    else
        //        return 3.0 * x * x / 2.0 - 2.0 * x;
        //}

        const double tol = 1e-6;
        public static double NotZero(double val)
        {
            return val < tol ? val > -tol ? tol : val : val;
        }

        public static void SetValuesOutsideBBox(Voxels<int> mask, Voxels<Vector3d> disp, BoundingBox bbox, int val, Func<Point3d, Vector3d> func)
        {
            int n = mask.n;
            for (int i = 0; i < n * n * n; i++)
            {
                Point3d pt = mask.IndexToPoint(i);
                if (!bbox.Contains(pt))
                {
                    Vector3d u = func(pt);
                    if (u.IsValid)
                    {
                        disp.cellValues[i] = u;
                        mask.cellValues[i] = val;
                    }
                }
            }
        }

        public static double ErrorNorm(
            List<Point3d> pts, 
            List<Vector3d> disp, 
            double lengthScale, 
            int power, 
            Func<Point3d, Vector3d> func)
        {
            List<Vector3d> trueDisp = pts.Select(x => func(x)).ToList();

            double n = 1.0 / (double)pts.Count;
            var res = disp.Zip(trueDisp, 
                (x, y) =>
                {
                    if (!y.IsValid)
                        return 0;
                    return Math.Pow((x - y).Length / lengthScale, power) * n;
                }).Sum();

            return Math.Pow(res, 1.0 / (double)power);
        }



        public static void DoComm<T>(Dictionary<Coord, Voxels<T>> voxels, int p)
        {
            foreach (KeyValuePair<Coord, Voxels<T>> pair in voxels)
            {
                Coord current = pair.Key;
                Voxels<T> voxel = pair.Value;
                int n = voxel.n;

                Coord offset;
                // Faces
                offset = new Coord(1, 0, 0);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, 0, 0);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, 1, 0);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, -1, 0);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, 0, 1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, 0, -1);
                commdir(voxels, voxel, current, offset, n, p);
                // Edges
                offset = new Coord(1, 1, 0);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(1, -1, 0);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, 1, 0);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, -1, 0);
                commdir(voxels, voxel, current, offset, n, p);

                offset = new Coord(1, 0, 1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, 1, 1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, -1, 1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, 0, 1);
                commdir(voxels, voxel, current, offset, n, p);

                offset = new Coord(1, 0, -1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, 1, -1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(0, -1, -1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, 0, -1);
                commdir(voxels, voxel, current, offset, n, p);
                // Corners
                offset = new Coord(1, 1, 1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, 1, 1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, -1, 1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(1, -1, 1);
                commdir(voxels, voxel, current, offset, n, p);

                offset = new Coord(1, 1, -1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, 1, -1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(-1, -1, -1);
                commdir(voxels, voxel, current, offset, n, p);
                offset = new Coord(1, -1, -1);
                commdir(voxels, voxel, current, offset, n, p);
            }
        }

        private static void commdir<T>(Dictionary<Coord, Voxels<T>> voxels, Voxels<T> voxel, Coord current, Coord offset, int n, int p)
        {
            Func<int, int> min = ii => ii == 0 ? p : ii == 1 ? n - p : 0;
            Func<int, int> max = ii => ii == 0 ? n - p : ii == 1 ? n : p;
            Func<int, int> off = ii => ii == 0 ? 0 : ii == 1 ? -n + p * 2 : n - p * 2;

            Coord other = current;
            other.X += offset.X;
            other.Y += offset.Y;
            other.Z += offset.Z;

            if (voxels.TryGetValue(other, out Voxels<T> voxel2))
            {
                voxel.commRel(voxel2,
                    new Coord(min(offset.X), min(offset.Y), min(offset.Z)),
                    new Coord(max(offset.X), max(offset.Y), max(offset.Z)),
                    voxel.ToLinearIndex(off(offset.X), off(offset.Y), off(offset.Z)));
            }
        }

    }
}
