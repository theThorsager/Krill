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

            return result.Where(x => x > 0).ToHashSet().ToArray();
        }

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

            double n = 1 / pts.Count;
            var res = disp.Zip(trueDisp, 
                (x, y) =>
                {
                    if (!y.IsValid)
                        return 0;
                    return Math.Pow((x - y).Length / lengthScale, power) * n;
                }).Sum();

            return Math.Pow(res, 1.0 / (double)power);
        }
    }
}
