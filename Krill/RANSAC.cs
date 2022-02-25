using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class RANSAC
    {
        public Line DoRANSAC(List<Point3d> data, int n, int k, double tol, int d)
        {
            Line bestFit = new Line();

            double bestErr = double.MaxValue;

            Random random = new Random();

            for (int i = 0; i < k; i++)
            {
                List<Point3d> maybeInliers = new List<Point3d>();
                List<Point3d> ptNOTinMaybeInliers = new List<Point3d>(data);
                
                for (int j = 0; j < n; j++)
                {
                    int ind = random.Next(data.Count - 1);
                    maybeInliers.Add(data[ind]);
                    ptNOTinMaybeInliers.Remove(data[ind]);
                }

                if (!Line.TryFitLineToPoints(maybeInliers, out Line maybeModel))
                    continue;

                List<Point3d> alsoInliers = new List<Point3d>();

                for (int j = 0; j < ptNOTinMaybeInliers.Count; j++)
                {
                    if (maybeModel.MinimumDistanceTo(ptNOTinMaybeInliers[j]) < tol)
                        alsoInliers.Add(ptNOTinMaybeInliers[j]);
                }

                if (alsoInliers.Count > d)
                {
                    alsoInliers.AddRange(maybeInliers);
                    Line.TryFitLineToPoints(alsoInliers, out Line betterModel);

                    double thisErr = 0;
                    for (int j = 0; j < alsoInliers.Count; j++)
                    {
                        thisErr += betterModel.MinimumDistanceTo(alsoInliers[j]);
                    }

                    if (thisErr < bestErr)
                    {
                        bestFit = betterModel;
                        bestErr = thisErr;
                    }
                }
            }
            return bestFit;
        }
    }
}
