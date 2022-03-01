using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class Kmean
    {
        public List<Point3d> points;
        public Voxels<double> weights;
        public int[] offsets;

        public Kmean(Voxels<double> weights, double r)
        {
            this.weights = weights;
            offsets = Utility.GetNeighbourOffsets(weights.n, r);
        }

        bool inside(int i)
        {
            int n = weights.n;
            weights.To3DIndex(ref i, out int j, out int k);
            return i >= 0 && i < n &&
                   j >= 0 && j < n &&
                   k >= 0 && k < n;
        }

        public void Iterate()
        {
            for (int a = 0; a < points.Count; ++a)
            {
                int i = weights.PointToIndex(points[a]);
                if (!inside(i))
                    continue;

                Point3d average = weights.IndexToPoint(i) * weights.cellValues[i];
                double sum = weights.cellValues[i];
                for (int jj = 0; jj < offsets.Length; jj++)
                {
                    int j = i + offsets[jj];
                    if (inside(j))
                    {
                        average += weights.IndexToPoint(j) * weights.cellValues[j];
                        sum += weights.cellValues[j];
                    }

                    j = i - offsets[jj];
                    if (inside(j))
                    {
                        average += weights.IndexToPoint(j) * weights.cellValues[j];
                        sum += weights.cellValues[j];
                    }
                }
                points[a] = average / sum;
            }
        }
    }
}
