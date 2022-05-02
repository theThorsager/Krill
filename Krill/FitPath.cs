using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class FitPath
    {
        public List<Point3d> points;
        public List<Point3d> Notpoints;
        public Voxels<double> weights;
        public int[] offsets;
        public double length;

        public List<Vector3d> dirs;

        public FitPath(Voxels<double> weights, double r)
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
            length = 0;
            for (int i = 1; i < points.Count; i++)
            {
                length += points[i].DistanceTo(points[i - 1]);
            }
            length /= (points.Count - 1);
            length *= 0.95;

            for (int a = 1; a < points.Count - 1; ++a)
            {
                int i = weights.PointToIndex(points[a]);
                if (!inside(i))
                    continue;

                // Direction of decent
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

                average /= sum;
                
                Vector3d force = average - points[a];
                force *= 0.5;

                // project
                Vector3d normal = points[a + 1] - points[a - 1];
                normal.Unitize();
                double dist = force * normal;

                force -= dist * normal;

                Vector3d dir1 = points[a + 1] - points[a];
                double l1 = dir1.Length;
                dir1 /= l1;
                double s1 = (l1 - length) / length;
                force += s1 * dir1;

                Vector3d dir2 = points[a - 1] - points[a];
                double l2 = dir2.Length;
                dir2 /= l2;
                double s2 = (l2 - length) / length;
                force += s2 * dir2;

                Notpoints[a] = points[a] + force * 0.05;
                dirs[a] = force;
            }

            {
                var temp = points;
                points = Notpoints;
                Notpoints = temp;
            }

        }
    }
}
