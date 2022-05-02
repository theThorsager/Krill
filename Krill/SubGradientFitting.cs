using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class SubGradientFitting
    {
        public List<Point3d> points;
        public List<Point3d> Notpoints;
        public Voxels<double> weights;

        public VoxelSubgradient subgradient;

        public List<Vector3d> dirs;

        public SubGradientFitting(Voxels<double> weights, List<Point3d> pts)
        {
            subgradient = new VoxelSubgradient(weights);
            this.weights = weights;
            points = pts;
            Notpoints = new List<Point3d>(pts);

            dirs = new Vector3d[pts.Count].ToList();
        }

        bool inside(int i)
        {
            int n = weights.n;
            weights.To3DIndex(ref i, out int j, out int k);
            return i >= 1 && i < n - 2 &&
                   j >= 1 && j < n - 2 &&
                   k >= 1 && k < n - 2;
        }

        //bool inside(int i)
        //{
        //    int n = weights.n;
        //    weights.To3DIndex(ref i, out int j, out int k);
        //    return i >= 0 && i < n &&
        //           j >= 0 && j < n &&
        //           k >= 0 && k < n;
        //}

        
        public void Iterate()
        {
            double length = 0;
            for (int i = 1; i < points.Count; i++)
            {
                length += points[i].DistanceTo(points[i - 1]);
            }
            length /= (points.Count - 1);
            length *= 0.75; // Make the string always stretched

            for (int a = 1; a < points.Count - 1; ++a)
            {
                int i = weights.PointToIndex(points[a]);
                if (!inside(i))
                    continue;

                // Direction of decent
                Vector3d force = subgradient.SubGradientAt(points[a]);
                force *= 0.5;

                // project
                Vector3d normal = points[a + 1] - points[a - 1];
                normal.Unitize();
                double dist = force * normal;

                force -= dist * normal;

                // dont go further than two delta
                double factor = (weights.delta * weights.delta * 4) / force.SquareLength;
                if (!double.IsNaN(factor) && factor < 1)
                    force *= factor;

                dirs[a] = force; // for debugging

                // string ness
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
                
                Notpoints[a] = points[a] + force * 0.1;
            }

            {
                var temp = points;
                points = Notpoints;
                Notpoints = temp;
            }

        }
    }
}
