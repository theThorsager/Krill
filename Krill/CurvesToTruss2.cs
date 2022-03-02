using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    internal class CurvesToTruss2
    {
        public Voxels<double> weights;

        public VoxelSubgradient subgradient;

        public CurvesToTruss2(Voxels<double> weights)
        {
            this.weights = weights;
            subgradient = new VoxelSubgradient(weights);
        }

        bool inside(int i)
        {
            int n = weights.n;
            weights.To3DIndex(ref i, out int j, out int k);
            return i >= 1 && i < n - 2 &&
                   j >= 1 && j < n - 2 &&
                   k >= 1 && k < n - 2;
        }

        public List<Polyline> Method(List<Polyline> plines, double d)
        {
            double sqd = d * d;
            List<Point3d> allPoints = plines.SelectMany(pline => pline.ToList()).ToList();

            // todo, if this all works
            //Rhino.Geometry.RTree tree = new RTree();
            //allPoints.ForEach(pt => tree.Insert(pt, 0));

            List<Polyline> results = new List<Polyline>();
            // for every point
            // take the average point of nearby points
            // this is new curve :)
            foreach (var pline in plines)
            {
                double length = 0;
                for (int i = 1; i < pline.Count; i++)
                {
                    length += pline[i].DistanceTo(pline[i - 1]);
                }
                length /= (pline.Count - 1);
                length *= 0.75; // Make the string always stretched


                Polyline res = new Polyline() { pline[0] };
                for (int i = 1; i < pline.Count - 1; i++)
                {
                    int ii = weights.PointToIndex(pline[i]);
                    if (!inside(ii))
                    {
                        res.Add(pline[i]);
                        continue;
                    }

                    // Direction of decent
                    Vector3d force = subgradient.SubGradientAt(pline[i]);
                    force *= 0.5;


                    int count = 0;
                    Point3d avePt = new Point3d();
                    foreach (var opt in allPoints)
                    {
                        if (pline[i].DistanceToSquared(opt) < sqd)
                        {
                            count++;
                            avePt += opt;
                        }
                    }
                    avePt /= count;
                    // project to tangent plane
                    Vector3d tan = pline[i + 1] - pline[i - 1];
                    tan.Unitize();
                    Vector3d move = (avePt - pline[i]) * 2.0 + force;
                    double dist = Vector3d.Multiply(move, tan);

                    move -= dist * tan;

                    // dont go further than two delta
                    double factor = (weights.delta * weights.delta * 4) / move.SquareLength;
                    if (!double.IsNaN(factor) && factor < 1)
                        move *= factor;


                    // string ness
                    Vector3d dir1 = pline[i + 1] - pline[i];
                    double l1 = dir1.Length;
                    dir1 /= l1;
                    double s1 = (l1 - length) / length;
                    move += s1 * dir1;

                    Vector3d dir2 = pline[i - 1] - pline[i];
                    double l2 = dir2.Length;
                    dir2 /= l2;
                    double s2 = (l2 - length) / length;
                    move += s2 * dir2;


                    res.Add(pline[i] + move * 0.1);
                }
                res.Add(pline[pline.Count - 1]);
                results.Add(res);
            }
            
            return results;
        }


    }
}
