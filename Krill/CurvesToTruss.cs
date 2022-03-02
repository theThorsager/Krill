using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    internal static class CurvesToTruss
    {
        public static List<Polyline> Method(List<Polyline> plines, double d)
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
                Polyline res = new Polyline() { pline[0] };
                for (int i = 1; i < pline.Count - 1; i++)
                {
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
                    Vector3d move = avePt - pline[i];
                    double dist = Vector3d.Multiply(move, tan);

                    move -= dist * tan;

                    res.Add(pline[i] + move);
                }
                res.Add(pline[pline.Count - 1]);
                results.Add(res);
            }
            
            return results;
        }


    }
}
