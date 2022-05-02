using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class PathToTruss
    {
        const int maskbit = 0x000000FF;

        public Polyline loadPath;
        public Polyline trussElement;
        private double tolerance;

        public PathToTruss(Polyline loadPath, double tolerance)
        {
            this.loadPath = loadPath;
            this.tolerance = tolerance;
        }

        public void ConstructStrutTruss()
        {
            // Begin from one end, find the first direction and move along it until the distance to the lPath is too large

            Vector3d startDir = loadPath[1] - loadPath[0];

            double step = startDir.Length;
            int noPts = loadPath.Count;

            startDir.Unitize();
            Point3d node1 = loadPath[0];

            for (int i = 0; i < noPts; i++)
            {
                node1 += startDir * step;

                double t = loadPath.ClosestParameter(node1);

                if (node1.DistanceTo(loadPath.PointAt(t)) > tolerance)
                    break;
            }

            // For now assume two points along the path is sufficient
            Vector3d endDir = loadPath[noPts - 2] - loadPath[noPts - 1];
            endDir.Unitize();
            Point3d node2 = loadPath[noPts - 1];

            for (int i = 0; i < noPts; i++)
            {
                node2 += endDir * step;

                double t = loadPath.ClosestParameter(node2);

                if (node2.DistanceTo(loadPath.PointAt(t)) > tolerance)
                    break;
            }

            List<Point3d> nodes = new List<Point3d>()
            {
                node1,
                node2
            };

            for (int i = 0; i < 15; i++)
            {
                int n = nodes.Count;
                if (MoreNodesReq(nodes[n - 2], nodes[n - 1], out Point3d node12))
                    nodes.Insert(n - 1, node12);
                else
                    break;
            }

            List<Point3d> allNodes = new List<Point3d>();
            allNodes.Add(loadPath[0]);
            allNodes.AddRange(nodes);
            allNodes.Add(loadPath[(noPts - 1)]);

            trussElement = new Polyline(allNodes);
        }

        private bool MoreNodesReq(Point3d n1, Point3d n2, out Point3d n12)
        {
            bool test = false;
            n12 = new Point3d();

            Line midSeg = new Line(n1, n2);
            Line[] allSeg = loadPath.GetSegments();

            int i1 = (int)Math.Floor(loadPath.ClosestParameter(n1));
            int i2 = (int)Math.Floor(loadPath.ClosestParameter(n2));

            double maxDist = double.MinValue;
            int maxInd = int.MaxValue;

            for (int i = (i1 + 1); i < i2; i++)
            {
                double dist = midSeg.MinimumDistanceTo(allSeg[i]);
                if (dist > tolerance)
                {
                    if (maxDist < dist)
                    {
                        maxDist = dist;
                        maxInd = i;
                    }
                }
            }

            if (maxInd < int.MaxValue)  // Kanske lite fult villkor men borde fungera fint
            {
                n12 = allSeg[maxInd].PointAtLength(allSeg[maxInd].Length * 0.5);
                double factor = 2.5;

                // Har nu ett avstånd runt varje nod där det inte kan skapas någon ny
                if (n1.DistanceTo(n12) > tolerance * factor && n2.DistanceTo(n12) > tolerance * factor)
                    test = true;
            }
            
            return test;
        } 
    }
}
