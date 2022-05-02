using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    public static class DualCurve
    {
        private static List<double> SmoothData(List<double> list, int range)
        {
            // Average value within range

            var result = new List<double>();
            
            for (int i = 0; i < list.Count; i++)
            {
                int count = 0;
                double sum = 0;
                for (int jj = -range; jj <= range; jj++)
                {
                    int j = i + jj;
                    if (j < 0 || j >= list.Count)
                        continue;

                    count++;
                    sum += list[j];
                }
                result.Add(sum / count);
            }

            return result;
        }

        public static Polyline ReduceCurve(this Polyline pline, double tol)
        {
            List<Point3d> pts = pline.ToList();
            var angles = new List<double>();
            for (int i = 1; i < pts.Count - 1; i++)
            {
                double angle = Vector3d.VectorAngle(pts[i - 1] - pts[i], pts[i] - pts[i + 1]);
                angles.Add(angle);
            }

            for (int i = pts.Count - 2; i >= 1; --i)
            {
                if (angles[i - 1] < tol)
                {
                    pts.RemoveAt(i);
                }
            }
            return new Polyline(pts);
        }

        public static void ReduceToMinimumCurvature(Polyline pline, out List<Point3d> points, out List<Vector3d> tangents)
        {
            // Equal polyline segment length
            // compare different angles
            List<Point3d> pts = pline.ToList();
            points = new List<Point3d>();
            tangents = new List<Vector3d>();
            var angles = new List<double>() { 0 };
            for (int i = 1; i < pts.Count - 1; i++)
            {
                double angle = Vector3d.VectorAngle(pts[i - 1] - pts[i], pts[i] - pts[i + 1]);
                angles.Add(angle);
            }
            angles.Add(0);

            for (int i = 1; i < 30; ++i)
                angles = SmoothData(angles, 1);
            
            angles[0] = 0.0;

            for (int i = 1; i < pts.Count - 1; i++)
            {
                double angle = angles[i];
                angles.Add(angle);

                if (angle > angles[i-1] && angles[i-1] <= angles[Math.Max(i - 2, 0)])
                {
                    points.Add(pts[i - 1]);
                    Vector3d tangent = pts[Math.Max(i - 2, 0)] - pts[i];
                    tangents.Add(tangent);
                }
            }
            points.Add(pts[pts.Count - 1]);
            Vector3d temp = pts[pts.Count - 2] - pts[pts.Count - 1];
            tangents.Add(temp);

            //double oldoldangle = 0;
            //double oldangle = 0;
            //for (int i = 1; i < pts.Count - 1; i++)
            //{
            //    double angle = Vector3d.VectorAngle(pts[i - 1] - pts[i], pts[i] - pts[i + 1]);
            //    angles.Add(angle);

            //    if (angle > oldangle && oldangle <= oldoldangle)
            //    {
            //        points.Add(pts[i - 1]);
            //        Vector3d tangent = pts[Math.Max(i - 2, 0)] - pts[i];
            //        tangent.Unitize();
            //        tangents.Add(tangent);
            //    }
            //    oldoldangle = oldangle;
            //    oldangle = angle;
            //}
            //points.Add(pts[pts.Count - 1]);
            //Vector3d temp = pts[pts.Count - 2] - pts[pts.Count - 1];
            //temp.Unitize();
            //tangents.Add(temp);
        }

        public static Polyline ConstructDualCurve(List<Point3d> points, List<Vector3d> tangents)
        { 
            List<Point3d> dual = new List<Point3d>();
            dual.Add(points[0]);
            Line lineA = new Line(points[0], tangents[0]);
            Line lineB = new Line(points[1], tangents[1]);
            Rhino.Geometry.Intersect.Intersection.LineLine(lineA, lineB, out double a, out double b);
            dual.Add(lineA.PointAt(a));

            for (int i = 1; i < points.Count - 2; i++)
            {
                lineA = new Line(points[i], tangents[i]);
                lineB = new Line(points[i + 1], tangents[i + 1]);
                Rhino.Geometry.Intersect.Intersection.LineLine(lineA, lineB, out a, out b);
                dual.Add((lineA.PointAt(a) + lineB.PointAt(b)) * 0.5);
            }

            int j = points.Count - 2;
            lineA = new Line(points[j], tangents[j]);
            lineB = new Line(points[j + 1], tangents[j + 1]);
            Rhino.Geometry.Intersect.Intersection.LineLine(lineA, lineB, out a, out b);
            dual.Add(lineB.PointAt(b));
            dual.Add(points[j + 1]);
            return new Polyline(dual);
        }
    }
}
