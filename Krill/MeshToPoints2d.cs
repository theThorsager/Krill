using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    class MeshToPoints2d
    {
        Brep mesh;
        int index = 0;

        public Voxels2d<int> voxels;

        public List<Point3d> points = new List<Point3d> ();     // For debugging, remove

        public MeshToPoints2d(Brep mesh, double Delta, double delta)
        {
            this.mesh = mesh;

            BoundingBox bbox = mesh.GetBoundingBox(true);

            int nPadding = (int)Math.Floor(delta);

            // Temporary
            //nPadding *= 2;

            int n = (int) Math.Ceiling((bbox.Diagonal.MaximumCoordinate) / Delta + 0.002);
            n += nPadding * 2;
            Point2d origin = new Point2d(bbox.Min.X, bbox.Min.Y) - new Vector2d(
                (nPadding) * Delta, 
                (nPadding) * Delta);

            voxels = new Voxels2d<int>(origin, Delta, n);
        }

        public void FillBoundaryValues(int newvalue = 1)
        {

            double del = voxels.delta / Math.Sqrt(2);
            foreach (Curve curve in mesh.Edges.Select(x => (Curve)x))
            {
                List<Point3d> pts = new List<Point3d>();

                //double curveLength = curve.GetLength();
                //double maxt = curve.Domain.Max;
                //double x = maxt / curveLength;
                //for (double i = 0; i < maxt; i += del * x * 0.1)
                //{
                //    pts.Add(curve.PointAt(i));
                //}
                //pts.Add(curve.PointAtEnd);

                pts.AddRange(curve.DivideEquidistant(del));
                points.AddRange(pts);

                // change this to be not be in a separate loop later
                foreach (Point3d pt in pts) 
                    voxels.cellValues[voxels.PointToIndex(new Point2d(pt.X, pt.Y))] = newvalue;
            }
        }


        public void FillBoundaryValuesBC(Curve curve, int newvalue, double delta)
        {
            double del = voxels.delta / Math.Sqrt(2);

            var indecies = new HashSet<int>();
            List<Point3d> pts = new List<Point3d>();

            //double curveLength = curve.GetLength();
            //double maxt = curve.Domain.Max;
            //double x = maxt / curveLength;
            //for (double i = 0; i < maxt; i += del * x * 0.1)
            //{
            //    pts.Add(curve.PointAt(i));
            //}
            //pts.Add(curve.PointAtEnd);

            pts.AddRange(curve.DivideEquidistant(del));
            points.AddRange(pts);

            foreach (Point3d pt in pts)
                indecies.Add(voxels.PointToIndex(new Point2d(pt.X, pt.Y)));


            var pts2d = new List<Point2d>();
            foreach (int i in indecies)
            {
                Point2d p = voxels.IndexToPoint(i);
                curve.ClosestPoint(new Point3d(p.X, p.Y, 0), out var t);
                var normal = Vector3d.CrossProduct(curve.TangentAt(t), Vector3d.ZAxis);
                normal *= del;
                var normal2d = new Vector2d(normal.X, normal.Y);
                for (int j = (int)-(voxels.delta * delta / del); j < voxels.delta * delta / del; j++)
                {
                    pts2d.Add(p + normal2d * j);
                }
            }
            //points.AddRange(pts);

            // change this to be not be in a separate loop later
            foreach (Point2d pt in pts2d)
            {
                int i = voxels.PointToIndex(pt);
                voxels.cellValues[i] |= newvalue;
            }
        }
        

        
        public void SetBCD(Containers.IBoundaryCondition2d bc, double delta, int val)
        {
            FillBoundaryValuesBC(bc.curve, val, delta);
        }

        public void SetBCN(Containers.IBoundaryCondition2d bc, int val)
        {
            FillBoundaryValuesBC(bc.curve, val, this.voxels.delta * 4);
        }
        
        public void RefineBoundaries()
        {
            for (int i = 0; i < voxels.cellValues.Length; i++)
            {
                if (voxels.cellValues[i] == 1)
                {
                    Point2d pt = voxels.IndexToPoint(i);
                    bool inside = IsPointOn(pt);
                    voxels.cellValues[i] = inside ? 1 : 0;
                }
            }
        }

        public bool IsPointOn(Point2d pt)
        {
            Point3d pt3d = new Point3d(pt.X, pt.Y, 0);
            var on = mesh.ClosestPoint(pt3d);
            return on.DistanceToSquared(pt3d) < 1e-9;
        }

        public void FillInternalValues()
        {
            while (FindFirstInside(0, true, out Coord coord))
            {
                FloodFill(coord, 0, 2);
            }
        }

        bool FindFirstInside(int val, bool inside, out Coord coord)
        {
            // Find a cell with the right value
            // check if it is inside the mesh, else iterate until the we pass a border again
            // Uses a global index such that a new interior point is found each time the method is called, until all points are found
            for (; index < voxels.cellValues.Length; index++)
            {
                if (voxels.cellValues[index] != val)
                    continue;

                if (IsPointOn(voxels.IndexToPoint(index)) != inside)
                {
                    index++;
                    for (; index < voxels.cellValues.Length; index++)
                    {
                        if (voxels.cellValues[index] != val)
                            break;
                    }
                }
                else
                {
                    coord = voxels.IndexToCoord(index);
                    index++;
                    return true;
                }
            }

            coord = new Coord();
            return false;
        }


        public void FloodFill(Coord first, int from, int to)
        {
            int n = voxels.n;
            Stack <Coord> stack = new Stack<Coord>();
            stack.Push(first);
            while (stack.Count > 0)
            {
                Coord temp = stack.Pop();
                int x1 = temp.X;
                while (x1 > 0 && voxels.cellValues[voxels.ToLinearIndex(x1 - 1, temp.Y)] == from)
                    x1--;

                bool spanLeft = false;
                bool spanRight = false;
                int offset = voxels.ToLinearIndex(0, temp.Y);
                while (x1 < n && voxels.cellValues[x1 + offset] == from)
                {
                    voxels.cellValues[x1 + offset] = to;
                    if (!spanLeft && temp.Y > 0 && voxels.cellValues[x1 + offset - n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y - 1));
                        spanLeft = true;
                    }
                    else if (spanLeft && temp.Y - 1 == 0 && voxels.cellValues[x1 + offset - n] != from)
                    {
                        spanLeft = false;
                    }

                    if (!spanRight && temp.Y < n -1 && voxels.cellValues[x1 + offset + n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y + 1));
                        spanRight = true;
                    }
                    else if (spanRight && temp.Y + 1 < n - 1 && voxels.cellValues[x1 + offset + n] != from)
                    {
                        spanRight = false;
                    }
                    
                    x1++;
                }
            }
        }


        
    }
}
