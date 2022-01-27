using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    struct Coord
    {
        public int X;
        public int Y;
        public int Z;
        public Coord(int x, int y, int z)
        {
            X = x; Y = y; Z = z; 
        }

        public static Coord operator-(Coord a, Coord b)
        {
            return new Coord(a.X-b.X, a.Y-b.Y, a.Z-b.Z);
        }
    }
    class MeshToPoints
    {
        Mesh mesh;
        Point3d origin;
        double delta;
        int n;
        int[] cellValues;
        int index = 0;

        public List<Point3d> points = new List<Point3d> ();     // For debugging, remove

        public MeshToPoints(Mesh mesh_, double delta_)
        {
            mesh = mesh_;
            delta = delta_;
            
            if (!mesh.IsSolid)
            {
                Console.WriteLine("Non Solid Mesh");
            }
            

            BoundingBox bbox = mesh.GetBoundingBox(true);

            n = (int) Math.Ceiling((bbox.Diagonal.MaximumCoordinate + 6 * delta) / delta);
            origin = bbox.Min - new Vector3d(3 * delta, 3 * delta, 3 * delta);
            cellValues = new int[n * n * n];
        }

        public void FillBoundaryValues()
        {
            double del = delta / Math.Sqrt(2);
            foreach (MeshFace face in mesh.Faces)
            {
                bool isfirst = true;
                Point3d a = mesh.Vertices[face.A];
                Point3d b = mesh.Vertices[face.B];
                Point3d c = mesh.Vertices[face.C];

            Begining:
                Vector3d ab = b - a;
                Vector3d ac = c - a;
                Vector3d bc = c - b;

                List<Point3d> pts = new List<Point3d>();

                double cos = ab * ac / (ab.Length * ac.Length);
                Vector3d dirC = ac * del / ac.Length;
                Vector3d dirB = (ab / ab.Length) * del / Math.Sqrt(1 - cos * cos); // Makes the length orthogonal to dirC = delta
                double length = ab.Length / dirB.Length;
                double reduction = length / (ac.Length / del);
                for (int j = 0; j < ac.Length / del; j++)
                {
                    Point3d pt = a + dirC * j;
                    for (int i = 0; i < length; i++)
                    {
                        pts.Add(pt);
                        pt += dirB;
                    }
                    length -= reduction;
                }

                // Specific fill for the other edges
                dirB = ab * del / ab.Length;
                for (int i = 0; i < ab.Length / del; i++)
                {
                    Point3d pt = b - dirB * i;
                    pts.Add(pt);
                }
                Vector3d dirBC = bc * del / bc.Length;
                for (int i = 0; i < bc.Length / del; i++)
                {
                    Point3d pt = c - dirBC * i;
                    pts.Add(pt);
                }

                points.AddRange(pts);

                foreach (Point3d pt in pts)
                    cellValues[PointToIndex(pt)] = 1;

                if (face.IsQuad && isfirst)
                {
                    isfirst = false;
                    b = mesh.Vertices[face.D];
                    goto Begining;
                }
            }
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
            for (; index < cellValues.Length; index++)
            {
                if (cellValues[index] != val)
                    continue;

                if (mesh.IsPointInside(IndexToPoint(index) + new Vector3d(delta / 2, delta / 2, delta / 2), 1e-3, true) != inside)
                {
                    index++;
                    for (; index < cellValues.Length; index++)
                    {
                        if (cellValues[index] != val)
                            break;
                    }
                }
                else
                {
                    coord = IndexToCoord(index);
                    index++;
                    return true;
                }
            }

            coord = new Coord();
            return false;
        }


        public void FloodFill(Coord first, int from, int to)
        {
            Stack<Coord> stack = new Stack<Coord>();
            stack.Push(first);
            while (stack.Count > 0)
            {
                Coord temp = stack.Pop();
                int x1 = temp.X;
                while (x1 > 0 && cellValues[ToLinearIndex(x1 - 1, temp.Y, temp.Z)] == from)
                    x1--;

                bool spanLeft = false;
                bool spanRight = false;
                bool spanUp = false;
                bool spanDown = false;
                int offset = ToLinearIndex(0, temp.Y, temp.Z);
                while (x1 < n && cellValues[x1 + offset] == from)
                {
                    cellValues[x1 + offset] = to;
                    if (!spanLeft && temp.Y > 0 && cellValues[x1 + offset - n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y - 1, temp.Z));
                        spanLeft = true;
                    }
                    else if (spanLeft && temp.Y - 1 == 0 && cellValues[x1 + offset - n] != from)
                    {
                        spanLeft = false;
                    }

                    if (!spanRight && temp.Y < n -1 && cellValues[x1 + offset + n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y + 1, temp.Z));
                        spanRight = true;
                    }
                    else if (spanRight && temp.Y + 1 < n - 1 && cellValues[x1 + offset + n] != from)
                    {
                        spanRight = false;
                    }

                    if (!spanDown && temp.Z > 0 && cellValues[x1 + offset - n*n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y, temp.Z - 1));
                        spanDown = true;
                    }
                    else if (spanDown && temp.Z - 1 == 0 && cellValues[x1 + offset - n*n] != from)
                    {
                        spanDown = false;
                    }

                    if (!spanUp && temp.Z < n - 1 && cellValues[x1 + offset + n * n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y, temp.Z + 1));
                        spanUp = true;
                    }
                    else if (spanUp && temp.Z + 1 < n - 1 && cellValues[x1 + offset + n * n] != from)
                    {
                        spanUp = false;
                    }
                    x1++;
                }
            }
        }

        public List<Point3d> GetPointsAt(int val)
        {
            List<Point3d> result = new List<Point3d>(); 
            for (int i = 0; i < cellValues.Length; i++)
            {
                if (cellValues[i] == val)
                    result.Add(IndexToPoint(i));
            }
            return result;
        }

        Point3d IndexToPoint(int i, int j, int k)
        {
            return new Point3d(origin.X + delta * i, origin.Y + delta * j, origin.Z + delta * k);
        }

        Coord IndexToCoord(int i)
        {
            To3DIndex(ref i, out int j, out int k);
            return new Coord(i, j, k);
        }
        Point3d IndexToPoint(int i)
        {
            To3DIndex(ref i, out int j, out int k);
            return new Point3d(origin.X + delta * i, origin.Y + delta * j, origin.Z + delta * k);
        }

        int PointToIndex(Point3d pt)
        {
            pt -= (Vector3d)origin;
            pt /= delta;
            return ToLinearIndex((int)pt.X, (int)pt.Y, (int)pt.Z);
        }
        int CoordToIndex(Coord pt)
        {
            return ToLinearIndex(pt.X, pt.Y, pt.Z);
        }
        Coord PointToCoord(Point3d pt)
        {
            pt -= (Vector3d)origin;
            pt /= delta;
            return new Coord((int)pt.X, (int)pt.Y, (int)pt.Z);
        }

        int ToLinearIndex(int i, int j, int k)
        {
            return i + j*n + k*n*n;
        }

        void To3DIndex(ref int i, out int j, out int k)
        {
            k = i / (n * n);
            j = (i - k*n*n) / n;
            i = i - k * n * n - j * n;
        }
    }
}
