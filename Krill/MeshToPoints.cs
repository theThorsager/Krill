using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    class MeshToPoints
    {
        Mesh mesh;

        public Voxels<int> voxels;

        public List<Point3d> points = new List<Point3d> ();     // For debugging, remove

        public MeshToPoints(Mesh mesh, double Delta, double delta)
        {
            this.mesh = mesh;
            
            if (!mesh.IsSolid)
            {
                Console.WriteLine("Non Solid Mesh");
            }

            BoundingBox bbox = mesh.GetBoundingBox(true);

            int nPadding = (int)Math.Floor(delta);

            // Temporary
            //nPadding *= 2;

            int n = (int) Math.Ceiling((bbox.Diagonal.MaximumCoordinate) / Delta + 0.002);
            n += nPadding * 2;
            Point3d origin = bbox.Min - new Vector3d(
                (nPadding) * Delta, 
                (nPadding) * Delta, 
                (nPadding) * Delta);

            voxels = new Voxels<int>(origin, Delta, n);
        }

        public MeshToPoints(Voxels<int> mask)
        {
            voxels = mask;
        }

        public void FillBoundaryValues(int newvalue = 1)
        {
            double del = voxels.delta / Math.Sqrt(2);
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

                //points.AddRange(pts);

                // change this to be not be in a separate loop later
                foreach (Point3d pt in pts) 
                    voxels.cellValues[voxels.PointToIndex(pt)] = newvalue;

                if (face.IsQuad && isfirst)
                {
                    isfirst = false;
                    b = mesh.Vertices[face.D];
                    goto Begining;
                }
            }
        }

        public void FillBoundaryValuesBC(int newvalue, double delta)
        {
            double del = voxels.delta / Math.Sqrt(2);
            foreach (MeshFace face in mesh.Faces)
            {
                var indecies = new HashSet<int>();
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

                //points.AddRange(pts);

                foreach (Point3d pt in pts)
                    indecies.Add(voxels.PointToIndex(pt));

                if (face.IsQuad && isfirst)
                {
                    isfirst = false;
                    b = mesh.Vertices[face.D];
                    goto Begining;
                }

                pts.Clear();

                b = mesh.Vertices[face.B];
                var normal = Vector3d.CrossProduct(b - a, ac);
                normal /= normal.Length;
                normal *= del;
                foreach (int i in indecies)
                {
                    Point3d p = voxels.IndexToPoint(i);
                    for (int j = (int)-(voxels.delta * delta / del); j < voxels.delta * delta / del; j++)
                    {
                        pts.Add(p + normal * j);
                    }
                }
                points.AddRange(pts);

                // change this to be not be in a separate loop later
                foreach (Point3d pt in pts)
                {
                    int i = voxels.PointToIndex(pt);
                    voxels.cellValues[i] |= newvalue;
                }
            }
        }

        public void SetBCD(Containers.IBoundaryCondition bc, double delta, int val)
        {
            Mesh temp = mesh;
            mesh = bc.area;
            FillBoundaryValuesBC(val, delta);

            mesh = temp;
        }

        public void SetBCN(Containers.IBoundaryCondition bc, double delta, int val)
        {
            Mesh temp = mesh;
            mesh = bc.area;
            FillBoundaryValuesBC(val, delta);

            mesh = temp;
        }

        public void RefineBoundaries()
        {
            for (int i = 0; i < voxels.cellValues.Length; i++)
            {
                if (voxels.cellValues[i] == 1)
                {
                    Point3d pt = voxels.IndexToPoint(i);
                    bool inside = mesh.IsPointInside(pt, 0, true);
                    voxels.cellValues[i] = inside ? 1 : 0;
                }
            }
        }

        public void FillInternalValues()
        {
            // Find a cell with the right value
            // check if it is inside the mesh, else iterate until the we pass a border again
            // Uses a global index such that a new interior point is found each time the method is called, until all points are found
            for (int index = 0; index < voxels.cellValues.Length; index++)
            {
                if (voxels.cellValues[index] != 0)
                    continue;

                if (mesh.IsPointInside(voxels.IndexToPoint(index), 1e-3, true))
                {
                    FloodFill(voxels.IndexToCoord(index), 0, 2);
                }
                else
                {
                    FloodFill(voxels.IndexToCoord(index), 0, 8);
                }
            }

            voxels.SetValues(voxels, 8, 0);
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
                while (x1 > 0 && voxels.cellValues[voxels.ToLinearIndex(x1 - 1, temp.Y, temp.Z)] == from)
                    x1--;

                bool spanLeft = false;
                bool spanRight = false;
                bool spanUp = false;
                bool spanDown = false;
                int offset = voxels.ToLinearIndex(0, temp.Y, temp.Z);
                while (x1 < n && voxels.cellValues[x1 + offset] == from)
                {
                    voxels.cellValues[x1 + offset] = to;
                    if (!spanLeft && temp.Y > 0 && voxels.cellValues[x1 + offset - n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y - 1, temp.Z));
                        spanLeft = true;
                    }
                    else if (spanLeft && temp.Y - 1 == 0 && voxels.cellValues[x1 + offset - n] != from)
                    {
                        spanLeft = false;
                    }

                    if (!spanRight && temp.Y < n -1 && voxels.cellValues[x1 + offset + n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y + 1, temp.Z));
                        spanRight = true;
                    }
                    else if (spanRight && temp.Y + 1 < n - 1 && voxels.cellValues[x1 + offset + n] != from)
                    {
                        spanRight = false;
                    }

                    if (!spanDown && temp.Z > 0 && voxels.cellValues[x1 + offset - n*n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y, temp.Z - 1));
                        spanDown = true;
                    }
                    else if (spanDown && temp.Z - 1 == 0 && voxels.cellValues[x1 + offset - n*n] != from)
                    {
                        spanDown = false;
                    }

                    if (!spanUp && temp.Z < n - 1 && voxels.cellValues[x1 + offset + n * n] == from)
                    {
                        stack.Push(new Coord(x1, temp.Y, temp.Z + 1));
                        spanUp = true;
                    }
                    else if (spanUp && temp.Z + 1 < n - 1 && voxels.cellValues[x1 + offset + n * n] != from)
                    {
                        spanUp = false;
                    }
                    x1++;
                }
            }
        }


        
    }
}
