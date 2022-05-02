using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    public class Voxels2d<T>
    {
        public Point2d origin;
        public double delta;
        public int n;
        public T[] cellValues;

        public Voxels2d()
        { }

        public Voxels2d(Point2d origin, double delta, int n)
        {
            this.origin = origin;
            this.delta = delta;
            this.n = n;
            cellValues = new T[n*n];
        }

        public void SetValues(Voxels2d<int> mask, int maskbit, T val)
        {
            for (int i = 0; i < cellValues.Length; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                {
                    cellValues[i] = val;
                }
            }
        }

        public static void MaskValues(Voxels2d<int> mask, int maskbit)
        {
            for (int i = 0; i < mask.cellValues.Length; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                {
                    mask.cellValues[i] &= ~maskbit;
                }
            }
        }

        public List<Point2d> GetPointsAt(T val)
        {
            List<Point2d> result = new List<Point2d>();
            for (int i = 0; i < cellValues.Length; i++)
            {
                if (cellValues[i].Equals(val))
                    result.Add(IndexToPoint(i));
            }
            return result;
        }
        public List<Point2d> GetPointsNotAt(T val)
        {
            List<Point2d> result = new List<Point2d>();
            for (int i = 0; i < cellValues.Length; i++)
            {
                if (!cellValues[i].Equals(val))
                    result.Add(IndexToPoint(i));
            }
            return result;
        }
        public static List<Point2d> GetPoints(Voxels2d<int> mask, Voxels2d<Vector2d> disp, double factor = 10, uint maskbit = 0xFFFFFFFF)
        {
            List<Point2d> result = new List<Point2d>();
            int n = Math.Min(disp.cellValues.Length, mask.cellValues.Length);
            for (int i = 0; i < n; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                    result.Add(disp.IndexToPoint(i) + factor * disp.cellValues[i]);
            }
            return result;
        }

        public List<T> GetValues(Voxels2d<int> mask, uint maskbit = 0xFFFFFFFF)
        {
            List<T> result = new List<T>();
            for (int i = 0; i < mask.cellValues.Length; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                    result.Add(this.cellValues[i]);
            }
            return result;
        }

        public Point2d IndexToPoint(int i, int j)
        {
            return new Point2d(origin.X + delta * i, origin.Y + delta * j);
        }

        public Coord IndexToCoord(int i)
        {
            To2DIndex(ref i, out int j);
            return new Coord(i, j);
        }
        public Point2d IndexToPoint(int i)
        {
            To2DIndex(ref i, out int j);
            return new Point2d(origin.X + delta * (i + 0.5), origin.Y + delta * (j + 0.5));
        }

        public int PointToIndex(Point2d pt)
        {
            pt -= new Vector2d(origin.X, origin.Y);
            pt /= delta;
            return ToLinearIndex((int)pt.X, (int)pt.Y);
        }
        public int CoordToIndex(Coord pt)
        {
            return ToLinearIndex(pt.X, pt.Y);
        }
        public Coord PointToCoord(Point2d pt)
        {
            pt -= new Vector2d(origin.X, origin.Y);
            pt /= delta;
            return new Coord((int)pt.X, (int)pt.Y);
        }

        public int ToLinearIndex(int i, int j)
        {
            return i + j * n;
        }

        public void To2DIndex(ref int i, out int j)
        {
            j = i / n;
            i = i - j * n;
        }

        public static void SoftenNearMask(Voxels2d<int> mask, Voxels2d<Vector2d> voxel, int[] nlist)
        {
            for (int j = 0; j < voxel.n; j++)
            {
                for (int i = 0; i < voxel.n; i++)
                {
                    int I = voxel.ToLinearIndex(i, j);
                    if ((mask.cellValues[I] & 3) == 0)
                        continue;

                    for (int a = 0; a < nlist.Length; a++)
                    {
                        int J = I + nlist[a];
                        if ((mask.cellValues[J] & 3) == 0)
                        {
                            voxel.cellValues[J] += voxel.cellValues[I];
                            int old = mask.cellValues[J] >> 8;
                            old++;
                            mask.cellValues[J] = old << 8;
                        }

                        J = I - nlist[a];
                        if ((mask.cellValues[J] & 3) == 0)
                        {
                            voxel.cellValues[J] += voxel.cellValues[I];
                            int old = mask.cellValues[J] >> 8;
                            old++;
                            mask.cellValues[J] = old << 8;
                        }
                    }
                }
            }

            double max = nlist.Length * 2;
            //Func<double, double> f = x => x * x / max - 0.5 * x + max * 0.5;

            double minor = max * 0.2;
            Func<double, double> f = x => Math.Max(x, minor);

            for (int j = 0; j < voxel.n; j++)
            {
                for (int i = 0; i < voxel.n; i++)
                {
                    int I = voxel.ToLinearIndex(i, j);
                    if ((mask.cellValues[I] & 3) != 0 || mask.cellValues[I] == 0)
                        continue;

                    int count = mask.cellValues[I] >> 8;

                    double norm = 1 / f(count);
                    voxel.cellValues[I] *= norm;
                    mask.cellValues[I] = 0;
                }
            }
        }
        public static void LERPFrom(Voxels2d<Vector2d> voxel, Voxels2d<Vector2d> other)
        {
            Point2d min = other.origin + other.delta * (0.5001) * new Vector2d(1, 1);
            Point2d max = other.origin + other.delta * (other.n - 1.5001) * new Vector2d(1, 1);

            for (int j = 0; j < voxel.n; j++)
            {
                for (int i = 0; i < voxel.n; i++)
                {
                    Point2d pt = new Point2d(voxel.origin.X + voxel.delta * (i + 0.5), voxel.origin.Y + voxel.delta * (j + 0.5));
                    if (pt.X < min.X || pt.Y < min.Y ||
                        pt.X > max.X || pt.Y > max.Y)
                        continue;

                    int I = voxel.ToLinearIndex(i, j);
                    voxel.cellValues[I] = LERPVoxels.ValueAt(other, pt);
                }
            }
        }
        public void commRel(Voxels2d<T> other, Coord min, Coord max, int offset)
        {
            for (int k = min.Z; k < max.Z; k++)
            {
                for (int j = min.Y; j < max.Y; j++)
                {
                    for (int i = min.X; i < max.X; i++)
                    {
                        int I = ToLinearIndex(i, j);
                        int J = I + offset;

                        this.cellValues[I] = other.cellValues[J];
                    }
                }
            }
        }

        public static Mesh GetBoundaryMesh(Voxels2d<int> voxels)
        {
            int n = voxels.n;

            List<MeshFace> faces = new List<MeshFace>();
            Dictionary<int, int> map = new Dictionary<int, int>();

            List<Point3d> points = new List<Point3d>();

            int I = voxels.ToLinearIndex(1, 0);
            int J = voxels.ToLinearIndex(0, 1);

            for (int j = 0; j < n - 1; j++)
            {
                for (int i = 0; i < n - 1; i++)
                {
                    int Ind = voxels.ToLinearIndex(i, j);
                    bool current = voxels.cellValues[Ind] != 0;

                    if (current)
                    {
                        MeshFace face = 
                            new MeshFace(Ind, Ind + I, Ind + J + I, Ind + J);
                        faces.Add(face);
                        addToMap(map, points, voxels, face);
                    }
                }
            }

            faces = faces.Select(x => new MeshFace(map[x.A], map[x.B], map[x.C], map[x.D])).ToList();

            Mesh mesh = new Mesh();
            mesh.Faces.AddFaces(faces);
            mesh.Vertices.AddVertices(points);

            //mesh.MergeAllCoplanarFaces(1e-3);

            return mesh;
        }

        private static void addToMap(Dictionary<int, int> map, List<Point3d> points, Voxels2d<int> voxels, MeshFace face)
        {
            Func<Point2d, Point3d> func = x => new Point3d(x.X, x.Y, 0);
            Vector3d off = -0.5 * new Vector3d(voxels.delta, voxels.delta, 0);
            if (!map.ContainsKey(face.A))
            {
                map.Add(face.A, points.Count);
                points.Add(func(voxels.IndexToPoint(face.A)) + off);
            }
            if (!map.ContainsKey(face.B))
            {
                map.Add(face.B, points.Count);
                points.Add(func(voxels.IndexToPoint(face.B)) + off);
            }
            if (!map.ContainsKey(face.C))
            {
                map.Add(face.C, points.Count);
                points.Add(func(voxels.IndexToPoint(face.C)) + off);
            }
            if (!map.ContainsKey(face.D))
            {
                map.Add(face.D, points.Count);
                points.Add(func(voxels.IndexToPoint(face.D)) + off);
            }
        }
    }
}
