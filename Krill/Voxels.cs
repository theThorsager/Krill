using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    public class Voxels<T>
    {
        public Point3d origin;
        public double delta;
        public int n;
        public T[] cellValues;

        public Voxels(Point3d origin, double delta, int n)
        {
            this.origin = origin;
            this.delta = delta;
            this.n = n;
            cellValues = new T[n*n*n];
        }

        public void SetValues(Voxels<int> mask, int maskbit, T val)
        {
            for (int i = 0; i < cellValues.Length; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                {
                    cellValues[i] = val;
                }
            }
        }

        public static void MaskValues(Voxels<int> mask, int maskbit)
        {
            for (int i = 0; i < mask.cellValues.Length; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                {
                    mask.cellValues[i] &= ~maskbit;
                }
            }
        }

        public List<Point3d> GetPointsAt(T val)
        {
            List<Point3d> result = new List<Point3d>();
            for (int i = 0; i < cellValues.Length; i++)
            {
                if (cellValues[i].Equals(val))
                    result.Add(IndexToPoint(i));
            }
            return result;
        }
        public List<Point3d> GetPointsNotAt(T val)
        {
            List<Point3d> result = new List<Point3d>();
            for (int i = 0; i < cellValues.Length; i++)
            {
                if (!cellValues[i].Equals(val))
                    result.Add(IndexToPoint(i));
            }
            return result;
        }
        public static List<Point3d> GetPoints(Voxels<int> mask, Voxels<Vector3d> disp, double factor = 10, uint maskbit = 0xFFFFFFFF)
        {
            List<Point3d> result = new List<Point3d>();
            for (int i = 0; i < mask.cellValues.Length; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                    result.Add(disp.IndexToPoint(i) + factor * disp.cellValues[i]);
            }
            return result;
        }

        public List<T> GetValues(Voxels<int> mask, uint maskbit = 0xFFFFFFFF)
        {
            List<T> result = new List<T>();
            for (int i = 0; i < mask.cellValues.Length; i++)
            {
                if ((mask.cellValues[i] & maskbit) != 0)
                    result.Add(this.cellValues[i]);
            }
            return result;
        }

        public Point3d IndexToPoint(int i, int j, int k)
        {
            return new Point3d(origin.X + delta * i, origin.Y + delta * j, origin.Z + delta * k);
        }

        public Coord IndexToCoord(int i)
        {
            To3DIndex(ref i, out int j, out int k);
            return new Coord(i, j, k);
        }
        public Point3d IndexToPoint(int i)
        {
            To3DIndex(ref i, out int j, out int k);
            return new Point3d(origin.X + delta * (i + 0.5), origin.Y + delta * (j + 0.5), origin.Z + delta * (k + 0.5));
        }

        public int PointToIndex(Point3d pt)
        {
            pt -= (Vector3d)origin;
            pt /= delta;
            return ToLinearIndex((int)pt.X, (int)pt.Y, (int)pt.Z);
        }
        public int CoordToIndex(Coord pt)
        {
            return ToLinearIndex(pt.X, pt.Y, pt.Z);
        }
        public Coord PointToCoord(Point3d pt)
        {
            pt -= (Vector3d)origin;
            pt /= delta;
            return new Coord((int)pt.X, (int)pt.Y, (int)pt.Z);
        }

        public int ToLinearIndex(int i, int j, int k)
        {
            return i + j * n + k * n * n;
        }

        public void To3DIndex(ref int i, out int j, out int k)
        {
            k = i / (n * n);
            j = (i - k * n * n) / n;
            i = i - k * n * n - j * n;
        }

        public void commRel(Voxels<T> other, Coord min, Coord max, int offset)
        {
            for (int k = min.Z; k < max.Z; k++)
            {
                for (int j = min.Y; j < max.Y; j++)
                {
                    for (int i = min.X; i < max.X; i++)
                    {
                        int I = ToLinearIndex(i, j, k);
                        int J = I + offset;

                        this.cellValues[I] = other.cellValues[J];
                    }
                }
            }
        }

        public static Mesh GetBoundaryMesh(Voxels<int> voxels)
        {
            int n = voxels.n;

            List<MeshFace> faces = new List<MeshFace>();
            Dictionary<int, int> map = new Dictionary<int, int>();

            List<Point3d> points = new List<Point3d>();

            int I = voxels.ToLinearIndex(1, 0, 0);
            int J = voxels.ToLinearIndex(0, 1, 0);
            int K = voxels.ToLinearIndex(0, 0, 1);

            for (int k = 0; k < n - 1; k++)
            {
                for (int j = 0; j < n - 1; j++)
                {
                    for (int i = 0; i < n - 1; i++)
                    {
                        int Ind = voxels.ToLinearIndex(i, j, k);
                        bool current = voxels.cellValues[Ind] == 0;
                        if (current ^ voxels.cellValues[Ind + I] == 0)
                        {
                            MeshFace face = current ?
                                new MeshFace(Ind + I, Ind + I + K, Ind + J + I + K, Ind + I + J) :
                                new MeshFace(Ind + I, Ind + I + J, Ind + J + I + K, Ind + I + K);
                            faces.Add(face);
                            addToMap(map, points, voxels, face);
                        }
                        if (current ^ voxels.cellValues[Ind + J] == 0)
                        {
                            MeshFace face = current ?
                                new MeshFace(Ind + J, Ind + J + I, Ind + J + I + K, Ind + J + K) :
                                new MeshFace(Ind + J, Ind + J + K, Ind + J + I + K, Ind + J + I);
                            faces.Add(face);
                            addToMap(map, points, voxels, face);
                        }
                        if (current ^ voxels.cellValues[Ind + K] == 0)
                        {
                            MeshFace face = current ?
                                new MeshFace(Ind + K, Ind + K + J, Ind + J + I + K, Ind + I + K) :
                                new MeshFace(Ind + K, Ind + K + I, Ind + J + I + K, Ind + K + J);
                            faces.Add(face);
                            addToMap(map, points, voxels, face);
                        }
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

        private static void addToMap(Dictionary<int, int> map, List<Point3d> points, Voxels<int> voxels, MeshFace face)
        {
            Vector3d off = -0.5 * new Vector3d(voxels.delta, voxels.delta, voxels.delta);
            if (!map.ContainsKey(face.A))
            {
                map.Add(face.A, points.Count);
                points.Add(voxels.IndexToPoint(face.A) + off);
            }
            if (!map.ContainsKey(face.B))
            {
                map.Add(face.B, points.Count);
                points.Add(voxels.IndexToPoint(face.B) + off);
            }
            if (!map.ContainsKey(face.C))
            {
                map.Add(face.C, points.Count);
                points.Add(voxels.IndexToPoint(face.C) + off);
            }
            if (!map.ContainsKey(face.D))
            {
                map.Add(face.D, points.Count);
                points.Add(voxels.IndexToPoint(face.D) + off);
            }
        }
    }
}
