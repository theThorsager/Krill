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
            for (int i = 0; i < mask.cellValues.Length; i++)
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

    }
}
