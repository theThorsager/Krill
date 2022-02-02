﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class Voxels<T>
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
        public static List<Point3d> GetPoints(Voxels<int> mask, Voxels<Vector3d> disp)
        {
            List<Point3d> result = new List<Point3d>();
            for (int i = 0; i < mask.cellValues.Length; i++)
            {
                if (mask.cellValues[i] != 0)
                    result.Add(disp.IndexToPoint(i) + disp.cellValues[i]);
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
            return new Point3d(origin.X + delta * i, origin.Y + delta * j, origin.Z + delta * k);
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
    }
}