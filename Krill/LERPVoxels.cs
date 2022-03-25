using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    public static class LERPVoxels
    {
        public static double ValueAt(this Voxels<double> voxels, Point3d pt)
        {
            pt -= (Vector3d)voxels.origin;
            pt /= voxels.delta;

            double u = pt.X;
            double v = pt.Y;
            double w = pt.Z;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int k0 = (int)Math.Floor(w - 0.5);
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            int INDi0j0k0 = voxels.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = voxels.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = voxels.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = voxels.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = voxels.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = voxels.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = voxels.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = voxels.ToLinearIndex(i1, j1, k1);


            return (1 - a) * (1 - b) * (1 - c) * voxels.cellValues[INDi0j0k0]
                    + a * (1 - b) * (1 - c) * voxels.cellValues[INDi1j0k0]
                    + (1 - a) * b * (1 - c) * voxels.cellValues[INDi0j1k0]
                    + a * b * (1 - c) * voxels.cellValues[INDi1j1k0]
                    + (1 - a) * (1 - b) * c * voxels.cellValues[INDi0j0k1]
                    + a * (1 - b) * c * voxels.cellValues[INDi1j0k1]
                    + (1 - a) * b * c * voxels.cellValues[INDi0j1k1]
                    + a * b * c * voxels.cellValues[INDi1j1k1];
        }

        public static Vector3d ValueAt(this Voxels<Vector3d> voxels, Point3d pt)
        {
            pt -= (Vector3d)voxels.origin;
            pt /= voxels.delta;

            double u = pt.X;
            double v = pt.Y;
            double w = pt.Z;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int k0 = (int)Math.Floor(w - 0.5);
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            int INDi0j0k0 = voxels.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = voxels.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = voxels.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = voxels.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = voxels.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = voxels.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = voxels.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = voxels.ToLinearIndex(i1, j1, k1);

            return (1 - a) * (1 - b) * (1 - c) * voxels.cellValues[INDi0j0k0]
                    + a * (1 - b) * (1 - c) * voxels.cellValues[INDi1j0k0]
                    + (1 - a) * b * (1 - c) * voxels.cellValues[INDi0j1k0]
                    + a * b * (1 - c) * voxels.cellValues[INDi1j1k0]
                    + (1 - a) * (1 - b) * c * voxels.cellValues[INDi0j0k1]
                    + a * (1 - b) * c * voxels.cellValues[INDi1j0k1]
                    + (1 - a) * b * c * voxels.cellValues[INDi0j1k1]
                    + a * b * c * voxels.cellValues[INDi1j1k1];
        }



        public static double ValueAt(this Voxels2d<double> voxels, Point2d pt)
        {
            pt -= new Vector2d(voxels.origin.X, voxels.origin.Y);
            pt /= voxels.delta;

            double u = pt.X;
            double v = pt.Y;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int i1 = i0 + 1;
            int j1 = j0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);

            int INDi0j0k0 = voxels.ToLinearIndex(i0, j0);
            int INDi1j0k0 = voxels.ToLinearIndex(i1, j0);
            int INDi0j1k0 = voxels.ToLinearIndex(i0, j1);
            int INDi1j1k0 = voxels.ToLinearIndex(i1, j1);

            return (1 - a) * (1 - b) * voxels.cellValues[INDi0j0k0]
                    + a * (1 - b) * voxels.cellValues[INDi1j0k0]
                    + (1 - a) * b * voxels.cellValues[INDi0j1k0]
                    + a * b * voxels.cellValues[INDi1j1k0];
        }

        public static Vector2d ValueAt(this Voxels2d<Vector2d> voxels, Point2d pt)
        {
            pt -= new Vector2d(voxels.origin.X, voxels.origin.Y);
            pt /= voxels.delta;

            double u = pt.X;
            double v = pt.Y;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int i1 = i0 + 1;
            int j1 = j0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);

            int INDi0j0k0 = voxels.ToLinearIndex(i0, j0);
            int INDi1j0k0 = voxels.ToLinearIndex(i1, j0);
            int INDi0j1k0 = voxels.ToLinearIndex(i0, j1);
            int INDi1j1k0 = voxels.ToLinearIndex(i1, j1);

            return (1 - a) * (1 - b) * voxels.cellValues[INDi0j0k0]
                    + a * (1 - b) * voxels.cellValues[INDi1j0k0]
                    + (1 - a) * b * voxels.cellValues[INDi0j1k0]
                    + a * b * voxels.cellValues[INDi1j1k0];
        }

    }
}
