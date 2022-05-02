using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using CSparse;
using CSparse.Double;

namespace Krill
{
    internal class SDFTruss
    {
        public BoxSDF SDF = null;

        public double[] xs = null;
        public double[] dxs = null;
        public Tuple<int, int>[] connections = null;

        public List<bool> lockedDOF = new List<bool>();

        public double cover = 0.0;

        public double gamma = 1.0;

        public void Init(Containers.TrussGeometry truss, BoxSDF SDF, double cover)
        {
            this.SDF = SDF;
            this.cover = cover;

            xs = truss.Nodes.SelectMany(x => new double[] { x.X, x.Y, x.Z }).ToArray();
            connections = truss.Connections.ToArray();

            dxs = new double[xs.Length];
        }

        public void LockDOFs(List<bool> locked)
        {
            lockedDOF = locked.ToList();
        }

        //public double EvaluateTruss(double cutoff)
        //{
        //    double sum = 0;
        //    for (int i = 0; i < xs.Length; i += 3)
        //    {
        //        sum += EvaluateNode(i);
        //    }
        //    return sum;
        //}

        public double SetGradients(double cutoff)
        {
            double sum = 0;
            Vector.Clear(dxs);
            for (int i = 0; i < xs.Length; i += 3)
            {
                double temp = EvaluateNode(i);
                if (temp > 0)
                {
                    SetGradients(i);
                    sum += temp;
                }
            }
            return sum;
        }

        public void ApplyGradient(double factor = 1)
        {
            Vector.Add(xs.Length, factor, dxs, xs, xs);
        }
        double EvaluateNode(int nIndex)
        {
            var pt = new Point3d(xs[nIndex], xs[nIndex+1], xs[nIndex+2]);

            var box = SDF.BoxValueAt(pt);

            double resX = lockedDOF[nIndex] ? 0 : Math.Max(0, cover - box.X * 0.5);
            double resY = lockedDOF[nIndex+1] ? 0 : Math.Max(0, cover - box.Y * 0.5);
            double resZ = lockedDOF[nIndex+2] ? 0 : Math.Max(0, cover - box.Z * 0.5);
            double res = Math.Max(Math.Max(resX, resY), resZ);
            return res * res * gamma;
        }

        void SetGradients(int nIndex)
        {
            var pt = new Point3d(xs[nIndex], xs[nIndex + 1], xs[nIndex + 2]);

            var grad = SDF.BoxGradientAt(pt, cover, gamma);

            dxs[nIndex] += lockedDOF[nIndex] ? 0 : grad.X;
            dxs[nIndex+1] += lockedDOF[nIndex+1] ? 0 : grad.Y;
            dxs[nIndex+2] += lockedDOF[nIndex+2] ? 0 : grad.Z;
        }
    }
}
