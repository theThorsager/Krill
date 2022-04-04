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
    internal class OrthonogalityTruss
    {
        public double[] xs = null;
        public double[] dxs = null;
        public Tuple<int, int>[] connections = null;

        public List<bool> lockedDOF = new List<bool>();

        public void Init(Containers.TrussGeometry truss)
        {
            xs = truss.Nodes.SelectMany(x => new double[] { x.X, x.Y, x.Z }).ToArray();
            connections = truss.Connections.ToArray();

            dxs = new double[xs.Length];
        }

        public void LockDOFs(List<bool> locked)
        {
            lockedDOF = locked.ToList();
        }

        public double EvaluateTruss(double cutoff)
        {
            double sum = 0;
            for (int i = 0; i < connections.Length; i++)
            {
                sum += EvaluateElement(i, cutoff);
            }
            return sum;
        }

        public void SetGradients(double cutoff)
        {
            Vector.Clear(dxs);
            for (int i = 0; i < connections.Length; i++)
            {
                SetGradients(i, cutoff);
            }
        }

        public void ApplyGradient(double factor = 1)
        {
            Vector.Add(xs.Length, factor, dxs, xs, xs);
        }

        double EvaluateElement(int eIndex, double cutoff)
        {
            var i = connections[eIndex].Item1 * 3;
            var j = connections[eIndex].Item2 * 3;

            double x = xs[j] - xs[i];
            double y = xs[j+1] - xs[i+1];
            double z = xs[j+2] - xs[i+2];
            double norm = x * x + y * y + z * z;
            double l = Math.Sqrt(norm);

            if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(z))
            {
                x = 0;
            }
            else if (Math.Abs(y) > Math.Abs(z))
            {
                y = 0;
            }
            else
            {
                z = 0;
            }

            return Math.Min(cutoff * cutoff * 0.4, (x * x + y * y + z * z) / norm);
        }
        void SetGradients(int eIndex, double f)
        {
            var i = connections[eIndex].Item1 * 3;
            var j = connections[eIndex].Item2 * 3;

            double x = xs[j] - xs[i];
            double y = xs[j + 1] - xs[i + 1];
            double z = xs[j + 2] - xs[i + 2];
            double norm = x * x + y * y + z * z;
            double l = Math.Sqrt(norm);
            double sqnorm = 1 / (norm * l);
            double rx = 0;
            double ry = 0;
            double rz = 0;
            if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(z))
            {
                rx = -2 * x * (y * y + z * z) * sqnorm;
                ry = 2 * x * x * y * sqnorm;
                rz = 2 * x * x * z * sqnorm;
            }
            else if (Math.Abs(y) > Math.Abs(z))
            {
                rx = 2 * y * y * x * sqnorm;
                ry = -2 * y * (x * x + z * z) * sqnorm;
                rz = 2 * y * y * z * sqnorm;
            }
            else
            {
                rx = 2 * z * z * x * sqnorm;
                ry = 2 * z * z * y * sqnorm;
                rz = -2 * z * (x * x + y * y) * sqnorm;
            }
            SmoothConstrain(ref rx, ref ry, ref rz, f);
            dxs[i] += rx / (l * 2);
            dxs[i+1] += ry / (l * 2);
            dxs[i+2] += rz / (l * 2);
            dxs[j] -= rx / (l * 2);
            dxs[j+1] -= ry / (l * 2);
            dxs[j+2] -= rz / (l * 2);
        }
        void SmoothConstrain(ref double x, ref double y, ref double z, double f)
        {
            const double lower = 0.01;
            const double fade = 0.15;
            double sqL = x * x + y * y + z * z;
            double lf = f - fade;
            double uf = f + fade;
            if (sqL < lf * lf)
            {
            }
            else if (sqL < uf * uf)
            {
                double l = Math.Sqrt(sqL);
                double t = (l - lf) / (2 * fade);
                double newLength = (t * lower + (1 - t) * (f - fade));
                x *= newLength / l;
                y *= newLength / l;
                z *= newLength / l;
            }
            else
            {
                double l = Math.Sqrt(sqL);
                x *= lower / l;
                y *= lower / l;
                z *= lower / l;
            }
        }
    }
}
