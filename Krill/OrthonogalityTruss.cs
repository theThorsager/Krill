using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Krill
{
    internal class OrthonogalityTruss
    {
        public List<Point3d> nodes = new List<Point3d>();
        public List<Vector3d> gradients = new List<Vector3d>();
        public List<Tuple<int, int>> connections = new List<Tuple<int, int>>();

        public List<bool> lockedDOF = new List<bool>();

        public void LockDOFs(List<bool> locked)
        {
            lockedDOF = locked.ToList();
        }

        public double EvaluateTruss(double cutoff)
        {
            double sum = 0;
            for (int i = 0; i < connections.Count; i++)
            {
                sum += EvaluateElement(i, cutoff);
            }
            return sum;
        }

        public void SetGradients(double cutoff)
        {
            for (int i = 0; i < gradients.Count; i++)
            {
                gradients[i] = new Vector3d();
            }
            for (int i = 0; i < connections.Count; i++)
            {
                SetGradients(i, cutoff);
            }
        }

        public void ApplyGradient(double factor = 1)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                if (!lockedDOF[i])
                    nodes[i] += gradients[i] * factor;
            }
        }

        double EvaluateElement(int eIndex, double cutoff)
        {
            var a = nodes[connections[eIndex].Item1];
            var b = nodes[connections[eIndex].Item2];

            var vec = b - a;
            double norm = vec.SquareLength;
            double l = Math.Sqrt(norm);

            if (Math.Abs(vec.X) > Math.Abs(vec.Y) && Math.Abs(vec.X) > Math.Abs(vec.Z))
            {
                vec.X = 0;
            }
            else if (Math.Abs(vec.Y) > Math.Abs(vec.Z))
            {
                vec.Y = 0;
            }
            else
            {
                vec.Z = 0;
            }

            return Math.Min(cutoff * cutoff * 0.4, vec.SquareLength / (norm));
        }
        void SetGradients(int eIndex, double f)
        {
            int i = connections[eIndex].Item1;
            int j = connections[eIndex].Item2;
            var a = nodes[i];
            var b = nodes[j];

            var vec = b - a;
            double norm = vec.SquareLength;
            double l = Math.Sqrt(norm);
            double sqnorm = 1 / (norm * l);
            Vector3d res = new Vector3d();
            if (Math.Abs(vec.X) > Math.Abs(vec.Y) && Math.Abs(vec.X) > Math.Abs(vec.Z))
            {
                res.X = -2 * vec.X * (vec.Y * vec.Y + vec.Z * vec.Z) * sqnorm;
                res.Y = 2 * vec.X * vec.X * vec.Y * sqnorm;
                res.Z = 2 * vec.X * vec.X * vec.Z * sqnorm;
            }
            else if (Math.Abs(vec.Y) > Math.Abs(vec.Z))
            {
                res.X = 2 * vec.Y * vec.Y * vec.X * sqnorm;
                res.Y = -2 * vec.Y * (vec.X * vec.X + vec.Z * vec.Z) * sqnorm;
                res.Z = 2 * vec.Y * vec.Y * vec.Z * sqnorm;
            }
            else
            {
                res.X = 2 * vec.Z * vec.Z * vec.X * sqnorm;
                res.Y = 2 * vec.Z * vec.Z * vec.Y * sqnorm;
                res.Z = -2 * vec.Z * (vec.X * vec.X + vec.Y * vec.Y) * sqnorm;
            }
            res = SmoothConstrain(res, f) / (l * 2);
            gradients[i] += res;
            gradients[j] -= res;
        }
        Vector3d SmoothConstrain(Vector3d v, double f)
        {
            const double lower = 0.01;
            const double fade = 0.15;
            double lf = f - fade;
            double uf = f + fade;
            if (v.SquareLength < lf * lf)
            {
                return v;
            }
            else if (v.SquareLength < uf * uf)
            {
                double l = v.Length;
                double t = (l - lf) / (2 * fade);
                double newLength = (t * lower + (1 - t) * (f - fade));
                v *= newLength / l;
                return v;
            }
            else
            {
                v.Unitize();
                v *= lower;
                return v;
            }
        }
    }
}
