using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    internal class VoxelSubgradient
    {
        public Voxels<double> voxels;

        public VoxelSubgradient(Voxels<double> voxels)
        {
            this.voxels = voxels;
        }

        double LERP(double factor, double a, double b)
        {
            return b * factor + a * (1 - factor);
        }

        public Vector3d SubGradientAt(Point3d pos)
        {
            pos -= (Vector3d)voxels.origin;
            pos /= voxels.delta;

            double u = pos.X;
            double v = pos.Y;
            double w = pos.Z;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int k0 = (int)Math.Floor(w - 0.5);

            double X, Y, Z;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);
            // if x close to zero, lerp X, etc
            const double border = 0.05;  /* 0 < border < 0.5 */
            const double F = 0.5 / border;
            if (a < border)
                X = LERP((border + a) * F, SubGradientXAt(i0 - 1, j0, k0, 1, v, w), SubGradientXAt(i0, j0, k0, 0, v, w));
            else if (a > 1.0 - border)
                X = LERP((1.0 + border - a) * F, SubGradientXAt(i0 + 1, j0, k0, 0, v, w), SubGradientXAt(i0, j0, k0, 1, v, w));
            else
                X = SubGradientXAt(i0, j0, k0, u, v, w);

            if (b < border)
                Y = LERP((border + b) * F, SubGradientYAt(i0, j0 - 1, k0, u, 0, w), SubGradientYAt(i0, j0, k0, u, 0, w));
            else if (b > 1.0 - border)
                Y = LERP((1.0 + border - b) * F, SubGradientYAt(i0, j0 + 1, k0, u, 0, w), SubGradientYAt(i0, j0, k0, u, 0, w));
            else
                Y = SubGradientYAt(i0, j0, k0, u, v, w);

            if (c < border)
                Z = LERP((border + c) * F, SubGradientZAt(i0, j0, k0 - 1, u, v, 0), SubGradientZAt(i0, j0, k0, u, v, 0));
            else if (c > 1.0 - border)
                Z = LERP((1.0 + border - c) * F, SubGradientZAt(i0, j0, k0 + 1, u, v, 0), SubGradientZAt(i0, j0, k0, u, v, 0));
            else
                Z = SubGradientZAt(i0, j0, k0, u, v, w);

            return new Vector3d(X, Y, Z);
        }

        double SubGradientXAt(int i0, int j0, int k0, double u, double v, double w)
        {
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

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

            return -(1 - b) * (1 - c) * voxels.cellValues[INDi0j0k0]
                   + (1 - b) * (1 - c) * voxels.cellValues[INDi1j0k0]
                   - b * (1 - c) * voxels.cellValues[INDi0j1k0]
                   + b * (1 - c) * voxels.cellValues[INDi1j1k0]
                   - (1 - b) * c * voxels.cellValues[INDi0j0k1]
                   + (1 - b) * c * voxels.cellValues[INDi1j0k1]
                   - b * c * voxels.cellValues[INDi0j1k1]
                   + b * c * voxels.cellValues[INDi1j1k1];
        }


        double SubGradientYAt(int i0, int j0, int k0, double u, double v, double w)
        {
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            int INDi0j0k0 = voxels.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = voxels.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = voxels.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = voxels.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = voxels.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = voxels.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = voxels.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = voxels.ToLinearIndex(i1, j1, k1);

            return -(1 - a) * (1 - c) * voxels.cellValues[INDi0j0k0]
                   - a * (1 - c) * voxels.cellValues[INDi1j0k0]
                   + (1 - a) * (1 - c) * voxels.cellValues[INDi0j1k0]
                   + a * (1 - c) * voxels.cellValues[INDi1j1k0]
                   - (1 - a) * c * voxels.cellValues[INDi0j0k1]
                   - a * c * voxels.cellValues[INDi1j0k1]
                   + (1 - a) * c * voxels.cellValues[INDi0j1k1]
                   + a * c * voxels.cellValues[INDi1j1k1];
        }

        double SubGradientZAt(int i0, int j0, int k0, double u, double v, double w)
        {
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);

            int INDi0j0k0 = voxels.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = voxels.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = voxels.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = voxels.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = voxels.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = voxels.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = voxels.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = voxels.ToLinearIndex(i1, j1, k1);

            return -(1 - a) * (1 - b) * voxels.cellValues[INDi0j0k0]
                    - a * (1 - b) * voxels.cellValues[INDi1j0k0]
                    - (1 - a) * b * voxels.cellValues[INDi0j1k0]
                    - a * b * voxels.cellValues[INDi1j1k0]
                    + (1 - a) * (1 - b) * voxels.cellValues[INDi0j0k1]
                    + a * (1 - b) * voxels.cellValues[INDi1j0k1]
                    + (1 - a) * b * voxels.cellValues[INDi0j1k1]
                    + a * b * voxels.cellValues[INDi1j1k1];
        }

        /*
        Vector3d SubGradientAt(int i0, int j0, int k0, double u, double v, double w)
        {
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            int INDi0j0k0 = weights.ToLinearIndex(i0, j0, k0);
            int INDi1j0k0 = weights.ToLinearIndex(i1, j0, k0);
            int INDi0j1k0 = weights.ToLinearIndex(i0, j1, k0);
            int INDi1j1k0 = weights.ToLinearIndex(i1, j1, k0);
            int INDi0j0k1 = weights.ToLinearIndex(i0, j0, k1);
            int INDi1j0k1 = weights.ToLinearIndex(i1, j0, k1);
            int INDi0j1k1 = weights.ToLinearIndex(i0, j1, k1);
            int INDi1j1k1 = weights.ToLinearIndex(i1, j1, k1);

            Vector3d result = new Vector3d
            {
                X = -(1 - b) * (1 - c) * weights.cellValues[INDi0j0k0]
                       + (1 - b) * (1 - c) * weights.cellValues[INDi1j0k0]
                       - b * (1 - c) * weights.cellValues[INDi0j1k0]
                       + b * (1 - c) * weights.cellValues[INDi1j1k0]
                       - (1 - b) * c * weights.cellValues[INDi0j0k1]
                       + (1 - b) * c * weights.cellValues[INDi1j0k1]
                       - b * c * weights.cellValues[INDi0j1k1]
                       + b * c * weights.cellValues[INDi1j1k1],

                Y = -(1 - a) * (1 - c) * weights.cellValues[INDi0j0k0]
                       - a * (1 - c) * weights.cellValues[INDi1j0k0]
                       + (1 - a) * (1 - c) * weights.cellValues[INDi0j1k0]
                       + a * (1 - c) * weights.cellValues[INDi1j1k0]
                       - (1 - a) * c * weights.cellValues[INDi0j0k1]
                       - a * c * weights.cellValues[INDi1j0k1]
                       + (1 - a) * c * weights.cellValues[INDi0j1k1]
                       + a * c * weights.cellValues[INDi1j1k1],

                Z = -(1 - a) * (1 - b) * weights.cellValues[INDi0j0k0]
                       - a * (1 - b) * weights.cellValues[INDi1j0k0]
                       - (1 - a) * b * weights.cellValues[INDi0j1k0]
                       - a * b * weights.cellValues[INDi1j1k0]
                       + (1 - a) * (1 - b) * weights.cellValues[INDi0j0k1]
                       + a * (1 - b) * weights.cellValues[INDi1j0k1]
                       + (1 - a) * b * weights.cellValues[INDi0j1k1]
                       + a * b * weights.cellValues[INDi1j1k1]
            };

            return result;
        }
        */

    }
}
