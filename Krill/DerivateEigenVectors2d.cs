using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class DerivateEigenVectors2d
    {
        const int maskbit = 0x000000FF;
        public Voxels2d<int> startVoxels;
        public Voxels2d<Matrix> stressTensor;
        public Voxels2d<Vector3d> princpStress;
        public Voxels2d<Vector3d[]> princpDir;

        public DerivateEigenVectors2d(Voxels2d<int> startVoxels, Voxels2d<Matrix> stressTensor, Voxels2d<Vector3d> princpStress, Voxels2d<Vector3d[]> princpDir)
        {
            this.startVoxels = startVoxels;
            this.stressTensor = stressTensor;
            this.princpStress = princpStress;
            this.princpDir = princpDir;
        }

        public Vector3d[] DerEigenVec(int ind)
        {
            Vector3d eigenVal = princpStress.cellValues[ind];
            Vector3d[] eigenVecs = princpDir.cellValues[ind];

            Vector3d[] derEigenVec = new Vector3d[2];

            Point2d pos = startVoxels.IndexToPoint(ind);

            for (int i = 0; i < 2; i++)
            {
                Matrix dB = StressLerpDer(pos, eigenVecs[i]);
                Matrix ni = new Matrix(2, 1);
                ni[0, 0] = eigenVecs[i].X;
                ni[1, 0] = eigenVecs[i].Y;

                Matrix dBni = dB * ni;

                Vector3d dBniVec = new Vector3d();
                dBniVec.X = dBni[0, 0];
                dBniVec.Y = dBni[1, 0];

                for (int j = 0; j < 2; j++)
                {
                    if (i == j)
                        continue;

                    derEigenVec[i] += 1 / (eigenVal[i] - eigenVal[j]) * (dBniVec * eigenVecs[j]) * eigenVecs[j];
                }
            }
            return derEigenVec;
        }

        private Matrix StressLerpDer(Point2d pos, Vector3d ni)
        {
            Matrix derStress = new Matrix(2, 2);

            pos -= new Vector2d(startVoxels.origin.X, startVoxels.origin.Y);
            pos /= startVoxels.delta;

            double u = pos.X;
            double v = pos.Y;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int i1 = (int)Math.Floor(u - 0.5) + 1;
            int j1 = (int)Math.Floor(v - 0.5) + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);

            Coord i0j0k0 = new Coord(i0, j0);
            Coord i1j0k0 = new Coord(i1, j0);
            Coord i0j1k0 = new Coord(i0, j1);
            Coord i1j1k0 = new Coord(i1, j1);

            int INDi0j0k0 = startVoxels.CoordToIndex(i0j0k0);
            int INDi1j0k0 = startVoxels.CoordToIndex(i1j0k0);
            int INDi0j1k0 = startVoxels.CoordToIndex(i0j1k0);
            int INDi1j1k0 = startVoxels.CoordToIndex(i1j1k0);

            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    derStress[i, j] = ((-1 + b) * TensorVal(INDi0j0k0, i, j)
                              + (1 - b) * TensorVal(INDi1j0k0, i ,j)
                              - b * TensorVal(INDi0j1k0, i, j)
                              + b * TensorVal(INDi1j1k0, i ,j)) * ni.X

                              + ((-1 + a) * TensorVal(INDi0j0k0, i, j)
                              - a * TensorVal(INDi1j0k0, i, j)
                              + (1 - a) * TensorVal(INDi0j1k0, i, j)
                              + a * TensorVal(INDi1j1k0, i, j)) * ni.Y;
                }
            }
            return derStress;
        }

        private double TensorVal(int ind, int i, int j)
        {
            double val;

            if ((startVoxels.cellValues[ind] & maskbit) == 0)
                val = 0;
            else            
                val = stressTensor.cellValues[ind][i, j];
            
            return val;
        }
    }
}
