using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class StressLine
    {
        const int maskbit = 0x000000FF;

        private Voxels<int> startVoxels;
        private Voxels<Matrix> stressTensor;

        public StressLine(Containers.LinearSolution linSol)
        {
            startVoxels = linSol.mask;
            OutputResults results = new OutputResults(linSol);
            results.UpdateFakeStrains(linSol.displacments);
            results.UpdateStresses();

            int noVoxels = startVoxels.n * startVoxels.n * startVoxels.n;

            stressTensor = new Voxels<Matrix>(startVoxels.origin, startVoxels.delta, startVoxels.n);

            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix tensor = new Matrix(3, 3);
                tensor[0, 0] = results.stressXX.cellValues[i];
                tensor[0, 1] = results.stressXY.cellValues[i];
                tensor[0, 2] = results.stressXZ.cellValues[i];

                tensor[1, 0] = results.stressXY.cellValues[i];
                tensor[1, 1] = results.stressYY.cellValues[i];
                tensor[1, 2] = results.stressYZ.cellValues[i];

                tensor[2, 0] = results.stressXZ.cellValues[i];
                tensor[2, 1] = results.stressYZ.cellValues[i];
                tensor[2, 2] = results.stressZZ.cellValues[i];

                stressTensor.cellValues[i] = tensor;
            }

        }

        public List<double> LERPstressAlongLine(Line line)
        {
            List<double> stressList = new List<double>();
            Vector3d n = line.Direction;
            n.Unitize();

            Matrix dir = new Matrix(3, 1);
            dir[0, 0] = n.X;
            dir[1, 0] = n.Y;
            dir[2, 0] = n.Z;

            Matrix dirT = dir.Duplicate();
            dirT.Transpose();

            Point3d pos = line.From;
            double noSteps = 100;
            double step = line.Length / noSteps;

            for (int i = 0; i < noSteps; i++)
            {
                Matrix tensor = LERPstress(pos);

                Matrix s = dirT * tensor * dir;

                double stress = s[0, 0];

                stressList.Add(stress);

                pos += n * step;

            }
            return stressList;
        }

        Matrix LERPstress(Point3d pos)
        {
            pos -= (Vector3d)startVoxels.origin;
            pos /= startVoxels.delta;

            double u = pos.X;
            double v = pos.Y;
            double w = pos.Z;

            int i0 = (int)Math.Floor(u - 0.5);
            int j0 = (int)Math.Floor(v - 0.5);
            int k0 = (int)Math.Floor(w - 0.5);
            int i1 = (int)Math.Floor(u - 0.5) + 1;
            int j1 = (int)Math.Floor(v - 0.5) + 1;
            int k1 = (int)Math.Floor(w - 0.5) + 1;

            double a = (u - 0.5) - Math.Floor(u - 0.5);
            double b = (v - 0.5) - Math.Floor(v - 0.5);
            double c = (w - 0.5) - Math.Floor(w - 0.5);

            Coord i0j0k0 = new Coord(i0, j0, k0);
            Coord i1j0k0 = new Coord(i1, j0, k0);
            Coord i0j1k0 = new Coord(i0, j1, k0);
            Coord i1j1k0 = new Coord(i1, j1, k0);
            Coord i0j0k1 = new Coord(i0, j0, k1);
            Coord i1j0k1 = new Coord(i1, j0, k1);
            Coord i0j1k1 = new Coord(i0, j1, k1);
            Coord i1j1k1 = new Coord(i1, j1, k1);

            int INDi0j0k0 = startVoxels.CoordToIndex(i0j0k0);
            int INDi1j0k0 = startVoxels.CoordToIndex(i1j0k0);
            int INDi0j1k0 = startVoxels.CoordToIndex(i0j1k0);
            int INDi1j1k0 = startVoxels.CoordToIndex(i1j1k0);
            int INDi0j0k1 = startVoxels.CoordToIndex(i0j0k1);
            int INDi1j0k1 = startVoxels.CoordToIndex(i1j0k1);
            int INDi0j1k1 = startVoxels.CoordToIndex(i0j1k1);
            int INDi1j1k1 = startVoxels.CoordToIndex(i1j1k1);

            Matrix tensorLERP = new Matrix(3, 3);

            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++)
                {
                    tensorLERP[row, col] = (1 - a) * (1 - b) * (1 - c) * tensorValue(INDi0j0k0, row, col)
                                            + a * (1 - b) * (1 - c) * tensorValue(INDi1j0k0, row, col)
                                            + (1 - a) * b * (1 - c) * tensorValue(INDi0j1k0, row, col)
                                            + a * b * (1 - c) * tensorValue(INDi1j1k0, row, col)
                                            + (1 - a) * (1 - b) * c * tensorValue(INDi0j0k1, row, col)
                                            + a * (1 - b) * c * tensorValue(INDi1j0k1, row, col)
                                            + (1 - a) * b * c * tensorValue(INDi0j1k1, row, col)
                                            + a * b * c * tensorValue(INDi1j1k1, row, col);
                }
            }
            return tensorLERP;
        }

        private double tensorValue(int ind, int row, int col)
        {
            double val;
            if ((startVoxels.cellValues[ind] & maskbit) == 0)
                val = 0;
            else
                val = stressTensor.cellValues[ind][row, col];

            return val;
        }
    }
}
