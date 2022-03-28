using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class StressLine2d
    {
        const int maskbit = 0x000000FF;

        private Voxels2d<int> startVoxels;
        private Voxels2d<Matrix> stressTensor;

        public StressLine2d(Containers.PostProcessingResults2d post)
        {
            startVoxels = post.mask;
            //OutputResults2d results = new OutputResults2d(post);
            //results.UpdateFakeStrains(post.displacments);
            //results.UpdateStresses();

            int noVoxels = startVoxels.n * startVoxels.n;

            stressTensor = new Voxels2d<Matrix>(startVoxels.origin, startVoxels.delta, startVoxels.n);

            for (int i = 0; i < noVoxels; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;

                Matrix tensor = new Matrix(2, 2);
                tensor[0, 0] = post.stressXX.cellValues[i];
                tensor[0, 1] = post.stressXY.cellValues[i];

                tensor[1, 0] = post.stressXY.cellValues[i];
                tensor[1, 1] = post.stressYY.cellValues[i];

                stressTensor.cellValues[i] = tensor;
            }

        }

        public List<double> LERPstressAlongLine(Line line)
        {
            List<double> stressList = new List<double>();
            Vector3d n = line.Direction;
            n.Unitize();

            Matrix dir = new Matrix(2, 1);
            dir[0, 0] = n.X;
            dir[1, 0] = n.Y;

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
            pos -= new Vector3d(startVoxels.origin.X, startVoxels.origin.Y, 0);
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

            Matrix tensorLERP = new Matrix(2, 2);

            for (int row = 0; row < 2; row++)
            {
                for (int col = 0; col < 2; col++)
                {
                    tensorLERP[row, col] = (1 - a) * (1 - b) * tensorValue(INDi0j0k0, row, col)
                                            + a * (1 - b) * tensorValue(INDi1j0k0, row, col)
                                            + (1 - a) * b * tensorValue(INDi0j1k0, row, col)
                                            + a * b * tensorValue(INDi1j1k0, row, col);
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
