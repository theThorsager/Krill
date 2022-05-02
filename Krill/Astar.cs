using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Krill
{
    internal class Astar
    {
        public Voxels<double> weigths;
        public int[] offsets;

        public Astar(Voxels<double> weigths)
        {
            this.weigths = weigths;
            offsets = Utility.GetNeighbourOffsets(weigths.n, Math.Sqrt(2) + 0.001);
        }

        private List<int> reconstructPath(Dictionary<int, int> cameFrom, int current)
        {
            var path = new List<int>() { current };
            while (cameFrom[current] != -1)
            {
                current = cameFrom[current];
                path.Add(current);
            }
            return path;
        }

        private double h(int i, int to)
        {
            return (weigths.IndexToPoint(i) - weigths.IndexToPoint(to)).Length;
        }

        private double d(int current, int neigbour)
        {
            return (weigths.IndexToPoint(neigbour) - weigths.IndexToPoint(current)).Length + 
                Math.Abs(weigths.cellValues[neigbour] - weigths.cellValues[current]);
        }

        bool inside(int i)
        {
            int n = weigths.n;
            weigths.To3DIndex(ref i, out int j, out int k);
            return i >= 0 && i < n &&
                   j >= 0 && j < n &&
                   k >= 0 && k < n;
        }

        public List<int> FindPath(int from, int to)
        {
            var openSet = new List<int>() { from };
            var cameFrom = new Dictionary<int, int>();
            cameFrom[from] = -1;

            var gscore = new Dictionary<int, Tuple<double, double>>();
            gscore[from] = new Tuple<double, double>(0, h(from, to));

            while (openSet.Count > 0)
            {
                int current = openSet[0];
                int opensetIndex = 0;
                double min = gscore[current].Item2;
                for (int i = 1; i < openSet.Count; i++)
                {
                    if (gscore[openSet[i]].Item2 < min)
                    {
                        current = openSet[i];
                        opensetIndex = i;
                        min = gscore[current].Item2;
                    }
                }

                if (current == to)
                    return reconstructPath(cameFrom, current);

                openSet.RemoveAt(opensetIndex);
                for (int jj = 0; jj < offsets.Length; jj++)
                {
                    int neighbour = current + offsets[jj];
                    if (inside(neighbour))
                    {
                        double tenativeGscore = gscore[current].Item1 + d(current, neighbour);
                        bool got = gscore.TryGetValue(neighbour, out var score);
                        if (!got || tenativeGscore < score.Item1)
                        {
                            cameFrom[neighbour] = current;
                            gscore[neighbour] = new Tuple<double, double>(tenativeGscore, tenativeGscore + h(neighbour, to));
                            if (!openSet.Contains(neighbour))
                                openSet.Add(neighbour);
                        }
                    }

                    neighbour = current - offsets[jj];
                    if (inside(neighbour))
                    {
                        double tenativeGscore = gscore[current].Item1 + d(current, neighbour);
                        bool got = gscore.TryGetValue(neighbour, out var score);
                        if (!got || tenativeGscore < score.Item1)
                        {
                            cameFrom[neighbour] = current;
                            gscore[neighbour] = new Tuple<double, double>(tenativeGscore, tenativeGscore + h(neighbour, to));
                            if (!openSet.Contains(neighbour))
                                openSet.Add(neighbour);
                        }
                    }
                }
            }

            return null;
        }


    }
}
