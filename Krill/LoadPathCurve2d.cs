using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class LoadPathCurve2d
    {
        const int maskbit = 0x000000FF;
        public Voxels2d<int> startVoxels;
        public Point2d startPt;
        public Vector2d startVec;
        public Voxels2d<Vector3d[]> princpDir;
        public Voxels2d<Vector3d> princpStress; 
        public Polyline loadPath;

        public Voxels2d<Vector3d> normals;

        private double d;
        private int[] nListForN;
        double nListRadius = 3.1;

        double stressCutOffLim;

        public LoadPathCurve2d(Voxels2d<int> startVoxels, Point3d startPt, Vector3d startVec,
                                Voxels2d<Vector3d[]> princpDir, Voxels2d<Vector3d> princpStress)
        {
            this.startVoxels = startVoxels;
            this.startPt = new Point2d(startPt.X, startPt.Y);
            this.startVec = new Vector2d(startVec.X, startVec.Y);
            this.princpDir = princpDir;
            this.princpStress = princpStress;

            d = startVoxels.delta;

            normals = new Voxels2d<Vector3d>(startVoxels.origin, d, startVoxels.n);
            
            nListForN = Utility.GetNeighbourOffsets2d(startVoxels.n, nListRadius);

            double maxPstress = double.MinValue;
            for (int i = 0; i < startVoxels.n * startVoxels.n; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;
                for (int j = 0; j < 2; j++)
                {
                    double curVal = Math.Abs(princpStress.cellValues[i][j]);
                    if (curVal > maxPstress)
                        maxPstress = curVal;
                }
            }

            stressCutOffLim = maxPstress / 500.0;
        }

        public void ConstructLoadPath(double scaleStep)
        {

            int maxNoStep = (int)1e4;   // Max number of steps

            Vector2d moveVec = startVec;
            moveVec.Unitize();
            Point2d x1 = startPt;
            List<Point2d> pathPts = new List<Point2d>();

            double step = d * scaleStep;

            pathPts.Add(x1);

            //x1 += step * moveVec;
            //pathPts.Add(x1);

            for (int i = 0; i < maxNoStep; i++)
            {
                Vector2d oldVec = moveVec;
                bool endCond;
                Point2d x0 = x1;
                Vector2d dfx0 = CLKFilterLinearMoveVec(x0, oldVec, out endCond);
                Vector2d aveDf = dfx0;

                Vector2d oldAveDf;

                //int oldSign = Math.Sign(LerpPstress(x0, dfx0));

                for (int j = 0; j < 10; j++)
                {
                    x1 = x0 + step * aveDf;
                    Vector2d dfx1 = CLKFilterLinearMoveVec(x1, dfx0, out endCond);
                    oldAveDf = aveDf;
                    aveDf = (dfx0 + dfx1) * 0.5;
                                  
                    if (j == 9)
                        break;

                    if ((aveDf - oldAveDf).SquareLength < 1e-18)
                        break;                    
                }

                moveVec = aveDf;

                x1 = x0 + step * moveVec;

                // If the stress at p-stress at point x1 is too low, break
                double pStress = Math.Abs(LerpPstress(x1, moveVec));
                if (pStress < stressCutOffLim)
                    break;

                //int newSign = Math.Sign(LerpPstress(x1, moveVec));

                //if (Math.Abs(oldSign - newSign) == 2)
                //    break;

                if (endCond)
                    break;

                pathPts.Add(x1);                
            }
            loadPath = new Polyline(pathPts.Select(x => new Point3d(x.X, x.Y, 0)));
        }

        public bool SecondaryLoadPath(double scaleStep, double tolerance, List<Point3d> nodes, out int index)
        {
            int maxNoStep = (int)1e4;   // Max number of steps

            index = 1000;

            Vector2d moveVec = startVec;
            moveVec.Unitize();
            Point2d x1 = startPt;
            List<Point3d> pathPts = new List<Point3d>();

            double step = d * scaleStep;

            Point3d startPt3d = new Point3d(startPt.X, startPt.Y, 0);

            pathPts.Add(startPt3d);

            //x1 += 2 * step * moveVec;)
            //pathPts.Add(x1);

            tolerance *= tolerance;

            for (int i = 0; i < maxNoStep; i++)
            {
                Vector2d oldVec = moveVec;
                bool endCond;
                Point2d x0 = x1;
                Vector2d dfx0 = CLKFilterLinearMoveVec(x0, oldVec, out endCond);
                Vector2d aveDf = dfx0;

                Vector2d oldAveDf;

                for (int j = 0; j < 10; j++)
                {
                    x1 = x0 + step * aveDf;
                    Vector2d dfx1 = CLKFilterLinearMoveVec(x1, dfx0, out endCond);
                    oldAveDf = aveDf;
                    aveDf = (dfx0 + dfx1) * 0.5;

                    if (j == 9)
                        break;

                    if ((aveDf - oldAveDf).SquareLength < 1e-18)
                        break;
                }

                moveVec = aveDf;

                x1 = x0 + step * moveVec;

                Point3d x1_3d = new Point3d(x1.X, x1.Y, 0);

                pathPts.Add(x1_3d);

                for (int j = 0; j < nodes.Count; j++)
                {
                    if (x1_3d.DistanceToSquared(nodes[j]) < tolerance && startPt3d.DistanceToSquared(nodes[j]) > tolerance)
                    {
                        index = j;
                        loadPath = new Polyline(pathPts);
                        return true;
                    }
                }

                if (endCond)
                {
                    loadPath = new Polyline(pathPts);
                    return false;
                }
            }
            loadPath = new Polyline(pathPts);
            return false;
        }

        private Vector2d CLKFilterLinearMoveVec(Point2d pos, Vector2d previousDir, out bool basicEnd)
        {
            Vector2d dir = new Vector2d();
            basicEnd = false;
            pos -= new Vector2d(startVoxels.origin.X, startVoxels.origin.Y);
            pos /= d;

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

            // Very basic break-statement
            if ((startVoxels.cellValues[INDi0j0k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k0] & maskbit) == 0 &&
                (startVoxels.cellValues[INDi0j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k0] & maskbit) == 0)
            {
                basicEnd = true;
                return Vector2d.Zero;
            }
            
            List<int> INDs = new List<int>();

            INDs.Add(INDi0j0k0);
            INDs.Add(INDi1j0k0);
            INDs.Add(INDi0j1k0);
            INDs.Add(INDi1j1k0);

            Vector2d[] dirLERP = new Vector2d[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < 4; i++)
            {
                if ((startVoxels.cellValues[INDs[i]] & maskbit) == 0)
                    inside[i] = false;
                else
                {
                    inside[i] = true;
                    dirLERP[i] = CorrectPrincpDir(INDs[i], previousDir);
                }                    
            }

            Vector2d avgPdir = new Vector2d();
            if (!inside[0] || !inside[1] || !inside[2] || !inside[3])
            {
                for (int i = 0; i < INDs.Count; i++)
                {
                    if (!inside[i])
                        continue;

                    avgPdir += CorrectPrincpDir(INDs[i], previousDir);
                }
                avgPdir.Unitize();
            }

            for (int i = 0; i < INDs.Count; i++)
            {
                if (inside[i])
                    continue;

                // Construct normal vector for that voxel
                Vector2d normal = new Vector2d();
                for (int ii = 0; ii < nListForN.Length; ii++)
                {
                    int j = INDs[i] + nListForN[ii];
                    if (j < startVoxels.cellValues.Length && startVoxels.cellValues[j] != 0)
                    {
                        Vector2d vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(INDs[i]);
                        if (vec.SquareLength < nListRadius * nListRadius)
                            normal += vec;
                    }

                    j = INDs[i] - nListForN[ii];
                    if (j >= 0 && startVoxels.cellValues[j] != 0)
                    {
                        Vector2d vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(INDs[i]);
                        if (vec.SquareLength < nListRadius * nListRadius)
                            normal += vec;
                    }
                }
                normal.Unitize();

                Vector3d norm3d = new Vector3d(normal.X, normal.Y, 0);
                normals.cellValues[INDs[i]] = norm3d;

                // Mirror the P-dir and assign value
                Vector2d mirrPdir = avgPdir - (avgPdir * normal) * normal * 2;

                // Check that it is in the right direction
                if (mirrPdir * previousDir < 0)
                    mirrPdir = -mirrPdir;

                dirLERP[i] = mirrPdir;
            }
            
            // Do the LERP
            dir.X = (1 - a) * (1 - b) * dirLERP[0].X
                    + a * (1 - b) * dirLERP[1].X
                    + (1 - a) * b * dirLERP[2].X
                    + a * b * dirLERP[3].X;

            dir.Y = (1 - a) * (1 - b) * dirLERP[0].Y
                    + a * (1 - b) * dirLERP[1].Y
                    + (1 - a) * b * dirLERP[2].Y
                    + a * b * dirLERP[3].Y;

            dir.Unitize();

            return dir;
        }

        private Vector2d CorrectPrincpDir(int indVoxel, Vector2d prevDir)
        {
            Vector3d[] pDir = new Vector3d[6];
            double[] cos = new double[6];
            double maxCos = double.MinValue;
            int maxInd = -1;
            Vector2d direction;

            Vector3d prevDir3d = new Vector3d(prevDir.X, prevDir.Y, 0);

            // This if-statement should never be true
            //if ((startVoxels.cellValues[indVoxel] & maskbit) == 0 && princpDir.cellValues[indVoxel] == null)
            //{
            //    //return Vector2d.Zero;
            //    return prevDir;
            //}

            for (int k = 0; k < 3; k++)
            {
                pDir[k] = princpDir.cellValues[indVoxel][k];
                pDir[k + 3] = -pDir[k];

                cos[k] = (prevDir3d * pDir[k]);
                cos[k + 3] = -cos[k];
            }

            for (int k = 0; k < 6; k++)
            {
                if (cos[k] > maxCos)
                {
                    maxCos = cos[k];
                    maxInd = k;
                }
            }

            direction = new Vector2d(pDir[maxInd].X, pDir[maxInd].Y);

            return direction;
        }

        private double LerpPstress(Point2d pos, Vector2d previousDir)
        {
            pos -= new Vector2d(startVoxels.origin.X, startVoxels.origin.Y);
            pos /= d;

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

            List<int> INDs = new List<int>();

            INDs.Add(INDi0j0k0);
            INDs.Add(INDi1j0k0);
            INDs.Add(INDi0j1k0);
            INDs.Add(INDi1j1k0);

            double[] valLERP = new double[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < 4; i++)
            {
                if ((startVoxels.cellValues[INDs[i]] & maskbit) == 0)
                    inside[i] = false;
                else
                {
                    inside[i] = true;
                    valLERP[i] = CorrectPrincpStress(INDs[i], previousDir);
                }
            }

            double avgVal = 0;
            double noInside = 0;
            if (!inside[0] || !inside[1] || !inside[2] || !inside[3])
            {
                for (int i = 0; i < INDs.Count; i++)
                {
                    if (!inside[i])
                        continue;

                    avgVal += CorrectPrincpStress(INDs[i], previousDir);
                    noInside++;
                }
                avgVal /= (double)noInside;
            }

            for (int i = 0; i < INDs.Count; i++)
            {
                if (inside[i])
                    continue;

                valLERP[i] = avgVal;
            }

            double pStress;

            pStress = (1 - a) * (1 - b) * valLERP[0]
                      + a * (1 - b) * valLERP[1]
                      + (1 - a) * b * valLERP[2]
                      + a * b * valLERP[3];

            return pStress;
        }

        private double CorrectPrincpStress(int indVoxel, Vector2d prevDir)
        {
            Vector3d[] pDir = new Vector3d[6];
            double[] cos = new double[6];
            double[] pStress = new double[6];
            double maxCos = double.MinValue;
            int maxInd = -1;

            Vector3d prevDir3d = new Vector3d(prevDir.X, prevDir.Y, 0);

            // Kolla närmare på vad som händer längst med en kant
            //if ((startVoxels.cellValues[indVoxel] & maskbit) == 0)
            //{
            //    return 0;
            //}

            for (int k = 0; k < 3; k++)
            {
                pDir[k] = princpDir.cellValues[indVoxel][k];
                pDir[k + 3] = -pDir[k];

                cos[k] = (prevDir3d * pDir[k]);
                cos[k + 3] = -cos[k];

                pStress[k] = princpStress.cellValues[indVoxel][k];
                pStress[k + 3] = pStress[k];
            }

            for (int k = 0; k < 6; k++)
            {
                if (cos[k] > maxCos)
                {
                    maxCos = cos[k];
                    maxInd = k;
                }
            }

            return pStress[maxInd];
        }
    }
}
