using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class LoadPathCurve
    {
        const int maskbit = 0x000000FF;
        public Voxels<int> startVoxels;
        public Voxels<Vector3d[]> princpDir;
        public Voxels<Vector3d> princpStress;

        public Voxels<Vector3d> normals;

        private double d;
        private int[] nListForN;
        double nListRadius = 3.1;

        double stressCutOffLim;

        public LoadPathCurve(Voxels<int> startVoxels, Voxels<Vector3d[]> princpDir, Voxels<Vector3d> princpStress)
        {
            this.startVoxels = startVoxels;
            this.princpDir = princpDir;
            this.princpStress = princpStress;

            d = startVoxels.delta;

            normals = new Voxels<Vector3d>(startVoxels.origin, d, startVoxels.n);

            nListForN = Utility.GetNeighbourOffsets(startVoxels.n, nListRadius);

            double maxPstress = double.MinValue;
            for (int i = 0; i < startVoxels.n * startVoxels.n * startVoxels.n; i++)
            {
                if ((startVoxels.cellValues[i] & maskbit) == 0)
                    continue;
                for (int j = 0; j < 3; j++)
                {
                    double curVal = Math.Abs(princpStress.cellValues[i][j]);
                    if (curVal > maxPstress)
                        maxPstress = curVal;
                }
            }

            stressCutOffLim = maxPstress / 5000.0;
        }

        public Polyline ConstructLoadPath(Point3d startPt, Vector3d startVec, double scaleStep, bool incSignChange)
        {

            int maxNoStep = (int)1e3;   // Max number of steps

            Vector3d moveVec = startVec;
            moveVec.Unitize();
            Point3d x1 = startPt;
            List<Point3d> pathPts = new List<Point3d>();

            double step = d * scaleStep;

            pathPts.Add(x1);

            //x1 += step * moveVec;
            //pathPts.Add(x1);

            for (int i = 0; i < maxNoStep; i++)
            {
                Vector3d oldVec = moveVec;
                bool endCond;
                Point3d x0 = x1;
                Vector3d dfx0 = CLKFilterLinearMoveVec(x0, oldVec, out endCond);
                Vector3d aveDf = dfx0;

                Vector3d oldAveDf;

                int oldSign = Math.Sign(LerpPstress(x0, dfx0));

                for (int j = 0; j < 10; j++)
                {
                    x1 = x0 + step * aveDf;
                    Vector3d dfx1 = CLKFilterLinearMoveVec(x1, dfx0, out endCond);
                    oldAveDf = aveDf;
                    aveDf = (dfx0+ dfx1)*0.5;
                                  
                    if (j == 9)
                        break;

                    if ((aveDf - oldAveDf).SquareLength < 1e-18)
                        break;                    
                }

                moveVec = aveDf;

                x1 = x0 + step * moveVec;

                if (endCond)
                    break;

                pathPts.Add(x1);

                // Is this break-statement creating more problems than it solves? For some problems it seems to work better without it
                // If the stress at p-stress at point x1 is too low, break
                double pStress = Math.Abs(LerpPstress(x1, moveVec));
                if (pStress < stressCutOffLim)
                    break;

                // If it loops around to the same position, end it
                if (i > 10 && x1.DistanceToSquared(startPt) < (step * 2) * (step * 2))
                    break;

                int newSign = Math.Sign(LerpPstress(x1, moveVec));

                if (incSignChange && Math.Abs(oldSign - newSign) == 2)
                    break;

            }
            return new Polyline(pathPts);
        }

        //public bool SecondaryLoadPath(double scaleStep, double tolerance, List<Point3d> nodes, out int index)
        //{
        //    int maxNoStep = (int)1e4;   // Max number of steps

        //    index = 1000;

        //    Vector3d moveVec = startVec;
        //    moveVec.Unitize();
        //    Point3d x1 = startPt;
        //    List<Point3d> pathPts = new List<Point3d>();

        //    double step = d * scaleStep;

        //    pathPts.Add(x1);

        //    x1 += 2 * step * moveVec;

        //    pathPts.Add(x1);

        //    tolerance *= tolerance;

        //    for (int i = 0; i < maxNoStep; i++)
        //    {
        //        Vector3d oldVec = moveVec;
        //        bool endCond;
        //        Point3d x0 = x1;
        //        Vector3d dfx0 = CLKFilterLinearMoveVec(x0, oldVec, out endCond);
        //        Vector3d aveDf = dfx0;

        //        Vector3d oldAveDf;

        //        for (int j = 0; j < 10; j++)
        //        {
        //            x1 = x0 + step * aveDf;
        //            Vector3d dfx1 = CLKFilterLinearMoveVec(x1, dfx0, out endCond);
        //            oldAveDf = aveDf;
        //            aveDf = (dfx0 + dfx1) * 0.5;

        //            if (j == 9)
        //                break;

        //            if ((aveDf - oldAveDf).SquareLength < 1e-18)
        //                break;
        //        }

        //        moveVec = aveDf;

        //        x1 = x0 + step * moveVec;

        //        pathPts.Add(x1);

        //        for (int j = 0; j < nodes.Count; j++)
        //        {
        //            if (x1.DistanceToSquared(nodes[j]) < tolerance && startPt.DistanceToSquared(nodes[j]) > tolerance)
        //            {
        //                index = j;
        //                loadPath = new Polyline(pathPts);
        //                return true;
        //            }
        //        }

        //        if (endCond)
        //        {
        //            loadPath = new Polyline(pathPts);
        //            return false;
        //        }                    
        //    }
        //    loadPath = new Polyline(pathPts);
        //    return false;
        //}

        private Vector3d CLKFilterLinearMoveVec(Point3d pos, Vector3d previousDir, out bool basicEnd)
        {
            Vector3d dir = new Vector3d();
            basicEnd = false;
            pos -= (Vector3d)startVoxels.origin;
            pos /= d;

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

            // Väldigt basic break-villkor, uppdatera och implementera ett bättre
            if ((startVoxels.cellValues[INDi0j0k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j0k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k1] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k1] & maskbit) == 0)
            {
                basicEnd = true;
                return Vector3d.Zero;
            }

            List<int> INDs = new List<int>();

            INDs.Add(INDi0j0k0);
            INDs.Add(INDi1j0k0);
            INDs.Add(INDi0j1k0);
            INDs.Add(INDi1j1k0);
            INDs.Add(INDi0j0k1);
            INDs.Add(INDi1j0k1);
            INDs.Add(INDi0j1k1);
            INDs.Add(INDi1j1k1);

            Vector3d[] dirLERP = new Vector3d[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < INDs.Count; i++)
            {
                if ((startVoxels.cellValues[INDs[i]] & maskbit) == 0)
                    inside[i] = false;
                else
                {
                    inside[i] = true;
                    dirLERP[i] = CorrectPrincpDir(INDs[i], previousDir);
                }
            }

            Vector3d avgPdir = new Vector3d();
            if (!inside[0] || !inside[1] || !inside[2] || !inside[3] ||
                !inside[4] || !inside[5] || !inside[6] || !inside[7])
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
                Vector3d normal = new Vector3d();
                for (int ii = 0; ii < nListForN.Length; ii++)
                {
                    int j = INDs[i] + nListForN[ii];
                    if (j < startVoxels.cellValues.Length && startVoxels.cellValues[j] != 0)
                    {
                        Vector3d vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(INDs[i]);
                        if (vec.SquareLength < nListRadius * nListRadius)
                            normal += vec;
                    }

                    j = INDs[i] - nListForN[ii];
                    if (j >= 0 && startVoxels.cellValues[j] != 0)
                    {
                        Vector3d vec = startVoxels.IndexToPoint(j) - startVoxels.IndexToPoint(INDs[i]);
                        if (vec.SquareLength < nListRadius * nListRadius)
                            normal += vec;
                    }
                }
                normal.Unitize();
                normals.cellValues[INDs[i]] = normal;

                // Mirror the P-dir and assign value
                Vector3d mirrPdir = avgPdir - (avgPdir * normal) * normal * 2;

                // Check that it is in the right direction
                if (mirrPdir * previousDir < 0)
                    mirrPdir = -mirrPdir;

                dirLERP[i] = mirrPdir;
            }

            dir.X = (1 - a) * (1 - b) * (1 - c) * dirLERP[0].X
                    + a * (1 - b) * (1 - c) * dirLERP[1].X
                    + (1 - a) * b * (1 - c) * dirLERP[2].X
                    + a * b * (1 - c) * dirLERP[3].X
                    + (1 - a) * (1 - b) * c * dirLERP[4].X
                    + a * (1 - b) * c * dirLERP[5].X
                    + (1 - a) * b * c * dirLERP[6].X
                    + a * b * c * dirLERP[7].X;

            dir.Y = (1 - a) * (1 - b) * (1 - c) * dirLERP[0].Y
                    + a * (1 - b) * (1 - c) * dirLERP[1].Y
                    + (1 - a) * b * (1 - c) * dirLERP[2].Y
                    + a * b * (1 - c) * dirLERP[3].Y
                    + (1 - a) * (1 - b) * c * dirLERP[4].Y
                    + a * (1 - b) * c * dirLERP[5].Y
                    + (1 - a) * b * c * dirLERP[6].Y
                    + a * b * c * dirLERP[7].Y;

            dir.Z = (1 - a) * (1 - b) * (1 - c) * dirLERP[0].Z
                    + a * (1 - b) * (1 - c) * dirLERP[1].Z
                    + (1 - a) * b * (1 - c) * dirLERP[2].Z
                    + a * b * (1 - c) * dirLERP[3].Z
                    + (1 - a) * (1 - b) * c * dirLERP[4].Z
                    + a * (1 - b) * c * dirLERP[5].Z
                    + (1 - a) * b * c * dirLERP[6].Z
                    + a * b * c * dirLERP[7].Z;

            dir.Unitize();
            return dir;
        }

        private Vector3d CLKFilterLinearMoveVecOld(Point3d pos, Vector3d previousDir, out bool basicEnd)
        {
            Vector3d dir = new Vector3d();
            basicEnd = false;
            pos -= (Vector3d)startVoxels.origin;
            pos /= d;

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

            // Väldigt basic break-villkor, uppdatera och implementera ett bättre
            if ((startVoxels.cellValues[INDi0j0k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j0k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k1] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k1] & maskbit) == 0)
                basicEnd = true;

            dir.X = (1 - a) * (1 - b) * (1 - c) * CorrectPrincpDir(INDi0j0k0, previousDir).X
                    + a * (1 - b) * (1 - c) * CorrectPrincpDir(INDi1j0k0, previousDir).X
                    + (1 - a) * b * (1 - c) * CorrectPrincpDir(INDi0j1k0, previousDir).X
                    + a * b * (1 - c) * CorrectPrincpDir(INDi1j1k0, previousDir).X
                    + (1 - a) * (1 - b) * c * CorrectPrincpDir(INDi0j0k1, previousDir).X
                    + a * (1 - b) * c * CorrectPrincpDir(INDi1j0k1, previousDir).X
                    + (1 - a) * b * c * CorrectPrincpDir(INDi0j1k1, previousDir).X
                    + a * b * c * CorrectPrincpDir(INDi1j1k1, previousDir).X;

            dir.Y = (1 - a) * (1 - b) * (1 - c) * CorrectPrincpDir(INDi0j0k0, previousDir).Y
                    + a * (1 - b) * (1 - c) * CorrectPrincpDir(INDi1j0k0, previousDir).Y
                    + (1 - a) * b * (1 - c) * CorrectPrincpDir(INDi0j1k0, previousDir).Y
                    + a * b * (1 - c) * CorrectPrincpDir(INDi1j1k0, previousDir).Y
                    + (1 - a) * (1 - b) * c * CorrectPrincpDir(INDi0j0k1, previousDir).Y
                    + a * (1 - b) * c * CorrectPrincpDir(INDi1j0k1, previousDir).Y
                    + (1 - a) * b * c * CorrectPrincpDir(INDi0j1k1, previousDir).Y
                    + a * b * c * CorrectPrincpDir(INDi1j1k1, previousDir).Y;

            dir.Z = (1 - a) * (1 - b) * (1 - c) * CorrectPrincpDir(INDi0j0k0, previousDir).Z
                    + a * (1 - b) * (1 - c) * CorrectPrincpDir(INDi1j0k0, previousDir).Z
                    + (1 - a) * b * (1 - c) * CorrectPrincpDir(INDi0j1k0, previousDir).Z
                    + a * b * (1 - c) * CorrectPrincpDir(INDi1j1k0, previousDir).Z
                    + (1 - a) * (1 - b) * c * CorrectPrincpDir(INDi0j0k1, previousDir).Z
                    + a * (1 - b) * c * CorrectPrincpDir(INDi1j0k1, previousDir).Z
                    + (1 - a) * b * c * CorrectPrincpDir(INDi0j1k1, previousDir).Z
                    + a * b * c * CorrectPrincpDir(INDi1j1k1, previousDir).Z;

            dir.Unitize();
            return dir;
        }

        private Vector3d CorrectPrincpDir(int indVoxel, Vector3d prevDir)
        {
            Vector3d[] pDir = new Vector3d[6];
            double[] cos = new double[6];
            double maxCos = double.MinValue;
            int maxInd = -1;
            Vector3d direction;

            // This if-statement should never be true
            //if ((startVoxels.cellValues[indVoxel] & maskbit) == 0)
            //{
            //    direction = prevDir;
            //    return direction;
            //}                
            
            for (int k = 0; k < 3; k++)
            {
                pDir[k] = princpDir.cellValues[indVoxel][k];
                pDir[k + 3] = -pDir[k];

                cos[k] = prevDir * pDir[k];
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

            direction = pDir[maxInd];
            return direction;
        }

        private double LerpPstress(Point3d pos, Vector3d previousDir)
        {
            pos -= (Vector3d)startVoxels.origin;
            pos /= d;

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

            if ((startVoxels.cellValues[INDi0j0k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j0k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k1] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k1] & maskbit) == 0)
            {
                return 0;
            }

            List<int> INDs = new List<int>();

            INDs.Add(INDi0j0k0);
            INDs.Add(INDi1j0k0);
            INDs.Add(INDi0j1k0);
            INDs.Add(INDi1j1k0);
            INDs.Add(INDi0j0k1);
            INDs.Add(INDi1j0k1);
            INDs.Add(INDi0j1k1);
            INDs.Add(INDi1j1k1);

            double[] valLERP = new double[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < INDs.Count; i++)
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
            if (!inside[0] || !inside[1] || !inside[2] || !inside[3] ||
                !inside[4] || !inside[5] || !inside[6] || !inside[7])
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

            pStress = (1 - a) * (1 - b) * (1 - c) * valLERP[0]
                    + a * (1 - b) * (1 - c) * valLERP[1]
                    + (1 - a) * b * (1 - c) * valLERP[2]
                    + a * b * (1 - c) * valLERP[3]
                    + (1 - a) * (1 - b) * c * valLERP[4]
                    + a * (1 - b) * c * valLERP[5]
                    + (1 - a) * b * c * valLERP[6]
                    + a * b * c * valLERP[7];

            return pStress;
        }

        private double CorrectPrincpStress(int indVoxel, Vector3d prevDir)
        {
            Vector3d[] pDir = new Vector3d[6];
            double[] cos = new double[6];
            double[] pStress = new double[6];
            double maxCos = double.MinValue;
            int maxInd = -1;

            // Kolla närmare på vad som händer längst med en kant
            if ((startVoxels.cellValues[indVoxel] & maskbit) == 0)
            {
                return 0;
            }


            for (int k = 0; k < 3; k++)
            {
                pDir[k] = princpDir.cellValues[indVoxel][k];
                pDir[k + 3] = -pDir[k];

                cos[k] = prevDir * pDir[k];
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
