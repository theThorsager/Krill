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
        public Point3d startPt;
        public Vector3d startVec;
        public Voxels<Vector3d[]> princpDir;
        public Voxels<Vector3d> princpStress;

        public Polyline loadPath;

        private double d;

        public LoadPathCurve(Voxels<int> startVoxels, Point3d startPt, Vector3d startVec,
                                Voxels<Vector3d[]> princpDir, Voxels<Vector3d> princpStress)
        {
            this.startVoxels = startVoxels;
            this.startPt = startPt;
            this.startVec = startVec;
            this.princpDir = princpDir;
            this.princpStress = princpStress;

            d = startVoxels.delta;
        }

        public void ConstructLoadPath(double scaleStep)
        {

            int maxNoStep = (int)1e4;   // Max number of steps

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

                int newSign = Math.Sign(LerpPstress(x1, moveVec));

                if (Math.Abs(oldSign - newSign) == 2)
                    break;

                if (endCond)
                    break;

                pathPts.Add(x1);                
            }
            loadPath = new Polyline(pathPts);
        }

        public bool SecondaryLoadPath(double scaleStep, double tolerance, List<Point3d> nodes, out int index)
        {
            int maxNoStep = (int)1e4;   // Max number of steps

            index = 1000;

            Vector3d moveVec = startVec;
            moveVec.Unitize();
            Point3d x1 = startPt;
            List<Point3d> pathPts = new List<Point3d>();

            double step = d * scaleStep;

            pathPts.Add(x1);

            x1 += 2 * step * moveVec;

            pathPts.Add(x1);

            tolerance *= tolerance;

            for (int i = 0; i < maxNoStep; i++)
            {
                Vector3d oldVec = moveVec;
                bool endCond;
                Point3d x0 = x1;
                Vector3d dfx0 = CLKFilterLinearMoveVec(x0, oldVec, out endCond);
                Vector3d aveDf = dfx0;

                Vector3d oldAveDf;

                for (int j = 0; j < 10; j++)
                {
                    x1 = x0 + step * aveDf;
                    Vector3d dfx1 = CLKFilterLinearMoveVec(x1, dfx0, out endCond);
                    oldAveDf = aveDf;
                    aveDf = (dfx0 + dfx1) * 0.5;

                    if (j == 9)
                        break;

                    if ((aveDf - oldAveDf).SquareLength < 1e-18)
                        break;
                }

                moveVec = aveDf;

                x1 = x0 + step * moveVec;

                pathPts.Add(x1);

                for (int j = 0; j < nodes.Count; j++)
                {
                    if (x1.DistanceToSquared(nodes[j]) < tolerance && startPt.DistanceToSquared(nodes[j]) > tolerance)
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
            double[] angle = new double[6];
            double minAng = double.MaxValue;
            int minInd = 0;
            Vector3d direction;

            // Kolla närmare på vad som händer längst med en kant
            if ((startVoxels.cellValues[indVoxel] & maskbit) == 0)
            {
                // Det fungerar poteniellt bättre än 0-vektorn, mer tester behövs
                direction = prevDir;
                return direction;
            }
                
            
            for (int k = 0; k < 3; k++)
            {
                pDir[k] = princpDir.cellValues[indVoxel][k];
                pDir[k + 3] = -pDir[k];

                angle[k] = Vector3d.VectorAngle(prevDir, pDir[k]);
                angle[k + 3] = Vector3d.VectorAngle(prevDir, pDir[k + 3]);
            }

            for (int k = 0; k < 6; k++)
            {
                if (angle[k] < minAng)
                {
                    minAng = angle[k];
                    minInd = k;
                }
            }

            direction = pDir[minInd];
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

            double pStress;

            pStress = (1 - a) * (1 - b) * (1 - c) * CorrectPrincpStress(INDi0j0k0, previousDir)
                    + a * (1 - b) * (1 - c) * CorrectPrincpStress(INDi1j0k0, previousDir)
                    + (1 - a) * b * (1 - c) * CorrectPrincpStress(INDi0j1k0, previousDir)
                    + a * b * (1 - c) * CorrectPrincpStress(INDi1j1k0, previousDir)
                    + (1 - a) * (1 - b) * c * CorrectPrincpStress(INDi0j0k1, previousDir)
                    + a * (1 - b) * c * CorrectPrincpStress(INDi1j0k1, previousDir)
                    + (1 - a) * b * c * CorrectPrincpStress(INDi0j1k1, previousDir)
                    + a * b * c * CorrectPrincpStress(INDi1j1k1, previousDir);

            return pStress;
        }

        private double CorrectPrincpStress(int indVoxel, Vector3d prevDir)
        {
            Vector3d[] pDir = new Vector3d[6];
            double[] angle = new double[6];
            double[] pStress = new double[6];
            double minAng = double.MaxValue;
            int minInd = 0;

            // Kolla närmare på vad som händer längst med en kant
            if ((startVoxels.cellValues[indVoxel] & maskbit) == 0)
            {
                return 0;
            }


            for (int k = 0; k < 3; k++)
            {
                pDir[k] = princpDir.cellValues[indVoxel][k];
                pDir[k + 3] = -pDir[k];

                angle[k] = Vector3d.VectorAngle(prevDir, pDir[k]);
                angle[k + 3] = Vector3d.VectorAngle(prevDir, pDir[k + 3]);

                pStress[k] = princpStress.cellValues[indVoxel][k];
                pStress[k + 3] = pStress[k];
            }

            for (int k = 0; k < 6; k++)
            {
                if (angle[k] < minAng)
                {
                    minAng = angle[k];
                    minInd = k;
                }
            }

            return pStress[minInd];
        }
    }
}
