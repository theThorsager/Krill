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
        // public Voxels<Vector3d> princpStress;

        public Polyline loadPath;

        private double d;

        public LoadPathCurve2d(Voxels2d<int> startVoxels, Point3d startPt, Vector3d startVec,
                                Voxels2d<Vector3d[]> princpDir)
        {
            this.startVoxels = startVoxels;
            this.startPt = new Point2d(startPt.X, startPt.Y);
            this.startVec = new Vector2d(startVec.X, startVec.Y);
            this.princpDir = princpDir;
            // this.princpStress = princpStress;

            d = startVoxels.delta;
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

                for (int j = 0; j < 10; j++)
                {
                    x1 = x0 + step * aveDf;
                    Vector2d dfx1 = CLKFilterLinearMoveVec(x1, oldVec, out endCond);
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
                    Vector2d dfx1 = CLKFilterLinearMoveVec(x1, oldVec, out endCond);
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

            // Väldigt basic break-villkor, uppdatera och implementera ett bättre
            if ((startVoxels.cellValues[INDi1j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi0j0k0] & maskbit) == 0)
                basicEnd = true;

            dir.X = (1 - a) * (1 - b) * CorrectPrincpDir(INDi0j0k0, previousDir).X
                    + a * (1 - b) * CorrectPrincpDir(INDi1j0k0, previousDir).X
                    + (1 - a) * b * CorrectPrincpDir(INDi0j1k0, previousDir).X
                    + a * b * CorrectPrincpDir(INDi1j1k0, previousDir).X;

            dir.Y = (1 - a) * (1 - b) * CorrectPrincpDir(INDi0j0k0, previousDir).Y
                    + a * (1 - b) * CorrectPrincpDir(INDi1j0k0, previousDir).Y
                    + (1 - a) * b * CorrectPrincpDir(INDi0j1k0, previousDir).Y
                    + a * b * CorrectPrincpDir(INDi1j1k0, previousDir).Y;

            dir.Unitize();
            return dir;
        }

        private Vector2d CorrectPrincpDir(int indVoxel, Vector2d prevDir)
        {
            Vector3d[] pDir = new Vector3d[6];
            double[] angle = new double[6];
            double minAng = double.MaxValue;
            int minInd = 0;
            Vector2d direction;

            Vector3d prevDir3d = new Vector3d(prevDir.X, prevDir.Y, 0);

            // Kolla närmare på vad som händer längst med en kant
            if ((startVoxels.cellValues[indVoxel] & maskbit) == 0)
            {
                // Det fungerar poteniellt bättre än 0-vektorn, mer tester behövs
                return prevDir;
            }
                
            
            for (int k = 0; k < 3; k++)
            {
                pDir[k] = princpDir.cellValues[indVoxel][k];
                pDir[k + 3] = -pDir[k];

                angle[k] = Vector3d.VectorAngle(prevDir3d, pDir[k]);
                angle[k + 3] = Vector3d.VectorAngle(prevDir3d, pDir[k + 3]);
            }

            for (int k = 0; k < 6; k++)
            {
                if (angle[k] < minAng)
                {
                    minAng = angle[k];
                    minInd = k;
                }
            }

            direction = new Vector2d(pDir[minInd].X, pDir[minInd].Y);
            return direction;
        }
    }
}
