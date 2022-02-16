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

            int maxNoStep = (int)1e6;   // Max number of steps

            Vector3d moveVec = startVec;
            moveVec.Unitize();
            Point3d pos = startPt;
            List<Point3d> pathPts = new List<Point3d>();

            double step = d * scaleStep;

            pathPts.Add(pos);

            pos += step * moveVec;
            pathPts.Add(pos);

            for (int i = 0; i < maxNoStep; i++)
            {
                Vector3d oldVec = moveVec;
                bool endCond;
                Vector3d dfx0 = CLKFilterLinearMoveVec(pos, oldVec, out endCond);
                Vector3d aveDf = dfx0;

                Vector3d oldAveDf;
                Point3d x1 = pos;

                for (int j = 0; j < 10; j++)
                {
                    Vector3d dfx1 = CLKFilterLinearMoveVec(x1, oldVec, out endCond);
                    oldAveDf = aveDf;
                    aveDf = (dfx0+ dfx1)*0.5;

                    x1 = pos + step * aveDf;
                    if ((aveDf - oldAveDf).SquareLength < 1e-12)
                        break;                    
                }

                moveVec = aveDf;
                pos = x1;

                if (endCond)
                    break;

                pathPts.Add(pos);                
            }
            loadPath = new Polyline(pathPts);
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
            if ((startVoxels.cellValues[INDi1j1k1] & maskbit) == 0 && (startVoxels.cellValues[INDi0j0k0] & maskbit) == 0)
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
                // Det fungerar poteniellt bättre än 0-vektorn, mer tester kan behövas
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
    }
}
