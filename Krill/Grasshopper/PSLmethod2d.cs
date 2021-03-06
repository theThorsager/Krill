using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Krill.Containers;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Krill.Grasshopper
{
    public class PSLmethod2d : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the PSLmethod class.
        /// </summary>
        public PSLmethod2d()
          : base("PSLmethod2d", "PSL2d",
              "Description",
              "Krill", "Utility")
        {
            BaseWorker = new PSLWorker2d();
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            this.RequestCancellation();
            BaseWorker.SetData(null);   // Dirty way to disable the conduit
            base.RemovedFromDocument(document);
        }

        public override bool Locked
        {
            get => base.Locked;
            set
            {
                if (value)
                    this.RequestCancellation();
                base.Locked = value;
            }
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.PostProcessingResultsParam2d());
            pManager.AddParameter(new Param.BoundaryConditionParam2d(), "BoundaryConditions", "BCs", "", GH_ParamAccess.list);
            pManager.AddPointParameter("StartPoints", "pts", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("ScaleFactorForDelta", "sf", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("ToleranceNodeRegion", "tolNode", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("OffsetNodeRegion", "offsetNode", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("curve", "c", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("PhaseICrvs", "Icrvs", "", GH_ParamAccess.list);
            pManager.AddPointParameter("LocalMaxVonMises", "maxPts", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("PhaseIICrvs", "IIcrvs", "", GH_ParamAccess.list);
            pManager.AddLineParameter("InitialTruss", "truss", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        //protected override void SolveInstance(IGH_DataAccess DA)
        //{
        //}

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("34319B02-57F1-40E9-AA17-F0E2CE064FEE"); }
        }
    }
    class PSLWorker2d : WorkerInstance
    {
        Containers.PostProcessingResults2d post = null;
        List<IBoundaryCondition2d> BCs { get; set; } = null;        
        List<Point3d> startPoints { get; set; }
        double scaleDelta { get; set; }
        double tol { get; set; }
        double offsetTol { get; set; }

        List<Polyline> pLine { get; set; } = null;
        List<Polyline> phaseIcrvs { get; set; } = null;
        List<Point3d> localMax { get; set; } = null;
        List<Polyline> phaseIIcrvs { get; set; } = null;
        List<Line> truss { get; set; } = null;
        const int maskbit = 0x000000FF;
        public PSLWorker2d() : base(null)
        { }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (CancellationToken.IsCancellationRequested) return;
            if (post is null)
                return;
            // ...
            ReportProgress(Id, 0);

            pLine = new List<Polyline>();
            phaseIcrvs = new List<Polyline>();
            localMax = new List<Point3d>();
            phaseIIcrvs = new List<Polyline>();
            truss = new List<Line>();

            Vector3d[] startDirs = new Vector3d[4];
            startDirs[0] = Vector3d.XAxis;
            startDirs[1] = -Vector3d.XAxis;
            startDirs[2] = Vector3d.YAxis;
            startDirs[3] = -Vector3d.YAxis;

            double delta = post.mask.delta;

            List<Curve> dCrvs = new List<Curve>();
            List<Curve> nCrvs = new List<Curve>();

            List<PhaseI> pI = new List<PhaseI>();
            List<PhaseII> pII = new List<PhaseII>();

            List<Point3d> nodes = new List<Point3d>();

            if (CancellationToken.IsCancellationRequested)
                return;

            foreach (IBoundaryCondition2d bc in BCs)
            {
                if (bc is BoundaryConditionDirechlet2d bcD)                
                    dCrvs.Add(bcD.curve);
                else if (bc is BoundaryConditionNuemann2d bcN)
                    nCrvs.Add(bcN.curve);
            }

            ///////////////////////////////////////////////////////////////////////////////////////////
            // Constructing the first p-lines
            for (int i = 0; i < startPoints.Count; i++)
            {
                if (CancellationToken.IsCancellationRequested)
                    return;

                for (int j = 0; j < startDirs.Length; j++)
                {
                    LoadPathCurve2d lPath = new LoadPathCurve2d(post.mask, startPoints[i], startDirs[j], post.princpDir, post.princpStress);
                    lPath.ConstructLoadPath(scaleDelta);
                    if (lPath.loadPath.IsValid)
                        pLine.Add(lPath.loadPath);
                }                
            }

            bool[] plToBc = new bool[pLine.Count];
            for (int i = 0; i < plToBc.Length; i++)
                plToBc[i] = false;

            // Searching against D-boundary
            for (int i = 0; i < dCrvs.Count; i++)
            {
                for (int j = 0; j < pLine.Count; j++)
                {
                    if (dCrvs[i].ClosestPoint(pLine[j].Last, out double t, delta * 2))
                    {
                        PhaseI newPI = new PhaseI(pLine[j]);
                        newPI.startPt = newPI.pLine.First;
                        newPI.endPt = dCrvs[i].PointAt(t);

                        pI.Add(newPI);

                        phaseIcrvs.Add(newPI.pLine);

                        plToBc[j] = true;

                        CorrectNode(ref nodes, newPI.startPt, tol, out bool bol);
                        CorrectNode(ref nodes, newPI.endPt, tol, out bol);
                    }                                          
                }                
            }

            for (int i = 0; i < pLine.Count; i++)
            {
                for (int j = 0; j < startPoints.Count; j++)
                {
                    Point3d pt = pLine[i].ClosestPoint(startPoints[j]);

                    if (pt.DistanceToSquared(startPoints[j]) < tol * tol && pt.DistanceToSquared(pLine[i].First) > tol * tol)
                    {
                        PhaseI newPI = new PhaseI(pLine[i]);
                        newPI.startPt = newPI.pLine.First;
                        newPI.endPt = startPoints[j];
                        
                        if (pI.Last().startPt.DistanceToSquared(newPI.endPt) > tol &&
                            pI.Last().endPt.DistanceToSquared(newPI.startPt) > tol)
                        {
                            pI.Add(newPI);

                            phaseIcrvs.Add(newPI.pLine);

                            CorrectNode(ref nodes, newPI.startPt, tol, out bool bol);
                            CorrectNode(ref nodes, newPI.endPt, tol, out bol);

                            plToBc[i] = true;
                        }
                    }
                }
            }

            

            // Finding the relevant local max vonMises points

            for (int i = 0; i < pI.Count; i++)
            {
                pI[i].locMaxPts = FindLocalMaxPtsOnCurve(pI[i].pLine, dCrvs, nCrvs);
                for (int j = 0; j < pI[i].locMaxPts.Count; j++)
                {
                    double t = pI[i].pLine.ClosestParameter(pI[i].locMaxPts[j]);
                    Vector3d tangent = pI[i].pLine.TangentAt(t);
                    Vector3d normal = Vector3d.CrossProduct(tangent, Vector3d.ZAxis);
                    pI[i].normalsAtT.Add(normal);
                    pI[i].tangentsAtT.Add(tangent);
                    pI[i].interTvals.Add(t);

                    CorrectNode(ref nodes, pI[i].locMaxPts[j], tol, out bool bol);
                }
            }

            //////////////////////////////////////////////////////////////////////////////////////////////////////
            // Check if one of the first curves intersect a phaseI curve

            double noOldPI = pI.Count;

            for (int i = 0; i < pLine.Count; i++)
            {
                if (plToBc[i])
                    continue;

                Curve curCrv = PolylineToCurve(pLine[i], 1);

                for (int j = 0; j < noOldPI; j++)
                {
                    CurveIntersections crvInter = Intersection.CurveCurve(pI[j].pLineCurve, curCrv, 0.01, 0.0);
                    if (crvInter != null && crvInter.Count > 0)
                    {
                        IntersectionEvent intEvent = crvInter[0];

                        if (intEvent.PointA.DistanceToSquared(pLine[i].First) > tol * tol)
                        {
                            PhaseI newPI = new PhaseI(pLine[i]);
                            newPI.startPt = pLine[i].First;
                            newPI.endPt = CorrectNode(ref nodes, intEvent.PointA, tol, out bool bol);

                            newPI.locMaxPts = FindLocalMaxPtsOnCurve(newPI.pLine, dCrvs, nCrvs);
                            for (int k = 0; k < newPI.locMaxPts.Count; k++)
                            {
                                double t = newPI.pLine.ClosestParameter(newPI.locMaxPts[k]);
                                Vector3d tangent = newPI.pLine.TangentAt(t);
                                Vector3d normal = Vector3d.CrossProduct(tangent, Vector3d.ZAxis);
                                newPI.normalsAtT.Add(normal);
                                newPI.tangentsAtT.Add(tangent);
                                newPI.interTvals.Add(t);

                                phaseIcrvs.Add(newPI.pLine);

                                pI.Add(newPI);

                                CorrectNode(ref nodes, newPI.locMaxPts[k], tol, out bol);
                            }

                        }
                    }

                }
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////
            // Constructing new load paths from max loc pts and intersections

            for (int i = 0; i < pI.Count; i++)
            {
                for (int j = 0; j < pI[i].locMaxPts.Count; j++)
                {
                    Point3d pt = pI[i].locMaxPts[j];

                    Point3d[] pts = new Point3d[4];
                    pts[0] = pt + pI[i].normalsAtT[j] * offsetTol;
                    pts[1] = pt - pI[i].normalsAtT[j] * offsetTol;
                    pts[2] = pt + pI[i].tangentsAtT[j] * offsetTol;
                    pts[3] = pt - pI[i].tangentsAtT[j] * offsetTol;

                    for (int k = 0; k < pts.Length; k++)
                    {
                        for (int m = 0; m < startDirs.Length; m++)
                        {
                            LoadPathCurve2d lPath = new LoadPathCurve2d(post.mask, pts[k], startDirs[m], post.princpDir, post.princpStress);
                            lPath.ConstructLoadPath(scaleDelta);

                            if (lPath.loadPath.IsValid)
                            {
                                pI[i].potIIpls.Add(lPath.loadPath);
                                pI[i].potIIplsStartPt.Add(pt);
                            }
                        }
                    }
                }               
            }

            for (int i = 0; i < pI.Count; i++)
            {
                for (int j = 0; j < pI[i].potIIpls.Count; j++)
                {
                    Curve currentPotIIcrv = PolylineToCurve(pI[i].potIIpls[j], 1);
                    Point3d currentLocMaxPt = pI[i].potIIplsStartPt[j];

                    bool intersect = false;

                    for (int k = 0; k < pI.Count; k++)
                    {
                        if (k == i)
                            continue;

                        CurveIntersections crvInter = Intersection.CurveCurve(pI[k].pLineCurve, currentPotIIcrv, 0.01, 0.0);
                        if (crvInter != null && crvInter.Count > 0)
                        {
                            IntersectionEvent intEvent = crvInter[0];

                            bool connectLocalMax = false;   // Or as a previous line

                            for (int kk = 0; kk < pI[k].interTvals.Count; kk++)
                            {
                                Point3d pt = pI[k].pLine.PointAt(pI[k].interTvals[kk]);
                                if (intEvent.PointA.DistanceToSquared(pt) < tol * tol)
                                {
                                    Line l = new Line(currentLocMaxPt, pt);
                                    truss.Add(l);
                                    connectLocalMax = true;
                                    intersect = true;
                                    phaseIIcrvs.Add(pI[i].potIIpls[j]);
                                    CorrectNode(ref nodes, pt, tol, out bool bol);
                                }
                            }

                            if (!connectLocalMax)
                            {
                                if (pI[k].startPt.DistanceToSquared(intEvent.PointA) > tol * tol &&
                                    pI[k].endPt.DistanceToSquared(intEvent.PointA) > tol * tol)
                                {
                                    Polyline pl = pI[i].potIIpls[j];
                                    List<Point3d> cutPlPts = new List<Point3d>();

                                    for (int ii = 0; ii < intEvent.ParameterB; ii++)
                                        cutPlPts.Add(pl[ii]);
                                    
                                    Polyline cutPl = new Polyline(cutPlPts);
                                    Line l = new Line(currentLocMaxPt, intEvent.PointA);

                                    if (cutPl.Length < l.Length * 1.15)
                                    {
                                        pI[k].interTvals.Add(intEvent.ParameterA);

                                        PhaseII newPII = new PhaseII(pl, i);
                                        newPII.startPt = currentLocMaxPt;
                                        newPII.endPt = intEvent.PointA;

                                        CorrectNode(ref nodes, intEvent.PointA, tol, out bool bol);

                                        pII.Add(newPII);

                                        truss.Add(l);
                                        intersect = true;

                                        phaseIIcrvs.Add(pI[i].potIIpls[j]);
                                    }
                                }                                
                            }
                        }
                    }

                    // If it does not intersect anything but connects in a new way to a BC
                    if (!intersect)
                    {
                        foreach (Curve dCrv in dCrvs)
                        {
                            if (dCrv.ClosestPoint(pI[i].potIIpls[j].Last, out double t, delta * 2))
                            {
                                double bcTol = dCrv.GetLength() / 2;

                                Line l = new Line(currentLocMaxPt, CorrectNode(ref nodes, dCrv.PointAt(t), bcTol, out bool bol));
                                if (!bol)
                                {
                                    phaseIIcrvs.Add(pI[i].potIIpls[j]);
                                    truss.Add(l);
                                }
                            }
                        }
                    }
                }
            }

            // Check if potII curves intersect
            for (int i = 0; i < pI.Count; i++)
            {
                for (int j = 0; j < pI[i].potIIpls.Count; j++)
                {
                    Curve currentIIcrv = PolylineToCurve(pI[i].potIIpls[j], 1);

                    for (int k = 0; k < pII.Count; k++)
                    {
                        if (i == pII[k].indPI)
                            continue;

                        CurveIntersections crvInter = Intersection.CurveCurve(pII[k].pLineCurve, currentIIcrv, 0.01, 0.0);
                        if (crvInter != null && crvInter.Count > 0)
                        {
                            IntersectionEvent intEvent = crvInter[0];

                            Point3d sPt = pI[i].potIIplsStartPt[j];
                            Point3d ePt;

                            // Does not create a new node on pII curves, instead connects to closest end point
                            if (intEvent.PointA.DistanceToSquared(pII[k].startPt) < intEvent.PointA.DistanceToSquared(pII[k].endPt))
                                ePt = pII[k].startPt;
                            else
                                ePt = pII[k].endPt;
                            
                            Line l = new Line(sPt, ePt);
                            truss.Add(l);
                            //phaseIIcrvs.Add(pI[i].potIIpls[j]);
                        }
                    }
                }
            }


            // Constructing the phaseI lines
            for (int i = 0; i < pI.Count; i++)
            {
                pI[i].interTvals.Sort();
                List<Point3d> pts = new List<Point3d>();
                pts.Add(pI[i].startPt);
                for (int j = 0; j < pI[i].interTvals.Count; j++)
                {
                    pts.Add(pI[i].pLine.PointAt(pI[i].interTvals[j]));
                }
                pts.Add(pI[i].endPt);

                for (int j = 0; j < pts.Count - 1; j++)
                {
                    Line l = new Line(pts[j], pts[j + 1]);
                    truss.Add(l);
                }
            }

            /////////////////////////////////////////////////////////////////////////////////////////
            // Phase III (handeling the beam case)
            for (int i = 0; i < dCrvs.Count; i++)
            {
                if (dCrvs.Count < 2)
                    break;

                Point3d startPt = dCrvs[i].PointAtNormalizedLength(0.5);

                for (int j = 0; j < startDirs.Length; j++)
                {
                    LoadPathCurve2d lPath = new LoadPathCurve2d(post.mask, startPt, startDirs[j], post.princpDir, post.princpStress);
                    lPath.ConstructLoadPath(scaleDelta);

                    if (lPath.loadPath.IsValid)
                    {
                        Curve potIII = PolylineToCurve(lPath.loadPath, 1);

                        List<Point3d> ptsL = new List<Point3d>();

                        for (int ii = 0; ii < pI.Count; ii++)
                        {
                            CurveIntersections crvInter = Intersection.CurveCurve(pI[ii].pLineCurve, potIII, tol, 0.0);
                            if (crvInter != null && crvInter.Count > 0)
                            {
                                IntersectionEvent intEvent = crvInter[0];                                    

                                // It is enough to only measure against the end pts
                                if (intEvent.PointA.DistanceToSquared(pI[ii].endPt) < tol * tol)
                                    ptsL.Add(pI[ii].endPt);                                                                 
                            }
                        }

                        if (ptsL.Count > 1)
                        {
                            for (int ii = 0; ii < ptsL.Count - 1; ii++)
                            {
                                Line l = new Line(ptsL[ii], ptsL[ii + 1]);
                                truss.Add(l);
                            }
                        }
                    }                        
                }
            }

            //localMax = nodes;

            if (CancellationToken.IsCancellationRequested)
                return;

            Done();
        }
        

        internal class PhaseI
        {
            public Polyline pLine;
            public Curve pLineCurve;
            public Point3d startPt;
            public Point3d endPt;
            public List<double> interTvals;
            public List<Point3d> locMaxPts;
            public List<Vector3d> normalsAtT;
            public List<Vector3d> tangentsAtT;
            public List<Polyline> potIIpls;
            public List<Point3d> potIIplsStartPt;

            public PhaseI(Polyline loadPath)
            {
                pLine = loadPath;
                pLineCurve = PolylineToCurve(pLine, 1);
                interTvals = new List<double>();
                locMaxPts = new List<Point3d>();
                potIIpls = new List<Polyline>();
                normalsAtT = new List<Vector3d>();
                tangentsAtT = new List<Vector3d>();
                potIIplsStartPt = new List<Point3d>();                
            }
        }

        internal class PhaseII
        {
            public Polyline pLine;
            public Curve pLineCurve;
            public Point3d startPt;
            public Point3d endPt;
            public int indPI;

            public PhaseII(Polyline loadPath, int indPhaseIcrv)
            {
                pLine = loadPath;
                pLineCurve = PolylineToCurve(pLine, 1);
                indPI = indPhaseIcrv;
            }
        }

        private static Point3d CorrectNode(ref List<Point3d> nodes, Point3d newNode, double tol, out bool alreadyExists)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                if (newNode.DistanceToSquared(nodes[i]) < tol * tol)
                {
                    alreadyExists = true;
                    return nodes[i];
                }
                    
            }
            alreadyExists = false;
            nodes.Add(newNode);
            return newNode;
        }

        private static Curve PolylineToCurve(Polyline polyline, int degree)
        {
            List<Point3d> pts = new List<Point3d>();
            for (int i = 0; i < polyline.Count; i++)
            {
                pts.Add(polyline[i]);
            }
            return Curve.CreateInterpolatedCurve(pts, degree);
        }
        private List<Point3d> FindLocalMaxPtsOnCurve(Polyline polyline, List<Curve> dCrvs, List<Curve> nCrvs)
        {
            List<Point3d> ptsOnCrv = new List<Point3d>();
            List<double> vonAtCurve = new List<double>();

            for (int i = 0; i < polyline.Count; i++)
            {
                Point2d pos = new Point2d(polyline[i].X, polyline[i].Y);
                vonAtCurve.Add(LERPvonMises(pos));
            }

            for (int i = 0; i < 10; i++)
            {
                vonAtCurve = SmoothData(vonAtCurve, 1);
            }

            int neighbour = 1;

            for (int i = neighbour; i < vonAtCurve.Count - neighbour; i++)
            {
                if (vonAtCurve[i] > vonAtCurve[i - neighbour] &&
                    vonAtCurve[i] > vonAtCurve[i + neighbour])
                {
                    bool closeToBC = false;

                    double bcTol = tol;
                    if (tol < post.mask.delta * 3)
                        bcTol = post.mask.delta * 3;

                    foreach (Curve dCrv in dCrvs)
                    {
                        if (dCrv.ClosestPoint(polyline[i], out double t, bcTol))
                        {
                            closeToBC = true;
                            break;
                        }
                    }
                    foreach (Curve nCrv in nCrvs)
                    {
                        if (nCrv.ClosestPoint(polyline[i], out double t, bcTol))
                        {
                            closeToBC = true;
                            break;
                        }
                    }

                    if (!closeToBC)
                    {
                        Point2d pos = new Point2d(polyline[i].X, polyline[i].Y);
                        Vector2d pStress = LERPpStress(pos);
                        double p1 = Math.Abs(pStress.X);
                        double p2 = Math.Abs(pStress.Y);
                        double factor = 0.1;

                        if (p1 > p2 && p2 > p1 * factor)
                        {
                            localMax.Add(polyline[i]);
                            ptsOnCrv.Add(polyline[i]);
                        }
                            
                        else if (p2 > p1 && p1 > p2 * factor)
                        {
                            localMax.Add(polyline[i]);
                            ptsOnCrv.Add(polyline[i]);
                        }
                    }                    
                }                    
            }
            return ptsOnCrv;
        }

        private static List<double> SmoothData(List<double> list, int range)
        {
            // Average value within range

            var result = new List<double>();

            for (int i = 0; i < list.Count; i++)
            {
                int count = 0;
                double sum = 0;
                for (int jj = -range; jj <= range; jj++)
                {
                    int j = i + jj;
                    if (j < 0 || j >= list.Count)
                        continue;

                    count++;
                    sum += list[j];
                }
                result.Add(sum / count);
            }

            return result;
        }

        private double LERPvonMises(Point2d pos)
        {
            Voxels2d<int> startVoxels = post.mask;
            Voxels2d<double> vonMises = post.vonMises;

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

            // Very basic break-statement
            if ((startVoxels.cellValues[INDi0j0k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k0] & maskbit) == 0 &&
                (startVoxels.cellValues[INDi0j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k0] & maskbit) == 0)
            {
                return 0;
            }

            List<int> INDs = new List<int>();

            INDs.Add(INDi0j0k0);
            INDs.Add(INDi1j0k0);
            INDs.Add(INDi0j1k0);
            INDs.Add(INDi1j1k0);

            double[] lerpVals = new double[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < 4; i++)
            {
                if ((startVoxels.cellValues[INDs[i]] & maskbit) == 0)
                    inside[i] = false;
                else
                {
                    inside[i] = true;
                    lerpVals[i] = vonMises.cellValues[INDs[i]];
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

                    avgVal += vonMises.cellValues[INDs[i]];
                    noInside++;
                }
                avgVal /= (double)noInside;
            }

            for (int i = 0; i < INDs.Count; i++)
            {
                if (inside[i])
                    continue;

                lerpVals[i] = avgVal;
            }

            // Do the LERP
            double val = (1 - a) * (1 - b) * lerpVals[0]
                    + a * (1 - b) * lerpVals[1]
                    + (1 - a) * b * lerpVals[2]
                    + a * b * lerpVals[3];

            return val;
        }
        private Vector2d LERPpStress(Point2d pos)
        {
            Voxels2d<int> startVoxels = post.mask;
            Voxels2d<Vector3d> pStress = post.princpStress;

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

            // Very basic break-statement
            if ((startVoxels.cellValues[INDi0j0k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k0] & maskbit) == 0 &&
                (startVoxels.cellValues[INDi0j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k0] & maskbit) == 0)
            {
                return Vector2d.Zero;
            }

            List<int> INDs = new List<int>();

            INDs.Add(INDi0j0k0);
            INDs.Add(INDi1j0k0);
            INDs.Add(INDi0j1k0);
            INDs.Add(INDi1j1k0);

            Vector2d[] lerpVals = new Vector2d[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < 4; i++)
            {
                if ((startVoxels.cellValues[INDs[i]] & maskbit) == 0)
                    inside[i] = false;
                else
                {
                    inside[i] = true;
                    lerpVals[i].X = pStress.cellValues[INDs[i]].X;
                    lerpVals[i].Y = pStress.cellValues[INDs[i]].Y;
                }
            }

            Vector2d avgVal = new Vector2d();
            double noInside = 0;
            if (!inside[0] || !inside[1] || !inside[2] || !inside[3])
            {
                for (int i = 0; i < INDs.Count; i++)
                {
                    if (!inside[i])
                        continue;

                    avgVal.X += pStress.cellValues[INDs[i]].X;
                    avgVal.Y += pStress.cellValues[INDs[i]].Y;
                    noInside++;
                }
                avgVal /= (double)noInside;
            }

            for (int i = 0; i < INDs.Count; i++)
            {
                if (inside[i])
                    continue;

                lerpVals[i] = avgVal;
            }

            // Do the LERP
            Vector2d val = new Vector2d();
            val.X = (1 - a) * (1 - b) * lerpVals[0].X
                    + a * (1 - b) * lerpVals[1].X
                    + (1 - a) * b * lerpVals[2].X
                    + a * b * lerpVals[3].X;

            val.Y = (1 - a) * (1 - b) * lerpVals[0].Y
                    + a * (1 - b) * lerpVals[1].Y
                    + (1 - a) * b * lerpVals[2].Y
                    + a * b * lerpVals[3].Y;

            return val;
        }
        private void FindLocalMaxPts(double factorOfMaxStress, double horizon, Voxels2d<double> vonMises, List<Curve> dCrvs, List<Curve> nCrvs, Voxels2d<Vector3d> principalStress)
        {
            double minLocalMaxStress = vonMises.cellValues.Max() * factorOfMaxStress;   // godtyckligt vald faktor

            int[] nList = Utility.GetNeighbourOffsets2d(vonMises.n, horizon);

            for (int i = 0; i < vonMises.n * vonMises.n; i++)
            {
                if ((post.mask.cellValues[i] & maskbit) == 0)
                    continue;

                bool localMaxBool = true;

                for (int ii = 0; ii < nList.Length; ii++)
                {
                    int j = i + nList[ii];
                    if (j < post.mask.cellValues.Length && post.mask.cellValues[j] != 0)
                    {
                        if (vonMises.IndexToPoint(i).DistanceTo(vonMises.IndexToPoint(j)) < horizon)
                        {
                            if (vonMises.cellValues[i] < vonMises.cellValues[j])
                            {
                                localMaxBool = false;
                                break;
                            }
                        }
                        
                    }

                    j = i - nList[ii];
                    if (j >= 0 && post.mask.cellValues[j] != 0)
                    {
                        if (vonMises.IndexToPoint(i).DistanceTo(vonMises.IndexToPoint(j)) < horizon)
                        {
                            if (vonMises.cellValues[i] < vonMises.cellValues[j])
                            {
                                localMaxBool = false;
                                break;
                            }
                        }
                    }
                }

                if (localMaxBool)
                {
                    if (vonMises.cellValues[i] > minLocalMaxStress)
                    {
                        Point2d pt = vonMises.IndexToPoint(i);
                        Point3d pt3d = new Point3d(pt.X, pt.Y, 0);
                        bool closeToBC = false;

                        foreach (Curve dCrv in dCrvs)
                        {
                            if (dCrv.ClosestPoint(pt3d, out double t, tol))
                            {
                                closeToBC = true;
                                break;
                            }
                        }

                        foreach (Curve nCrv in nCrvs)
                        {
                            if (nCrv.ClosestPoint(pt3d, out double t, tol))
                            {
                                closeToBC = true;
                                break;
                            }
                        }

                        if (!closeToBC)
                        {
                            double p1 = Math.Abs(principalStress.cellValues[i].X);
                            double p2 = Math.Abs(principalStress.cellValues[i].Y);
                            double factor = 0.1;

                            if (p1 > p2 && p2 > p1 * factor)
                                localMax.Add(pt3d);
                            else if (p2 > p1 && p1 > p2 * factor)
                                localMax.Add(pt3d);
                        }

                    }
                }
            }
        }

        public override WorkerInstance Duplicate() => new PSLWorker2d();

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Param.PostProcessingResults2dGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            post = res.Value;

            var bcs = new List<Param.BoundaryCondition2dGoo>();
            DA.GetDataList(1, bcs);
            if (!(bcs is null))
                this.BCs = bcs.Select(x => x.Value).ToList();

            List<Point3d> startPts = new List<Point3d>();
            DA.GetDataList(2, startPts);
            startPoints = startPts;

            double scale = new double();
            DA.GetData(3, ref scale);
            scaleDelta = scale;

            double tol = new double();
            DA.GetData(4, ref tol);
            this.tol = tol;

            double offsetTol = new double();
            DA.GetData(5, ref offsetTol);
            this.offsetTol = offsetTol;
        }

        public override void SetData(IGH_DataAccess DA)
        {
            if (CancellationToken.IsCancellationRequested)
                return;

            if (!(pLine is null))
                DA.SetDataList(0, pLine);
            if (!(phaseIcrvs is null))
                DA.SetDataList(1, phaseIcrvs);
            if (!(localMax is null))
                DA.SetDataList(2, localMax);
            if (!(phaseIIcrvs is null))
                DA.SetDataList(3, phaseIIcrvs);
            if (!(truss is null))
                DA.SetDataList (4, truss);
        }
    }
}