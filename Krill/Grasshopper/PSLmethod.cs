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
    public class PSLmethod : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the PSLmethod class.
        /// </summary>
        public PSLmethod()
          : base("PSLmethod", "PSL",
              "Description",
              "Krill", "Utility")
        {
            BaseWorker = new PSLWorker();
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
            pManager.AddParameter(new Param.PostProcessingResultsParam());
            pManager.AddParameter(new Param.BoundaryConditionParam(), "BoundaryConditions", "BCs", "", GH_ParamAccess.list);
            pManager.AddPointParameter("StartPoints", "pts", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("ScaleFactorForDelta", "sf", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("ToleranceNodeRegion", "tolNode", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("OffsetNodeRegion", "offsetNode", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("ToleranceIntersection", "tolInt", "", GH_ParamAccess.item);
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
            get { return new Guid("F1731901-9D9A-43E2-9501-D9E23635B6B9"); }
        }
    }
    class PSLWorker : WorkerInstance
    {
        Containers.PostProcessingResults post = null;
        List<IBoundaryCondition> BCs { get; set; } = null;        
        List<Point3d> startPoints { get; set; }
        double scaleDelta { get; set; }
        double tol { get; set; }
        double offsetTol { get; set; }
        double intTol { get; set; }

        List<Polyline> pLine { get; set; } = null;
        List<Polyline> phaseIcrvs { get; set; } = null;
        List<Point3d> localMax { get; set; } = null;
        List<Polyline> phaseIIcrvs { get; set; } = null;
        List<Line> truss { get; set; } = null;
        const int maskbit = 0x000000FF;
        public PSLWorker() : base(null)
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

            Vector3d[] startDirs = new Vector3d[6]
            {
                Vector3d.XAxis,
                -Vector3d.XAxis,
                Vector3d.YAxis,
                -Vector3d.YAxis,
                Vector3d.ZAxis,
                -Vector3d.ZAxis
            };
            double delta = post.mask.delta;

            List<Mesh> dAreas = new List<Mesh>();
            List<Mesh> nAreas = new List<Mesh>();

            List<PhaseI> pI = new List<PhaseI>();
            List<PhaseII> pII = new List<PhaseII>();

            List<Point3d> nodes = new List<Point3d>();

            if (CancellationToken.IsCancellationRequested)
                return;

            foreach (IBoundaryCondition bc in BCs)
            {
                if (bc is BoundaryConditionDirechlet bcD)
                    dAreas.Add(bcD.area);                        
                else if (bc is BoundaryConditionNuemann bcN)
                    nAreas.Add(bcN.area);
            }

            ///////////////////////////////////////////////////////////////////////////////////////////
            // Constructing the first p-lines
            for (int i = 0; i < startPoints.Count; i++)
            {
                if (CancellationToken.IsCancellationRequested)
                    return;

                for (int j = 0; j < startDirs.Length; j++)
                {
                    LoadPathCurve lPath = new LoadPathCurve(post.mask, startPoints[i], startDirs[j], post.princpDir, post.princpStress);
                    lPath.ConstructLoadPath(scaleDelta, false);
                    if (lPath.loadPath.IsValid)
                        pLine.Add(lPath.loadPath);
                }                
            }

            bool[] plToBc = new bool[pLine.Count];            

            // Check if it connects to another start point
            for (int i = 0; i < pLine.Count; i++)
            {
                //List<double> closeTvals = new List<double>();
                //List<int> jVals = new List<int>();

                List<Tuple<double, int>> tValsAndInd = new List<Tuple<double, int>>();

                double startTol = delta;

                for (int j = 0; j < startPoints.Count; j++)
                {
                    
                    double t = pLine[i].ClosestParameter(startPoints[j]);
                    Point3d pt = pLine[i].PointAt(t);

                    if (pt.DistanceToSquared(startPoints[j]) < startTol * startTol && pt.DistanceToSquared(pLine[i].First) > startTol * startTol)
                        tValsAndInd.Add(new Tuple<double, int>(t, j));                    
                }

                if (tValsAndInd.Count > 0)
                {
                    tValsAndInd = tValsAndInd.OrderBy(x => x.Item1).ToList();
                    pLine[i] = CutPolyline(pLine[i], tValsAndInd[0].Item1);
                    PhaseI newPI = new PhaseI(pLine[i]);
                    newPI.startPt = newPI.pLine.First;
                    newPI.endPt = startPoints[tValsAndInd[0].Item2];

                    bool add = true;

                    for (int k = 0; k < pI.Count; k++)
                    {
                        // Duplicate check
                        if ((pI[k].startPt.DistanceToSquared(newPI.startPt) < startTol * startTol && pI[k].endPt.DistanceToSquared(newPI.endPt) < startTol * startTol) ||
                            (pI[k].startPt.DistanceToSquared(newPI.endPt) < startTol * startTol && pI[k].endPt.DistanceToSquared(newPI.startPt) < startTol * startTol))
                        {
                            add = false;
                            break;
                        }
                    }

                    if (add)
                    {
                        pI.Add(newPI);

                        phaseIcrvs.Add(newPI.pLine);

                        CorrectNode(ref nodes, newPI.startPt, tol, out bool bol);
                        CorrectNode(ref nodes, newPI.endPt, tol, out bol);

                        plToBc[i] = true;
                    }
                }
            }

            // Searching against D-boundary
            for (int i = 0; i < dAreas.Count; i++)
            {
                for (int j = 0; j < pLine.Count; j++)
                {
                    if (dAreas[i].ClosestPoint(pLine[j].Last, out Point3d ptOnMesh, delta * 2) != -1)
                    {
                        PhaseI newPI = new PhaseI(pLine[j]);
                        newPI.startPt = newPI.pLine.First;
                        newPI.endPt = ptOnMesh;

                        pI.Add(newPI);

                        phaseIcrvs.Add(newPI.pLine);

                        plToBc[j] = true;

                        CorrectNode(ref nodes, newPI.startPt, tol, out bool bol);
                        CorrectNode(ref nodes, newPI.endPt, tol, out bol);
                    }
                }
            }


            // Finding the relevant local max vonMises points

            for (int i = 0; i < pI.Count; i++)
            {
                pI[i].locMaxPts = FindLocalMaxPtsOnCurve(pI[i].pLine, dAreas, nAreas);
                for (int j = 0; j < pI[i].locMaxPts.Count; j++)
                {
                    double t = pI[i].pLine.ClosestParameter(pI[i].locMaxPts[j]);
                    Vector3d tangent = pI[i].pLine.TangentAt(t);
                    Plane normalPlane = new Plane(pI[i].pLine.PointAt(t), tangent);
                    pI[i].normalsAtT.Add(normalPlane);
                    pI[i].tangentsAtT.Add(tangent);
                    pI[i].interTvals.Add(t);

                    CorrectNode(ref nodes, pI[i].locMaxPts[j], tol, out bool bol);
                }
            }

            if (CancellationToken.IsCancellationRequested)
                return;
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
                    CurveIntersections crvInter = Intersection.CurveCurve(pI[j].pLineCurve, curCrv, intTol, 0.0);
                    if (crvInter != null && crvInter.Count > 0)
                    {
                        IntersectionEvent intEvent = crvInter[0];

                        if (intEvent.PointA.DistanceToSquared(pLine[i].First) > tol * tol)
                        {
                            PhaseI newPI = new PhaseI(pLine[i]);
                            newPI.startPt = pLine[i].First;
                            newPI.endPt = CorrectNode(ref nodes, intEvent.PointA, tol, out bool bol);

                            newPI.locMaxPts = FindLocalMaxPtsOnCurve(newPI.pLine, dAreas, nAreas);
                            for (int k = 0; k < newPI.locMaxPts.Count; k++)
                            {
                                double t = newPI.pLine.ClosestParameter(newPI.locMaxPts[k]);
                                Vector3d tangent = newPI.pLine.TangentAt(t);
                                Plane normalPlane = new Plane(newPI.pLine.PointAt(t), tangent);
                                newPI.normalsAtT.Add(normalPlane);
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

            if (CancellationToken.IsCancellationRequested)
                return;
            ////////////////////////////////////////////////////////////////////////////////////////////////
            // Constructing new load paths from max loc pts and intersections

            for (int i = 0; i < pI.Count; i++)
            {
                for (int j = 0; j < pI[i].locMaxPts.Count; j++)
                {
                    Point3d pt = pI[i].locMaxPts[j];

                    Point3d[] pts = new Point3d[6];
                    pts[0] = pt + pI[i].normalsAtT[j].XAxis * offsetTol;
                    pts[1] = pt - pI[i].normalsAtT[j].XAxis * offsetTol;
                    pts[2] = pt + pI[i].normalsAtT[j].YAxis * offsetTol;
                    pts[3] = pt - pI[i].normalsAtT[j].YAxis * offsetTol;
                    pts[4] = pt + pI[i].tangentsAtT[j] * offsetTol;
                    pts[5] = pt - pI[i].tangentsAtT[j] * offsetTol;

                    for (int k = 0; k < pts.Length; k++)
                    {
                        for (int m = 0; m < startDirs.Length; m++)
                        {
                            LoadPathCurve lPath = new LoadPathCurve(post.mask, pts[k], startDirs[m], post.princpDir, post.princpStress);
                            lPath.ConstructLoadPath(scaleDelta, true);

                            if (lPath.loadPath.IsValid)
                            {
                                pI[i].potIIpls.Add(lPath.loadPath);
                                pI[i].potIIplsStartPt.Add(pt);
                                //phaseIIcrvs.Add(lPath.loadPath);
                            }
                        }
                    }
                }               
            }


            if (CancellationToken.IsCancellationRequested)
                return;
            // Checking the new curves for intersections
            for (int i = 0; i < pI.Count; i++)
            {
                for (int j = 0; j < pI[i].potIIpls.Count; j++)
                {
                    Curve currentPotIIcrv = PolylineToCurve(pI[i].potIIpls[j], 1);
                    Point3d currentLocMaxPt = pI[i].potIIplsStartPt[j];

                    bool connectLocalMax = false;   // Or as a previous line

                    List<Tuple<double, int, IntersectionEvent>> intProp = new List<Tuple<double, int, IntersectionEvent>>();

                    for (int k = 0; k < pI.Count; k++)
                    {
                        // Not searching against the curve that it originates from
                        if (k == i)
                            continue;

                        CurveIntersections crvInter = Intersection.CurveCurve(pI[k].pLineCurve, currentPotIIcrv, intTol, 0.0);

                        if (crvInter != null && crvInter.Count > 0)
                        {
                            IntersectionEvent intEvent = crvInter[0];
                            intProp.Add(new Tuple<double, int, IntersectionEvent>(intEvent.ParameterB, k, intEvent));
                        }
                    }

                    // At least one intersection is found, the first one is relevant, not the rest
                    if (intProp.Count > 0)
                    {
                        intProp = intProp.OrderBy(x => x.Item1).ToList();

                        Polyline potPl = null;
                        int indTval = 0;

                        // Kolla vinkeln så att den skapade linjen går ungefär i samma rikning som polylinen
                        for (int k = 0; k < intProp.Count; k++)
                        {
                            potPl = CutPolyline(pI[i].potIIpls[j], intProp[k].Item1);
                            if (!potPl.IsValid)
                                continue;
                            Vector3d dirPl = potPl.Last - potPl.First;
                            dirPl.Unitize();
                            Vector3d dirLn = intProp[k].Item3.PointA - potPl.First;
                            dirLn.Unitize();

                            double cos = Math.Abs(dirPl * dirPl);

                            if (cos > 0.5)
                            {
                                indTval = k;
                                break;
                            }
                        }
                        Polyline curPl = CutPolyline(pI[i].potIIpls[j], intProp[indTval].Item1);
                        pI[i].potIIpls[j] = curPl;

                        if (!curPl.IsValid)
                            continue;

                        int ind = intProp[indTval].Item2; // Index of the pI curve that it intersects with
                        Point3d intPtA = intProp[indTval].Item3.PointA;
                        double paramA = intProp[indTval].Item3.ParameterA;

                        // Checking if it connects to a local maximum or another kink in the curve
                        for (int k = 0; k < pI[ind].interTvals.Count; k++)
                        {
                            Point3d pt = pI[ind].pLine.PointAt(pI[ind].interTvals[k]);

                            if (intPtA.DistanceToSquared(pt) < tol * tol)
                            {
                                Line l = new Line(currentLocMaxPt, pt);
                                truss.Add(l);
                                connectLocalMax = true;
                                CorrectNode(ref nodes, pt, tol, out bool bol);

                                phaseIIcrvs.Add(curPl);

                                break;
                            }
                        }

                        if (!connectLocalMax)
                        {
                            Line l = new Line(currentLocMaxPt, intPtA);

                            if (pI[ind].startPt.DistanceToSquared(intPtA) > tol * tol &&
                                pI[ind].endPt.DistanceToSquared(intPtA) > tol * tol &&
                                (curPl.Length - offsetTol) < l.Length * 1.25)
                            {
                                pI[ind].interTvals.Add(paramA);

                                PhaseII newPII = new PhaseII(curPl, i, ind);
                                newPII.startPt = currentLocMaxPt;
                                newPII.endPt = intPtA;

                                newPII.normalAtEnd = Vector3d.CrossProduct(pI[ind].pLineCurve.TangentAt(paramA), l.Direction);
                                //newPII.normalAtEnd = Vector3d.CrossProduct(pI[k].pLineCurve.TangentAt(intEvent.ParameterA), currentPotIIcrv.TangentAt(intEvent.ParameterB));

                                CorrectNode(ref nodes, intPtA, tol, out bool bol);

                                pII.Add(newPII);

                                truss.Add(l);

                                phaseIIcrvs.Add(newPII.pLine);
                            }
                        }

                    }
                    // If it does not intersect anything but connects in a new way to a BC
                    else
                    {
                        foreach (Mesh dArea in dAreas)
                        {
                            if (dArea.ClosestPoint(pI[i].potIIpls[j].Last, out Point3d ptOnMesh, delta * 2) != -1)
                            {
                                //double bcTol = dArea.GetLength() / 2;

                                Line l = new Line(currentLocMaxPt, CorrectNode(ref nodes, ptOnMesh, tol, out bool bol));
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

            if (CancellationToken.IsCancellationRequested)
                return;
            // New lines from intersection points, but only in the normal direction (normal is not the correct word)
            // The direction that is perpendicular to the tangents of both curves at the intersection
            for (int i = 0; i < pII.Count; i++)
            {
                Vector3d[] dirs = new Vector3d[2];
                dirs[0] = pII[i].normalAtEnd;
                dirs[1] = -dirs[0];

                for (int j = 0; j < dirs.Length; j++)
                {
                    LoadPathCurve lPath = new LoadPathCurve(post.mask, pII[i].endPt, dirs[j], post.princpDir, post.princpStress);
                    lPath.ConstructLoadPath(scaleDelta, true);

                    //phaseIIcrvs.Add(lPath.loadPath);

                    if (!lPath.loadPath.IsValid)
                        continue;

                    Curve curCrv = PolylineToCurve(lPath.loadPath, 1);

                    List<Tuple<double, int, IntersectionEvent>> intProp = new List<Tuple<double, int, IntersectionEvent>>();

                    for (int k = 0; k < pI.Count; k++)
                    {
                        if (k == pII[i].indPIstart || k == pII[i].indPIend)
                            continue;

                        CurveIntersections crvInter = Intersection.CurveCurve(pI[k].pLineCurve, curCrv, intTol, 0.0);
                        if (crvInter != null && crvInter.Count > 0)
                        {
                            IntersectionEvent intEvent = crvInter[0];
                            intProp.Add(new Tuple<double, int, IntersectionEvent>(intEvent.ParameterB, k, intEvent));
                        }                    
                    }

                    if (intProp.Count > 0)
                    {
                        intProp = intProp.OrderBy(x => x.Item1).ToList();

                        int ind = intProp[0].Item2; // Index of the pI curve that it intersects with
                        Point3d intPtA = intProp[0].Item3.PointA;

                        Point3d pt = CorrectNode(ref nodes, intPtA, tol, out bool alreadyExists);
                        if (alreadyExists)
                        {
                            Line l = new Line(pII[i].endPt, pt);
                            if (l.IsValid)
                                truss.Add(l);
                        }
                    }
                }
            }



            if (CancellationToken.IsCancellationRequested)
                return;
            // Check if potII curves intersect
            for (int i = 0; i < pI.Count; i++)
            {
                for (int j = 0; j < pI[i].potIIpls.Count; j++)
                {
                    // Validity check for the polyline
                    if (!pI[i].potIIpls[j].IsValid)
                        continue;

                    Curve currentIIcrv = PolylineToCurve(pI[i].potIIpls[j], 1);

                    List<Tuple<double, int, IntersectionEvent>> intProp = new List<Tuple<double, int, IntersectionEvent>>();

                    for (int k = 0; k < pII.Count; k++)
                    {
                        if (i == pII[k].indPIstart)
                            continue;

                        CurveIntersections crvInter = Intersection.CurveCurve(pII[k].pLineCurve, currentIIcrv, intTol, 0.0);
                        if (crvInter != null && crvInter.Count > 0)
                        {
                            IntersectionEvent intEvent = crvInter[0];
                            intProp.Add(new Tuple<double, int, IntersectionEvent>(intEvent.ParameterB, k, intEvent));
                        }
                    }

                    if (intProp.Count > 0)
                    {
                        intProp = intProp.OrderBy(x => x.Item1).ToList();

                        int ind = intProp[0].Item2; // Index of the pII curve that it intersects with
                        Point3d intPtA = intProp[0].Item3.PointA;

                        Point3d sPt = pI[i].potIIplsStartPt[j];
                        Point3d ePt;

                        // Does not create a new node on pII curves, instead connects to closest end point
                        if (intPtA.DistanceToSquared(pII[ind].startPt) < intPtA.DistanceToSquared(pII[ind].endPt))
                            ePt = pII[ind].startPt;
                        else
                            ePt = pII[ind].endPt;

                        Line l = new Line(sPt, ePt);
                        truss.Add(l);
                        phaseIIcrvs.Add(pI[i].potIIpls[j]);
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

            if (CancellationToken.IsCancellationRequested)
                return;
            /////////////////////////////////////////////////////////////////////////////////////////
            // Phase III (handeling the beam case)
            for (int i = 0; i < dAreas.Count; i++)
            {
                if (dAreas.Count < 2)
                    break;

                Point3d[] vertices = dAreas[i].Vertices.ToPoint3dArray();
                Point3d startPt = new Point3d();

                for (int j = 0; j < vertices.Length; j++)
                {
                    startPt += vertices[j];
                }

                startPt /= vertices.Length;

                Vector3d avgNormal = new Vector3d();

                if (dAreas[i].FaceNormals.ComputeFaceNormals())
                {
                    for (int j = 0; j < dAreas[i].FaceNormals.Count; j++)
                    {
                        Vector3f mVec = dAreas[i].FaceNormals[j];
                        Vector3d vec = new Vector3d(mVec.X, mVec.Y, mVec.Z);
                        vec.Unitize();
                        avgNormal += vec;
                    }
                }

                avgNormal.Unitize();

                startPt -= avgNormal * offsetTol;   // The method to find this pt needs to be clarified

                Plane norPl = new Plane(startPt, avgNormal);

                Vector3d[] pIIIstartDirs = new Vector3d[4];
                pIIIstartDirs[0] = norPl.XAxis;
                pIIIstartDirs[1] = -norPl.XAxis;
                pIIIstartDirs[2] = norPl.YAxis;
                pIIIstartDirs[3] = -norPl.YAxis;

                for (int j = 0; j < pIIIstartDirs.Length; j++)
                {
                    LoadPathCurve lPath = new LoadPathCurve(post.mask, startPt, pIIIstartDirs[j], post.princpDir, post.princpStress);
                    lPath.ConstructLoadPath(scaleDelta, false);

                    if (!lPath.loadPath.IsValid)
                        continue;


                    phaseIIcrvs.Add(lPath.loadPath);

                    Curve potIII = PolylineToCurve(lPath.loadPath, 1);

                    List<Tuple<double, Point3d>> tValsAndPts = new List<Tuple<double, Point3d>>();

                    for (int ii = 0; ii < pI.Count; ii++)
                    {
                        if (potIII.ClosestPoint(pI[ii].endPt, out double t, intTol))
                        {
                            tValsAndPts.Add(new Tuple<double, Point3d>(t, pI[ii].endPt));
                        }
                    }

                    for (int ii = 0; ii < pII.Count; ii++)
                    {
                        if (potIII.ClosestPoint(pII[ii].endPt, out double t, intTol))
                        {
                            tValsAndPts.Add(new Tuple<double, Point3d>(t, pII[ii].endPt));
                        }
                    }

                    List<Point3d> pts = new List<Point3d>();

                    if (tValsAndPts.Count > 1)
                    {
                        tValsAndPts = tValsAndPts.OrderBy(x => x.Item1).ToList();

                        for (int ii = 0; ii < tValsAndPts.Count; ii++)
                        {
                            pts.Add(tValsAndPts[ii].Item2);
                        }
                    }

                    if (pts.Count > 1)
                    {
                        for (int ii = 0; ii < pts.Count - 1; ii++)
                        {
                            Line l = new Line(pts[ii], pts[ii + 1]);
                            truss.Add(l);
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
            public List<Plane> normalsAtT;
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
                normalsAtT = new List<Plane>();
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
            public int indPIstart;
            public int indPIend;
            public Vector3d normalAtEnd;

            public PhaseII(Polyline loadPath, int indPhaseIcrv, int indSecondPhaseIcrv)
            {
                pLine = loadPath;
                pLineCurve = PolylineToCurve(pLine, 1);
                indPIstart = indPhaseIcrv;
                indPIend = indSecondPhaseIcrv;
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

        private List<Point3d> FindLocalMaxPtsOnCurve(Polyline polyline, List<Mesh> dAreas, List<Mesh> nAreas)
        {
            List<Point3d> ptsOnCrv = new List<Point3d>();
            List<double> vonAtCurve = new List<double>();

            for (int i = 0; i < polyline.Count; i++)
            {
                vonAtCurve.Add(LERPvonMises(polyline[i]));
            }

            for (int i = 0; i < 10; i++)
            {
                vonAtCurve = SmoothData(vonAtCurve, 1);
            }

            double vonCutOff = vonAtCurve.Max() * 0.333;

            for (int i = 1; i < vonAtCurve.Count - 1; i++)
            {
                if (vonAtCurve[i] > vonAtCurve[i - 1] &&
                    vonAtCurve[i] > vonAtCurve[i + 1] &&
                    vonAtCurve[i] > vonCutOff)
                {
                    bool closeToBC = false;

                    double bcTol = tol;
                    if (tol < post.mask.delta * 3)
                        bcTol = post.mask.delta * 3;

                    foreach (Mesh dArea in dAreas)
                    {
                        
                        if (dArea.ClosestPoint(polyline[i], out Point3d ptOnMesh, bcTol) != -1)
                        {
                            closeToBC = true;
                            break;
                        }
                    }
                    foreach (Mesh nArea in nAreas)
                    {
                        if (nArea.ClosestPoint(polyline[i], out Point3d ptOnMesh, bcTol) != -1)
                        {
                            closeToBC = true;
                            break;
                        }
                    }

                    if (!closeToBC)
                    {
                        Vector3d pStress = LERPpStress(polyline[i]);
                        pStress.X = Math.Abs(pStress.X);
                        pStress.Y = Math.Abs(pStress.Y);
                        pStress.Z = Math.Abs(pStress.Z);
                        double factor = 0.1;

                        double maxVal = pStress.MaximumCoordinate;

                        if (pStress.X > pStress.Y && pStress.X > pStress.Z)
                        {
                            if (pStress.Y > maxVal * factor || pStress.Z > maxVal * factor)
                            {
                                localMax.Add(polyline[i]);
                                ptsOnCrv.Add(polyline[i]);
                            }
                        }
                        else if (pStress.Y > pStress.Z)
                        {
                            if (pStress.X > maxVal * factor || pStress.Z > maxVal * factor)
                            {
                                localMax.Add(polyline[i]);
                                ptsOnCrv.Add(polyline[i]);
                            }
                        }
                        else
                        {
                            if (pStress.Y > maxVal * factor || pStress.X > maxVal * factor)
                            {
                                localMax.Add(polyline[i]);
                                ptsOnCrv.Add(polyline[i]);
                            }
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

        private double LERPvonMises(Point3d pos)
        {
            Voxels<int> startVoxels = post.mask;
            Voxels<double> vonMises = post.vonMises;

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

            // Very basic break-statement
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

            double[] lerpVals = new double[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < INDs.Count; i++)
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
            if (!inside[0] || !inside[1] || !inside[2] || !inside[3] ||
                !inside[4] || !inside[5] || !inside[6] || !inside[7])
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
            double val = (1 - a) * (1 - b) * (1 - c) * lerpVals[0]
                    + a * (1 - b) * (1 - c) * lerpVals[1]
                    + (1 - a) * b * (1 - c) * lerpVals[2]
                    + a * b * (1 - c) * lerpVals[3]
                    + (1 - a) * (1 - b) * c * lerpVals[4]
                    + a * (1 - b) * c * lerpVals[5]
                    + (1 - a) * b * c * lerpVals[6]
                    + a * b * c * lerpVals[7];

            return val;
        }
        
        private Vector3d LERPpStress(Point3d pos)
        {
            Voxels<int> startVoxels = post.mask;
            Voxels<Vector3d> pStress = post.princpStress;

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

            // Very basic break-statement
            if ((startVoxels.cellValues[INDi0j0k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k0] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k0] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j0k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j0k1] & maskbit) == 0
                 && (startVoxels.cellValues[INDi0j1k1] & maskbit) == 0 && (startVoxels.cellValues[INDi1j1k1] & maskbit) == 0)
            {
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

            Vector3d[] lerpVals = new Vector3d[INDs.Count];

            bool[] inside = new bool[INDs.Count];
            for (int i = 0; i < INDs.Count; i++)
            {
                if ((startVoxels.cellValues[INDs[i]] & maskbit) == 0)
                    inside[i] = false;
                else
                {
                    inside[i] = true;
                    lerpVals[i].X = pStress.cellValues[INDs[i]].X;
                    lerpVals[i].Y = pStress.cellValues[INDs[i]].Y;
                    lerpVals[i].Z = pStress.cellValues[INDs[i]].Z;
                }
            }

            Vector3d avgVal = new Vector3d();
            double noInside = 0;
            if (!inside[0] || !inside[1] || !inside[2] || !inside[3] ||
                !inside[4] || !inside[5] || !inside[6] || !inside[7])
            {
                for (int i = 0; i < INDs.Count; i++)
                {
                    if (!inside[i])
                        continue;

                    avgVal.X += pStress.cellValues[INDs[i]].X;
                    avgVal.Y += pStress.cellValues[INDs[i]].Y;
                    avgVal.Z += pStress.cellValues[INDs[i]].Z;
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
            Vector3d val = new Vector3d();
            val.X = (1 - a) * (1 - b) * (1 - c) * lerpVals[0].X
                    + a * (1 - b) * (1 - c) * lerpVals[1].X
                    + (1 - a) * b * (1 - c) * lerpVals[2].X
                    + a * b * (1 - c) * lerpVals[3].X
                    + (1 - a) * (1 - b) * c * lerpVals[4].X
                    + a * (1 - b) * c * lerpVals[5].X
                    + (1 - a) * b * c * lerpVals[6].X
                    + a * b * c * lerpVals[7].X;

            val.Y = (1 - a) * (1 - b) * (1 - c) * lerpVals[0].Y
                    + a * (1 - b) * (1 - c) * lerpVals[1].Y
                    + (1 - a) * b * (1 - c) * lerpVals[2].Y
                    + a * b * (1 - c) * lerpVals[3].Y
                    + (1 - a) * (1 - b) * c * lerpVals[4].Y
                    + a * (1 - b) * c * lerpVals[5].Y
                    + (1 - a) * b * c * lerpVals[6].Y
                    + a * b * c * lerpVals[7].Y;

            val.Z = (1 - a) * (1 - b) * (1 - c) * lerpVals[0].Z
                    + a * (1 - b) * (1 - c) * lerpVals[1].Z
                    + (1 - a) * b * (1 - c) * lerpVals[2].Z
                    + a * b * (1 - c) * lerpVals[3].Z
                    + (1 - a) * (1 - b) * c * lerpVals[4].Z
                    + a * (1 - b) * c * lerpVals[5].Z
                    + (1 - a) * b * c * lerpVals[6].Z
                    + a * b * c * lerpVals[7].Z;

            return val;
        }

        private static Polyline CutPolyline(Polyline polyline, double t)
        {
            List<Point3d> pts = new List<Point3d>();
            for (int i = 0; i < t; i++)
            {
                pts.Add(polyline[i]);
            }

            Polyline cutPl = new Polyline(pts);
            return cutPl;
        }

        private bool FindRelevantIntersection(ref Polyline pLine, List<Curve> crvsA, int[] indSkip, out int indIntCrv, out IntersectionEvent intEvent)
        {
            Curve crvB = PolylineToCurve(pLine, 1);

            List<Tuple<double, int, IntersectionEvent>> intProp = new List<Tuple<double, int, IntersectionEvent>>();

            for (int i = 0; i < crvsA.Count; i++)
            {
                // Not searching against the curve with indicated index
                bool skip = false;
                for (int j = 0; j < indSkip.Length; j++)
                {
                    if (i == indSkip[j])
                    {
                        skip = true;
                        break;
                    }
                }
                if (skip)
                    continue;

                CurveIntersections crvInter = Intersection.CurveCurve(crvsA[i], crvB, intTol, 0.0);

                if (crvInter != null && crvInter.Count > 0)
                {
                    IntersectionEvent inter = crvInter[0];
                    intProp.Add(new Tuple<double, int, IntersectionEvent>(inter.ParameterB, i, inter));
                }
            }

            // At least one intersection is found, the first one is relevant, not the rest
            if (intProp.Count > 0)
            {
                intProp = intProp.OrderBy(x => x.Item1).ToList();

                Polyline potPl = null;
                int indTval = 0;

                // Kolla vinkeln så att den skapade linjen går ungefär i samma rikning som polylinen
                for (int i = 0; i < intProp.Count; i++)
                {
                    potPl = CutPolyline(pLine, intProp[i].Item1);
                    if (!potPl.IsValid)
                        continue;

                    Vector3d dirPl = potPl.Last - potPl.First;
                    dirPl.Unitize();
                    Vector3d dirLn = intProp[i].Item3.PointA - potPl.First;
                    dirLn.Unitize();

                    double cos = Math.Abs(dirPl * dirPl);

                    if (cos > 0.5)
                    {
                        indTval = i;
                        break;
                    }
                }

                if (potPl.IsValid)
                {
                    pLine = potPl;
                    indIntCrv = intProp[indTval].Item2;
                    intEvent = intProp[indTval].Item3;
                    return true;
                }
            }

            indIntCrv = -1;
            intEvent = null;
            return false;
        }

        // Old method, not used anymore
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

        public override WorkerInstance Duplicate() => new PSLWorker();

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Param.PostProcessingResultsGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            post = res.Value;

            var bcs = new List<Param.BoundaryConditionGoo>();
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

            double intTol = new double();
            DA.GetData(6, ref intTol);
            this.intTol = intTol;
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