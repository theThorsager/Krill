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
            pManager.AddNumberParameter("Tolerance", "tol", "", GH_ParamAccess.item);
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
        double tol { get; set; }    // I vissa delar av koden borde tol bytas ut mot delta*factor

        List<Polyline> pLine { get; set; } = null;
        List<Polyline> phaseIcrvs { get; set; } = null;
        List<Point3d> localMax { get; set; } = null;
        List<Polyline> phaseIIcrvs { get; set; } = null;
        List<Line> truss { get; set; } = null;
        public PSLWorker2d() : base(null)
        { }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (CancellationToken.IsCancellationRequested) return;
            if (post is null)
                return;
            // ...
            ReportProgress(Id, 0);

            const int maskbit = 0x000000FF;

            pLine = new List<Polyline>();
            phaseIcrvs = new List<Polyline>();
            localMax = new List<Point3d>();
            phaseIIcrvs = new List<Polyline>();
            truss = new List<Line>();

            Voxels2d<double> vonMises = post.vonMises;

            Vector3d[] startDirs = new Vector3d[4];
            startDirs[0] = Vector3d.XAxis;
            startDirs[1] = -Vector3d.XAxis;
            startDirs[2] = Vector3d.YAxis;
            startDirs[3] = -Vector3d.YAxis;

            double delta = post.mask.delta;

            List<Curve> dCrvs = new List<Curve>();
            List<Curve> nCrvs = new List<Curve>();

            if (CancellationToken.IsCancellationRequested)
                return;

            foreach (IBoundaryCondition2d bc in BCs)
            {
                if (bc is BoundaryConditionDirechlet2d bcD)                
                    dCrvs.Add(bcD.curve);
                else if (bc is BoundaryConditionNuemann2d bcN)
                    nCrvs.Add(bcN.curve);
            }

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

            List<PhaseI> pI = new List<PhaseI>();

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
                    }                                          
                }                
            }
            // Searching against N-boundary
            //for (int i = 0; i < nCrvs.Count; i++)
            //{
            //    for (int j = 0; j < pLine.Count; j++)
            //    {
            //        if (nCrvs[i].ClosestPoint(pLine[j].Last, out double t, delta * 2) &&
            //            !nCrvs[i].ClosestPoint(pLine[j].First, out double t2, delta * 2))
            //        {
            //            PhaseI newPI = new PhaseI(pLine[j]);
            //            newPI.startPt = newPI.pLine.First;
            //            newPI.endPt = nCrvs[i].PointAt(t);

            //            pI.Add(newPI);

            //            phaseIcrvs.Add(newPI.pLine);
            //        }
            //    }
            //}

            for (int i = 0; i < pLine.Count; i++)
            {
                for (int j = 0; j < startPoints.Count; j++)
                {
                    Point3d pt = pLine[i].ClosestPoint(startPoints[j]);

                    if (pt.DistanceToSquared(startPoints[j]) < tol*tol && pt.DistanceToSquared(pLine[i].First) > tol * tol)
                    {
                        Line l = new Line(pLine[i].First, startPoints[j]);
                        truss.Add(l);
                    }
                }
            }

            double minLocalMaxStress = vonMises.cellValues.Max() * 0.75;   // godtyckligt vald faktor

            int[] nList = Utility.GetNeighbourOffsets2d(vonMises.n, 5.1);

            // Kanske lägg det här i en egen funktion
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
                        if (vonMises.cellValues[i] < vonMises.cellValues[j])
                        {
                            localMaxBool = false;
                            break;
                        }
                    }

                    j = i - nList[ii];
                    if (j >= 0 && post.mask.cellValues[j] != 0)
                    {
                        if (vonMises.cellValues[i] < vonMises.cellValues[j])
                        {
                            localMaxBool = false;
                            break;
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
                            localMax.Add(pt3d);
                    }
                }
            }

            List<Point3d> pIIpts = new List<Point3d>();
            List<Vector3d> pInormals = new List<Vector3d>();
            List<int> pIcrvInd = new List<int>();

            for (int i = 0; i < pI.Count; i++)
            {
                for (int j = 0; j < localMax.Count; j++)
                {
                    double t = pI[i].pLine.ClosestParameter(localMax[j]);
                    Point3d clPt = pI[i].pLine.PointAt(t);                    

                    if (clPt.DistanceToSquared(localMax[j]) < tol * tol)
                    {
                        pIIpts.Add(clPt);

                        Vector3d tangent = pI[i].pLine.TangentAt(t);
                        Vector3d normal = Vector3d.CrossProduct(tangent, Vector3d.ZAxis);
                        pInormals.Add(normal);

                        pIcrvInd.Add(i);

                        pI[i].interTvals.Add(t);
                    }                                            
                }
            }

            List<Polyline>[] potIIcrvs = new List<Polyline>[pIIpts.Count];

            for (int i = 0; i < pIIpts.Count; i++)
            {
                Point3d pt1 = pIIpts[i] + pInormals[i] * tol;
                Point3d pt2 = pIIpts[i] - pInormals[i] * tol;

                potIIcrvs[i] = new List<Polyline>();

                for (int j = 0; j < startDirs.Length; j++)
                {
                    LoadPathCurve2d lPath1 = new LoadPathCurve2d(post.mask, pt1, startDirs[j], post.princpDir, post.princpStress);
                    LoadPathCurve2d lPath2 = new LoadPathCurve2d(post.mask, pt2, startDirs[j], post.princpDir, post.princpStress);
                    lPath1.ConstructLoadPath(scaleDelta);
                    lPath2.ConstructLoadPath(scaleDelta);

                    if (lPath1.loadPath.IsValid)
                        potIIcrvs[i].Add(lPath1.loadPath);
                    if (lPath2.loadPath.IsValid)
                        potIIcrvs[i].Add(lPath2.loadPath);
                }                
            }

            foreach (List<Polyline> IIcrvsPL in potIIcrvs)
            {
                Curve[] pIcrvs = new Curve[pI.Count];
                for (int i = 0; i < pI.Count; i++)
                    pIcrvs[i] = PolylineToCurve(pI[i].pLine, 1);                
                
                Curve[] IIcrvs = new Curve[IIcrvsPL.Count];
                for (int i = 0; i < IIcrvsPL.Count; i++)
                    IIcrvs[i] = PolylineToCurve(IIcrvsPL[i], 1);

                for (int i = 0; i < pIcrvInd.Count; i++)
                {
                    for (int j = 0; j < pI.Count; j++)
                    {
                        if (j == pIcrvInd[i])
                            continue;

                        for (int ii = 0; ii < IIcrvs.Length; ii++)
                        {
                            CurveIntersections crvInter = Intersection.CurveCurve(pIcrvs[j], IIcrvs[ii], 0.01, 0.0);
                            if (crvInter != null && crvInter.Count > 0)
                            {
                                phaseIIcrvs.Add(IIcrvsPL[ii]);

                                IntersectionEvent intEvent = crvInter[0];

                                pI[j].interTvals.Add(intEvent.ParameterA);

                                int ind = Array.IndexOf(potIIcrvs, IIcrvsPL);
                                Line l = new Line(pIIpts[ind], intEvent.PointA);
                                truss.Add(l);
                            }
                        }
                    }
                }
                
                //for (int i = 0; i < pIcrvs.Length; i++)
                //{
                //    // This needs to be redone to work for several points
                //    for (int j = 0; j < pIcrvInd.Count; j++)
                //    {
                //        if (i == pIcrvInd[j])
                //            goto BreakPoint;
                //    }

                //    for (int ii = 0; ii < IIcrvs.Length; ii++)
                //    {
                //        CurveIntersections crvInter = Intersection.CurveCurve(pIcrvs[i], IIcrvs[ii], 0.01, 0.0);
                //        if (crvInter != null && crvInter.Count > 0)
                //        {
                //            phaseIIcrvs.Add(IIcrvsPL[ii]);

                //            IntersectionEvent intEvent = crvInter[0];

                //            pI[i].interTvals.Add(intEvent.ParameterA);

                //            int ind = Array.IndexOf(potIIcrvs, IIcrvsPL);
                //            Line l = new Line(pIIpts[ind], intEvent.PointA);
                //            truss.Add(l);
                //        }
                //    }
                //BreakPoint:;
                //}
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


            // Phase III (handeling the beam case)
            for (int i = 0; i < dCrvs.Count; i++)
            {
                if (dCrvs.Count < 1)
                    break;

                Point3d startPt = dCrvs[i].PointAtNormalizedLength(0.5);

                for (int j = 0; j < startDirs.Length; j++)
                {
                    LoadPathCurve2d lPath = new LoadPathCurve2d(post.mask, startPt, startDirs[j], post.princpDir, post.princpStress);
                    lPath.ConstructLoadPath(scaleDelta);

                    if (lPath.loadPath.IsValid)
                    {
                        Curve[] pIcrvs = new Curve[pI.Count];
                        for (int ii = 0; ii < pI.Count; ii++)
                            pIcrvs[ii] = PolylineToCurve(pI[ii].pLine, 1);

                        Curve potIII = PolylineToCurve(lPath.loadPath, 1);

                        List<Point3d> ptsL = new List<Point3d>();

                        for (int ii = 0; ii < pIcrvs.Length; ii++)
                        {
                            CurveIntersections crvInter = Intersection.CurveCurve(pIcrvs[ii], potIII, tol, 0.0);
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

            

            if (CancellationToken.IsCancellationRequested)
                return;

            Done();
        }

        private Curve PolylineToCurve(Polyline polyline, int degree)
        {
            List<Point3d> pts = new List<Point3d>();
            for (int i = 0; i < polyline.Count; i++)
            {
                pts.Add(polyline[i]);
            }
            return Curve.CreateInterpolatedCurve(pts, degree);
        }

        internal class PhaseI
        {
            public Polyline pLine;
            public Point3d startPt;
            public Point3d endPt;
            public List<double> interTvals;

            public PhaseI(Polyline loadPath)
            {
                pLine = loadPath;
                interTvals = new List<double>();
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