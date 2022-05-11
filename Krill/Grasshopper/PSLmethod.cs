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
            pManager.AddCurveParameter("PhaseICrvs", "Icrvs", "", GH_ParamAccess.list);
            pManager.AddPointParameter("LocalMaxVonMises", "maxPts", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("PhaseIICrvs", "IIcrvs", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("PhaseIIICrvs", "IIIcrvs", "", GH_ParamAccess.list);
            pManager.AddLineParameter("InitialTruss", "truss", "", GH_ParamAccess.list);
            pManager.AddLineParameter("SupportLines", "dLines", "", GH_ParamAccess.list);
            pManager.AddLineParameter("LoadLines", "nLines", "", GH_ParamAccess.list);
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

        List<Polyline> phaseIcrvs { get; set; } = null;
        List<Point3d> localMax { get; set; } = null;
        List<Polyline> phaseIIcrvs { get; set; } = null;
        List<Polyline> phaseIIIcrvs { get; set; } = null;
        List<Line> truss { get; set; } = null;
        List<Line> dLines { get; set; } = null;
        List<Line> nLines { get; set; } = null;

        public PSLWorker() : base(null)
        { }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (CancellationToken.IsCancellationRequested) return;
            if (post is null)
                return;
            // ...
            ReportProgress(Id, 0);

            phaseIcrvs = new List<Polyline>();
            localMax = new List<Point3d>();
            phaseIIcrvs = new List<Polyline>();
            phaseIIIcrvs = new List<Polyline>();
            truss = new List<Line>();
            dLines = new List<Line>();
            nLines = new List<Line>();

            PSL psl = new PSL(post, BCs, startPoints, scaleDelta, tol, offsetTol, intTol);

            psl.DoPhaseI();

            if (CancellationToken.IsCancellationRequested)
                return;

            psl.DoPhaseII();

            if (CancellationToken.IsCancellationRequested)
                return;

            psl.DoPhaseIII();

            if (CancellationToken.IsCancellationRequested)
                return;

            psl.ConstructPhaseIlines();

            if (CancellationToken.IsCancellationRequested)
                return;

            for (int i = 0; i < psl.pI.Count; i++)
            {
                phaseIcrvs.Add(psl.pI[i].pLine);
                localMax.AddRange(psl.pI[i].locMaxPts);
            }                

            for (int i = 0; i < psl.pIIcrvs.Count; i++)
                phaseIIcrvs.Add(psl.pIIcrvs[i]);

            for (int i = 0; i < psl.pIIIcrvs.Count; i++)
                phaseIIIcrvs.Add(psl.pIIIcrvs[i]);

            truss.AddRange(psl.truss);
            dLines.AddRange(psl.supportLines);
            nLines.AddRange(psl.loadLines);
            

            if (CancellationToken.IsCancellationRequested)
                return;

            Done();
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

            if (!(phaseIcrvs is null))
                DA.SetDataList(0, phaseIcrvs);
            if (!(localMax is null))
                DA.SetDataList(1, localMax);
            if (!(phaseIIcrvs is null))
                DA.SetDataList(2, phaseIIcrvs);
            if (!(phaseIIIcrvs is null))
                DA.SetDataList(3, phaseIIIcrvs);
            if (!(truss is null))
                DA.SetDataList(4, truss);
            if (!(dLines is null))
                DA.SetDataList(5, dLines);
            if (!(nLines is null))
                DA.SetDataList(6, nLines);

        }
    }
}