using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

using Krill.Containers;

namespace Krill.Grasshopper
{
    public class LoadPaths : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public LoadPaths()
          : base("LoadPaths", "LoadP",
              "Description",
              "Krill", "Utility")
        {
            BaseWorker = new LoadPathsWorker();
        }
        
        public override void RemovedFromDocument(GH_Document document)
        {
            this.RequestCancellation();
            BaseWorker.SetData(null);   // Dirty way to disable the conduit
            base.RemovedFromDocument(document);
        }

        public override bool Locked { get => base.Locked; 
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
            pManager.AddPointParameter("StartPoints", "pts", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("StartVectors", "vecs", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("ScaleFactorForDelta", "sf", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("curve", "c", "", GH_ParamAccess.list);
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
            get { return new Guid("E71269F1-683A-4A5B-9998-13D385DCDD94"); }
        }
    }

    class LoadPathsWorker : WorkerInstance
    {
        Containers.PostProcessingResults post = null;
        List<Polyline> pLine { get; set; } = null;
        List<Point3d> startPoints { get; set; }
        List<Vector3d> startVectors { get; set; }
        double scaleDelta { get; set; }
        public LoadPathsWorker() : base(null)
        { }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (CancellationToken.IsCancellationRequested) return;
            if (post is null)
                return;
            // ...
            ReportProgress(Id, 0);
            pLine = new List<Polyline>();

            //OutputResults outputR = new OutputResults(post);
            //outputR.UpdateFakeStrains(post.displacments);
            //outputR.UpdateStresses();
            //outputR.UpdateVonMises();
            //outputR.UpdatePrincipalStresses();

            Voxels<int> startVoxels = post.mask;
            Voxels<Vector3d[]> princpDirections = post.princpDir;
           
            int n = startVoxels.n;

            if (CancellationToken.IsCancellationRequested)
                return;

            for (int i = 0; i < startPoints.Count; i++)
            {
                ReportProgress(Id, (double)i / startPoints.Count);

                if (CancellationToken.IsCancellationRequested)
                    return;

                LoadPathCurve lPath = new LoadPathCurve(startVoxels, startPoints[i], startVectors[i], princpDirections, post.princpStress);

                lPath.ConstructLoadPath(scaleDelta, false);
                pLine.Add(lPath.loadPath);
            }
            ReportProgress(Id, 1);

            if (CancellationToken.IsCancellationRequested)
                return;

            Done();
        }

        public override WorkerInstance Duplicate() => new LoadPathsWorker();

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Param.PostProcessingResultsGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            post = res.Value;

            List<Point3d> startPts = new List<Point3d>();
            DA.GetDataList(1, startPts);
            startPoints = startPts;

            List<Vector3d> startVecs = new List<Vector3d>();
            DA.GetDataList(2, startVecs);
            startVectors = startVecs;

            double scale = new double();
            DA.GetData(3, ref scale);
            scaleDelta = scale;
        }

        public override void SetData(IGH_DataAccess DA)
        {
            if (CancellationToken.IsCancellationRequested)
                return;

            if (!(pLine is null))
                DA.SetDataList(0, pLine);
        }
    }
}