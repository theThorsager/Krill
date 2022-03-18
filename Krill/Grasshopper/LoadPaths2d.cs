using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

using Krill.Containers;

namespace Krill.Grasshopper
{
    public class LoadPaths2d : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public LoadPaths2d()
          : base("LoadPaths2d", "LoadP2d",
              "Description",
              "Krill", "Solvers")
        {
            BaseWorker = new LoadPathsWorker2d();
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
            pManager.AddParameter(new Param.LinearSolutionParam2d());
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
            get { return new Guid("AEB3A86D-C9A7-4FD0-87F5-442C4293C846"); }
        }
    }

    class LoadPathsWorker2d : WorkerInstance
    {
        Containers.LinearSolution2d linearSolution = null;
        List<Polyline> pLine { get; set; } = null;
        List<Point3d> startPoints { get; set; }
        List<Vector3d> startVectors { get; set; }
        double scaleDelta { get; set; }
        public LoadPathsWorker2d() : base(null)
        { }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (CancellationToken.IsCancellationRequested) return;
            if (linearSolution is null)
                return;
            // ...
            ReportProgress(Id, 0);
            pLine = new List<Polyline>();

            OutputResults2d outputR = new OutputResults2d(linearSolution);
            outputR.UpdateFakeStrains(linearSolution.displacments);
            outputR.UpdateStresses();
            outputR.UpdateVonMises();
            outputR.UpdatePrincipalStresses();

            Voxels2d<int> startVoxels = linearSolution.mask;
            Voxels2d<Vector3d[]> princpDirections = outputR.princpDir;
            Voxels2d<Vector3d> princpStress = outputR.princpStress;
           
            int n = startVoxels.n;

            if (CancellationToken.IsCancellationRequested)
                return;

            for (int i = 0; i < startPoints.Count; i++)
            {
                if (CancellationToken.IsCancellationRequested)
                    return;
                LoadPathCurve2d lPath = new LoadPathCurve2d(startVoxels, startPoints[i], startVectors[i], princpDirections, princpStress);

                lPath.ConstructLoadPath(scaleDelta);

                pLine.Add(lPath.loadPath);
            }

            if (CancellationToken.IsCancellationRequested)
                return;

            Done();
        }

        public override WorkerInstance Duplicate() => new LoadPathsWorker2d();

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Param.LinearSolution2dGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            linearSolution = res.Value;

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