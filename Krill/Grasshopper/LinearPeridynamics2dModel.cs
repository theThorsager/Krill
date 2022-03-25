using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

using Krill.Containers;
using System.Linq;

namespace Krill.Grasshopper
{
    public class LinearPeridynamics2dModel : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public LinearPeridynamics2dModel()
          : base("LinearPeridynamics2dModel", "LinPeri2dModel",
              "Description",
              "Krill", "Solvers")
        {
            BaseWorker = new LinearPeridynamics2dModelWorker();
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            this.RequestCancellation();
            //BaseWorker.SetData(null);   // Dirty way to disable the conduit
            base.RemovedFromDocument(document);
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("surface", "s", "", GH_ParamAccess.item);
            pManager.AddParameter(new Param.BoundaryConditionParam2d(), "BCs", "BCs", "", GH_ParamAccess.list);
            pManager[1].Optional = true; 
            pManager.AddParameter(new Param.SettingsParam());
            pManager[2].Optional = true;
            pManager.AddBooleanParameter("PreivewMesh", "preview", "", GH_ParamAccess.item, true);
            pManager[3].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddIntegerParameter("int", "i", "", GH_ParamAccess.item);
            pManager.AddParameter(new Param.LinearPeridynamics2dModelParam());
            pManager.AddMeshParameter("Boundary Mesh", "BMesh", "", GH_ParamAccess.list);
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
            get { return new Guid("50E29E5E-0C15-4CA3-922F-18E1E6A1B48F"); }
        }
    }

    class LinearPeridynamics2dModelWorker : WorkerInstance
    {
        Brep mesh { get; set; } = null;
        List<IBoundaryCondition2d> BCs { get; set; } = null;
        Settings settings { get; set; } = null;
        List<Mesh> BoundaryMeshes { get; set; } = new List<Mesh>();
        bool preview = true;
        Krill.Grasshopper.Param.LinearPeridynamics2dModelGoo LinModel { get; set; } = null;
        public LinearPeridynamics2dModelWorker() : base(null)
        {
        }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            ReportProgress(Id, 0);

            if (settings is null)
                settings = new Settings();

            if (CancellationToken.IsCancellationRequested) return;

            // make mesh to voxel thing
            MeshToPoints2d meshToPoints = new MeshToPoints2d(mesh, settings.Delta, settings.delta);
            meshToPoints.FillBoundaryValues();
            if (CancellationToken.IsCancellationRequested) return;
            ReportProgress(Id, 0.33);
            meshToPoints.FillInternalValues();
            if (CancellationToken.IsCancellationRequested) return;
            ReportProgress(Id, 0.66);
            meshToPoints.RefineBoundaries();
            if (CancellationToken.IsCancellationRequested) return;
            ReportProgress(Id, 1);


            Voxels2d<int> mask = meshToPoints.voxels;

            if (preview)
            {
                ReportProgress(Id, 0);
                BoundaryMeshes.Add(Voxels2d<int>.GetBoundaryMesh(mask));
                ReportProgress(Id, 0.5);
                if (CancellationToken.IsCancellationRequested) return;

                var dummy = new Voxels2d<int>(mask.origin - mask.delta * new Vector2d(1,1), mask.delta, mask.n + 2);
                meshToPoints.voxels = dummy;

                foreach (IBoundaryCondition2d bc in BCs)
                {
                    if (CancellationToken.IsCancellationRequested) return;

                    if (bc is BoundaryConditionDirechlet2d bcD)
                    {
                        meshToPoints.SetBCD(bcD, settings.delta, 1);
                        BoundaryMeshes.Add(Voxels2d<int>.GetBoundaryMesh(dummy));
                        Voxels2d<int>.MaskValues(dummy, 1);
                    }
                    else if (bc is BoundaryConditionNuemann2d bcN)
                    {
                        meshToPoints.SetBCN(bcN, 1);
                        BoundaryMeshes.Add(Voxels2d<int>.GetBoundaryMesh(dummy));
                        Voxels2d<int>.MaskValues(dummy, 1);
                    }
                    else
                    {
                        // error
                    }
                }
                ReportProgress(Id, 1);

            }

            LinModel = new Param.LinearPeridynamics2dModelGoo(new Containers.LinearPeridynamics2dModel()
            {
                mask = mask,
                boundaryConditions = BCs,
                settings = settings,
                tag = new Random().Next(int.MaxValue)
            });

            Done();
        }

        public override WorkerInstance Duplicate() => new LinearPeridynamics2dModelWorker();

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Brep mesh = null;
            DA.GetData(0, ref mesh);
            this.mesh = mesh;

            var bcs = new List<Param.BoundaryCondition2dGoo>();
            DA.GetDataList(1, bcs);
            if (!(bcs is null))
                this.BCs = bcs.Select(x => x.Value).ToList();

            Param.SettingsGoo settings = null;
            if (DA.GetData(2, ref settings))
                this.settings = settings.Value;

            DA.GetData(3, ref preview);
        }

        public override void SetData(IGH_DataAccess DA)
        {
            if (!(LinModel is null))
                DA.SetData(0, LinModel);

            if (CancellationToken.IsCancellationRequested)
                return;

            if (!(BoundaryMeshes is null))
                DA.SetDataList(1, BoundaryMeshes);

            if (CancellationToken.IsCancellationRequested)
                return;
        }
    }
}