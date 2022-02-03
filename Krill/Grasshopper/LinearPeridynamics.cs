using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class LinearPeridynamics : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public LinearPeridynamics()
          : base("LinearPeridynamics", "LinPeri",
              "Description",
              "Krill", "Solvers")
        {
            BaseWorker = new LinearPeridynamicsWorker(null);
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            this.RequestCancellation();
            BaseWorker.SetData(null);   // Dirty way to disable the conduit
            base.RemovedFromDocument(document);
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("mesh", "m", "", GH_ParamAccess.item);
            //pManager.AddParameter(null, )
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddIntegerParameter("int", "i", "", GH_ParamAccess.item);
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
            get { return new Guid("4605351A-95A9-4DD5-96D7-19AC5C6D0578"); }
        }
    }

    class LinearPeridynamicsWorker : WorkerInstance
    {
        Mesh mesh { get; set; } = null;
        Settings settings { get; set; } = null;
        VoxelConduit conduit { get; set; }
        public LinearPeridynamicsWorker(VoxelConduit vcon) : base(null)
        {
            if (vcon is null)
                conduit = new VoxelConduit();
            else
                conduit = vcon;

            conduit.Enabled = true;
        }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (settings is null)
                settings = new Settings();

            if (CancellationToken.IsCancellationRequested) return;

            // make mesh to voxel thing
            MeshToPoints meshToPoints = new MeshToPoints(mesh, settings.Delta, settings.delta);
            meshToPoints.FillBoundaryValues();
            meshToPoints.FillInternalValues();

            Voxels<int> mask = meshToPoints.voxels;

            // Create the neighbour offsets
            int[] neighOff = Utility.GetNeighbourOffsets(mask.n, settings.delta);

            // Construct the BBperi model to allocate data
            BBperi model = new BBperi(mask, settings.bond_stiffness, neighOff, 
                settings.volume,
                settings.delta);

            conduit.mask = mask;

            int n = mask.n;

            model.SetDensities(settings.delta*settings.Delta);

            // Make a looop
            for (int i = 0; i < settings.n_timesteps; i++)
            {
                // compute the acceleration
                model.UpdateForce();
                // Lock one point for testing or something
                model.forceVoxels.cellValues[n * n * n / 2] = Rhino.Geometry.Vector3d.Zero;
                // Verlet integration, to update pos
                double c = model.CalculateDampening();
                model.UpdateDisp(c);

                if (i % 10 == 0)
                {
                    if (CancellationToken.IsCancellationRequested) return;

                    conduit.SetDisplacments(model.dispVoxels);
                    conduit.Update();
                    ReportProgress(Id, ((double)i)/settings.n_timesteps);
                }

                // Check termination criteria

            }

            // Display data
            conduit.SetDisplacments(model.dispVoxels);
            conduit.Update();

            Done();
        }

        public override WorkerInstance Duplicate() => new LinearPeridynamicsWorker(conduit);

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Mesh mesh = null;
            DA.GetData(0, ref mesh);
            this.mesh = mesh;
        }

        public override void SetData(IGH_DataAccess DA)
        {
            if (CancellationToken.IsCancellationRequested)
                return;

            if (DA is null && !(conduit is null))
            {
                conduit.Enabled = false;
                conduit.Update();
            }
        }
    }
}