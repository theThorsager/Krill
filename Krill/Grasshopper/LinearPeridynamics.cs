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
            pManager.AddMeshParameter("mesh", "m", "", GH_ParamAccess.item);
            pManager.AddMeshParameter("bcD", "bcD", "", GH_ParamAccess.item);
            pManager[1].Optional = true; 
            pManager.AddMeshParameter("bcN", "bcN", "", GH_ParamAccess.item);
            pManager[2].Optional = true;
            pManager.AddParameter(new Param.SettingsParam());
            pManager[3].Optional = true;
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
        BoundaryCondition bcsD { get; set; } = null;
        BoundaryCondition bcsN { get; set; } = null;
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
            double tolerance = 1e-6;
            double logtol = Math.Log(tolerance);

            if (settings is null)
                settings = new Settings();

            if (settings.delta <= Math.Sqrt(2))
            {
                this.Parent.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "delta is clamped to sqrt(2)");
                settings.delta = Math.Sqrt(2) + 0.0001;
            }
            if (CancellationToken.IsCancellationRequested) return;

            // make mesh to voxel thing
            MeshToPoints meshToPoints = new MeshToPoints(mesh, settings.Delta, settings.delta);
            meshToPoints.FillBoundaryValues();
            meshToPoints.FillInternalValues();

            if (!(bcsD is null))
                meshToPoints.SetBC(bcsD, settings.delta, 0x00000100);

            if (!(bcsN is null))
                meshToPoints.SetBC(bcsN, settings.delta, 3);

            Voxels<int> mask = meshToPoints.voxels;

            // Create the neighbour offsets
            int[] neighOff = Utility.GetNeighbourOffsets(mask.n, settings.delta);

            // Construct the BBperi model to allocate data
            BBperi model = new BBperi(mask, settings.bond_stiffness, neighOff, 
                settings.Delta * settings.Delta * settings.Delta,
                settings.delta);

            conduit.mask = mask;

            int n = mask.n;

            model.SetDensities(settings.delta*settings.Delta);

            double residual_scale = 1;
            // Make a looop
            for (int i = 0; i < settings.n_timesteps; i++)
            {
                // compute the acceleration
                model.UpdateForce();
                // Lock one point for testing or something
                //model.forceVoxels.cellValues[n * n * n / 2] = Rhino.Geometry.Vector3d.Zero;
                // Verlet integration, to update pos
                double c = model.CalculateDampening();
                model.UpdateDisp(c);

                double residual = model.ComputeResidual();
                if (i % 10 == 0)
                {
                    if (CancellationToken.IsCancellationRequested) return;

                    conduit.SetDisplacments(model.dispVoxels);
                    conduit.Update();
                    if (i == 0)
                    {
                        residual_scale = Math.Log(residual);
                        ReportProgress(Id, 0);
                    }
                    else
                        ReportProgress(Id, (Math.Log(residual) - residual_scale) / logtol);
                }

                // Check termination criteria
                if (residual < tolerance)
                    break;
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

            Mesh mesh2 = null;
            DA.GetData(1, ref mesh2);
            if (!(mesh2 is null))
                this.bcsD = new BoundaryCondition() { mesh = mesh2 };

            Mesh mesh3 = null;
            DA.GetData(2, ref mesh3);
            if (!(mesh3 is null))
                this.bcsN = new BoundaryCondition() { mesh = mesh3 };

            Param.SettingsGoo settings = null;
            if (DA.GetData(3, ref settings))
                this.settings = settings.Value;
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