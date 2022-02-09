using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

using Krill.Containers;
using System.Linq;

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
            pManager.AddParameter(new Param.BoundaryConditionParam(), "BCs", "BCs", "", GH_ParamAccess.list);
            pManager[1].Optional = true; 
            pManager.AddParameter(new Param.SettingsParam());
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddIntegerParameter("int", "i", "", GH_ParamAccess.item);
            pManager.AddParameter(new Param.LinearSolutionParam());
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
        List<IBoundaryCondition> BCs { get; set; } = null;
        Settings settings { get; set; } = null;
        VoxelConduit conduit { get; set; }

        Param.LinearSolutionGoo solution { get; set; } = null;
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

            

            Voxels<int> mask = meshToPoints.voxels;

            // Create the neighbour offsets
            int[] neighOff = Utility.GetNeighbourOffsets(mask.n, settings.delta);

            // Construct the BBperi model to allocate data
            BBperi model = new BBperi(mask, settings.bond_stiffness, neighOff, 
                settings.Delta * settings.Delta * settings.Delta,
                settings.delta);
            

            var BCD = new List<BoundaryConditionDirechlet>();
            var BCN = new List<BoundaryConditionNuemann>();
            

            foreach (IBoundaryCondition bc in BCs)
            {
                if (bc is BoundaryConditionDirechlet)
                {
                    BCD.Add(bc as BoundaryConditionDirechlet);
                    meshToPoints.SetBC(bc, settings.delta, BCD.Count << 8);
            
                    model.dispVoxels.SetValues(model.startVoxels, 0xFF00, BCD.Last().displacement);
                }
                else if (bc is BoundaryConditionNuemann)
                {
                    BCN.Add(bc as BoundaryConditionNuemann);
                    meshToPoints.SetBC(bc, settings.delta, BCN.Count << 4);
                }
                else
                {
                    // ?
                }
            }

            model.BCD = BCD;
            model.BCN = BCN;
            conduit.mask = mask;

            model.SetDensities(settings.delta*settings.Delta);

            double residual_scale = 1;
            // Make a looop
            for (int i = 0; i < settings.n_timesteps; i++)
            {
                // compute the acceleration
                model.UpdateForce();
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

            // List<BoundaryConditionNuemann> converted = ToNuemannBCs(BCs)
            List<BoundaryConditionNuemann> nuemanns = null;

            solution = new Param.LinearSolutionGoo(
                new LinearSolution() { 
                    mask = model.startVoxels, 
                    displacments = model.dispVoxels,
                    boundaryConditions = nuemanns
                });

            Done();
        }

        public override WorkerInstance Duplicate() => new LinearPeridynamicsWorker(conduit);

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Mesh mesh = null;
            DA.GetData(0, ref mesh);
            this.mesh = mesh;

            var bcs = new List<Param.BoundaryConditionGoo>();
            DA.GetDataList(1, bcs);
            if (!(bcs is null))
                this.BCs = bcs.Select(x => x.Value).ToList();

            Param.SettingsGoo settings = null;
            if (DA.GetData(2, ref settings))
                this.settings = settings.Value;
        }

        public override void SetData(IGH_DataAccess DA)
        {
            if (!(solution is null))
                DA.SetData(0, solution);

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