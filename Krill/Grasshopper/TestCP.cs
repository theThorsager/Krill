using System;
using System.Collections.Generic;
using System.Diagnostics;
using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;
using System.Linq;

using Krill.Containers;

namespace Krill.Grasshopper
{
    public class TestCP : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public TestCP()
          : base("TestCP", "TestCP",
              "Description",
              "Krill", "Solvers")
        {
            BaseWorker = new TestCPWorker(null);
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
            pManager.AddNumberParameter("a", "a", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("h", "h", "", GH_ParamAccess.item);
            pManager.AddParameter(new Param.SettingsParam());
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("points", "pts", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("disp", "d", "", GH_ParamAccess.list);
            pManager.AddTextParameter("results", "r", "", GH_ParamAccess.item);
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
            get { return new Guid("63D5D3BC-C017-4751-AC6B-50F513F7C067"); }
        }
    }

    class TestCPWorker : WorkerInstance
    {
        Mesh mesh { get; set; } = null;
        Settings settings { get; set; } = null;
        VoxelConduit conduit { get; set; }
        double a = 1;
        double h = 1;

        List<Point3d> points = null;
        List<Vector3d> vectors = null;
        string results = null;
        public TestCPWorker(VoxelConduit vcon) : base(null)
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

            Stopwatch watchsetup = new Stopwatch();
            Stopwatch watchloop = new Stopwatch();

            double factor = 3;
            mesh = Mesh.CreateFromBox(
                new BoundingBox(
                    new Point3d(-factor * a, -factor * a, -factor * a), 
                    new Point3d(factor * a, factor * a, -0.01*a)), 1, 1, 1);

            watchsetup.Start();
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

            watchsetup.Stop();

            var bc = new BoundaryConditionDirechlet()
            {
                area = Mesh.CreateFromClosedPolyline(new Circle(a).ToNurbsCurve().ToPolyline(1e-6, 1e-3, 0.01, 0.1).ToPolyline()),
                displacement = new Vector3d(0, 0, h)
            };
            int tag = 1 << 8;
            meshToPoints.SetBC(bc, settings.delta, tag);
            bc.tag = tag;

            model.SetDirechlets(bc);
            model.startVoxels.SetValues(mask, tag, 0);

            Utility.SetValuesOutsideBBox(
                model.startVoxels, 
                model.dispVoxels, 
                mesh.GetBoundingBox(true), 
                0x00000100, // mask
                x => AnalyticalSolutions.CylinderPunch(x, a, h, 0.25));

            conduit.mask = mask;

            int n = mask.n;

            watchsetup.Start();
            model.SetDensities(settings.delta*settings.Delta);
            watchsetup.Stop();
            watchloop.Start();
            double residual_scale = 1;
            // Make a looop
            for (int i = 0; i < settings.n_timesteps; i++)
            {
                // compute the acceleration
                model.UpdateForce();
                double c = model.CalculateDampening();
                // Verlet integration, to update pos
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
                        ReportProgress(Id, (Math.Log(residual) - residual_scale) / logtol );
                }

                // Check termination criteria
                if (residual < tolerance)
                    break;
            }
            watchloop.Stop();
            // Display data
            conduit.SetDisplacments(model.dispVoxels);
            conduit.Update();

            points = Voxels<bool>.GetPoints(model.startVoxels, model.dispVoxels, 0, 0x000000FF);
            vectors = model.dispVoxels.GetValues(model.startVoxels, 0x000000FF);

            int power = 2; /* the 2 norm */
            double errornorm = Utility.ErrorNorm(points, vectors, a, power , 
                x => AnalyticalSolutions.CylinderPunch(x, a, h, 0.25));

            results = $"Cylinder Punch test: \n" +
                $"  Input values: a = {a}, Delta = {settings.Delta}, delta = {settings.delta} \n" +
                $"  Error in the {power} norm: {errornorm}\n" +
                $"  Time to setup: {watchsetup.ElapsedMilliseconds}ms\n" +
                $"  Time for the main loop: {watchloop.ElapsedMilliseconds}ms\n";

            Done();
        }

        public override WorkerInstance Duplicate() => new TestCPWorker(conduit);

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            DA.GetData(0, ref a);
            DA.GetData(1, ref h);

            Param.SettingsGoo settings = null;
            if (DA.GetData(2, ref settings))
                this.settings = settings.Value;
        }

        public override void SetData(IGH_DataAccess DA)
        {
            if (!(points is null))
                DA.SetDataList(0, points);

            if (!(vectors is null))
                DA.SetDataList(1, vectors);

            if (!(results is null))
                DA.SetData(2, results);

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