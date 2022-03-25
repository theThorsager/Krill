using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

using Krill.Containers;
using System.Linq;

namespace Krill.Grasshopper
{
    public class LinearPeridynamics2d : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public LinearPeridynamics2d()
          : base("LinearPeridynamics2d", "LinPeri2d",
              "Description",
              "Krill", "Solvers")
        {
            BaseWorker = new LinearPeridynamics2dWorker(null, this);
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

        bool WasHidden = false;
        public override void DocumentContextChanged(GH_Document document, GH_DocumentContext context)
        {
            base.DocumentContextChanged(document, context);

            switch (context)
            {
                case GH_DocumentContext.Loaded:
                case GH_DocumentContext.Open:
                    Hidden = WasHidden;
                    break;
                case GH_DocumentContext.Close:
                case GH_DocumentContext.Unloaded:
                    WasHidden = Hidden;
                    Hidden = true;
                    break;
                case GH_DocumentContext.Lock:
                case GH_DocumentContext.Unlock:
                case GH_DocumentContext.None:
                case GH_DocumentContext.Unknown:
                default:
                    break;
            }
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("surface", "surf", "", GH_ParamAccess.item);
            pManager.AddParameter(new Param.BoundaryConditionParam2d(), "BCs", "BCs", "", GH_ParamAccess.list);
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
            pManager.AddParameter(new Param.LinearSolutionParam2d());
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
            get { return new Guid("12243A86-75C4-4F02-BE22-184A4C9E950E"); }
        }
    }

    class LinearPeridynamics2dWorker : WorkerInstance
    {
        Brep brep { get; set; } = null;
        List<IBoundaryCondition2d> BCs { get; set; } = null;
        Settings settings { get; set; } = null;
        Voxel2dConduit conduit { get; set; }

        Param.LinearSolution2dGoo solution { get; set; } = null;
        public LinearPeridynamics2dWorker(Voxel2dConduit vcon, GH_Component parent) : base(null)
        {
            if (vcon is null)
                conduit = new Voxel2dConduit();
            else
                conduit = vcon;

            conduit.Enabled = true;
            Parent = parent;
        }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            ReportProgress(Id, 0);

            double tolerance = 1e-6;
            double logtol = Math.Log(tolerance);

            if (settings is null)
                settings = new Settings();

            if (CancellationToken.IsCancellationRequested) return;

            // make mesh to voxel thing
            MeshToPoints2d meshToPoints = new MeshToPoints2d(brep, settings.Delta, settings.delta);
            meshToPoints.FillBoundaryValues();
            meshToPoints.FillInternalValues();
            meshToPoints.RefineBoundaries();


            Voxels2d<int> mask = meshToPoints.voxels;

            // Create the neighbour offsets
            int[] neighOff = Utility.GetNeighbourOffsets2d(mask.n, settings.delta);

            // Construct the BBperi model to allocate data
            BBperi2d model = new BBperi2d(mask, settings.bond_stiffness, neighOff,
                settings.Delta * settings.Delta,
                settings.delta);
            //model.kernel = Utility.getKernelWeights(mask.n, settings.delta, neighOff);

            model.SetVolumes();

            int count = 0;
            foreach (IBoundaryCondition2d bc in BCs)
            {
                if (bc is BoundaryConditionDirechlet2d bcD)
                {
                    count++;
                    int tag = 4 << 8;
                    meshToPoints.SetBCD(bcD, settings.delta, tag);
                    bcD.tag = tag;
                    model.SetDirechlets(bcD);
                    //model.startVoxels.SetValues(mask, tag, 0);
                    Voxels2d<int>.MaskValues(mask, tag);
                }
                else if (bc is BoundaryConditionNuemann2d bcN)
                {
                    count++;
                    int tag = 1 << 3;
                    meshToPoints.SetBCN(bcN, tag);
                    model.SetNuemann(bcN, tag);
                    // removing the tag should not be needed :)
                    Voxels2d<int>.MaskValues(mask, tag);
                    //model.startVoxels.SetValues(mask, tag, 1);
                }
                else
                {
                    // error
                }
            }

            model.SetVolumesStiffness();

            conduit.mask = mask;
            conduit.component = Parent;
            conduit.SetDisplacments(model.dispVoxels);
            conduit.Update();

            model.SetDensities(settings.delta * settings.Delta, 4);

            double F = model.ComputeF(BCs, settings.E);

            // Load Stepping
            const int n_load_stepping = 50;
            int i = 0;
            for (; i < n_load_stepping; i++)
            {
                // compute the acceleration
                model.UpdateForce(Math.Min((double)(i + 1.0) / n_load_stepping, 1));
                // Verlet integration, to update pos
                double c = model.CalculateDampening();
                model.UpdateDisp(c);

                if (i % 50 == 0)
                {
                    if (CancellationToken.IsCancellationRequested) return;

                    conduit.SetDisplacments(model.dispVoxels);
                    conduit.Update();
                }
                ReportProgress(Id, (double)i / n_load_stepping);
            }

            double residual_scale = 1;
            // Try to converge
            for (; i < settings.n_timesteps; i++)
            {
                // compute the acceleration
                model.UpdateForce();
                // Verlet integration, to update pos
                double c = model.CalculateDampening();
                model.UpdateDisp(c);

                double residual = model.ComputeResidual(F);
                if (i % 50 == n_load_stepping % 50)
                {
                    if (CancellationToken.IsCancellationRequested) return;

                    conduit.SetDisplacments(model.dispVoxels);
                    conduit.Update();
                    if (i == n_load_stepping)
                    {
                        residual_scale = Math.Log(residual);
                        ReportProgress(Id, 0);
                    }
                    else
                        ReportProgress(Id, (Math.Log(residual) - residual_scale) / (logtol - residual_scale));
                }
                // Check termination criteria
                if (residual < tolerance)
                    break;
            }

            // Try to converge with relaxing of tension
            //ReportProgress(Id, 0);
            //model.relaxTension = true;
            //for (; i < settings.n_timesteps; i++)
            //{
            //    // compute the acceleration
            //    model.UpdateForce();
            //    // Verlet integration, to update pos
            //    double c = model.CalculateDampening();
            //    model.UpdateDisp(c * 0.5);

            //    double residual = model.ComputeResidual(F);
            //    if (i % 50 == n_load_stepping % 50)
            //    {
            //        if (CancellationToken.IsCancellationRequested) return;

            //        conduit.SetDisplacments(model.dispVoxels);
            //        conduit.Update();
            //        ReportProgress(Id, (Math.Log(residual) - residual_scale) / (logtol - residual_scale));
            //    }
            //    // Check termination criteria
            //    if (residual < tolerance)
            //        break;
            //}

            // Display data
            conduit.SetDisplacments(model.dispVoxels);
            conduit.Update();

            solution = new Param.LinearSolution2dGoo(
                new LinearSolution2d()
                {
                    mask = model.startVoxels,
                    displacments = model.dispVoxels,
                    peridelta = settings.delta,
                    elasticModulus = settings.E,
                    bondStiffness = settings.bond_stiffness,
                    nList = model.nlist,
                    springs = model.spring,
                    bodyload = model.bodyload,
                    utilization = model.utilization,
                    weighting = model.weighting,
                    relaxTension = model.relaxTension,
                    oldcuttoff = model.oldcuttoff
                });

            Done();
        }

        public override WorkerInstance Duplicate() => new LinearPeridynamics2dWorker(conduit, Parent);

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Brep brep = null;
            DA.GetData(0, ref brep);
            this.brep = brep;

            var bcs = new List<Param.BoundaryCondition2dGoo>();
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