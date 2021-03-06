using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

using Krill.Containers;
using System.Linq;

using System.Windows.Forms;

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
            BaseWorker = new LinearPeridynamics2dWorker(new Voxel2dConduit(), this);
        }

        public override bool IsPreviewCapable => true;

        public override void RemovedFromDocument(GH_Document document)
        {
            this.RequestCancellation();
            BaseWorker.SetData(null);   // Dirty way to disable the conduit
            base.RemovedFromDocument(document);
        }

        public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
        {
            base.AppendAdditionalMenuItems(menu);
            Menu_AppendItem(menu, "Cancel", (s, e) =>
            {
                RequestCancellation();
            });
        }

        public override void DisplayProgress(object sender, System.Timers.ElapsedEventArgs e)
        {
            if (Workers.Count == 0 || ProgressReports.Values.Count == 0)
            {
                return;
            }

            double min = double.MaxValue;
            foreach (var kvp in ProgressReports)
            {
                min = kvp.Value < min ? kvp.Value : min;
            }
            int i = (int)min;
            min -= i;
            switch (i)
            {
                case 0:
                    Message = min.ToString("0.00%");
                    break;
                case 1:
                    Message = "Applying BCs";
                    break;
                case 2:
                    Message = "Load stepping: " + min.ToString("0.00%");
                    break;
                case 3:
                    Message = "Converging: " + min.ToString("0.00%");
                    break;
                case 4:
                    Message = "Relaxing: " + min.ToString("0.00%");
                    break;
                default:
                    break;
            }

            Rhino.RhinoApp.InvokeOnUiThread((Action)delegate
            {
                OnDisplayExpired(true);
            });
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
            pManager.AddParameter(new Param.LinearPeridynamics2dModelParam());
            pManager.AddBooleanParameter("reset", "reset", "", GH_ParamAccess.item, false);
            pManager[1].Optional = true;

            //pManager.AddBrepParameter("surface", "surf", "", GH_ParamAccess.item);
            //pManager.AddParameter(new Param.BoundaryConditionParam2d(), "BCs", "BCs", "", GH_ParamAccess.list);
            //pManager[1].Optional = true;
            //pManager.AddParameter(new Param.SettingsParam());
            //pManager[2].Optional = true;
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

        internal BBperi2dState oldstate = null;
        public object balanceLock = new object();
    }

    class LinearPeridynamics2dWorker : WorkerInstance
    {
        List<IBoundaryCondition2d> BCs { get; set; } = null;
        Settings settings { get; set; } = null;
        Voxel2dConduit conduit { get; set; }
        Param.LinearPeridynamics2dModelGoo input { get; set; } = null;
        Param.LinearPeridynamics2dModelGoo oldinput { get; set; } = null;

        Param.LinearSolution2dGoo solution { get; set; } = null;
        LinearPeridynamics2d RealParent = null;
        public LinearPeridynamics2dWorker(Voxel2dConduit vcon, LinearPeridynamics2d parent) : base(null)
        {
            if (!(vcon is null))
            {
                conduit = vcon;
                conduit.Enabled = true;
            }
            RealParent = parent;
        }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (input is null)
                return;

            ReportProgress(Id, 1);

            BCs = input.Value.boundaryConditions;
            settings = input.Value.settings;

            double tolerance = 1e-6;
            double logtol = Math.Log(tolerance);

            if (settings is null)
                settings = new Settings();

            if (CancellationToken.IsCancellationRequested) return;

            Voxels2d<int> mask = input.Value.mask;

            // Create the neighbour offsets
            int[] neighOff = Utility.GetNeighbourOffsets2d(mask.n, settings.delta);

            // Construct the BBperi model to allocate data
            BBperi2d model = new BBperi2d(mask, settings.bond_stiffness2d, neighOff,
                settings.Delta * settings.Delta,
                settings.delta);
            //model.kernel = Utility.getKernelWeights(mask.n, settings.delta, neighOff);

            int i = 0;
            object thelock = RealParent.balanceLock;
            lock (thelock)
            {
                BBperi2dState oldstate = RealParent?.oldstate;

                if (!(oldstate is null))
                {
                    model.ApplyState(oldstate);
                    i = oldstate.i;
                }

                oldstate = new BBperi2dState(model);
                RealParent.oldstate = oldstate;

                model.SetVolumes();

                int count = 0;
                MeshToPoints2d meshToPoints = new MeshToPoints2d(mask);

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
                        meshToPoints.SetBCN(bcN, settings.delta, tag);
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
                conduit.component = RealParent;
                conduit.SetDisplacments(model.dispVoxels);
                conduit.Update();

                model.SetDensities(settings.delta * settings.Delta, 4);

                double F = model.ComputeF(BCs, settings.E);

                // Load Stepping
                ReportProgress(Id, 2);
                const int n_load_stepping = 50;
                for (; i < n_load_stepping && i < settings.n_timesteps; i++)
                {
                    // compute the acceleration
                    model.UpdateForce(Math.Min((double)(i + 1.0) / n_load_stepping, 1));
                    // Verlet integration, to update pos
                    double c = model.CalculateDampening();
                    model.UpdateDisp(c);

                    if (CancellationToken.IsCancellationRequested)
                    {
                        oldstate.LastUpdate(mask, neighOff, model, i);
                        return;
                    }

                    if (i % 50 == 0)
                    {
                        conduit.SetDisplacments(model.dispVoxels);
                        conduit.Update();
                    }
                    ReportProgress(Id, 2.0 + (double)i / n_load_stepping);
                }

                double residual_scale = double.MinValue;
                ReportProgress(Id, 3);
                // Try to converge
                for (; i < settings.n_timesteps; i++)
                {
                    // compute the acceleration
                    model.UpdateForce();
                    // Verlet integration, to update pos
                    double c = model.CalculateDampening();
                    model.UpdateDisp(c);

                    if (CancellationToken.IsCancellationRequested)
                    {
                        oldstate.LastUpdate(mask, neighOff, model, i);
                        return;
                    }

                    double residual = model.ComputeResidual(F);
                    if (i % 50 == n_load_stepping % 50)
                    {
                        conduit.SetDisplacments(model.dispVoxels);
                        conduit.Update();

                        double temp = Math.Log(residual);
                        residual_scale = temp > residual_scale ? temp : residual_scale;
                        ReportProgress(Id, 3.0 + Math.Max(0, (Math.Log(residual) - residual_scale) / (logtol - residual_scale)));

                        //if (i == n_load_stepping)
                        //{
                        //    residual_scale = Math.Log(residual);
                        //    ReportProgress(Id, 3);
                        //}
                        //else
                        //    ReportProgress(Id, 3.0 + Math.Max(0, (Math.Log(residual) - residual_scale) / (logtol - residual_scale)));
                    }
                    // Check termination criteria
                    if (residual < tolerance)
                        break;
                }

                ///////////////////////
                //Evolutionary structural optimisation

                //double ER = 0.005;

                //for (double RR = 0.01; RR < 0.6; RR += ER)
                //{
                //    for (int j = 0; j < 50; j++)
                //    {
                //        //if (!model.RemoveUnderUtilizedVoxels(RR))
                //        //    break;

                //        if (!model.RemoveUnstressedVoxels(RR, settings.E))
                //            break;

                //        for (; i < settings.n_timesteps; i++)
                //        {
                //            // compute the acceleration
                //            model.UpdateForce();
                //            // Verlet integration, to update pos
                //            double c = model.CalculateDampening();
                //            model.UpdateDisp(c * 0.5);

                //            double residual = model.ComputeResidual(F);
                //            if (i % 10 == n_load_stepping % 10)
                //            {
                //                if (CancellationToken.IsCancellationRequested) return;

                //                conduit.SetDisplacments(model.dispVoxels);
                //                conduit.Update();
                //                if (i == n_load_stepping)
                //                {
                //                    residual_scale = Math.Log(residual);
                //                    ReportProgress(Id, 0);
                //                }
                //                else
                //                    ReportProgress(Id, (Math.Log(residual) - residual_scale) / (logtol - residual_scale));
                //            }
                //            // Check termination criteria
                //            if (residual < tolerance)
                //                break;
                //        }
                //    }
                //}

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
                //        ReportProgress(Id, 4.0  + Math.Max(0 ,(Math.Log(residual) - residual_scale) / (logtol - residual_scale)));
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
                        bondStiffness = settings.bond_stiffness2d,
                        nList = model.nlist,
                        springs = model.spring,
                        bodyload = model.bodyload,
                        utilization = model.utilization,
                        weighting = model.weighting,
                        relaxTension = model.relaxTension,
                        oldcuttoff = model.oldcuttoff
                    });

                oldinput = input;

                oldstate.LastUpdate(mask, neighOff, model, i);
            }
            Done();
        }

        public override WorkerInstance Duplicate() => new LinearPeridynamics2dWorker(conduit, RealParent);

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Param.LinearPeridynamics2dModelGoo tempInput = null;
            DA.GetData(0, ref tempInput);
            input = tempInput;

            bool reset = false;
            DA.GetData(1, ref reset);
            if (reset)
            {
                RealParent.balanceLock = new object();
                RealParent.oldstate = null;
            }
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