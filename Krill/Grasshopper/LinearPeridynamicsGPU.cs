using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

using Krill.Containers;
using System.Linq;
using System.Windows.Forms;
using GPUCompute;

namespace Krill.Grasshopper
{
    public class LinearPeridynamicsGPU : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public LinearPeridynamicsGPU()
          : base("LinearPeridynamicsGPU", "LinPeriGPU",
              "Description",
              "Krill", "Solvers")
        {
            BaseWorker = new LinearPeridynamicsGPUWorker(new VoxelConduit(), this);
        }

        public override bool IsPreviewCapable => true;

        public override void AddedToDocument(GH_Document document)
        {
            wrapper = new SilkWrapper();
            string errors = wrapper.Init();

            if (!(errors is null))
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, errors);

            base.AddedToDocument(document);
        }
        public override void RemovedFromDocument(GH_Document document)
        {
            this.RequestCancellation();
            BaseWorker.SetData(null);   // Dirty way to disable the conduit
            wrapper.Finilize();
            base.RemovedFromDocument(document);
        }
        public SilkWrapper wrapper;

        public override bool Locked { get => base.Locked; 
            set 
            {
                if (value)
                    this.RequestCancellation();
                base.Locked = value; 
            } 
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
                case 10:
                    Message = "Writing Buffers";
                    break;
                case 11:
                    Message = "Reading Buffers";
                    break;
                default:
                    break;
            }

            Rhino.RhinoApp.InvokeOnUiThread((Action)delegate
            {
                OnDisplayExpired(true);
            });
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
            pManager.AddParameter(new Param.LinearPeridynamicsModelParam());
            pManager.AddBooleanParameter("reset", "reset", "", GH_ParamAccess.item, false);
            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
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
            get { return new Guid("3E7F4465-7D00-4B32-97FF-C1B8CD4C9B9A"); }
        }

        internal BBperiState oldState = null;
        public object balanceLock = new object();
        public object wrapperLock = new object();
    }

    class LinearPeridynamicsGPUWorker : WorkerInstance
    {
        List<IBoundaryCondition> BCs { get; set; } = null;
        Settings settings { get; set; } = null;
        VoxelConduit conduit { get; set; }
        Param.LinearPeridynamicsModelGoo input { get; set; } = null;
        Param.LinearPeridynamicsModelGoo oldinput { get; set; } = null;

        Param.LinearSolutionGoo solution { get; set; } = null;

        LinearPeridynamicsGPU RealParent = null;
        public LinearPeridynamicsGPUWorker(VoxelConduit vcon, LinearPeridynamicsGPU parent) : base(null)
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

            Voxels<int> mask = input.Value.mask;

            // Create the neighbour offsets
            int[] neighOff = Utility.GetNeighbourOffsets(mask.n, settings.delta);

            // Construct the BBperi model to allocate data
            BBperi model = new BBperi(mask, settings.bond_stiffness, neighOff, 
                settings.Delta * settings.Delta * settings.Delta,
                settings.delta);

            int i = 0;
            object thelock = RealParent.balanceLock;
            lock (thelock)
            {
                BBperiState oldstate = RealParent?.oldState;

                if (!(oldstate is null))
                {
                    model.ApplyState(oldstate);
                    i = oldstate.i;
                }
                oldstate = new BBperiState(model);
                RealParent.oldState = oldstate;

                model.SetVolumes();

                int count = 0;
                MeshToPoints meshToPoints = new MeshToPoints(mask);

                foreach (IBoundaryCondition bc in BCs)
                {
                    if (bc is BoundaryConditionDirechlet bcD)
                    {
                        count++;
                        int tag = 4 << 8;
                        meshToPoints.SetBCD(bcD, settings.delta, tag);
                        bcD.tag = tag;
                        model.SetDirechlets(bcD);
                        //model.startVoxels.SetValues(mask, tag, 0);
                        Voxels<int>.MaskValues(mask, tag);
                    }
                    else if (bc is BoundaryConditionNuemann bcN)
                    {
                        count++;
                        int tag = 1 << 3;
                        meshToPoints.SetBCN(bcN, settings.delta, tag);
                        model.SetNuemann(bcN, tag);
                        // removing the tag should not be needed :)
                        Voxels<int>.MaskValues(mask, tag);
                        //model.startVoxels.SetValues(mask, tag, 1);
                    }
                    else
                    {
                        // error
                    }
                }

                conduit.mask = mask;
                conduit.component = RealParent;
                conduit.SetDisplacments(model.dispVoxels);
                conduit.Update();

                model.SetDensities(settings.delta * settings.Delta, 10);
                double F = model.ComputeF(BCs, settings.E);
                int n_particles = mask.cellValues.Count(x => x > 0);

                SilkWrapper wrap = RealParent.wrapper;
                double vol = settings.Delta * settings.Delta * settings.Delta;

                ReportProgress(Id, 10.0);
                var disp = RhinoVectorConversion.SetValues(model.dispVoxels.cellValues, mask.cellValues, neighOff.Length * 4 + 2);
                var vel = RhinoVectorConversion.SetValues(model.velVoxels.cellValues);
                var force = RhinoVectorConversion.SetValues(model.forceVoxels.cellValues);

                int xi_n = (int)Math.Floor(settings.delta) * 2 + 1;
                lock (RealParent.wrapperLock)
                {
                    wrap.AssignBuffers(
                    disp, vel, force,
                    RhinoVectorConversion.SetValues(model.densities.cellValues),
                    RhinoVectorConversion.SetValues(model.bodyload.cellValues.Select(x => x / vol).ToArray()),
                    RhinoVectorConversion.SetValues(model.spring.cellValues),
                    Utility.GetNeighbourXiGPU(settings.delta, settings.Delta),
                    xi_n, mask.n);

                    wrap.SetKernelArguments((int)settings.delta, (float)(settings.bond_stiffness * vol), mask.n);

                    ReportProgress(Id, 3.0);
                    double residual_scale = double.MinValue;
                    for (i = 0; i < settings.n_timesteps; i++)
                    {
                        wrap.EnqueueKernel(mask.n);

                        if (CancellationToken.IsCancellationRequested)
                        {
                            break;
                        }

                        if (i % 100 == 1)
                        {
                            double residual = wrap.CheckResidual(mask.n, F, n_particles);
                            if (residual < tolerance)
                                break;

                            double temp = Math.Log(residual);
                            residual_scale = temp > residual_scale ? temp : residual_scale;
                            ReportProgress(Id, 3.0 + Math.Max(0, (Math.Log(residual) - residual_scale) / (logtol - residual_scale)));

                            wrap.ReadDisp(disp, mask.n);
                            RhinoVectorConversion.GetValues(disp, ref model.dispVoxels.cellValues);
                            conduit.SetDisplacments(model.dispVoxels);
                            conduit.Update();
                        }
                    }

                    ReportProgress(Id, 11.0);
                    wrap.ReadBuffers(disp, vel, force, mask.n);

                    wrap.ReleaseBuffers();
                }

                RhinoVectorConversion.GetValues(disp, ref model.dispVoxels.cellValues);
                RhinoVectorConversion.GetValues(vel, ref model.velVoxels.cellValues);
                RhinoVectorConversion.GetValues(force, ref model.forceVoxels.cellValues);

                // Display data
                conduit.SetDisplacments(model.dispVoxels);
                conduit.Update();


                solution = new Param.LinearSolutionGoo(
                    new LinearSolution()
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
                        weighting = model.weighting
                    });


                oldinput = input;

                oldstate.LastUpdate(mask, neighOff, model, i);
            }
            Done();
        }

        public override WorkerInstance Duplicate() => new LinearPeridynamicsGPUWorker(conduit, RealParent);

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            Param.LinearPeridynamicsModelGoo tempInput = null;
            DA.GetData(0, ref tempInput);
            input = tempInput;

            bool reset = false;
            DA.GetData(1, ref reset);
            if (reset)
            {
                RealParent.balanceLock = new object();
                RealParent.oldState = null;
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