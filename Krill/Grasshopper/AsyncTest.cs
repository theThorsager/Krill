using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using GrasshopperAsyncComponent;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class AsyncTest : GH_AsyncComponent
    {
        /// <summary>
        /// Initializes a new instance of the AsyncTest class.
        /// </summary>
        public AsyncTest()
          : base("AsyncTest", "async",
              "Description",
              "Krill", "TestComponents")
        {
            BaseWorker = new testWorker(null);
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
            pManager.AddIntegerParameter("int", "i", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddIntegerParameter("int", "i", "", GH_ParamAccess.item);
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
            get { return new Guid("4605351A-104D-4DD5-96D7-19AC4C6D0578"); }
        }
    }

    class testWorker : WorkerInstance
    {
        int TheInt { get; set; } = 1;
        RhinoConduit conduit { get; set; }
        public testWorker(RhinoConduit rcon) : base(null)
        {
            if (rcon is null)
                conduit = new RhinoConduit();
            else
                conduit = rcon;

            conduit.Enabled = true;
        }

        public override void DoWork(Action<string, double> ReportProgress, Action Done)
        {
            if (CancellationToken.IsCancellationRequested) return;
            for (int i = 0; i < TheInt; i++)
            {
                if (CancellationToken.IsCancellationRequested) return;
                System.Threading.Thread.Sleep(10);

                double k = 0.01;
                conduit.Pt = new Point3d(Math.Cos(i * k), Math.Sin(i * k), 0);

                conduit.Update();

                ReportProgress(Id, ((double)i)/TheInt);
            }

            Done();
        }

        public override WorkerInstance Duplicate() => new testWorker(conduit);

        public override void GetData(IGH_DataAccess DA, GH_ComponentParamServer Params)
        {
            int i = 0;
            DA.GetData(0, ref i);
            TheInt = i;
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
            else
                DA.SetData(0, TheInt);
        }
    }
}