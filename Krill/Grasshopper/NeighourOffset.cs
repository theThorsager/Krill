using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Krill.Grasshopper
{
    public class NeighourOffset : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public NeighourOffset()
          : base("NeighourOffset", "Noffset",
            "Description",
            "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("number", "n", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("radius", "r", "", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddIntegerParameter("offset", "offset", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int n = 0;
            double r = 0;
            DA.GetData(0, ref n);
            DA.GetData(1, ref r);
            if (n < 1 || r <= 0)
                return;

            Stopwatch watch = new Stopwatch();
            watch.Start();
            int[] offset = Utility.GetNeighbourOffsets(n, r);
            watch.Stop();

            DA.SetDataList(0, offset);

            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"Elapsed Time is {watch.ElapsedMilliseconds} ms");

        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => null;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("81E6EB96-95A9-3956-9CAD-95C647CBD90D");
    }
}