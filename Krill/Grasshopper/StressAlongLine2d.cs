using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class StressAlongLine2d : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the StressAlongLine2d class.
        /// </summary>
        public StressAlongLine2d()
          : base("StressAlongLine2d", "SaL2d",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearSolutionParam2d());
            pManager.AddLineParameter("Line", "L", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("StressAlongLine", "SaL", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Containers.LinearSolution2d linearSolution = null;
            Param.LinearSolution2dGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            linearSolution = res.Value;

            Line line = new Line();
            DA.GetData(1, ref line);

            StressLine2d stressLine = new StressLine2d(linearSolution);

            List<double> stresses = stressLine.LERPstressAlongLine(line);

            if (stresses != null)
                DA.SetDataList(0, stresses);
        }

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
            get { return new Guid("47B09452-F075-4C6E-AECC-908A67B20325"); }
        }
    }
}