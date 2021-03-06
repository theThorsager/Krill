using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Krill;

namespace Krill.Grasshopper.Param
{
    public class LinearSolution : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Settings class.
        /// </summary>
        public LinearSolution()
          : base("LinearSolution", "LinSol",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearSolutionParam(), "solution", "sol", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("points", "pts", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("displacements", "disp", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("utilization", "util", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("weighting", "weighting", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var res = new Param.LinearSolutionGoo();
            DA.GetData(0, ref res);
            if (res is null || res.Value is null)
                return;

            var points = Voxels<bool>.GetPoints(res.Value.mask, res.Value.displacments, 0, 0x000000FF);
            var displacemnts = res.Value.displacments.GetValues(res.Value.mask, 0x000000FF);
            var utilisations = res.Value.utilization?.GetValues(res.Value.mask, 0x000000FF);
            var weighting = res.Value.weighting?.GetValues(res.Value.mask, 0x000000FF);

            DA.SetDataList(0, points);
            DA.SetDataList(1, displacemnts);
            DA.SetDataList(2, utilisations);
            DA.SetDataList(3, weighting);
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
            get { return new Guid("CA0FDFF5-F32D-43C9-8285-3CECD5DC4C3F"); }
        }
    }
}