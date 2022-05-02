using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class CurveToTruss : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the DualCurve class.
        /// </summary>
        public CurveToTruss()
          : base("CurveToTruss", "CurveToTruss",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearSolutionParam());
            pManager.AddCurveParameter("plines", "pls", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("d", "d", "", GH_ParamAccess.item);
            pManager.AddIntegerParameter("n", "n", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("curves", "cs", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Param.LinearSolutionGoo lin = null;
            DA.GetData(0, ref lin);
            if (lin is null)
                return;


            List<Curve> curves = new List<Curve>();
            DA.GetDataList(1, curves);
            if (curves is null)
                return;

            List<Polyline> plines = new List<Polyline>();
            foreach (Curve curve in curves)
            {
                if (curve.TryGetPolyline(out Polyline pline))
                    plines.Add(pline);
                else
                    return;
            }
            double d = 0;
            DA.GetData(2, ref d);

            int n = 0;
            DA.GetData(3, ref n);

            var curvetoTruss = new Krill.CurvesToTruss2(lin.Value.utilization);

            for (int i = 0; i < n; i++)
                plines = curvetoTruss.Method(plines, d);

            DA.SetDataList(0, plines);
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
            get { return new Guid("D81F848C-31B7-4AF9-A272-B2694F0A9352"); }
        }
    }
}