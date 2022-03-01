using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class DualCurve : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the DualCurve class.
        /// </summary>
        public DualCurve()
          : base("DualCurve", "DualCurve",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("pline", "pl", "", GH_ParamAccess.item);
            pManager.AddIntegerParameter("n", "n", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("curveA", "cA", "", GH_ParamAccess.item);
            pManager.AddCurveParameter("curveB", "cB", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve curve = null;
            DA.GetData(0, ref curve);
            if (curve is null)
                return;


            if (!curve.TryGetPolyline(out Polyline pline))
                return;
            int n = 0;
            DA.GetData(1, ref n);

            Krill.DualCurve.ReduceToMinimumCurvature(pline, out List<Point3d> points, out List<Vector3d> tangents);
            Polyline resB;
            if (points.Count > 1)
                resB = Krill.DualCurve.ConstructDualCurve(points, tangents);
            else
                resB = new Polyline(new List<Point3d> { pline[0], pline[pline.Count - 1] });

            resB = Krill.DualCurve.ReduceCurve(resB, Math.PI / 180 * 12.5);

            DA.SetData(0, new Polyline(points));
            DA.SetData(1, resB);
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
            get { return new Guid("13072091-8921-4480-B2FB-B49E0752F85B"); }
        }
    }
}