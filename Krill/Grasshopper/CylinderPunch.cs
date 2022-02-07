using System;
using System.Numerics;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;


namespace Krill.Grasshopper
{
    public class CylinderPunch : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SphericalCavity class.
        /// </summary>
        public CylinderPunch()
          : base("CylinderPunch", "CylinderP",
              "Description",
              "Krill", "AnalyticalSolutions")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("pt", "pt", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("a", "a", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("h", "h", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("E", "E", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("nu", "nu", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("u", "u", "", GH_ParamAccess.item);
            // Stress ? :)
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Point3d x = new Point3d();
            double a = 0;
            double h = 0;
            double E = 0;
            double nu = 0;
            DA.GetData(0, ref x);
            DA.GetData(1, ref a);
            DA.GetData(2, ref h);
            DA.GetData(3, ref E);
            DA.GetData(4, ref nu);

            Vector3d u = AnalyticalSolutions.CylinderPunch(x, a, h, nu);

            DA.SetData(0, u);
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
            get { return new Guid("E4F8945D-0170-48A7-9523-948D5F2BB541"); }
        }
    }
}