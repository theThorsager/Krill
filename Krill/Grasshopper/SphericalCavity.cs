using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class SphericalCavity : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SphericalCavity class.
        /// </summary>
        public SphericalCavity()
          : base("SphericalCavity", "SphereC",
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
            pManager.AddNumberParameter("sigma", "sigma", "", GH_ParamAccess.item);
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
            double sigma = 0;
            double E = 0;
            double nu = 0;
            DA.GetData(0, ref x);
            DA.GetData(1, ref a);
            DA.GetData(2, ref sigma);
            DA.GetData(3, ref E);
            DA.GetData(4, ref nu);

            Vector3d u = AnalyticalSolutions.SphericalCavity(x, a, sigma, E, nu);

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
            get { return new Guid("69D23F5B-5ED4-46EA-952C-FBBF729231BE"); }
        }
    }
}