using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Krill;

namespace Krill.Grasshopper.Param
{
    public class Nuemann : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Settings class.
        /// </summary>
        public Nuemann()
          : base("Nuemann", "Nuemann",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            //pManager.AddParameter(new Param.LinearSolutionParam(), "solution", "sol", "", GH_ParamAccess.item);
            // Nuemann
            pManager.AddMeshParameter("area", "area", "", GH_ParamAccess.item);
            pManager.AddBooleanParameter("normal", "normal", "", GH_ParamAccess.item, true);
            pManager.AddVectorParameter("load", "load", "", GH_ParamAccess.item, Vector3d.ZAxis);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.BoundaryConditionParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh area = null;
            bool normal = true;
            Vector3d load = Vector3d.Unset;

            DA.GetData(0, ref area);
            DA.GetData(1, ref normal);
            DA.GetData(2, ref load);


            var nuemann = new Containers.BoundaryConditionNuemann()
            {
                area = area,
                normal = normal,
                load = load
            };

            DA.SetData(0, new BoundaryConditionGoo(nuemann));
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
            get { return new Guid("2AE931E1-1001-4D7D-B8EE-401B066EC1C0"); }
        }
    }
}