using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Krill;

namespace Krill.Grasshopper.Param
{
    public class DirechletPt : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Settings class.
        /// </summary>
        public DirechletPt()
          : base("DirechletPt", "DirechletPt",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // Direchlet
            pManager.AddPointParameter("point", "pt", "", GH_ParamAccess.item);
            pManager.AddBooleanParameter("x", "x", "", GH_ParamAccess.item, true);
            pManager.AddBooleanParameter("y", "y", "", GH_ParamAccess.item, true);
            pManager.AddBooleanParameter("z", "z", "", GH_ParamAccess.item, true);
            //pManager.AddVectorParameter("displacment", "disp", "", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.DiscreteBoundaryConditionParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var pt = new Point3d();
            bool x = true;
            bool y = true;
            bool z = true;

            DA.GetData(0, ref pt);
            DA.GetData(1, ref x);
            DA.GetData(2, ref y);
            DA.GetData(3, ref z);


            var direchlet = new Containers.DiscreteBoundaryConditionVariables()
            {
                point = pt,
                lockX = x,
                lockY = y,
                lockZ = z
            };

            DA.SetData(0, new DiscreteBoundaryConditionGoo(direchlet));
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
            get { return new Guid("3BB13152-E5D2-4AC3-AC8D-1012DF30E0EC"); }
        }
    }
}