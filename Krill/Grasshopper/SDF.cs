using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class SDF : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SDF class.
        /// </summary>
        public SDF()
          : base("SDF", "SDF",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearPeridynamicsModelParam());
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.BoxSDFParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Param.LinearPeridynamicsModelGoo modelGoo = null;
            DA.GetData(0, ref modelGoo);

            if (modelGoo?.Value?.mask is null)
                return;

            Krill.BoxSDF sdf = new BoxSDF(modelGoo.Value.mask);
            sdf.ConstructSDF();

            DA.SetData(0, new Param.BoxSDFGoo(sdf));
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
            get { return new Guid("C70E8704-2345-435A-979D-F25260EB7E71"); }
        }
    }
}