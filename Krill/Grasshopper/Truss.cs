using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class Truss : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Truss class.
        /// </summary>
        public Truss()
          : base("Truss", "Truss",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("pts", "pts", "", GH_ParamAccess.list);
            pManager.AddIntegerParameter("a", "a", "", GH_ParamAccess.list);
            pManager.AddIntegerParameter("b", "b", "", GH_ParamAccess.list);
            pManager.AddBooleanParameter("locked", "locked", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("lines", "l", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("values", "vals", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var nodes = new List<Point3d>();
            var a = new List<int>();
            var b = new List<int>();
            var locked = new List<bool>();
            DA.GetDataList(0, nodes);
            DA.GetDataList(1, a);
            DA.GetDataList(2, b);
            DA.GetDataList(3, locked);

            Krill.Truss truss = new Krill.Truss(nodes, locked, a, b);

            //truss.model.Solve();
            truss.model.Solve_MPC();


            DA.SetDataList(0, truss.ToLines());
            DA.SetDataList(1, truss.GetAxialForces());

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
            get { return new Guid("237B1730-D267-4C97-BFA0-4F06A5173753"); }
        }
    }
}