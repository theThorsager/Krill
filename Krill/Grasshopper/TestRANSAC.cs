using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class TestRANSAC : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the TestRANSAC class.
        /// </summary>
        public TestRANSAC()
          : base("TestRANSAC", "RANSAC",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("DataPoints", "pts", "", GH_ParamAccess.list);
            pManager.AddIntegerParameter("MimNumDataPtsReq", "n", "", GH_ParamAccess.item);
            pManager.AddIntegerParameter("MaxNumIt", "k", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("ThresholdVal", "tol", "",GH_ParamAccess.item);
            pManager.AddIntegerParameter("NumClosePtsGoodEnough", "d", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("BestModel", "Ln", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> data = new List<Point3d>();
            DA.GetDataList(0, data);

            int n = new int();
            DA.GetData(1, ref n);

            int k = new int();
            DA.GetData(2, ref k);

            double tol = new double();
            DA.GetData(3, ref tol);

            int d = new int();
            DA.GetData(4, ref d);

            RANSAC r = new RANSAC();
            Line model = r.DoRANSAC(data, n, k, tol, d);

            if (model != null)
                DA.SetData(0, model);
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
            get { return new Guid("408F711E-24FD-41BE-A9B5-11BFB0948C66"); }
        }
    }
}