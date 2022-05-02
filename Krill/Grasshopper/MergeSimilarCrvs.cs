using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class MergeSimilarCrvs : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MergeSimilarCrvs class.
        /// </summary>
        public MergeSimilarCrvs()
          : base("MergeSimilarCrvs", "mergeCrv",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curves", "Crvs", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("Tolerance", "tol", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Merged curve", "crv", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            DA.GetDataList(0, curves);

            double tol = 0;
            DA.GetData(1, ref tol);

            List<Curve> mCrvs = new List<Curve>(curves);

            //Add if statment before loops to determine if the curves should be merged or not

            for (int i = 0; i < curves.Count - 1; i++)
            {
                List<Curve> newCrvs = new List<Curve>();

                for (int j = 0; j < mCrvs.Count - 1; j++)
                {
                    newCrvs.Add(Curve.CreateMeanCurve(mCrvs[j], mCrvs[j + 1]));
                }

                mCrvs = new List<Curve>(newCrvs);
            }

            Curve mCrv = mCrvs[0];

            if (mCrvs.Count != 1)
                mCrv = null;

            if (mCrv != null)
                DA.SetData(0, mCrv);
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
            get { return new Guid("51BB5DBD-9D59-483D-90B5-CAE7A49DB6F6"); }
        }
    }
}