using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class GhDerivateEigenVectors2d : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GhDerivateEigenVectors2d class.
        /// </summary>
        public GhDerivateEigenVectors2d()
          : base("GhDerivateEigenVectors2d", "dP",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearSolutionParam2d());
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("OriginalPositions", "orgPt", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("DerDir1", "dP1", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("DerDir2", "dP2", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Get Data
            Containers.LinearSolution2d linearSolution = null;
            Param.LinearSolution2dGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            linearSolution = res.Value;

            OutputResults2d result = new OutputResults2d(linearSolution);
            result.UpdateFakeStrains(linearSolution.displacments);
            result.UpdateStresses();
            result.UpdatePrincipalStresses();

            DerivateEigenVectors2d dEigenVecs = new DerivateEigenVectors2d(linearSolution.mask, result.stressTensor, result.princpStress, result.princpDir);

            List<Point3d> orgPoints = new List<Point3d>();
            List<Vector3d> dP1 = new List<Vector3d>();
            List<Vector3d> dP2 = new List<Vector3d>();

            const int maskbit = 0x000000FF;

            for (int i = 0; i < linearSolution.mask.n * linearSolution.mask.n; i++)
            {
                if ((linearSolution.mask.cellValues[i] & maskbit) == 0)
                    continue;

                Point2d pt = linearSolution.mask.IndexToPoint(i);
                orgPoints.Add(new Point3d(pt.X, pt.Y, 0));

                Vector3d[] dE = dEigenVecs.DerEigenVec(i);

                dP1.Add(dE[0]);
                dP2.Add(dE[1]);
            }

            if (orgPoints != null)
                DA.SetDataList(0, orgPoints);
            if (dP1 != null)
                DA.SetDataList(1, dP1);
            if (dP2 != null)
                DA.SetDataList(2, dP2);
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
            get { return new Guid("EF00B8BA-A508-4F9C-B844-3F62C96A1F1C"); }
        }
    }
}