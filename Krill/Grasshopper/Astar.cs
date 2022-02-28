using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class Astar : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Astar class.
        /// </summary>
        public Astar()
          : base("Astar", "Astar",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("mesh", "m", "", GH_ParamAccess.item);
            pManager.AddPointParameter("From", "a", "", GH_ParamAccess.item);
            pManager.AddPointParameter("To", "b", "", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("c", "c", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            DA.GetData(0, ref mesh);
            Point3d A = new Point3d();
            Point3d B = new Point3d();
            DA.GetData(1, ref A);
            DA.GetData(2, ref B);

            MeshToPoints meshToPoints = new MeshToPoints(mesh, 0.5, 3);
            meshToPoints.FillBoundaryValues();
            meshToPoints.FillInternalValues();
            meshToPoints.RefineBoundaries();

            Voxels<double> weights = new Voxels<double>(meshToPoints.voxels.origin, meshToPoints.voxels.delta, meshToPoints.voxels.n);
            for (int i = 0; i < weights.cellValues.Length; i++)
            {
                weights.cellValues[i] = meshToPoints.voxels.cellValues[i] > 0 ? 1000000000000.0 : 0.0;
            }

            Krill.Astar astar = new Krill.Astar(weights);
            List<int> path = astar.FindPath(weights.PointToIndex(A), weights.PointToIndex(B));

            Polyline pline = new Polyline();
            foreach (int i in path)
            {
                pline.Add(weights.IndexToPoint(i));
            }

            DA.SetData(0, pline);
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
            get { return new Guid("6872C21F-67B2-4855-9BD7-D5CFD15F41F6"); }
        }
    }
}