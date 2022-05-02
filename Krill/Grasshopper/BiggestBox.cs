using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class BiggestBox : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the BiggestBox class.
        /// </summary>
        public BiggestBox()
          : base("BiggestBox", "BiggestBox",
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
            pManager.AddPointParameter("point", "pt", "", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBoxParameter("box", "box", "", GH_ParamAccess.item);
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

            Krill.Truss truss = new Krill.Truss(nodes, locked,  a, b);

            Mesh m = null;
            DA.GetData(0, ref m);

            Point3d pt = Point3d.Unset;
            DA.GetData(1, ref pt);

            MeshToPoints meshToPoints = new MeshToPoints(m, 1, 5);

            meshToPoints.FillBoundaryValues();
            meshToPoints.FillInternalValues();
            meshToPoints.RefineBoundaries();

            truss.mask = meshToPoints.voxels;

            BoxSDF sdf = new BoxSDF(meshToPoints.voxels);
            sdf.ConstructSDF();

            Vector3d v = sdf.BoxValueAt(pt);
            BoundingBox bbox = new BoundingBox(pt - v * 0.5, pt + 0.5 * v);

            DA.SetData(0, bbox);
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
            get { return new Guid("CB785553-A8B2-49B6-B7EB-B64A6B37C1D7"); }
        }
    }
}