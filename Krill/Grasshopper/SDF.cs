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
            pManager.AddMeshParameter("mesh", "m", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("d", "d", "", GH_ParamAccess.item);
            pManager.AddLineParameter("l", "l", "", GH_ParamAccess.item, Line.Unset);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("points", "pts", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("box", "b", "", GH_ParamAccess.list); 
            pManager.AddVectorParameter("union", "u", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            DA.GetData(0, ref mesh);
            double d = -1;
            DA.GetData(1, ref d);

            if (mesh is null)
                return;

            MeshToPoints meshToPoints = new MeshToPoints(mesh, d, 2);

            meshToPoints.FillBoundaryValues();
            meshToPoints.FillInternalValues();
            meshToPoints.RefineBoundaries();

            Krill.BoxSDF sdf = new BoxSDF(meshToPoints.voxels);
            sdf.ConstructSDF();

            //truss.ConstructSDF2();

            Line l = Line.Unset;
            DA.GetData(2, ref l);
            var boxes = new List<Vector3d>();
            if (l.IsValid)
            {
                for (int i = 0; i < 100; i++)
                {
                    boxes.Add(sdf.BoxValueAt(l.PointAt(i / (double)99)));
                }
            }

            DA.SetDataList(0, sdf.mask.GetPointsNotAt(0));
            DA.SetDataList(1, sdf.SDF.GetValues(sdf.mask));
            DA.SetDataList(2, boxes);
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