using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class AreaCubeTests : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the AreaCubeTests class.
        /// </summary>
        public AreaCubeTests()
          : base("AreaCubeTests", "AreaCubeTests",
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
            pManager.AddNumberParameter("areas", "areas", "", GH_ParamAccess.list);
            pManager.AddLineParameter("lines", "l", "", GH_ParamAccess.list);
            pManager.AddBoxParameter("box", "b", "", GH_ParamAccess.list);
            pManager.AddPointParameter("pts", "pts", "", GH_ParamAccess.list);
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
            DA.GetDataList(1, nodes);
            DA.GetDataList(2, a);
            DA.GetDataList(3, b);
            DA.GetDataList(4, locked);

            Krill.Truss truss = new Krill.Truss(nodes, locked, a, b);

            Mesh m = null;
            DA.GetData(0, ref m);

            MeshToPoints meshToPoints = new MeshToPoints(m, 1, 5);

            meshToPoints.FillBoundaryValues();
            meshToPoints.FillInternalValues();
            meshToPoints.RefineBoundaries();

            truss.SetVoxels(meshToPoints.voxels);

            truss.areas[0] = 12.0;
            if (truss.FindRestOfArea())
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "The elements do not fit in the system.");
            }


            DA.SetDataList(0, truss.areas);
            DA.SetDataList(1, truss.ToLines());
            DA.SetDataList(2, truss.boundingBoxes);
            DA.SetDataList(3, meshToPoints.voxels.GetPointsNotAt(0));
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
            get { return new Guid("8B12688E-EF0F-407E-8EB7-90E577010DF4"); }
        }
    }
}