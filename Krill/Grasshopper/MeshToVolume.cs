using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Krill.Grasshopper
{
    public class MeshToVolume : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MeshToVolume()
          : base("MeshToVolume", "MtV",
            "Description",
            "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("mesh", "mesh", "", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("ptsB", "ptsB", "", GH_ParamAccess.list);
            pManager.AddPointParameter("ptsI", "ptsI", "", GH_ParamAccess.list);
            pManager.AddPointParameter("ptsO", "ptsO", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            DA.GetData(0, ref mesh);
            Stopwatch watch = new Stopwatch();

            if (!(mesh is null))
            {
                MeshToPoints meshToPoints = new MeshToPoints(mesh, 0.5);

                meshToPoints.FillBoundaryValues();

                meshToPoints.FillInternalValues();
                watch.Start();

                DA.SetDataList(0, meshToPoints.GetPointsAt(1));
                DA.SetDataList(1, meshToPoints.GetPointsAt(2));
                DA.SetDataList(2, meshToPoints.GetPointsAt(0));
                //DA.SetDataList(1, meshToPoints.points);
                watch.Stop();


                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"Elapsed Time is {watch.ElapsedMilliseconds} ms");

            }
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => null;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("81E6EB96-95A9-4254-9CAD-95C647CBD90D");
    }
}