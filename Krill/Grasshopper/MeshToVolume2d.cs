using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace Krill.Grasshopper
{
    public class MeshToVolume2d : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MeshToVolume2d()
          : base("MeshToVolume2d", "MtV2d",
            "Description",
            "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("surface", "surface", "", GH_ParamAccess.item);
            pManager.AddMeshParameter("bc", "m", "", GH_ParamAccess.item);
            pManager[1].Optional = true;

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
            Brep mesh = null;
            DA.GetData(0, ref mesh);
            Mesh mesh2 = null;
            DA.GetData(1, ref mesh2);
            Stopwatch watch = new Stopwatch();

            //BoundaryCondition bcs = null;
            //if (mesh2 != null)
            //    bcs = new BoundaryCondition() { mesh = mesh2 };

            if (!(mesh is null))
            {

                watch.Start();
                MeshToPoints2d meshToPoints = new MeshToPoints2d(mesh, 0.05, 3.01);

                meshToPoints.FillBoundaryValues();
                watch.Stop();

                meshToPoints.FillInternalValues();

                meshToPoints.RefineBoundaries();

                //if (!(bcs is null))
                //    meshToPoints.SetBC(bcs, 3.01, 8);

                //conduit = new VoxelConduit();
                //conduit.mask = meshToPoints.voxels;
                //conduit.Enabled = true;
                //conduit.Update();
                DA.SetDataList(0, meshToPoints.voxels.GetPointsAt(1).Select(x => new Point3d(x.X, x.Y, 0)));
                DA.SetDataList(1, meshToPoints.voxels.GetPointsAt(2).Select(x => new Point3d(x.X, x.Y, 0)));
                //DA.SetDataList(2, meshToPoints.voxels.GetPointsAt(0).Select(x => new Point3d(x.X, x.Y, 0));
                //DA.SetDataList(1, meshToPoints.points);


                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"Elapsed Time is {watch.ElapsedMilliseconds} ms");

            }
        }

        VoxelConduit conduit;

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
        public override Guid ComponentGuid => new Guid("3BD67411-5A53-4CF2-ACC7-8C6363A81431");
    }
}