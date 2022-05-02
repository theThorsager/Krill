using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Krill.Grasshopper
{
    public class TestBonds : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public TestBonds()
          : base("TestBonds", "TestBonds",
            "Description",
            "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("m", "m", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("radius", "r", "", GH_ParamAccess.item);
            pManager.AddIntegerParameter("i", "i", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("bonds", "bonds", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double r = 0;
            int i = 0;
            Mesh mesh = null;
            DA.GetData(0, ref mesh);
            DA.GetData(1, ref r);
            DA.GetData(2, ref i);
            if (r <= 0 || i < 0)
                return;

            Stopwatch watch = new Stopwatch();
            watch.Start();
            MeshToPoints meshToPoints = new MeshToPoints(mesh, 1, r);
            int[] neighOff = Utility.GetNeighbourOffsets(meshToPoints.voxels.n, r);
            int n = meshToPoints.voxels.n;
            List<Line> results = new List<Line>();

            Point3d center = meshToPoints.voxels.IndexToPoint(Math.Min(i, n*n*n));
            Func<int, Point3d> f = ii => meshToPoints.voxels.IndexToPoint(Math.Max(Math.Min(ii, n * n * n), 0));
            for (int a = 0; a < neighOff.Length; a++)
            {
                int j = i + neighOff[a];
                results.Add(new Line(center, f(j)));

                j = i - neighOff[a];
                results.Add(new Line(center, f(j)));
            }

            watch.Stop();

            DA.SetDataList(0, results);

            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"Elapsed Time is {watch.ElapsedMilliseconds} ms");

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
        public override Guid ComponentGuid => new Guid("98BC92C2-CE55-49EC-9F6F-44B488CB9BB4");
    }
}