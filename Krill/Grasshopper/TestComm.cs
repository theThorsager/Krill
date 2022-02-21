using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace Krill.Grasshopper
{
    public class TestComm : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public TestComm()
          : base("TestComm", "TestComm",
            "Description",
            "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("n", "n", "", GH_ParamAccess.item, 10);
            pManager.AddBooleanParameter("com", "com", "", GH_ParamAccess.item);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("pts", "pts", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool com = false;
            DA.GetData(1, ref com);

            var voxeldic = new Dictionary<Coord, Voxels<Point3d>>();

            int padding = 3;
            int nwidth = 10;
            DA.GetData(0, ref nwidth);
            double delta = 0.1;
            double f = nwidth * delta;

            Voxels<Point3d> voxel;
            for (int k = -1; k < 2; k++)
            {
                for (int j = -1; j < 2; j++)
                {
                    for (int i = -1; i < 2; i++)
                    {
                        voxel = new Voxels<Point3d>(new Point3d(i*f - padding*delta, j* f - padding * delta, k* f - padding * delta), delta, nwidth + padding * 2);
                        for (int ii = 0; ii < voxel.n*voxel.n*voxel.n; ii++)
                            voxel.cellValues[ii] = voxel.IndexToPoint(ii);

                        voxeldic.Add(new Coord(i, j, k), voxel);
                    }
                }
            }

            voxel = new Voxels<Point3d>(Point3d.Origin, delta, nwidth + padding * 2);
            for (int ii = 0; ii < voxel.n * voxel.n * voxel.n; ii++)
                voxel.cellValues[ii] = -voxel.IndexToPoint(ii);
            voxeldic[new Coord(0, 0, 0)] = voxel;

            Stopwatch sw = new Stopwatch();
            sw.Start();
            if (com)
                Utility.DoComm(voxeldic, padding);
            sw.Stop();
            // 
            var displacemnts = voxeldic.SelectMany(x => x.Value.cellValues.Select(y => y)).ToList();

            DA.SetDataList(0, displacemnts);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"time: {sw.ElapsedMilliseconds}ms.");
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
        public override Guid ComponentGuid => new Guid("1FFA1D15-8825-463E-B6C0-499FEC8F3114");
    }
}