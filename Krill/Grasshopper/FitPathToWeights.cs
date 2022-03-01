using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class FitPathToWeights : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the FitPathToWeights class.
        /// </summary>
        public FitPathToWeights()
          : base("FitPathToWeights", "FitPath",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearSolutionParam());
            pManager.AddPointParameter("a", "a", "", GH_ParamAccess.item);
            pManager.AddPointParameter("b", "b", "", GH_ParamAccess.item);
            pManager.AddVectorParameter("normalA", "normalA", "", GH_ParamAccess.item);
            pManager.AddVectorParameter("normalB", "normalB", "", GH_ParamAccess.item);
            pManager.AddIntegerParameter("n_p", "n_p", "", GH_ParamAccess.item);
            pManager.AddIntegerParameter("n_i", "n_i", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("pts", "pts", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("vec", "vec", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Param.LinearSolutionGoo lin = null;
            DA.GetData(0, ref lin);
            if (lin is null)
                return;

            Point3d a = new Point3d();
            Point3d b = new Point3d();
            Vector3d va = new Vector3d();
            Vector3d vb = new Vector3d();
            DA.GetData(1, ref a);
            DA.GetData(2, ref b);
            DA.GetData(3, ref va);
            DA.GetData(4, ref vb);
            int n = 0;
            DA.GetData(6, ref n);

            int np = 0;
            DA.GetData(5, ref np);

            Point3d A = a + va;
            Point3d B = b + vb;
            var points = new List<Point3d>();
            for (int i = 0; i < np; i++)
            {
                double f = (double)i / (double)(np - 1.0);
                points.Add(A * (1.0 - f) + B * f);
            }

            var subGradient = new SubGradientFitting(lin.Value.utilization, points);

            for (int i = 0; i < n; i++)
            {
                subGradient.Iterate();
            }

            points.Add(b);
            points.Insert(0, a);
            DA.SetDataList(0, new Polyline(points));
            subGradient.dirs.Add(Vector3d.Zero);
            subGradient.dirs.Insert(0, Vector3d.Zero);
            DA.SetDataList(1, subGradient.dirs);
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
            get { return new Guid("BCD7F877-A096-4577-9B94-FDDF8621DA2D"); }
        }
    }
}