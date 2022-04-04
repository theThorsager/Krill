using System;
using System.Collections.Generic;
using System.Linq;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class TrussOrthogonality : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the TrussGradient class.
        /// </summary>
        public TrussOrthogonality()
          : base("TrussOrthogonality", "TrussOrtho",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.TrussGeometryParam());
            pManager.AddIntegerParameter("n", "n", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("alpha", "a", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("factor", "f", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("energy", "e", "", GH_ParamAccess.item);
            pManager.AddVectorParameter("gradient", "g", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("disp", "d", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("strains", "s", "", GH_ParamAccess.list);
            pManager.AddLineParameter("lines", "l", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Param.TrussGeometryGoo truss = null;
            DA.GetData(0, ref truss);

            int n = 1;
            DA.GetData(1, ref n);

            double a = 1;
            DA.GetData(2, ref a);

            double f = 1;
            DA.GetData(3, ref f);

            if (truss?.Value is null)
                return;

            var ortho = new OrthonogalityTruss();

            ortho.nodes = truss.Value.Nodes.ToList();
            ortho.connections = truss.Value.Connections.ToList();
            ortho.gradients = truss.Value.Nodes.Select(x => new Vector3d()).ToList();


            var locked = new bool[ortho.nodes.Count];
            var indecies = truss.Value.Connections.SelectMany(x => new int[] { x.Item1, x.Item2 }).ToList();
            for (int i = 0; i < ortho.nodes.Count; i++)
            {
                if (indecies.Count(x => x == i) == 1)
                {
                    locked[i] = true;
                }
            }
            ortho.LockDOFs(locked.ToList());


            double result = 0.0;
            for (int i = 0; i < n; i++)
            {
                result = ortho.EvaluateTruss(f);
                ortho.SetGradients(f);
                ortho.ApplyGradient(a);
            }

            //var displac = energyTruss.us;

            //var grad = new List<Vector3d>();
            //var disp = new List<Vector3d>();
            //var pts = new List<Point3d>();
            //for (int i = 0; i < energyTruss.nVariables / 3; i++)
            //{
            //    grad.Add(new Vector3d(gradient[i * 3], gradient[i * 3 + 1], gradient[i * 3 + 2]));
            //    disp.Add(new Vector3d(displac[i * 3], displac[i * 3 + 1], displac[i * 3 + 2]));
            //    pts.Add(new Point3d(energyTruss.xs[i * 3], energyTruss.xs[i * 3 + 1], energyTruss.xs[i * 3 + 2]));
            //}

            var lines = new List<Line>();
            for (int i = 0; i < ortho.connections.Count; i++)
            {
                lines.Add(new Line(ortho.nodes[ortho.connections[i].Item1], ortho.nodes[ortho.connections[i].Item2]));
            }

            //DA.SetData(0, Math.Log(result + f * f));
            DA.SetData(0, result);
            DA.SetDataList(1, ortho.gradients);
            //DA.SetDataList(2, disp);
            //DA.SetDataList(3, energyTruss.eps);
            DA.SetDataList(4, lines);
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
            get { return new Guid("6D2B14CE-82EE-4AE6-ACF6-3A0E234589E7"); }
        }
    }
}