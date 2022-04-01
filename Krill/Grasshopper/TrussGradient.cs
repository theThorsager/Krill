using System;
using System.Collections.Generic;
using System.Linq;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class TrussGradient : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the TrussGradient class.
        /// </summary>
        public TrussGradient()
          : base("TrussGradient", "TrussGrad",
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

            if (truss?.Value is null)
                return;

            var energyTruss = new InternalEnergyTruss();
            energyTruss.Init(truss.Value);
            
            var areas = new double[energyTruss.nElements];
            energyTruss.SetData(areas.Select(x => 1.0).ToArray());

            var locked = new bool[energyTruss.nVariables];
            var indecies = truss.Value.Connections.SelectMany(x => new int[] { x.Item1, x.Item2 }).ToList();
            for (int i = 0; i < energyTruss.nVariables / 3; i++)
            {
                if (indecies.Count(x => x == i) == 1)
                {
                    locked[i*3] = true;
                    locked[i*3+1] = true;
                    locked[i*3+2] = true;
                }
            }

            energyTruss.LockDOFs(locked);

            var gradient = new double[energyTruss.nVariables];
            double energy = -1;
            for (int i = 0; i < n; i++)
            {
                energy = energyTruss.ComputeValueAndGradient(ref gradient);
                energyTruss.ApplyGradient(gradient, a);
            }


            var displac = energyTruss.us;

            var grad = new List<Vector3d>();
            var disp = new List<Vector3d>();
            var pts = new List<Point3d>();
            for (int i = 0; i < energyTruss.nVariables / 3; i++)
            {
                grad.Add(new Vector3d(gradient[i * 3], gradient[i * 3 + 1], gradient[i * 3 + 2]));
                disp.Add(new Vector3d(displac[i * 3], displac[i * 3 + 1], displac[i * 3 + 2]));
                pts.Add(new Point3d(energyTruss.xs[i * 3], energyTruss.xs[i * 3 + 1], energyTruss.xs[i * 3 + 2]));
            }

            var lines = new List<Line>();
            for (int i = 0; i < energyTruss.nElements; i++)
            {
                lines.Add(new Line(pts[truss.Value.Connections[i].Item1], pts[truss.Value.Connections[i].Item2]));
            }

            DA.SetData(0, energy);
            DA.SetDataList(1, grad);
            DA.SetDataList(2, disp);
            DA.SetDataList(3, energyTruss.eps);
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
            get { return new Guid("1D149400-C0FA-4415-B3C9-A0563E402E48"); }
        }
    }
}