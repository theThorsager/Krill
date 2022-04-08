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
            pManager.AddParameter(new Param.DiscreteBoundaryConditionParam(), "BCs", "BCs", "", GH_ParamAccess.list);
            pManager.AddIntegerParameter("n", "n", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("alpha", "a", "", GH_ParamAccess.item);
            pManager.AddBooleanParameter("lockZ", "lockZ", "", GH_ParamAccess.item);
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

            var bcsGoo = new List<Param.DiscreteBoundaryConditionGoo>();
            DA.GetDataList(1, bcsGoo);
            var BCs = bcsGoo.Select(x => x.Value);

            int n = 1;
            DA.GetData(2, ref n);

            double a = 1;
            DA.GetData(3, ref a);

            bool lockZ = false;
            DA.GetData(4, ref lockZ);

            if (truss?.Value is null)
                return;

            var energyTruss = new InternalEnergyTruss();
            energyTruss.Init(truss.Value);
            bool first = true;
            foreach (var bc in BCs)
            {
                if (bc is Containers.DiscreteBoundaryConditionDirechlet bcD)
                {
                    int i = truss.Value.Nodes.FindIndex(x => x.DistanceToSquared(bc.line.From) < 1e-6);
                    int count = truss.Value.Connections.SelectMany(x => new[] {x.Item1, x.Item2}).Count(x => x == i);

                    if (count > 1)
                    {
                        energyTruss.LockElement(bcD.line.To, bcD.line.From, first);
                    }
                    else
                    {
                        energyTruss.LockElement(bcD.line.From, bcD.line.To, first);
                    }
                    first = false;
                }
                else if (bc is Containers.DiscreteBoundaryConditionNuemann bcN)
                {
                    bool flip = truss.Value.Nodes.Any(x => x.DistanceToSquared(bc.line.From) < 1e-6);
                    if (flip)
                        energyTruss.SetExtraElement(bcN.line.To, bcN.line.From, bcN.load);
                    else
                        energyTruss.SetExtraElement(bcN.line.From, bcN.line.To, bcN.load);
                }
            }
            if (lockZ)
                energyTruss.LockZ();

            energyTruss.BCPost();

            energyTruss.SetData(null);
            var gradient = new double[energyTruss.nVariables];
            double energy = energyTruss.ComputeValue();
            for (int i = 0; i < n; i++)
            {
                if (energyTruss.mechanisim || double.IsNaN(energy))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"The truss is an Mechanism and can not be solved. \n Occurred at iteration: {i}");
                    energy = double.NaN;
                    break;
                }

                energy = energyTruss.ComputeValueAndGradient(ref gradient);
                energyTruss.ConstrainToDirections(gradient);
                energyTruss.ApplyGradient(gradient, a);
                energyTruss.SetData(null);
            }


            // Post processing
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
            DA.SetDataList(3, energyTruss.Forces());
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