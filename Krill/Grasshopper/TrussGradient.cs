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
            pManager.AddParameter(new Param.BoxSDFParam());
            pManager[2].Optional = true;
            pManager.AddIntegerParameter("n", "n", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("alpha", "a", "", GH_ParamAccess.item);
            pManager.AddBooleanParameter("lockZ", "lockZ", "", GH_ParamAccess.item);
            pManager.AddParameter(new Param.SettingsSTMParam());
            pManager[6].Optional = true;
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
            pManager.AddNumberParameter("areas", "a", "", GH_ParamAccess.list);
            pManager.AddGenericParameter("test", "t", "", GH_ParamAccess.list);
            pManager.AddGenericParameter("graphVals", "vals", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("Penality modification location", "penLoc", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("STM location", "stmLoc", "", GH_ParamAccess.list);
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

            Param.BoxSDFGoo boxSDFGoo = null;
            DA.GetData(2, ref boxSDFGoo);

            int n = 1;
            DA.GetData(3, ref n);

            double a = 1;
            DA.GetData(4, ref a);
            double firstA = a;

            bool lockZ = false;
            DA.GetData(5, ref lockZ);

            Containers.SettingsSTM settings = null;
            Param.SettingsSTMGoo res = null;
            DA.GetData(6, ref res);
            if (res is null)
                settings = new Containers.SettingsSTM();
            else
                settings = res.Value;

            if (truss?.Value is null)
                return;

            var energyTruss = new InternalEnergyTruss();
            energyTruss.Init(truss.Value, settings);
            energyTruss.SDF = boxSDFGoo?.Value;
            foreach (var bc in BCs)
            {
                if (bc is Containers.DiscreteBoundaryConditionDirechlet bcD)
                {
                    int i = truss.Value.Nodes.FindIndex(x => x.DistanceToSquared(bcD.line.From) < 1e-6);
                    int count = truss.Value.Connections.SelectMany(x => new[] {x.Item1, x.Item2}).Count(x => x == i);

                    if (count > 1)
                    {
                        energyTruss.LockElement(bcD.line.To, bcD.line.From, bcD.Fixed);
                    }
                    else
                    {
                        energyTruss.LockElement(bcD.line.From, bcD.line.To, bcD.Fixed);
                    }
                }
                else if (bc is Containers.DiscreteBoundaryConditionNuemann bcN)
                {
                    bool flip = truss.Value.Nodes.Any(x => x.DistanceToSquared(bcN.line.From) < 1e-6);
                    if (flip)
                        energyTruss.SetExtraElement(bcN.line.To, bcN.line.From, bcN.load);
                    else
                        energyTruss.SetExtraElement(bcN.line.From, bcN.line.To, bcN.load);
                }
                else if (bc is Containers.DiscreteBoundaryConditionVariables bcV)
                {
                    energyTruss.LockVariable(bcV.point, bcV.lockX, bcV.lockY, bcV.lockZ);
                }
            }
            if (lockZ)
                energyTruss.LockZ();

            energyTruss.BCPost();

            var log = new List<string>();

            var gradient = new double[energyTruss.nVariables];
            var gradientA = new double[energyTruss.nElements];
            double intermidiateEnergy = double.MaxValue;
            double gamma = 1;
            double stepLength = double.MaxValue;
            double energy = double.MaxValue;

            List<Tuple<int, int, double, double>> funcVals = new List<Tuple<int, int, double, double>>();
            List<int> penLoc = new List<int>();
            List<int> stmLoc = new List<int>();
            
            int iter = 0;
            for (int stmIter = 0; stmIter < 10; stmIter++)
            {
                energyTruss.SetPenalties(1);
                energyTruss.SetData(null);
                energy = energyTruss.ComputeValue();

                for (int penIter = 0; penIter < 10; penIter++)
                {
                    intermidiateEnergy = double.MaxValue;
                    int outerInnerIter = 0;
                    while (Math.Abs(intermidiateEnergy - energy) > 1e-6 && iter < n && outerInnerIter <= 3)
                    {
                        outerInnerIter++;
                        a = firstA;
                        // Node Locations
                        iter++;
                        int innnerIter = 0;
                        for (; iter < n; iter++)
                        {
                            //if (energyTruss.mechanisim || double.IsNaN(energy))
                            //{
                            //    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"The truss is an Mechanism and can not be solved. \n Occurred at iteration: {iter}");
                            //    energy = double.NaN;
                            //    break;
                            //}

                            energyTruss.ComputeGradient(ref gradient);
                            energy = energyTruss.ArmijoStep(gradient, ref a, out stepLength, gamma);
                            log.Add($"location iteration: {iter} Steplength: {stepLength} Energy: {energy}");

                            funcVals.Add(new Tuple<int, int, double, double>(iter, 0, energyTruss.functionVals()[0], energy));

                            // Steplength is the square distance moved ish (as if everything is thought of as one vector)
                            if (stepLength < energyTruss.stepTol)
                                break;

                            innnerIter++;
                            if (innnerIter >= 50)
                                break;
                        }
                        intermidiateEnergy = energy;
                        a = firstA;
                        // Element size
                        iter++;
                        innnerIter = 0;
                        for (; iter < n; iter++)
                        {
                            //if (energyTruss.mechanisim || double.IsNaN(energy))
                            //{
                            //    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"The truss is an Mechanism and can not be solved. \n Occurred at iteration: {iter}");
                            //    energy = double.NaN;
                            //    break;
                            //}

                            energyTruss.ComputeGradientA(ref gradientA);
                            energy = energyTruss.ArmijoStepA(gradientA, ref a, out stepLength, gamma);
                            log.Add($"area iteration: {iter} Steplength: {stepLength} Energy: {energy}");

                            funcVals.Add(new Tuple<int, int, double, double>(iter, 1, energyTruss.functionVals()[0], energy));

                            // Steplength is the square distance moved ish (as if everything is thought of as one vector)
                            if (stepLength < energyTruss.stepTol)
                                break;

                            innnerIter++;
                            if (innnerIter >= 50)
                                break;
                        }
                    }
                    if (iter >= n)
                        break;


                    energyTruss.ModifyPenalties(10);
                    energyTruss.SetData(null);
                    energy = energyTruss.ComputeValue();
                    log.Add($"----- penalty modification -----  Energy: {energy}");
                    penLoc.Add(iter);
                }

                if (iter < n)
                {
                    energyTruss.ApplySTMConstraintsHueristic();
                    intermidiateEnergy = energy;
                    energy = energyTruss.ComputeValue();
                    log.Add($"////// STM modification ////// Energy: {energy}");
                    stmLoc.Add(iter);

                    if (Math.Abs(intermidiateEnergy - energy) < 1e-6)
                        break;
                }
            }
            // Post processing
            double[] test = energyTruss.functionVals();
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
            DA.SetDataList(5, energyTruss.Areas());
            DA.SetDataList(6, energyTruss.endAreas);
            DA.SetDataList(7, funcVals);
            DA.SetDataList(8, penLoc);
            DA.SetDataList(9, stmLoc);
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