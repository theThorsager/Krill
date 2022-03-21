using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

using System.Linq;

namespace Krill.Grasshopper
{
    public class Truss : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Truss class.
        /// </summary>
        public Truss()
          : base("Truss", "Truss",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
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
            pManager.AddLineParameter("lines", "l", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("values", "vals", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("areas", "areas", "", GH_ParamAccess.list);
            pManager.AddTextParameter("warning", "w", "", GH_ParamAccess.list);
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
            DA.GetDataList(0, nodes);
            DA.GetDataList(1, a);
            DA.GetDataList(2, b);
            DA.GetDataList(3, locked);

            Krill.Truss truss = new Krill.Truss(nodes, locked, a, b);

            //truss.SetVoxels(mask);  // Lär vilja slänga in en LinearSolution

            // initial evaluation of the truss to get forces
            truss.model.Solve_MPC();
            var forces = truss.GetAxialForces();

            /*
            for (int iter = 0; iter < 50; iter++)
            {

                // Set the area for all ties based on forces
                truss.SetAreasForTies(forces);
                // set the area for all supports based on support size
                truss.SetAreasForSupports();    // Not implemented

                // while (areas did not change) {
                // do a geometrical pass to see if any of the areas need to be reduced (due to lack of space)
                // What did I mean here? Like if an element became to thick on the middle? I will initially assume that cannot happen

                // Find the rest of the areas of the elements where node size is controlled by the least area
                truss.FindRestOfArea();
                // }

                truss.ApplyAreas();     // to the FEM model
                // Rerun the solver and see how much the solution changed, break if some tolerance otherwise start again
                var oldforces = new List<double>(forces);
                truss.model.Solve_MPC();
                forces = truss.GetAxialForces();


                if (forces.Zip(oldforces, (x, y) => Math.Abs(x - y)).All(x => x <= 1e-6 * forces.Average()))
                    break;
            }
            */

            // evaluate further geometry which is based on if things are struts or ties
            var res = truss.CheckAngles(forces);
            res.AddRange(truss.CheckAnglesConcentrated(forces, truss.GetConcentrated(locked)));

            // evaluate if nodes/struts/ties are broken

            // Done :)

            List<string> warnings = new List<string>();
            foreach (var warning in res)
            {
                warnings.Add($"{warning.pt} \n{warning.A} \n{warning.B} \n{warning.excetrcFromBeingSpanned} \n{warning.angle} \n{warning.concentrated}");
            }

            DA.SetDataList(0, truss.ToLines());
            DA.SetDataList(1, truss.GetAxialForces());
            DA.SetDataList(2, forces);

            DA.SetDataList(3, warnings);
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
            get { return new Guid("237B1730-D267-4C97-BFA0-4F06A5173753"); }
        }
    }
}