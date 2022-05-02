﻿using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Krill;

namespace Krill.Grasshopper.Param
{
    public class Nuemann2d : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Settings class.
        /// </summary>
        public Nuemann2d()
          : base("Nuemann2d", "Nuemann2d",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            //pManager.AddParameter(new Param.LinearSolutionParam(), "solution", "sol", "", GH_ParamAccess.item);
            // Nuemann
            pManager.AddCurveParameter("curve", "curve", "", GH_ParamAccess.item);
            pManager.AddBooleanParameter("normal", "normal", "", GH_ParamAccess.item, true);
            pManager.AddVectorParameter("load", "load", "", GH_ParamAccess.item, Vector3d.YAxis);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.BoundaryConditionParam2d());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve curve = null;
            bool normal = true;
            Vector3d load = Vector3d.Unset;

            DA.GetData(0, ref curve);
            DA.GetData(1, ref normal);
            DA.GetData(2, ref load);


            var nuemann = new Containers.BoundaryConditionNuemann2d()
            {
                curve = curve,
                normal = normal,
                load = load
            };

            DA.SetData(0, new BoundaryCondition2dGoo(nuemann));
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
            get { return new Guid("25C4E1DF-123A-4450-9561-66F810FC48B3"); }
        }
    }
}