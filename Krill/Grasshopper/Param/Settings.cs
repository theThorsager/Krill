using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Krill;

namespace Krill.Grasshopper.Param
{
    public class Settings : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Settings class.
        /// </summary>
        public Settings()
          : base("Settings", "settings",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            Krill.Settings setting = new Krill.Settings();
            pManager.AddNumberParameter("Delta", "D", "", GH_ParamAccess.item, setting.Delta);
            pManager.AddNumberParameter("delta", "d", "", GH_ParamAccess.item, setting.delta);
            pManager.AddIntegerParameter("timesteps", "nt", "", GH_ParamAccess.item, setting.n_timesteps);
            pManager.AddNumberParameter("bond stiffnes", "c", "", GH_ParamAccess.item, setting.bond_stiffness);
            pManager.AddNumberParameter("Youngs", "E", "", GH_ParamAccess.item, setting.E);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new SettingsParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double Delta = 0;
            double delta = 0;
            int n = 0;
            double stiff = 0;
            double e = 0;

            DA.GetData(0, ref Delta);
            DA.GetData(1, ref delta);
            DA.GetData(2, ref n);
            DA.GetData(3, ref stiff);
            DA.GetData(4, ref e);

            DA.SetData(0, new SettingsGoo(new Krill.Settings() 
            { 
                Delta = Delta, 
                delta = delta, 
                bond_stiffness = stiff, 
                n_timesteps = n, 
                E = e 
            }));
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
            get { return new Guid("C900D8DF-D528-4C71-9C17-FB1A83F4CA5B"); }
        }
    }
}