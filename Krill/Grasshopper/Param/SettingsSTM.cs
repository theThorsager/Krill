using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Krill;

namespace Krill.Grasshopper.Param
{
    public class SettingsSTM : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Settings class.
        /// </summary>
        public SettingsSTM()
          : base("SettingsSTM", "SettingsSTM",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            Containers.SettingsSTM settings = new Containers.SettingsSTM();
            pManager.AddNumberParameter("Mean elastic modulus of concrete [Pa]", "Ec", "", GH_ParamAccess.item, settings.Ec);
            pManager.AddNumberParameter("Characteristic compressive strength of concrete [Pa]", "fck", "", GH_ParamAccess.item, settings.fck);
            pManager.AddNumberParameter("Design compressive strength of concrete [Pa]", "fcd", "", GH_ParamAccess.item, settings.fcd);
            pManager.AddNumberParameter("Elastic modulus of reinforcing steel [Pa]", "Es", "", GH_ParamAccess.item, settings.Es);
            pManager.AddNumberParameter("Design yield strength of reinforcing steel [Pa]", "fyd", "", GH_ParamAccess.item, settings.fyd);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.SettingsSTMParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double Ec = 0;
            double fck = 0;
            double fcd = 0;
            double Es = 0;
            double fyd = 0;

            DA.GetData(0, ref Ec);
            DA.GetData(1, ref fck);
            DA.GetData(2, ref fcd);
            DA.GetData(3, ref Es);
            DA.GetData(4, ref fyd);

            Containers.SettingsSTM settings = new Containers.SettingsSTM()
            {
                Ec = Ec,
                fck = fck,
                fcd = fcd,
                Es = Es,
                fyd = fyd
            };

            DA.SetData(0, new SettingsSTMGoo(settings));
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
            get { return new Guid("48AEAB98-78AB-490A-B524-F5A8F1709FE4"); }
        }
    }
}