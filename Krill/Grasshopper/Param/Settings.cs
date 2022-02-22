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
            Krill.Containers.Settings setting = new Krill.Containers.Settings();
            pManager.AddNumberParameter("Delta", "D", "", GH_ParamAccess.item, setting.Delta);
            pManager.AddNumberParameter("delta", "d", "", GH_ParamAccess.item, setting.delta);
            pManager.AddIntegerParameter("timesteps", "nt", "", GH_ParamAccess.item, setting.n_timesteps);
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
            int n_t = 0;
            double E = 0;

            DA.GetData(0, ref Delta);
            DA.GetData(1, ref delta);
            DA.GetData(2, ref n_t);
            DA.GetData(3, ref E);

            if (delta <= Math.Sqrt(2))
            {
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "delta is clamped to sqrt(2)");
                delta = Math.Sqrt(2) + 0.0001;
            }

            // E to bondstiffness, see: Peridigm User Guide
            double d = Delta * delta;
            double K3d = E / (3.0 * (1.0 - 2.0 * 0.25));
            
            //double c = Analytical(K3d, d);
            //double c = StiffnessMod(K3d, Delta);
            double c = VolumeCorr(K3d, Delta, delta);

            DA.SetData(0, new SettingsGoo(new Krill.Containers.Settings() 
            { 
                Delta = Delta, 
                delta = delta, 
                bond_stiffness = c, 
                n_timesteps = n_t, 
                E = E
            }));
        }

        private double VolumeCorr(double K3d, double Delta, double delta)
        {
            int n = (int)Math.Ceiling(delta) * 2 + 1;
            int[] off = Utility.GetNeighbourOffsets(n, delta);
            Voxels<bool> dummy = new Voxels<bool>(Point3d.Origin, Delta, n);
            double c = 0;
            double volume = Delta * Delta * Delta;
            int I = n / 2;
            for (int i = 0; i < off.Length; i++)
            {
                double distance = (dummy.IndexToPoint(I) - dummy.IndexToPoint(I + off[i])).Length;
                c += volume * distance * 2;     // times two due to symmetry
            }
            return 18 * K3d / c;
        }

        private double StiffnessMod(double K3d, double Delta)
        {
            return 18 * K3d / (Delta * Delta * Delta);
        }

        private double Analytical(double K3d, double d)
        {
            // E to bondstiffness, see: Peridigm User Guide
            return 18.0 * K3d / (Math.PI * d * d * d * d);
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