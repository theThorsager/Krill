using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Krill;

namespace Krill.Grasshopper.Param
{
    public class Direchlet : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Settings class.
        /// </summary>
        public Direchlet()
          : base("Direchlet", "Direchlet",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // Direchlet
            pManager.AddMeshParameter("area", "area", "", GH_ParamAccess.item);
            pManager.AddBooleanParameter("normal", "normal", "", GH_ParamAccess.item, true);
            pManager.AddPlaneParameter("coordinate", "coord", "", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddBooleanParameter("x", "x", "", GH_ParamAccess.item, true);
            pManager.AddBooleanParameter("y", "y", "", GH_ParamAccess.item, true);
            pManager.AddBooleanParameter("z", "z", "", GH_ParamAccess.item, true);
            pManager.AddVectorParameter("displacment", "disp", "", GH_ParamAccess.item, Vector3d.Zero);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.BoundaryConditionParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh area = null;
            bool normal = true;
            Plane coordinate = Plane.Unset;
            bool x = false;
            bool y = false;
            bool z = false;
            Vector3d disp = Vector3d.Unset;

            DA.GetData(0, ref area);
            DA.GetData(1, ref normal);
            DA.GetData(2, ref coordinate);
            DA.GetData(3, ref x);
            DA.GetData(4, ref y);
            DA.GetData(5, ref z);
            DA.GetData(6, ref disp);


            var direchlet = new Containers.BoundaryConditionDirechlet()
            {
                area = area,
                normal = normal,
                Coordinate = coordinate,
                lockX = x,
                lockY = y,
                lockZ = z,
                displacement = disp
            };

            DA.SetData(0, new BoundaryConditionGoo(direchlet));
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
            get { return new Guid("F277F335-ABA8-43FD-A006-BA3D204946C8"); }
        }
    }
}