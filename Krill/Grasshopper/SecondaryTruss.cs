using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class SecondaryTruss : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SecondaryTruss class.
        /// </summary>
        public SecondaryTruss()
          : base("SecondaryTruss", "ST",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.PostProcessingResultsParam());
            pManager.AddCurveParameter("ExistingTruss", "pLine", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("ScaleFactorForDelta", "sf", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("Tolerance", "tol", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("SecondaryTruss", "sT", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("PrincipalLines", "princp", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Containers.PostProcessingResults post = null;
            Param.PostProcessingResultsGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            post = res.Value;

            List<Curve> curves = new List<Curve>();
            DA.GetDataList(1, curves);
            if (curves is null)
                return;

            List<Polyline> trusses = new List<Polyline>();
            for (int i = 0; i < curves.Count; i++)
            {
                if (!curves[i].TryGetPolyline(out Polyline truss))
                    return;
                trusses.Add(truss);
            }

            double scaleStep = new double();
            DA.GetData(2, ref scaleStep);

            double tol = new double();
            DA.GetData(3, ref tol);

            List<Line> secTruss = new List<Line>();
            List<Polyline> princpCrv = new List<Polyline>();

            //OutputResults results = new OutputResults(post);
            //results.UpdateFakeStrains(post.displacments);
            //results.UpdateStresses();
            //results.UpdatePrincipalStresses();

            List<Point3d> nds = new List<Point3d>();

            for (int i = 0; i < trusses.Count; i++)
            {
                for (int j = 1; j < trusses[i].Count - 1; j++)
                {
                    nds.Add(trusses[i][j]);
                }
            }

            for (int k = 0; k < trusses.Count; k++)
            {
                Polyline truss = trusses[k];

                for (int i = 1; i < truss.Count - 1; i++)
                {
                    Vector3d planeNormal = ((truss[i - 1] - truss[i]) + (truss[i] - truss[i + 1])) / 2.0;

                    Plane pl = new Plane(truss[i], planeNormal);

                    Vector3d[] vecs = new Vector3d[4];
                    vecs[0] = pl.XAxis;
                    vecs[1] = -pl.XAxis;
                    vecs[2] = pl.YAxis;
                    vecs[3] = -pl.YAxis;

                    for (int j = 0; j < 4; j++)
                    {
                        LoadPathCurve lPath = new LoadPathCurve(post.mask, truss[i], vecs[j], post.princpDir, post.princpStress);

                        if (lPath.SecondaryLoadPath(scaleStep, tol, nds, out int ind))
                        {
                            Line ln = new Line(truss[i], nds[ind]);
                            Line lnF = ln;
                            lnF.Flip();

                            if (!secTruss.Contains(ln) && !secTruss.Contains(lnF))
                                secTruss.Add(ln);
                        }
                        Polyline pL = lPath.loadPath;
                        princpCrv.Add(pL);
                    }
                }
            }

            if (secTruss != null)
                DA.SetDataList(0, secTruss);

            if (princpCrv != null)
                DA.SetDataList(1, princpCrv);

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
            get { return new Guid("1AD780DE-9165-4CDB-A74D-A02674469440"); }
        }
    }
}