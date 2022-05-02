using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

using System.Linq;

namespace Krill.Grasshopper
{
    public class ConstructTruss : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Truss class.
        /// </summary>
        public ConstructTruss()
          : base("ConstructTruss", "ConstructTruss",
              "Description",
              "Krill", "TestComponents")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("lines", "ls", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("areas", "as", "", GH_ParamAccess.list);
            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.TrussGeometryParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            List<Line> lines = new List<Line>();
            DA.GetDataList(0, curves);

            List<double> areas = new List<double>();
            DA.GetDataList(1, areas);

            double emergencyArea = areas.LastOrDefault();
            if (areas.Count == 0)
                emergencyArea = 1.0;

            List<double> freshAreas = new List<double>();

            foreach (var c in curves)
            {
                if (c.TryGetPolyline(out Polyline pline))
                {
                    lines.AddRange(pline.GetSegments());
                }
                else
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Non-linear curve was discarded.");
                }
            }

            var nodes = lines.Select(x => x.From).ToList();
            nodes.AddRange(lines.Select(x => x.To));

            const double tol = 1e-6;
            const double sqtol = tol * tol;

            // get a unique collection of nodes
            for (int j = 0; j < nodes.Count; j++)
            {
                Point3d pt = nodes[j];
                for (int i = nodes.Count - 1; i > j; i--)
                {
                    Point3d other = nodes[i];
                    if (pt.DistanceToSquared(other) < sqtol)
                    {
                        nodes.RemoveAt(i);
                    }
                }
            }

            var connections = new List<Tuple<int, int>>();
            for (int i = 0; i < lines.Count; i++)
            {
                Line l = lines[i];
                int a = -1;
                double mina = double.MaxValue;
                int b = -1;
                double minb = double.MaxValue;

                for (int j = 0; j < nodes.Count; j++)
                {
                    double da = l.From.DistanceToSquared(nodes[j]);
                    if (da < mina)
                    {
                        a = j;
                        mina = da;
                    }
                    double db = l.To.DistanceToSquared(nodes[j]);
                    if (db < minb)
                    {
                        b = j;
                        minb = db;
                    }
                }

                connections.Add(new Tuple<int, int>(a, b));
                freshAreas.Add(i < areas.Count ? areas[i] : emergencyArea);
            }

            for (int i = 0; i < connections.Count; i++)
            {
                for (int j = connections.Count - 1; j > i; j--)
                {
                    if (connections[i].Item1 == connections[j].Item2 && connections[i].Item2 == connections[j].Item1 ||
                        connections[i].Item1 == connections[j].Item1 && connections[i].Item2 == connections[j].Item2)
                    {
                        connections.RemoveAt(j);
                        if (j < areas.Count)
                        {
                            freshAreas[i] = (freshAreas[i] + freshAreas[j]) * 0.5;
                        }
                        freshAreas.RemoveAt(j);
                    }
                }
            }

            var truss = new Containers.TrussGeometry
            {
                Nodes = nodes,
                Connections = connections,
                Areas = freshAreas
            };

            DA.SetData(0, new Param.TrussGeometryGoo(truss));
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
            get { return new Guid("6EE44883-D0D1-4707-8C73-B787C8BE0805"); }
        }
    }
}