using System;
using System.Collections.Generic;
using System.Linq;

using Grasshopper.Kernel;
using Rhino.Geometry;

using Krill.Containers;

namespace Krill.Grasshopper
{
    public class MyComponent1 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public MyComponent1()
          : base("StabilizeTruss", "ST",
              "Description",
              "Krill", "Utility")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearPeridynamicsModelParam());
            pManager.AddParameter(new Param.TrussGeometryParam());
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.TrussGeometryParam());
            pManager.AddLineParameter("TrussLines", "NewLines", "", GH_ParamAccess.list);
        }

        const int maskbit = 0x000000FF;

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Containers.LinearPeridynamicsModel domain = null;
            Param.LinearPeridynamicsModelGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            domain = res.Value;

            Containers.TrussGeometry trussGeo = null;
            Param.TrussGeometryGoo res2 = null;
            DA.GetData(1, ref res2);
            if (res2 is null)
                return;
            trussGeo = res2.Value;

            

            Voxels<int> mask = domain.mask;
            List<Point3d> nodes = new List<Point3d>(trussGeo.Nodes);
            List<Tuple<int, int>> connections = new List<Tuple<int, int>>(trussGeo.Connections);

            List<Line> trussLines = new List<Line>();

            // Generate Node topology
            List<NodeTopology> nodeTop = new List<NodeTopology>();
            for (int i = 0; i < nodes.Count; i++)
            {
                nodeTop.Add(new NodeTopology(i));

                for (int j = 0; j < connections.Count; j++)
                {
                    if (i == connections[j].Item1)
                        nodeTop[i].connections.Add(connections[j].Item2);
                    if (i == connections[j].Item2)
                        nodeTop[i].connections.Add(connections[j].Item1);
                }
            }

            // For each node make sure that its neigbhours are connected
            for (int i = 0; i < nodeTop.Count; i++)
            {
                if (nodeTop[i].connections.Count <= 1)
                    continue;

                for (int j = 0; j < nodeTop[i].connections.Count; j++)
                {
                    int ind1 = nodeTop[i].connections[j];
                    if (nodeTop[ind1].connections.Count <= 1)
                        continue;

                    for (int k = j + 1; k < nodeTop[i].connections.Count; k++)
                    {
                        int ind2 = nodeTop[i].connections[k];
                        if (nodeTop[ind2].connections.Count <= 1)
                            continue;

                        if (!nodeTop[ind1].ConnectsTo(ind2))
                        {
                            Line l = new Line(nodes[ind1], nodes[ind2]);

                            // Check so that the line stays within the body and does not already exist
                            // Also check if it "skips" one node in between
                            if (InsideTest(l, mask) && 
                                !AlreadyExistsTest(ind1, ind2, connections))
                            {
                                connections.Add(new Tuple<int, int>(ind1, ind2));
                                trussLines.Add(l);
                            }

                            //if (InsideTest(l, mask) &&
                            //    !AlreadyExistsTest(ind1, ind2, connections) &&
                            //    !SkipNodeTest(ind1, ind2, nodes, mask.delta))
                            //{
                            //    connections.Add(new Tuple<int, int>(ind1, ind2));
                            //    trussLines.Add(l);
                            //}
                        }
                    }
                }
            }

            var extraAreas = new double[ connections.Count - trussGeo.Areas.Count].Select(x => 0.1);

            Param.TrussGeometryGoo newTrussGeo = new Param.TrussGeometryGoo(new TrussGeometry()
            {
                Nodes = nodes,
                Connections = connections,
                Areas = trussGeo.Areas.Concat(extraAreas).ToList()
            });

            if (newTrussGeo != null)
                DA.SetData(0, newTrussGeo);

            if (trussLines != null)
                DA.SetDataList(1, trussLines);

        }

        private class NodeTopology
        {
            public int ind;
            public List<int> connections;

            public NodeTopology(int ind)
            {
                this.ind = ind;
                connections = new List<int>();
            }

            public bool ConnectsTo(int index)
            {
                for (int i = 0; i < connections.Count; i++)
                {
                    if (index == connections[i])
                        return true;
                }
                return false;
            }
        }

        static private bool SkipNodeTest(int ind1, int ind2, List<Point3d> nodes, double tol)
        {
            Line l = new Line(nodes[ind1], nodes[ind2]);

            for (int i = 0; i < nodes.Count; i++)
            {
                if (i == ind1 || i == ind2)
                    continue;

                if (l.MinimumDistanceTo(nodes[i]) < tol)
                    return true;
            }

            return false;
        }

        static private bool AlreadyExistsTest(int ind1, int ind2, List<Tuple<int, int>> connections)
        {
            for (int k = 0; k < connections.Count; k++)
            {
                if ((connections[k].Item1 == ind1 && connections[k].Item2 == ind2) ||
                    (connections[k].Item2 == ind1 && connections[k].Item1 == ind2))
                    return true;                
            }
            return false;
        }

        static private bool FakeDeulaneyLine(int ind1, int ind2, List<Point3d> nodes)
        {
            Line l = new Line(nodes[ind1], nodes[ind2]);
            Point3d midPt = l.PointAt(0.5);

            double dist = 0.5 * l.Length;
            dist *= dist;

            for (int i = 0; i < nodes.Count; i++)
            {
                if (i == ind1 || i == ind2)
                    continue;

                if (nodes[i].DistanceToSquared(midPt) < dist)
                    return false;
                
            }
            return true;
        }

        static private bool InsideTest (Line l, Voxels<int> mask)
        {
            double stepSize = 0.5 * mask.delta / l.Length;

            int nMax = (int)Math.Floor(1 / stepSize);

            for (int n = 1; n < nMax; n++)
            {
                Point3d pos = l.PointAt(n * stepSize);

                pos -= (Vector3d)mask.origin;
                pos /= mask.delta;

                double u = pos.X;
                double v = pos.Y;
                double w = pos.Z;

                int i0 = (int)Math.Floor(u - 0.5);
                int j0 = (int)Math.Floor(v - 0.5);
                int k0 = (int)Math.Floor(w - 0.5);
                int i1 = (int)Math.Floor(u - 0.5) + 1;
                int j1 = (int)Math.Floor(v - 0.5) + 1;
                int k1 = (int)Math.Floor(w - 0.5) + 1;

                Coord i0j0k0 = new Coord(i0, j0, k0);
                Coord i1j0k0 = new Coord(i1, j0, k0);
                Coord i0j1k0 = new Coord(i0, j1, k0);
                Coord i1j1k0 = new Coord(i1, j1, k0);
                Coord i0j0k1 = new Coord(i0, j0, k1);
                Coord i1j0k1 = new Coord(i1, j0, k1);
                Coord i0j1k1 = new Coord(i0, j1, k1);
                Coord i1j1k1 = new Coord(i1, j1, k1);

                int INDi0j0k0 = mask.CoordToIndex(i0j0k0);
                int INDi1j0k0 = mask.CoordToIndex(i1j0k0);
                int INDi0j1k0 = mask.CoordToIndex(i0j1k0);
                int INDi1j1k0 = mask.CoordToIndex(i1j1k0);
                int INDi0j0k1 = mask.CoordToIndex(i0j0k1);
                int INDi1j0k1 = mask.CoordToIndex(i1j0k1);
                int INDi0j1k1 = mask.CoordToIndex(i0j1k1);
                int INDi1j1k1 = mask.CoordToIndex(i1j1k1);

                // Väldigt basic break-villkor, uppdatera och implementera ett bättre
                if ((mask.cellValues[INDi0j0k0] & maskbit) == 0 && (mask.cellValues[INDi1j0k0] & maskbit) == 0
                     && (mask.cellValues[INDi0j1k0] & maskbit) == 0 && (mask.cellValues[INDi1j1k0] & maskbit) == 0
                     && (mask.cellValues[INDi0j0k1] & maskbit) == 0 && (mask.cellValues[INDi1j0k1] & maskbit) == 0
                     && (mask.cellValues[INDi0j1k1] & maskbit) == 0 && (mask.cellValues[INDi1j1k1] & maskbit) == 0)
                {
                    return false;
                }
            }

            return true;
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
            get { return new Guid("5EA1AD31-3EAD-46C2-93E9-7ECD118BC40D"); }
        }
    }
}