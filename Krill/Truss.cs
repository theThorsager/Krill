using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using BriefFiniteElementNet;
using BriefFiniteElementNet.Elements;
using BriefFiniteElementNet.Sections;
using BriefFiniteElementNet.Materials;

namespace Krill
{
    public class Connection
    {
        public int IdA;
        public int IdB;
        public double Stiffness;
        public double Load;

        public Connection(int idA, int idB, double stiffness = 1.0)
        {
            IdA = idA;
            IdB = idB;
            Stiffness = stiffness;
            Load = 0.0;
        }
    }

    public class Truss
    {
        // Do we need the voxels here
        List<Point3d> nodes = new List<Point3d>();
        List<Connection> connections = new List<Connection>();

        public Model model;

        public Truss(List<Point3d> nodes, List<bool> locked, List<int> startNode, List<int> endNode)
        {
            double A = 0.01;
            double w = Math.Sqrt(A);
            double E = 1;

            model = new Model();

            for (int i = 0; i < nodes.Count; i++)
                model.Nodes.Add(new Node(nodes[i].X, nodes[i].Y, nodes[i].Z)
                { Constraints = locked[i] ? Constraints.MovementFixed : Constraints.Released, Label = $"node{i}" }); 

            UniformGeometric1DSection section = new UniformGeometric1DSection(SectionGenerator.GetRectangularSection(w, w));
            UniformIsotropicMaterial material = UniformIsotropicMaterial.CreateFromYoungPoisson(E, 0.25);

            for (int i = 0; i < startNode.Count; ++i)
                model.Elements.Add(new BarElement(model.Nodes[startNode[i]], model.Nodes[endNode[i]])
                { Behavior = BarElementBehaviours.FullFrame, Section = section, Material = material, Label = $"bar{i}" });

            for (int i = 0; i < nodes.Count; ++i)
                model.Nodes[i].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));

            
            //model.Nodes[2].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));
            //model.Nodes[3].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));
            //model.Nodes[4].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));

            // 0 2 3 4
            //model.Nodes[0].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));
            //model.Nodes[2].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));
            //model.Nodes[3].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));
            //model.Nodes[4].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));

        }

        public List<Line> ToLines()
        {
            List<Line> lines = new List<Line>();
            
            foreach (Element element in model.Elements)
            {
                var a = element.Nodes[0].Location;
                var b = element.Nodes[1].Location;
                lines.Add(new Line(new Point3d(a.X, a.Y, a.Z), new Point3d(b.X, b.Y, b.Z)));
            }
            return lines;
        }

        public List<double> GetAxialForces()
        {
            var axial = new List<double>();
            foreach (Element element in model.Elements)
            {
                axial.Add((element as BarElement).GetInternalForceAt(0.5).Fx);
            }
            return axial;
        }

        void ComputeForces()
        {
            var model = new Model();

            var FEMNodes = new List<Node>();
            foreach (var node in nodes)
                FEMNodes.Add(new Node(node.X, node.Y, node.Z) { Constraints = Constraints.RotationFixed });

            var FEMbars = new List<BarElement>();
            foreach (var connection in connections)
                FEMbars.Add(new BarElement(FEMNodes[connection.IdA], FEMNodes[connection.IdB]) { Behavior = BarElementBehaviours.Truss });

            model.Nodes.AddRange(FEMNodes);
            foreach (var bar in FEMbars)
                model.Elements.Add(bar);

            // Find more supports
            model.Nodes[0].Settlements.Add(new Settlement(new Displacement(0, 0, -0.01)));

            model.Solve();

            for (int i = 0; i < FEMbars.Count; i++)
                connections[i].Load = FEMbars[i].GetInternalForceAt(0.5).Fx;


        }

        public void ComputeStiffness(Voxels<int> voxels)
        {
            // check how much space is available for each connection depending on tensile or compressive load

            // transform to cylindrical coordinates
            // iterate outwards until the closest point radially, within the height, that is not valid for loads

            // Use that to set the stiffness
        }

        public bool StaticallyIndeterminate()
        {
            return false;
        }
    }
}
