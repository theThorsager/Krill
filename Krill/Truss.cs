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

    public class AngleWarning
    {
        public Point3d pt;
        public Vector3d A;
        public Vector3d B;
        public double angle;
        public double strutForce;
        // something to tell how well it's supported by multiple ties
        public double excetrcFromBeingSpanned;
        public bool concentrated = false;
    }

    public class Truss
    {
        // Do we need the voxels here
        public Voxels<int> mask;
        public BoxSDF boxSDF;

        public List<Point3d> nodes = new List<Point3d>();
        public List<Vector3d> nodeBBox = new List<Vector3d>();
        public List<Connection> connections = new List<Connection>();

        public Model model;
        public List<double> areas = new List<double>();
        public List<Vector3d> maxBox = new List<Vector3d>();


        // Debugging
        public List<BoundingBox> boundingBoxes = new List<BoundingBox>();

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
            {
                model.Elements.Add(new BarElement(model.Nodes[startNode[i]], model.Nodes[endNode[i]])
                { Behavior = BarElementBehaviours.FullFrame, Section = section, Material = material, Label = $"bar{i}" });

                areas.Add(0);
            }

            for (int i = 0; i < nodes.Count; ++i)
                model.Nodes[i].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));

            

            for (int i = 0; i < nodes.Count; ++i)
                boundingBoxes.Add(BoundingBox.Empty);

            //model.Nodes[2].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));
            //model.Nodes[3].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));
            //model.Nodes[4].Loads.Add(new NodalLoad(new Force(0, 0, -1, 0, 0, 0)));

            // 0 2 3 4
            //model.Nodes[0].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));
            //model.Nodes[2].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));
            //model.Nodes[3].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));
            //model.Nodes[4].Settlements.Add(new Settlement(new Displacement(0, 0, -0.1)));

        }

        public List<int> GetConcentrated(List<bool> locked)
        {
            List<int> indices2 = new List<int>();
            for (int i = 0; i < locked.Count; i++)
            {
                Node node = model.Nodes[i];
                List<int> indices = new List<int>();
                for (int j = 0; j < model.Elements.Count; j++)
                {
                    if (model.Elements[j].Nodes.Contains(node))
                    {
                        indices.Add(j);
                    }
                }
                if (indices.Count == 1)
                    indices2.Add(indices[0]);
            }
            return indices2;
        }

        public void SetAreasForTies(List<double> forces)
        {
            const double fyd = 300e6;    // This is a guess

            for (int i = 0; i < model.Elements.Count; i++)
            {
                if (forces[i] < 0)
                {
                    // Is does feel like there is more to it than this, but then again maybe not
                    areas[i] = forces[i] / fyd;  
                }
                else
                {
                    areas[i] = 0.0;
                }
            }

        }

        public void SetAreasForSupports()
        {

        }

        public void ApplyAreas()
        {
            for (int i = 0; i < model.Elements.Count; ++i)
            {
                double A = areas[i];
                double w = Math.Sqrt(A);
                UniformGeometric1DSection section = new UniformGeometric1DSection(SectionGenerator.GetRectangularSection(w, w));

                (model.Elements[i] as BarElement).Section = section;
            }
        }

        public void SetVoxels(Voxels<int> mask)
        {
            this.mask = mask;

            boxSDF = new BoxSDF(mask);
            boxSDF.ConstructSDF3();

            // old
            //for (int i = 0; i < model.Nodes.Count; ++i)
            //    maxBox.Add(boxSDF.BiggestBox(ToRhino(model.Nodes[i].Location)));
        }

        public List<AngleWarning> CheckAnglesConcentrated(List<double> forces, List<int> concentrated)
        {
            var warnings = new List<AngleWarning>();

            for (int ii = 0; ii < concentrated.Count; ii++)
            {
                int i = concentrated[ii];
                // check the nodes it connects to and update their areas
                for (int k = 0; k < 2; k++)
                {
                    Node node = model.Elements[i].Nodes[k];
                    Vector3d theNormal = ToRhino(node.Location - model.Elements[i].Nodes[(k + 1) % 2].Location);
                    theNormal.Unitize();
                    int index = model.Nodes.IndexOf(node);
                    // Find all elements which connects to this node
                    List<int> indices = new List<int>();
                    for (int j = 0; j < areas.Count; j++)
                    {
                        if (j != i && model.Elements[j].Nodes.Contains(node))
                        {
                            indices.Add(j);
                        }
                    }
                    // Get relevant normals
                    var normals = new List<Vector3d>();
                    for (int j = 0; j < indices.Count; j++)
                    {
                        var nodes = model.Elements[indices[j]].Nodes;
                        Vector3d normal = ToRhino(nodes.First(x => x != node).Location - node.Location);
                        normal.Unitize();
                        normals.Add(normal);
                    }

                    double minAngle = Math.PI;
                    int J = -1;
                    for (int j = 0; j < normals.Count; j++)
                    {
                        double angle = Vector3d.VectorAngle(theNormal, normals[j]);
                        if (angle < minAngle)
                        {
                            minAngle = angle;
                            J = j;
                        }
                    }
                    if (J != -1 && minAngle > Math.PI / 4)
                    {
                        warnings.Add(new AngleWarning()
                        {
                            pt = ToRhino(node.Location),
                            A = theNormal,
                            B = normals[J],
                            angle = minAngle,
                            strutForce = forces[i],
                            excetrcFromBeingSpanned = double.NaN,
                            concentrated = true
                        });
                    }

                }
            }

            return warnings;
        }

        public List<AngleWarning> CheckAngles(List<double> forces)
        {
            var warnings = new List<AngleWarning>();

            for (int i = 0; i < model.Nodes.Count; i++)
            {
                Node node = model.Nodes[i];
                // Find all elements which connects to this node
                List<int> indices = new List<int>();
                for (int j = 0; j < areas.Count; j++)
                {
                    if (model.Elements[j].Nodes.Contains(node))
                    {
                        indices.Add(j);
                    }
                }
                // Get relevant normals
                var normals = new List<Vector3d>();
                var normalsS = new List<Vector3d>();
                var normalsT = new List<Vector3d>();
                var subForces = new List<double>();
                var subForcesS = new List<double>();
                for (int j = 0; j < indices.Count; j++)
                {
                    var nodes = model.Elements[indices[j]].Nodes;
                    Vector3d normal = ToRhino(nodes.First(x => x != node).Location - node.Location);
                    normal.Unitize();

                    if (forces[indices[j]] < 0)
                    {
                        normalsS.Add(normal);
                        subForcesS.Add(forces[indices[j]]);
                    }
                    else
                        normalsT.Add(normal);

                    normals.Add(normal);
                    subForces.Add(forces[indices[j]]);
                }

                // Two perpendicular ties is the same as the strut being able to be spanned by the ties
                for (int j = 0; j < normalsS.Count; j++)
                {
                    var angles = new List<double>();
                    for (int k = 0; k < normalsT.Count; k++)
                    {
                        angles.Add(Vector3d.VectorAngle(normalsS[j], normalsT[k]));
                    }

                    for (int k = 0; k < normalsT.Count; k++)
                    {
                        double angle = angles[k];
                        // check if any is less than 45
                        if (angle < Math.PI / 4.0)
                        {
                            double spanned = Math.PI;
                            // check if it is spanned by more than one tie
                            for (int kk = 0; kk < normalsT.Count; kk++)
                            {
                                if (kk == k)
                                    continue;
                                // Think like great arc distance, if the sum of the distances is the same as the distance, then they are on the same arc
                                double temp = angle + angles[k] - Vector3d.VectorAngle(normalsT[k], normalsT[kk]);
                                spanned = temp < spanned ? temp : spanned;

                                // Add check for if it is spanned by three ties
                                for (int kkk = 0; kkk < normalsT.Count; kkk++)
                                {
                                    if (kkk == k || kkk == kk)
                                        continue;

                                    Rhino.Geometry.Matrix A = new Rhino.Geometry.Matrix(3,3);
                                    Rhino.Geometry.Matrix n = new Rhino.Geometry.Matrix(3, 1);
                                    for (int ii = 0; ii < 3; ii++)
                                    {
                                        A[ii, 0] = normalsT[k][ii];
                                        A[ii, 1] = normalsT[kk][ii];
                                        A[ii, 2] = normalsT[kkk][ii];
                                        n[ii, 0] = normalsS[j][ii];
                                    }
                                    A.Invert(1e-3);
                                    var w = A * n;

                                    temp = Math.Min(w[0, 0], Math.Min(w[1, 0], w[2, 0]));
                                    temp = temp > 0 ? 0 : -temp;

                                    spanned = temp < spanned ? temp : spanned;
                                }
                            }
                            
                            // Less than 30 is always pretty bad
                            if (angle < Math.PI / 6 || spanned > 1e-3)
                            {
                                warnings.Add(new AngleWarning()
                                {
                                    pt = ToRhino(node.Location),
                                    A = normalsS[j],
                                    B = normalsT[k],
                                    angle = angle,
                                    strutForce = subForcesS[j],
                                    excetrcFromBeingSpanned = spanned
                                });
                            }
                        }
                    }
                }
            }

            return warnings;
        }

        public bool FindRestOfArea()
        {
            bool problem = false;
            // Every node is an cuboid placed along the coordinate system
            // the projection of the cuboid along each member should have the correct area
            List<double> oldAreas;
            //do
            //{
            nodeBBox = new Vector3d[model.Nodes.Count].ToList();
            bool[] areaChecked = new bool[areas.Count];
            for (int iter = 0; iter < 1000; iter++)
            {
                problem = false;
                oldAreas = new List<double>(areas);

                // find an element with an known area
                for (int i = 0; i < areas.Count; i++)
                {
                    if (areas[i] <= 1e-6)
                        continue;

                    // check the nodes it connects to and update their areas
                    for (int k = 0; k < 2; k++)
                    {
                        Node node = model.Elements[i].Nodes[k];
                        int index = model.Nodes.IndexOf(node);
                        // Find all elements which connects to this node
                        List<int> indices = new List<int>();
                        for (int j = 0; j < areas.Count; j++)
                        {
                            if (model.Elements[j].Nodes.Contains(node))
                            {
                                indices.Add(j);
                            }
                        }
                        // Get relevant normals
                        var normals = new List<Vector3d>();
                        for (int j = 0; j < indices.Count; j++)
                        {
                            var nodes = model.Elements[indices[j]].Nodes;
                            Vector3d normal = ToRhino(nodes[0].Location - nodes[1].Location);
                            normal.Unitize();
                            normal.X = Math.Abs(normal.X);
                            normal.Y = Math.Abs(normal.Y);
                            normal.Z = Math.Abs(normal.Z);
                            normals.Add(normal);
                        }

                        // find the cuboid that fits them all the best
                        Vector3d bbox = GetBoundingBox(indices, normals, ToRhino(node.Location), index);
                        nodeBBox[index] = bbox;

                        var temp = ToRhino(node.Location);
                        boundingBoxes[index] = new BoundingBox(temp - bbox * 0.5, temp + 0.5 * bbox);

                        // Update the areas
                        for (int j = 0; j < indices.Count; j++)
                        {
                            Vector3d normal = normals[j];
                            areas[indices[j]] = ProjectedArea(bbox, normal);
                        }
                    }
                    if (true) //!areaChecked[i])
                    {
                        areaChecked[i] = true;  // Think this needs to know the node sizes, maybe do it every iteration and hope it evens out...
                        problem |= FitElementArea(i);
                    }
                }



                if (areas.Zip(oldAreas, (x, y) => Math.Abs(x - y) / x).All(x => x <= 1e-3))
                    break;
            }
            //    // break if no area changed more than the tolerance
            //} while (areas.Zip(oldAreas, (x, y) => Math.Abs(x - y)).Any(x => x > 1e-3));
            return problem;
        }

        public double ProjectedArea(Vector3d bbox, Vector3d normal)
        {
            return (normal.X) * bbox.Y * bbox.Z +
                (normal.Y) * bbox.X * bbox.Z +
                (normal.Z) * bbox.Y * bbox.X;
        }

        public Vector3d ToRhino(Vector vec)
        {
            return new Vector3d(vec.X, vec.Y, vec.Z);
        }
        public Point3d ToRhino(BriefFiniteElementNet.Point pt)
        {
            return new Point3d(pt.X, pt.Y, pt.Z);
        }

        public bool FitElementArea(int i)
        {
            bool problem = false;
            // Go through every element and step through their length such that the area fits

            // check the nodes it connects to and update their areas
            int k = 0;
            Node node = model.Elements[i].Nodes[k];
            double area = areas[i];

            // Get relevant normal
            var nodes = model.Elements[i].Nodes;
            Vector3d normal = ToRhino(nodes[1].Location - nodes[0].Location);
            int a = model.Nodes.IndexOf(nodes[0]);
            int b = model.Nodes.IndexOf(nodes[1]);
            Vector3d bboxA = nodeBBox[a];
            Vector3d bboxB = nodeBBox[b];

            double length = normal.Length;
            normal.Unitize();
            Vector3d realNormal = normal;
            normal.X = Math.Abs(normal.X);
            normal.Y = Math.Abs(normal.Y);
            normal.Z = Math.Abs(normal.Z);

            var pt = ToRhino(node.Location);
            //Vector3d bbox = SDF.BoxValueAt(pt);
            //area = Math.Min(area, ProjectedArea(bbox, normal));

            double sum = 0;
            int counter = 0;
            while (sum < length)
            {
                // Take the intersection between the max box at the point and the interpolated extreme boxes, this is to avoid too much shape distortion along the element
                Vector3d bbox = boxSDF.BoxValueAt(pt);
                //double t = StepLength(normal, bbox, area);

                Vector3d lerpBox = BoxLERP(bboxA, bboxB, sum / length);
                double t = StepLength(normal, bbox, lerpBox);
                if (t < length * 1e-3)
                {
                    t = mask.delta * 0.5;
                    if (sum + t > length)
                        break;
                    // See if we still fit our box here
                    Vector3d tempBox = boxSDF.BoxValueAt(pt + realNormal * t);
                    lerpBox = BoxLERP(bboxA, bboxB, (sum + t) / length);
                    if (StepLength(normal, tempBox, lerpBox) < - length * 1e-3)
                    {
                        // Decrease area
                        area = Math.Min(area, ProjectedArea(BoxIntersection(tempBox, lerpBox), normal));

                        // would be nice to eventually send back some shape information to the nodes.
                        // As it is now the nodes will only know that they need to decrease their area,
                        // which means it won't do things like getting taller to get around a corner

                        // Or give warning?
                        problem |= true;
                    }
                }

                sum += t;
                pt += realNormal * t;

                counter++;
                if (counter > 1000)
                    break;
            }

            areas[i] = area;
            return problem;
        }

        Vector3d BoxLERP(Vector3d a, Vector3d b, double f)
        {
            return a * (1 - f) + b * f;
        }

        double StepLength(Vector3d n, Vector3d b0, Vector3d b1)
        {
            double t1 = (b0.X - b1.X) / (2.0 * n.X);
            double t2 = (b0.Y - b1.Y) / (2.0 * n.Y);
            double t3 = (b0.Z - b1.Z) / (2.0 * n.Z);

            t1 = double.IsNaN(t1) ? double.PositiveInfinity : t1;
            t2 = double.IsNaN(t2) ? double.PositiveInfinity : t2;
            t3 = double.IsNaN(t3) ? double.PositiveInfinity : t3;

            return Math.Min(Math.Min(t1, t2), t3);
        }

        double StepLength(Vector3d n, Vector3d box, double area)
        {
            // Max step in box
            Vector3d nn = n;
            nn.X /= box.X;
            nn.Y /= box.Y;
            nn.Z /= box.Z;

            double max;
            if (nn.X > nn.Y && nn.X > nn.Z)
                max = box.X * 0.5 / n.X;
            else if (nn.Y > nn.Z)
                max = box.Y * 0.5 / n.Y;
            else
                max = box.Z * 0.5 / n.Z;

            // Exact step due to area
            double a = 12.0 * n.X * n.Y * n.Z;
            double b = 4.0 * (box.X * n.Y * n.Z + n.X * box.Y * n.Z + n.X * n.Y * box.Z);
            double c = n.X * box.Y * box.Z + box.X * n.Y * box.Z + box.X * box.Y * n.Z - area;

            if (a < 1e-6)
            {
                return Math.Min(b < 1e-6 ? n * box * 0.5 : c / b, max);
            }
            else
            {
                if (b * b - 4.0 * a * c < 0)
                    return max;
                else
                {
                    double t1 = (b + Math.Sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
                    double t2 = (b - Math.Sqrt(b * b - 4.0 * a * c)) / (2.0 * a);

                    return Math.Min(t1 > 0 && t1 < t2 ? t1 : t2 > 0 ? t2 : 0, max);
                }
            }
        }

        Vector3d BoxIntersection(Vector3d b1, Vector3d b2)
        {
            return new Vector3d(
                Math.Min(b1.X, b2.X),
                Math.Min(b1.Y, b2.Y),
                Math.Min(b1.Z, b2.Z));
        }

        public Vector3d GetBoundingBox(List<int> indices, List<Vector3d> normals, Point3d node, int index)
        {
            // Find the maximal space available
            //Vector3d bigBox = maxBox[index];
            Vector3d bigBox = boxSDF.BoxValueAt(node);

            // find all areas not zero
            var constraints = new List<Tuple<Vector3d, double>>();
            for (int i = 0; i < indices.Count; i++)
            {
                double area = areas[indices[i]];
                if (area >= 1e-6)
                {
                    constraints.Add(new Tuple<Vector3d, double>(normals[i], area));
                }
            }

            // Steepest ascent algorithm
            double norm = Math.Min(constraints.Min(x => x.Item2), bigBox.MaximumCoordinate * bigBox.MaximumCoordinate);
            double sqNorm = Math.Sqrt(norm);
            Point3d pt = new Point3d(sqNorm, sqNorm, sqNorm);

            for (int i = 0; i < 1000; i++)
            {
                var oldpt = pt;

                Vector3d gradient = new Vector3d();

                //gradient = new Vector3d(
                //    pt.Y * pt.Z,
                //    pt.X * pt.Z,
                //    pt.Y * pt.X);

                double temp = pt.Y * pt.Z + pt.X * (pt.Y + pt.Z);
                gradient.X = pt.Y * pt.Y * pt.Z * pt.Z / (temp * temp);

                temp = pt.X * pt.Z + pt.Y * (pt.X + pt.Z);
                gradient.Y = pt.X * pt.X * pt.Z * pt.Z / (temp * temp);

                temp = pt.Y * pt.X + pt.Z * (pt.Y + pt.X);
                gradient.Z = pt.Y * pt.Y * pt.X * pt.X / (temp * temp);

                // should probobly be scaled with smallest area
                gradient.Unitize();
                pt += 0.2 * sqNorm * gradient;// * (pt.X + pt.Y + pt.Z);
                // Could consider making it always have a fairly fixed step length applied after the projection, there would be issues in making it stop however

                // Project back into feasible space
                pt.X = pt.X > bigBox.X ? bigBox.X : pt.X;
                pt.Y = pt.Y > bigBox.Y ? bigBox.Y : pt.Y;
                pt.Z = pt.Z > bigBox.Z ? bigBox.Z : pt.Z;

                foreach (var constraint in constraints)
                {
                    var n = constraint.Item1;
                    var a = constraint.Item2;

                    if (AreaFunction(pt, n, a) > 0)
                        pt = GradientProjection(pt, n, a);

                }

                // The step length should be evaluated, 

                // And a break condition added
                if (pt.DistanceToSquared(oldpt) < 1e-18)
                    break;
            }

            return (Vector3d)pt;
        }

        double OriginAreaIntersectionSquared(Point3d pt, Vector3d normal, double area)
        {
            return area / (normal.X * pt.Y * pt.Z + normal.Y * pt.X * pt.Z + normal.Z * pt.Y * pt.X);
        }

        double AreaFunction(Point3d pt, Vector3d normal, double area)
        {
            return normal.X * pt.Y * pt.Z + normal.Y * pt.X * pt.Z + normal.Z * pt.Y * pt.X - area;
        }

        Point3d GradientProjection(Point3d pt, Vector3d normal, double area)
        {
            Vector3d gradient = new Vector3d(
                normal.Y*pt.Z + normal.Z*pt.Y,
                normal.X * pt.Z + normal.Z * pt.X,
                normal.Y * pt.X + normal.X * pt.Y);

            double a = normal.X * gradient.Y * gradient.Z + 
                       normal.Y * gradient.X * gradient.Z + 
                       normal.Z * gradient.Y * gradient.X;
            double b = normal.Z * gradient.Y * pt.X +
                       normal.Y * gradient.Z * pt.X +
                       normal.Z * gradient.X * pt.Y +
                       normal.X * gradient.Z * pt.Y +
                       normal.X * gradient.Y * pt.Z +
                       normal.Y * gradient.X * pt.Z;
            double c = normal.Z * pt.X * pt.Y + normal.Y * pt.X * pt.Z + normal.X * pt.Y * pt.Z - area;

            double t1 = (-b + Math.Sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
            double t2 = (-b - Math.Sqrt(b * b - 4.0 * a * c)) / (2.0 * a);

            return pt + gradient * t1;

            // This may be dumb and unnecessary
            // get largest negative solution
            double t = t1 < 0 && t1 > t2 ? t1 : t2 < 0 ? t2 : 0;

            var res = pt + gradient * t;
            return res;
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
