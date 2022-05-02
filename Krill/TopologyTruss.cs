using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    internal class TopologyTruss
    {
        public Voxels<double> weights;

        public VoxelSubgradient subgradient;

        const double lengthfactor = 0.75;
        const double movefactor = 0.1;
        const double forcefactor = 0.5;
        const double maxmove = 2.0;

        struct Indices
        {
            public int i;
            public bool start;
            public Indices(int i, bool start)
            {
                this.i = i;
                this.start = start;
            }
            public override bool Equals(object obj)
            {
                if (obj is int i)
                    return this.i == i;
                else if (obj is Indices I)
                    return this.i == I.i;
                else
                    return base.Equals(obj);
            }
            public override int GetHashCode()
            {
                return i.GetHashCode();
            }

            public static implicit operator Indices(int i)
            {
                return new Indices(i, true);
            }
        }

        class Intersection
        {
            public Point3d pos;
            public bool locked;
            public bool dead;
            public List<Indices> ind;
            public Intersection(Point3d pos, List<Indices> ind, bool locked = false)
            {
                this.pos = pos;
                this.locked = locked;
                this.ind = ind;
                this.dead = false;
            }
        }

        public TopologyTruss(Voxels<double> weights)
        {
            this.weights = weights;
            subgradient = new VoxelSubgradient(weights);
        }

        bool inside(int i)
        {
            int n = weights.n;
            weights.To3DIndex(ref i, out int j, out int k);
            return i >= 1 && i < n - 2 &&
                   j >= 1 && j < n - 2 &&
                   k >= 1 && k < n - 2;
        }

        public void Main(ref List<Polyline> plines, List<Point3d> intersections, int n)
        {
            double d = 0;
            foreach (var pline in plines)
            {
                double length = 0;
                for (int i = 1; i < pline.Count; i++)
                {
                    length += pline[i].DistanceTo(pline[i - 1]);
                }
                length /= (pline.Count - 1);
                length *= lengthfactor; // Make the string always stretched
                d += length;
            }
            d /= plines.Count;

            d *= 0.5;

            var intersect = ConstructIntersections(plines, intersections);

            for (int ii = 0; ii < n; ii++)
            {
                ModifyIntersections(plines, ref intersect, d);
                //UglyCleanup(intersect, plines);     // should probably not use this
                HandlePolylineLength(plines, intersect, d);
                for (int i = 0; i < 30; i++)
                {
                    plines = UpdateIntermidiate(plines, out var lengths);
                    UpdateIntersections(plines, lengths, intersect);
                }
            }
            //while (ModifyIntersections(plines, ref intersect, d))
            //{
            //    for (int i = 0; i < 30; i++)
            //    {
            //        plines = UpdateIntermidiate(plines, out var lengths);
            //        UpdateIntersections(plines, lengths, intersect);
            //    }
            //}

            // for debugging
            intersections.Clear();
            intersections.AddRange(intersect.Select(x => x.pos));

            GetEndPoints(intersect, plines);
        }

        public List<Point3d> ptsA = null;
        public List<Point3d> ptsB = null;
        void GetEndPoints(List<Intersection> intersections, List<Polyline> plines)
        {
            ptsA = new List<Point3d>();
            ptsB = new List<Point3d>();

            for (int i = 0; i < plines.Count; i++)
            {
                var pline = plines[i];

                if (pline.Count == 0)
                {
                    ptsA.Add(Point3d.Unset);
                    ptsB.Add(Point3d.Unset);
                    continue;
                }

                var res = intersections.Where(x => x.ind.Contains(i)).Select(x => x.pos);

                ptsA.Add(res.First());
                ptsB.Add(res.Last());

            }
        }

        void HandlePolylineLength(List<Polyline> plines, List<Intersection> intersections, double d)
        {
            const double factor = 0.3;
            double sqtol = d * d * factor * factor;

            for (int i = 0; i < plines.Count; i++)
            {
                var pline = plines[i];
                if (pline.Count == 0)
                    continue;

                if (pline.Count == 2)
                {
                    Vector3d rel = pline[0] - pline[1];
                    if (rel.SquareLength < sqtol)
                    {
                        // find the two intersections connecting to it and merge them
                        Point3d point = Point3d.Unset;
                        bool locked = false;

                        List<Indices> ind = new List<Indices>();

                        Point3d ave = Point3d.Unset;
                        for (int j = 0; j < intersections.Count; j++)
                        {
                            var inter = intersections[j];
                            if (inter.ind.Contains(i))
                            {
                                inter.dead = true;
                                if (inter.locked)
                                {
                                    point = inter.pos;
                                    locked = true;
                                }
                                for (int ii = 0; ii < inter.ind.Count; ii++)
                                {
                                    if (inter.ind[ii].i == i)
                                    {
                                        inter.ind.RemoveAt(ii);
                                        break;
                                    }
                                }

                                ind.AddRange(inter.ind);

                                if (!ave.IsValid)
                                {
                                    ave = inter.pos;
                                }
                                else
                                {
                                    ave += inter.pos;
                                    break;
                                }
                            }
                        }
                        if (!point.IsValid)
                            point = ave * 0.5;

                        RemoveDuplicates(ind, plines);
                        CorrectEndpoints(plines, ind, point);

                        Intersection res = new Intersection(point, ind, locked);
                        intersections.Add(res);
                        pline.Clear();
                    }
                }
            }
        }

        List<Intersection> ConstructIntersections(List<Polyline> plines, List<Point3d> intersections)
        {
            const double sqtol = 1e-6;

            var result = new List<Intersection>();
            for (int k = 0; k < intersections.Count; k++)
            {
                Point3d pt = intersections[k];
                // What is connecting up to this intersection?
                var ind = new List<Indices>();
                for (int j = 0; j < plines.Count; j++)
                {
                    if (plines[j].First.DistanceToSquared(pt) < sqtol)
                    {
                        ind.Add(new Indices(j, true));
                    }
                    else if (plines[j].Last.DistanceToSquared(pt) < sqtol)
                    {
                        ind.Add(new Indices(j, false));
                    }
                }
                result.Add(new Intersection(pt, ind, true));
            }

            return result;
        }
        /*
         * Check existing intersections if one can collapse and maybe move the intersection
         * rebuild the polylines
         * 
         * for {
         * update as usual for all intermidiate nodes
         * special update for the intersections
         * }
         * 
         * repeat until no more intersection movement can be made
         */

        bool ModifyIntersections(List<Polyline> plines, ref List<Intersection> intersections, double d)
        {
            bool result = false;
            for (int k = intersections.Count - 1; k >= 0; k--)
            {
                var inter = intersections[k];

                if (inter.dead)
                    continue;

                // What is connecting up to this intersection?
                var points = new List<Point3d>();
                for (int j = 0; j < inter.ind.Count; j++)
                {
                    int ind = inter.ind[j].i;
                    int jj = inter.ind[j].start ? 1 : plines[ind].Count - 2;
                    points.Add(plines[ind][jj]);
                }

                // can we collapse those into something new
                // cluster connections
                // start points on every point and let them step towards the distance-weighted average
                var indecies = points.Select((x, j) => j).ToList();
                double L = d * 1;       // factor seems to depend on the ratio between d and polyline point distance
                double sqL = L * L;
                var newPoints = new List<Point3d>();
                while (indecies.Count > 0)
                {
                    int index = indecies.Last();
                    indecies.RemoveAt(indecies.Count - 1);

                    Point3d current = points[index];
                    for (int i = 0; i < 30; i++)
                    {
                        Point3d mean = Point3d.Origin;
                        double count = 0;
                        for (int j = 0; j < points.Count; j++)
                        {
                            Vector3d rel = current - points[j];
                            if (rel.SquareLength < sqL)
                            {
                                double factor = -rel.Length / L + 1;
                                mean += points[j] * factor;
                                count += factor;
                            }
                        }
                        mean /= count;
                        current = mean;
                    }

                    const double fac = 0.3;
                    for (int j = 0; j < points.Count; j++)
                        if ((current - points[j]).SquareLength < (sqL * fac * fac))
                            indecies.Remove(j);

                    newPoints.Add(current);
                }

                for (int i = newPoints.Count - 1; i >= 0; i--)
                {
                    if (!newPoints[i].IsValid)
                        newPoints.RemoveAt(i);
                }

                // clear out duplicates
                // By angle
                double tol = 10 * 0.0174532925;
                for (int i = 0; i < newPoints.Count; i++)
                {
                    for (int j = newPoints.Count - 1; j > i; j--)
                    {
                        if (Vector3d.VectorAngle((newPoints[i] - inter.pos), (newPoints[j] - inter.pos)) < tol)
                            newPoints.RemoveAt(j);
                    }
                }

                if (newPoints.Count == points.Count)
                {
                    //CorrectEndpoints(plines, inter.curve, inter.ind, inter.pos);
                    // all is well and the intersection does not need to be modified
                    //newIntersections.Add(inter);
                    continue;
                }
                result = true;

                // Find how many buddies each newPoints has
                var inds = new List<Indices>[newPoints.Count];
                for (int ii = 0; ii < newPoints.Count; ii++)
                {
                    inds[ii] = new List<Indices>();
                }

                for (int ki = 0; ki < inter.ind.Count; ki++)
                {
                    int i = inter.ind[ki].i;
                    int j = inter.ind[ki].start ? 1 : plines[i].Count - 2;
                    Point3d pt = plines[i][j];

                    double min = double.MaxValue;
                    int ind = -1;

                    for (int ii = 0; ii < newPoints.Count; ii++)
                    {
                        double temp = pt.DistanceToSquared(newPoints[ii]);
                        if (temp < min)
                        {
                            min = temp;
                            ind = ii;
                        }
                    }
                    inds[ind].Add(inter.ind[ki]);
                }

                if (newPoints.Count == 2 && !inter.locked)
                {
                    // This intersection should be moved/removed/replaced
                    inter.dead = true;
                    if (inds[0].Count > 1 && inds[1].Count > 1)
                    {
                        // add new curve connecting them
                        Polyline pline = new Polyline() { newPoints[0], inter.pos, newPoints[1] };
                        plines.Add(pline);

                        for (int index = 0; index < 2; index++)
                        {
                            var ind = inds[index];

                            var extraind = new List<Indices>();
                            Point3d locked = Point3d.Unset;

                            for (int ii = ind.Count - 1; ii >= 0; ii--)
                            {
                                int i = ind[ii].i;
                                int j = ind[ii].start ? 0 : plines[i].Count - 1;
                                plines[i].RemoveAt(j);
                                if (plines[i].Count == 1)
                                {
                                    MergeIntersections(intersections, plines[i], i, extraind, ref locked, k);
                                    ind.RemoveAt(ii);
                                }
                            }

                            ind.Add(new Indices(plines.Count - 1, index == 0));

                            ind.AddRange(extraind);

                            // Remove duplicates
                            RemoveDuplicates(ind, plines);

                            CorrectEndpoints(plines, ind, locked.IsValid ? locked : newPoints[index]);

                            if (locked.IsValid)
                                intersections.Add(new Intersection(locked, ind, true));
                            else
                                intersections.Add(new Intersection(newPoints[index], ind));
                        }
                    }
                    else
                    {
                        int index = inds[0].Count > 1 ? 0 : 1;
                        var ind = inds[index];

                        var extraind = new List<Indices>();
                        Point3d locked = Point3d.Unset;

                        for (int ii = ind.Count - 1; ii >= 0; ii--)
                        {
                            int i = ind[ii].i;
                            int j = ind[ii].start ? 0 : plines[i].Count - 1;

                            plines[i].RemoveAt(j);
                            if (plines[i].Count == 1)
                            {
                                MergeIntersections(intersections, plines[i], i, extraind, ref locked, k);
                                ind.RemoveAt(ii);
                            }
                        }

                        {
                            // pull up the other polyline after it
                            int otherIndex = -(index - 1);
                            int i = inds[otherIndex][0].i;
                            int j = inds[otherIndex][0].start ? 0 : plines[i].Count;
                            plines[i].Insert(j, newPoints[index]);
                            ind.Add(inds[otherIndex][0]);
                        }

                        ind.AddRange(extraind);

                        // Remove duplicates
                        RemoveDuplicates(ind, plines);

                        CorrectEndpoints(plines, ind, locked.IsValid ? locked : newPoints[index]);

                        if (locked.IsValid)
                            intersections.Add(new Intersection(locked, ind, true));
                        else
                            intersections.Add(new Intersection(newPoints[index], ind));
                    }
                }
                else
                {
                    var oldIntInd = new List<Indices>();
                    // we need a new curve between this and any new intersections
                    for (int ii = 0; ii < inds.Length; ii++)
                    {
                        if (inds[ii].Count > 1)
                        {
                            Polyline pline = new Polyline() { inter.pos, newPoints[ii] };
                            plines.Add(pline);
                            var ind = inds[ii];

                            var extraind = new List<Indices>();
                            Point3d locked = Point3d.Unset;

                            for (int jj = ind.Count - 1; jj >= 0; jj--)
                            {
                                int i = ind[jj].i;
                                int j = ind[jj].start ? 0 : plines[i].Count - 1;
                                plines[i].RemoveAt(j);
                                if (plines[i].Count == 1)
                                {
                                    MergeIntersections(intersections, plines[i], i, extraind, ref locked, k);
                                    ind.RemoveAt(jj);
                                }
                            }

                            ind.Add(new Indices(plines.Count - 1, false));
                            oldIntInd.Add(new Indices(plines.Count - 1, true));

                            ind.AddRange(extraind);

                            // Remove duplicates
                            RemoveDuplicates(ind, plines);

                            CorrectEndpoints(plines, ind, locked.IsValid ? locked : newPoints[ii]);

                            if (locked.IsValid)
                                intersections.Add(new Intersection(locked, ind, true));
                            else
                                intersections.Add(new Intersection(newPoints[ii], ind));
                        }
                        else
                        {
                            oldIntInd.Add(inds[ii][0]);
                        }
                    }

                    CorrectEndpoints(plines, oldIntInd, inter.pos);
                    inter.ind = oldIntInd;
                    // This intersection remains
                    //newIntersections.Add(new Intersection(inter.pos, oldIntCurve, oldIntInd, inter.locked));
                }
            }

            intersections = intersections.Where(x => !x.dead).ToList();

            return result;
        }

        void RemoveIndex(List<Indices> ind, int index)
        {
            for (int j = ind.Count - 1; j >= 0; j--)
            {
                if (ind[j].i == index)
                {
                    ind.RemoveAt(j);
                }
            }
        }

        void RemoveDuplicates(List<Indices> ind, List<Polyline> plines)
        {
            for (int i = ind.Count - 1; i >= 1; i--)
            {
                for (int j = i - 1; j >= 0; j--)
                {
                    if (ind[i].i == ind[j].i)
                    {
                        int index = ind[i].i;
                        plines[index].Clear();

                        ind.RemoveAt(i);
                        ind.RemoveAt(j);

                        i--;
                        break;
                    }
                }
            }

        }

        void CorrectEndpoints(List<Polyline> plines, List<Indices> ind, Point3d pt)
        {
            for (int i = 0; i < ind.Count; i++)
            {
                if (ind[i].start)
                    plines[ind[i].i].First = pt;
                else
                    plines[ind[i].i].Last = pt;
            }
        }

        void MergeIntersections(List<Intersection> intersections, Polyline pline, int index, List<Indices> ind, ref Point3d locked, int me)
        {
            const double sqtol = 1e-6;
            // find the other intersection
            Intersection other = null;
            for (int i = 0; i < intersections.Count; i++)
            {
                if (i != me && !intersections[i].dead && intersections[i].ind.Contains(index))
                {
                    other = intersections[i];
                    break;
                }
            }
            if (other is null)
            {
                if (ind.Contains(index))
                {
                    // this curve already attatches to it self
                    RemoveIndex(ind, index);
                    pline.Clear();
                    return;
                }


                // let there be error
            }

            RemoveIndex(other.ind, index);

            ind.AddRange(other.ind);

            if (other.locked)
                locked = other.pos;

            other.dead = true;
            pline.Clear();
        }

        void UpdateIntersections(List<Polyline> plines, List<double> lengths, List<Intersection> intersections)
        {
            const double sqtol = 1e-6;
            var moves = new List<Vector3d>();
            foreach (var inter in intersections)
            {
                Vector3d force = new Vector3d();
                if (inter.locked || inter.dead)
                {
                    moves.Add(force);
                    continue;
                }
                // Add stringiness
                for (int i = 0; i < inter.ind.Count; i++)
                {
                    double length = lengths[inter.ind[i].i];
                    Polyline pline = plines[inter.ind[i].i];

                    if (inter.ind[i].start)
                    {
                        Vector3d dir = pline[1] - pline[0];
                        double l = dir.Length;
                        dir /= l;
                        double s1 = (l - length) / length;
                        force += s1 * dir;
                    }
                    else
                    {
                        Vector3d dir = pline[pline.Count - 2] - pline.Last;
                        double l = dir.Length;
                        dir /= l;
                        double s1 = (l - length) / length;
                        force += s1 * dir;
                    }
                }

                force *= 0.1;   // They move like a lot

                int ii = weights.PointToIndex(inter.pos);
                if (inside(ii))
                {
                    // Direction of decent
                    force += subgradient.SubGradientAt(inter.pos);
                    force *= forcefactor;
                }

                // dont go further than two delta
                double factor = (weights.delta * weights.delta * maxmove * maxmove) / force.SquareLength;
                if (!double.IsNaN(factor) && factor < 1)
                    force *= factor;

                moves.Add(force * movefactor * 0.1);
            }

            for (int i = 0; i < intersections.Count; i++)
            {
                intersections[i].pos += moves[i];
                for (int j = 0; j < intersections[i].ind.Count; j++)
                {
                    if (intersections[i].ind[j].start)
                    {
                        plines[intersections[i].ind[j].i].First += moves[i];
                    }
                    else
                    {
                        plines[intersections[i].ind[j].i].Last += moves[i];
                    }
                }
            }
        }

        List<Polyline> UpdateIntermidiate(List<Polyline> plines, out List<double> lengths)
        {
            List<Polyline> results = new List<Polyline>();
            lengths = new List<double>();
            foreach (var pline in plines)
            {
                if (pline.Count == 0)
                {
                    results.Add(pline);
                    lengths.Add(0);
                    continue;
                }
                double length = 0;
                for (int i = 1; i < pline.Count; i++)
                {
                    length += pline[i].DistanceTo(pline[i - 1]);
                }
                length /= (pline.Count - 1);
                length *= lengthfactor; // Make the string always stretched
                lengths.Add(length);

                Polyline res = new Polyline() { pline[0] };
                for (int i = 1; i < pline.Count - 1; i++)
                {
                    int ii = weights.PointToIndex(pline[i]);
                    if (!inside(ii))
                    {
                        res.Add(pline[i]);
                        continue;
                    }

                    // Direction of decent
                    Vector3d force = subgradient.SubGradientAt(pline[i]);
                    force *= forcefactor;

                    // project to tangent plane
                    Vector3d tan = pline[i + 1] - pline[i - 1];
                    tan.Unitize();
                    Vector3d move = force;
                    double dist = Vector3d.Multiply(move, tan);

                    move -= dist * tan;

                    // dont go further than two delta
                    double factor = (weights.delta * weights.delta * maxmove * maxmove) / move.SquareLength;
                    if (!double.IsNaN(factor) && factor < 1)
                        move *= factor;

                    // string ness
                    Vector3d dir1 = pline[i + 1] - pline[i];
                    double l1 = dir1.Length;
                    dir1 /= l1;
                    double s1 = (l1 - length) / length;
                    move += s1 * dir1;

                    Vector3d dir2 = pline[i - 1] - pline[i];
                    double l2 = dir2.Length;
                    dir2 /= l2;
                    double s2 = (l2 - length) / length;
                    move += s2 * dir2;


                    res.Add(pline[i] + move * movefactor);
                }
                res.Add(pline[pline.Count - 1]);
                results.Add(res);
            }

            return results;
        }
    }
}
