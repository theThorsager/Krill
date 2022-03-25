using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill.Containers
{
    public class TrussGeometry
    {
        public List<Point3d> Nodes { get; set; }
        public List<Tuple<int, int>> Connections { get; set; }
    }
}
