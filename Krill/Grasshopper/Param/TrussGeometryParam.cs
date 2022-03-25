using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;

namespace Krill.Grasshopper.Param
{
    class TrussGeometryParam : GH_Param<TrussGeometryGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("8F7A5A1C-6A94-4693-BB07-1D1F58875296"); }
        }

        public TrussGeometryParam() : 
            base("TrussGeometry", "TrussGeometry", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class TrussGeometryGoo : GH_Goo<Krill.Containers.TrussGeometry>
    {
        public TrussGeometryGoo()
        {
            this.Value = null;
        }
        public TrussGeometryGoo(Krill.Containers.TrussGeometry settings)
        {
            this.Value = settings;
        }
        public TrussGeometryGoo(TrussGeometryGoo settings)
        {
            this.Value = settings.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "TrussGeometry";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new TrussGeometryGoo(this);
        }

        public override string ToString()
        {
            return "A Krill TrussGeometry object";
        }
    }
}
