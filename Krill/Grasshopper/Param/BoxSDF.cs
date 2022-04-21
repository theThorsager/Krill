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
    class BoxSDFParam : GH_Param<BoxSDFGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("5631ABD4-D80E-4233-98E8-F3AA8D0CA29F"); }
        }

        public BoxSDFParam() : 
            base("BoxSDF", "BoxSDF", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class BoxSDFGoo : GH_Goo<Krill.BoxSDF>
    {
        public BoxSDFGoo()
        {
            this.Value = null;
        }
        public BoxSDFGoo(Krill.BoxSDF settings)
        {
            this.Value = settings;
        }
        public BoxSDFGoo(BoxSDFGoo settings)
        {
            this.Value = settings.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "BoxSDF";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new BoxSDFGoo(this);
        }

        public override string ToString()
        {
            return "A Krill BoxSDF object";
        }
    }
}
