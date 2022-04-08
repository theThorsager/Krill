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
    class DiscreteBoundaryConditionParam : GH_Param<DiscreteBoundaryConditionGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("0BD2646E-E723-41D7-BCB2-A229F0DA8654"); }
        }

        public DiscreteBoundaryConditionParam() : 
            base("BoundaryCondition", "BC", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class DiscreteBoundaryConditionGoo : GH_Goo<Krill.Containers.IDiscreteBoundaryCondition>
    {
        public DiscreteBoundaryConditionGoo()
        {
            this.Value = null;
        }
        public DiscreteBoundaryConditionGoo(Krill.Containers.IDiscreteBoundaryCondition bc)
        {
            this.Value = bc;
        }
        public DiscreteBoundaryConditionGoo(DiscreteBoundaryConditionGoo bc)
        {
            this.Value = bc.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "DiscreteBoundaryCondition";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new DiscreteBoundaryConditionGoo(this);
        }

        public override string ToString()
        {
            return "A Krill DiscreteBoundaryCondition object";
        }
    }
}
