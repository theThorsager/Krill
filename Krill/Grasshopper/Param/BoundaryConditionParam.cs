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
    class BoundaryConditionParam : GH_Param<BoundaryConditionGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("416130C7-14D0-4D5F-B3A5-F7983012E1E4"); }
        }

        public BoundaryConditionParam() : 
            base("BoundaryCondition", "BC", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class BoundaryConditionGoo : GH_Goo<Krill.Containers.IBoundaryCondition>
    {
        public BoundaryConditionGoo()
        {
            this.Value = null;
        }
        public BoundaryConditionGoo(Krill.Containers.IBoundaryCondition bc)
        {
            this.Value = bc;
        }
        public BoundaryConditionGoo(BoundaryConditionGoo bc)
        {
            this.Value = bc.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "BoundaryCondition";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new BoundaryConditionGoo(this);
        }

        public override string ToString()
        {
            return "A Krill BoundaryCondition object";
        }
    }
}
