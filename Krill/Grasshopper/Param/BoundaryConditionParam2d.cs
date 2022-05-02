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
    class BoundaryConditionParam2d : GH_Param<BoundaryCondition2dGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("17E95C68-EC41-47AF-9D29-DAC62D778DD6"); }
        }

        public BoundaryConditionParam2d() : 
            base("BoundaryCondition2d", "BC", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class BoundaryCondition2dGoo : GH_Goo<Krill.Containers.IBoundaryCondition2d>
    {
        public BoundaryCondition2dGoo()
        {
            this.Value = null;
        }
        public BoundaryCondition2dGoo(Krill.Containers.IBoundaryCondition2d bc)
        {
            this.Value = bc;
        }
        public BoundaryCondition2dGoo(BoundaryCondition2dGoo bc)
        {
            this.Value = bc.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "BoundaryCondition2d";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new BoundaryCondition2dGoo(this);
        }

        public override string ToString()
        {
            return "A Krill BoundaryCondition2d object";
        }
    }
}
