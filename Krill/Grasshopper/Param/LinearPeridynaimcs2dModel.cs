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
    class LinearPeridynamics2dModelParam : GH_Param<LinearPeridynamics2dModelGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("069BBFA0-3380-4882-A708-2453A4918EB5"); }
        }

        public LinearPeridynamics2dModelParam() : 
            base("LinearPeridynamics2dModel", "LinPeri2dModel", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class LinearPeridynamics2dModelGoo : GH_Goo<Krill.Containers.LinearPeridynamics2dModel>
    {
        public LinearPeridynamics2dModelGoo()
        {
            this.Value = null;
        }
        public LinearPeridynamics2dModelGoo(Krill.Containers.LinearPeridynamics2dModel linSol)
        {
            this.Value = linSol;
        }
        public LinearPeridynamics2dModelGoo(LinearPeridynamics2dModelGoo linSol)
        {
            this.Value = linSol.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "LinearPeridynamics2dModel";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new LinearPeridynamics2dModelGoo(this);
        }

        public override string ToString()
        {
            return "A Krill LinearPeridynamics2dModel object";
        }
    }
}
