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
    class LinearPeridynamicsModelParam : GH_Param<LinearPeridynamicsModelGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("9CBFDA6B-8C04-41C5-B6AD-BAEA61F588BC"); }
        }

        public LinearPeridynamicsModelParam() : 
            base("LinearPeridynamicsModel", "LinPeriModel", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class LinearPeridynamicsModelGoo : GH_Goo<Krill.Containers.LinearPeridynamicsModel>
    {
        public LinearPeridynamicsModelGoo()
        {
            this.Value = null;
        }
        public LinearPeridynamicsModelGoo(Krill.Containers.LinearPeridynamicsModel linSol)
        {
            this.Value = linSol;
        }
        public LinearPeridynamicsModelGoo(LinearPeridynamicsModelGoo linSol)
        {
            this.Value = linSol.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "LinearPeridynamicsModel";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new LinearPeridynamicsModelGoo(this);
        }

        public override string ToString()
        {
            return "A Krill LinearPeridynamicsModel object";
        }
    }
}
