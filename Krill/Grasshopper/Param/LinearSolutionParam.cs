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
    class LinearSolutionParam : GH_Param<LinearSolutionGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("4A58F0E2-9BEA-4AA2-86BA-E7003DF7EE3E"); }
        }

        public LinearSolutionParam() : 
            base("LinearSolution", "LinSol", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class LinearSolutionGoo : GH_Goo<Krill.Containers.LinearSolution>
    {
        public LinearSolutionGoo()
        {
            this.Value = null;
        }
        public LinearSolutionGoo(Krill.Containers.LinearSolution linSol)
        {
            this.Value = linSol;
        }
        public LinearSolutionGoo(LinearSolutionGoo linSol)
        {
            this.Value = linSol.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "LinearSolution";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new LinearSolutionGoo(this);
        }

        public override string ToString()
        {
            return "A Krill LinearSolution object";
        }
    }
}
