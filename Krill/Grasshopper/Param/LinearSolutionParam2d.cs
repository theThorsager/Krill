using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using GH_IO.Serialization;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;

namespace Krill.Grasshopper.Param
{
    class LinearSolutionParam2d : GH_Param<LinearSolution2dGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("4507538F-6EA2-41ED-9666-0317F946C3D7"); }
        }

        public LinearSolutionParam2d() : 
            base("LinearSolution", "LinSol", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class LinearSolution2dGoo : GH_Goo<Krill.Containers.LinearSolution2d>
    {
        public LinearSolution2dGoo()
        {
            this.Value = null;
        }
        public LinearSolution2dGoo(Krill.Containers.LinearSolution2d linSol)
        {
            this.Value = linSol;
        }
        public LinearSolution2dGoo(LinearSolution2dGoo linSol)
        {
            this.Value = linSol.Value;
        }

        public override bool Write(GH_IWriter writer)
        {
            Krill.Grasshopper.WritingHelper.GH_WriterHelper.Write(writer, "this", Value);
            return base.Write(writer);
        }

        public override bool Read(GH_IReader reader)
        {
            Value = WritingHelper.GH_ReaderHelper.Reader(reader, "this", new Containers.LinearSolution2d());
            return base.Read(reader);
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "LinearSolution2d";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new LinearSolution2dGoo(this);
        }

        public override string ToString()
        {
            return "A Krill LinearSolution2d object";
        }
    }
}
