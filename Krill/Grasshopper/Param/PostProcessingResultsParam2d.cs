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
    class PostProcessingResultsParam2d : GH_Param<PostProcessingResults2dGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("8D68A398-FA17-4C46-85D9-D4E27C63A063"); }
        }

        public PostProcessingResultsParam2d() : 
            base("PostProcessingResults", "Post", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class PostProcessingResults2dGoo : GH_Goo<Krill.Containers.PostProcessingResults2d>
    {
        public PostProcessingResults2dGoo()
        {
            this.Value = null;
        }
        public PostProcessingResults2dGoo(Krill.Containers.PostProcessingResults2d post)
        {
            this.Value = post;
        }
        public PostProcessingResults2dGoo(PostProcessingResults2dGoo post)
        {
            this.Value = post.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "PostProcessingResults2d";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new PostProcessingResults2dGoo(this);
        }

        public override string ToString()
        {
            return "A Krill PostProcessingResults2d object";
        }
    }
}
