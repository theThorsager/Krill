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
    class PostProcessingResultsParam : GH_Param<PostProcessingResultsGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("29F660B1-5ECC-41A5-88FE-D4F11D405B6E"); }
        }

        public PostProcessingResultsParam() : 
            base("PostProcessingResults", "Post", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class PostProcessingResultsGoo : GH_Goo<Krill.Containers.PostProcessingResults>
    {
        public PostProcessingResultsGoo()
        {
            this.Value = null;
        }
        public PostProcessingResultsGoo(Krill.Containers.PostProcessingResults post)
        {
            this.Value = post;
        }
        public PostProcessingResultsGoo(PostProcessingResultsGoo post)
        {
            this.Value = post.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "PostProcessingResults";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new PostProcessingResultsGoo(this);
        }

        public override string ToString()
        {
            return "A Krill PostProcessingResults object";
        }
    }
}
