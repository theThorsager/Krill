using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace Krill.Grasshopper
{
    public class KrillInfo : GH_AssemblyInfo
    {
        public override string Name => "Krill";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("D91DD523-6DD3-4A1C-BA77-D86AB326CE0B");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}