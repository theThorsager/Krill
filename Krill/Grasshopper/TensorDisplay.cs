using System;
using System.Collections.Generic;
using System.Linq;

using Grasshopper.Kernel;
using Rhino.Geometry;

using Krill.Containers;

namespace Krill.Grasshopper
{
    public class TensorDisplay : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the PostProcessing2d class.
        /// </summary>
        public TensorDisplay()
          : base("TensorDisplay", "TensorDisplay",
              "Description",
              "Krill", "Utility")
        {
        }

        public override bool IsPreviewCapable => true;

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.PostProcessingResultsParam());
            pManager.AddNumberParameter("scale", "s", "", GH_ParamAccess.item, 1);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Param.PostProcessingResultsGoo postResults = null;
            DA.GetData(0, ref postResults);

            double scale = 0;
            DA.GetData(1, ref scale);

            if (postResults?.Value is null)
                return;

            var post = postResults.Value;

            int n = post.mask.n;

            var t = new List<Line>();
            var c = new List<Line>();

            for (int i = 0; i < n * n * n; i++)
            {
                if (post.mask.cellValues[i] != 0)
                {
                    Point3d pt = post.mask.IndexToPoint(i);
                    for (int j = 0; j < 3; j++)
                    {
                        Vector3d dir = post.princpDir.cellValues[i][j];
                        double stress = post.princpStress.cellValues[i][j];

                        dir *= stress * scale;
                        Line line = new Line(pt - dir, pt + dir);

                        if (stress > 0)
                            t.Add(line);
                        else
                            c.Add(line);
                    }
                }
            }

            tension = t;
            compression = c;

            box = new BoundingBox(tension.Concat(compression).SelectMany(x => new[] { x.From, x.To }));
        }

        List<Line> tension = new List<Line>();
        List<Line> compression = new List<Line>();
        BoundingBox box = BoundingBox.Empty;
        public override BoundingBox ClippingBox
        {
            get
            {
                return box;
            }
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            args.Display.DrawLines(tension, System.Drawing.Color.Red);
            args.Display.DrawLines(compression, System.Drawing.Color.Blue);
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("6BDF7B77-A9E7-40B6-8214-460FAEE707FA"); }
        }
    }
}