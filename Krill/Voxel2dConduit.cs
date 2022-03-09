using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Threading;
using System.Drawing;
using Rhino;
using Rhino.Commands;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.DocObjects;
using Rhino.Display;
using Grasshopper.Kernel;

namespace Krill
{
    internal class Voxel2dConduit : Rhino.Display.DisplayConduit
    {
        // define a local copy of all geometry which is to be redrawn
        public Voxels2d<int> mask { get; set; } = null;
        public GH_Component component { get; set; } = null;
        public void SetDisplacments(Voxels2d<Vector2d> disp)
        {
            points = Voxels2d<bool>.GetPoints(mask, disp, 10);
            box = new BoundingBox(points.Where(x => x.IsValid).Select(x => new Point3d(x.X, x.Y, 0))); 
        }
        public List<Point2d> points { get; set; } = null;
        public BoundingBox box { get; set; } = BoundingBox.Empty;

        public Voxel2dConduit() : base()
        { }

        protected override void CalculateBoundingBox(CalculateBoundingBoxEventArgs e)
        {
            base.CalculateBoundingBox(e);

            if (component == null) return;
            if (component.Hidden) return;

            if (mask is null)
                return;

            if (points is null)
                return;

            if (box.IsValid)
            {
                e.IncludeBoundingBox(box);
            }
            else
            {
                e.BoundingBox.Union(new Point3d(mask.origin.X, mask.origin.Y, 0));
                double d = mask.n * mask.delta;
                Point2d pt = mask.origin + new Vector2d(d, d);
                e.BoundingBox.Union(new Point3d(pt.X, pt.Y, 0));
            }
        }

        protected override void PreDrawObjects(DrawEventArgs e)
        {
            base.PreDrawObjects(e);

            if (component == null) return;
            if (component.Hidden) return;

            if (mask is null)
                return;

            if (points is null)
                points = mask.GetPointsNotAt(0);
             
            e.Display.DrawPoints(points.Select(x => new Point3d(x.X, x.Y, 0)), PointStyle.RoundSimple, 2, Color.Black);
        }

        public void Update()
        {
            Rhino.RhinoDoc.ActiveDoc?.Views?.Redraw();
        }

    }
}