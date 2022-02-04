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

namespace Krill
{
    internal class VoxelConduit : Rhino.Display.DisplayConduit
    {
        // define a local copy of all geometry which is to be redrawn
        public Voxels<int> mask { get; set; } = null;
        public void SetDisplacments(Voxels<Vector3d> disp)
        {
            points = Voxels<bool>.GetPoints(mask, disp);
            box = new BoundingBox(points); 
        }
        public List<Point3d> points { get; set; } = null;
        public BoundingBox box { get; set; } = BoundingBox.Empty;

        public VoxelConduit() : base()
        { }

        protected override void CalculateBoundingBox(CalculateBoundingBoxEventArgs e)
        {
            base.CalculateBoundingBox(e);

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
                e.BoundingBox.Union(mask.origin);
                double d = mask.n * mask.delta;
                e.BoundingBox.Union(mask.origin + new Vector3d(d, d, d));
            }
        }

        protected override void PreDrawObjects(DrawEventArgs e)
        {
            base.PreDrawObjects(e);

            if (mask is null)
                return;

            if (points is null)
                points = mask.GetPointsNotAt(0);
             
            e.Display.DrawPoints(points, PointStyle.RoundSimple, 2, Color.Black);
        }

        public void Update()
        {
            Rhino.RhinoDoc.ActiveDoc?.Views?.Redraw();
        }

    }
}