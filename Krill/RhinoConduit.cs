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
    internal class RhinoConduit : Rhino.Display.DisplayConduit
    {
        // define a local copy of all geometry which is to be redrawn
        public Point3d Pt { get; set; } = Point3d.Unset;

        public RhinoConduit() : base()
        { }

        protected override void CalculateBoundingBox(CalculateBoundingBoxEventArgs e)
        {
            // call base?
            if (Pt != Point3d.Unset)
            {
                e.BoundingBox.Union(Pt);
            }
        }

        protected override void PreDrawObjects(DrawEventArgs e)
        {
            base.PreDrawObjects(e);

            if (Pt != Point3d.Unset)
                e.Display.DrawPoint(Pt, Color.Red);
        }

        public void Update()
        {
            Rhino.RhinoDoc.ActiveDoc?.Views?.Redraw();
        }

    }
}