using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

using Krill.Containers;

namespace Krill.Grasshopper
{
    public class PostProcessing2d : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the PostProcessing2d class.
        /// </summary>
        public PostProcessing2d()
          : base("PostProcessing2d", "post",
              "Description",
              "Krill", "Solvers")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearSolutionParam2d());
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.PostProcessingResultsParam2d());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Containers.LinearSolution2d linearSolution = null;
            Param.LinearSolution2dGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            linearSolution = res.Value;

            OutputResults2d results = new OutputResults2d(linearSolution);

            results.UpdateFakeStrains(linearSolution.displacments);
            results.UpdateFakeStress2(linearSolution.displacments);
            results.UpdateVonMises();
            results.UpdatePrincipalStresses();

            Param.PostProcessingResults2dGoo postResults = new Param.PostProcessingResults2dGoo(new PostProcessingResults2d() 
                { mask = linearSolution.mask,
                  strainXX = results.strainXX,
                  strainYY = results.strainYY,
                  strainXY = results.strainXY,
                  stressXX = results.stressXX,
                  stressYY = results.stressYY,
                  stressXY = results.stressXY,
                  vonMises = results.vonMises,
                  princpDir = results.princpDir,
                  princpStress = results.princpStress,
                  stressTensor = results.stressTensor
                    });

            if (!(postResults is null))
                DA.SetData(0, postResults);
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
            get { return new Guid("14B24CE0-E497-41AF-9E5F-29EFDBBF2473"); }
        }
    }
}