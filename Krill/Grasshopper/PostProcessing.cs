using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

using Krill.Containers;

namespace Krill.Grasshopper
{
    public class PostProcessing : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the PostProcessing2d class.
        /// </summary>
        public PostProcessing()
          : base("PostProcessing", "post",
              "Description",
              "Krill", "Solvers")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param.LinearSolutionParam());
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param.PostProcessingResultsParam());
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Containers.LinearSolution linearSolution = null;
            Param.LinearSolutionGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            linearSolution = res.Value;

            OutputResults results = new OutputResults(linearSolution);

            //results.UpdateFakeStrains(linearSolution.displacments);
            results.UpdateFakeStress2(linearSolution.displacments);
            results.UpdateVonMises();
            results.UpdatePrincipalStresses();
            results.CalcDivOfStressField();

            Param.PostProcessingResultsGoo postResults = new Param.PostProcessingResultsGoo(new PostProcessingResults() 
                { mask = linearSolution.mask,

                  strainXX = results.strainXX,
                  strainYY = results.strainYY,
                  strainZZ = results.strainZZ,
                  strainXY = results.strainXY,
                  strainXZ = results.strainXZ,
                  strainYZ = results.strainYZ,

                  stressXX = results.stressXX,
                  stressYY = results.stressYY,
                  stressZZ = results.stressZZ,
                  stressXY = results.stressXY,
                  stressXZ = results.stressXZ,
                  stressYZ = results.stressYZ,

                  vonMises = results.vonMises,
                  princpDir = results.princpDir,
                  princpStress = results.princpStress,
                  stressTensor = results.stressTensor,

                  lengthDivStress = results.lengthDivStress,
                  divOfStress = results.divOfStress
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
            get { return new Guid("53A7ADDF-06FE-4CF7-8FB8-15C7B321472A"); }
        }
    }
}