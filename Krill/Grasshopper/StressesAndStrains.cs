using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Krill.Grasshopper
{
    public class StressesAndStrains : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public StressesAndStrains()
          : base("StressesAndStrains", "SS",
              "Description",
              "Krill", "TestComponents")
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
            pManager.AddPointParameter("OriginalPositions", "orgPt", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("vonMisesStress", "eVM", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StressXX", "sXX", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StressYY", "sYY", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StressZZ", "sZZ", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StressXY", "sXY", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StressXZ", "sXZ", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StressYZ", "sYZ", "", GH_ParamAccess.list);

            pManager.AddNumberParameter("StrainXX", "eXX", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StrainYY", "eYY", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StrainZZ", "eZZ", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StrainXY", "eXY", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StrainXZ", "eXZ", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("StrainYZ", "eYZ", "", GH_ParamAccess.list);

            pManager.AddVectorParameter("PrincpDir1", "dir1", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("PrincpDir2", "dir2", "", GH_ParamAccess.list);
            pManager.AddVectorParameter("PrincpDir3", "dir3", "", GH_ParamAccess.list);

            pManager.AddNumberParameter("PrincpStress1", "s1", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("PrincpStress2", "s2", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("PrincpStress3", "s3", "", GH_ParamAccess.list);

            pManager.AddNumberParameter("SumOfCurl", "curl", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Get Data
            Containers.LinearSolution linearSolution = null;
            Param.LinearSolutionGoo res = null;
            DA.GetData(0, ref res);
            if (res is null)
                return;
            linearSolution = res.Value;

            // Do work
            OutputResults outputR = new OutputResults(linearSolution);
            outputR.UpdateFakeStrains(linearSolution.displacments);
            //outputR.UpdateFakeStress(linearSolution.displacments);
            outputR.UpdateStresses();
            outputR.UpdateVonMises();
            outputR.UpdatePrincipalStresses();
            outputR.CalcCurlOfStressField();

            List<Point3d> orgPoints = new List<Point3d>();

            List<double> vonMises = new List<double>();

            List<double> stressXX = new List<double>();
            List<double> stressYY = new List<double>();
            List<double> stressZZ = new List<double>();
            List<double> stressXY = new List<double>();
            List<double> stressXZ = new List<double>();
            List<double> stressYZ = new List<double>();

            List<double> strainXX = new List<double>();
            List<double> strainYY = new List<double>();
            List<double> strainZZ = new List<double>();
            List<double> strainXY = new List<double>();
            List<double> strainXZ = new List<double>();
            List<double> strainYZ = new List<double>();

            List<Vector3d> dir1 = new List<Vector3d>();
            List<Vector3d> dir2 = new List<Vector3d>();
            List<Vector3d> dir3 = new List<Vector3d>();

            List<double> stress1 = new List<double>();
            List<double> stress2 = new List<double>();
            List<double> stress3 = new List<double>();

            List<double> curlSum = new List<double>();

            const int maskbit = 0x000000FF;

            for (int i = 0; i < linearSolution.mask.n * linearSolution.mask.n * linearSolution.mask.n; i++)
            {
                if ((linearSolution.mask.cellValues[i] & maskbit) == 0)
                    continue;

                orgPoints.Add(linearSolution.mask.IndexToPoint(i));

                vonMises.Add(outputR.vonMises.cellValues[i]);

                stressXX.Add(outputR.stressXX.cellValues[i]);
                stressYY.Add(outputR.stressYY.cellValues[i]);
                stressZZ.Add(outputR.stressZZ.cellValues[i]);
                stressXY.Add(outputR.stressXY.cellValues[i]);
                stressXZ.Add(outputR.stressXZ.cellValues[i]);
                stressYZ.Add(outputR.stressYZ.cellValues[i]);

                strainXX.Add(outputR.strainXX.cellValues[i]);
                strainYY.Add(outputR.strainYY.cellValues[i]);
                strainZZ.Add(outputR.strainZZ.cellValues[i]);
                strainXY.Add(outputR.strainXY.cellValues[i]);
                strainXZ.Add(outputR.strainXZ.cellValues[i]);
                strainYZ.Add(outputR.strainYZ.cellValues[i]);

                dir1.Add(outputR.princpDir.cellValues[i][0]);
                dir2.Add(outputR.princpDir.cellValues[i][1]);
                dir3.Add(outputR.princpDir.cellValues[i][2]);

                stress1.Add(outputR.princpStress.cellValues[i].X);
                stress2.Add(outputR.princpStress.cellValues[i].Y);
                stress3.Add(outputR.princpStress.cellValues[i].Z);

                curlSum.Add(outputR.sumCurl.cellValues[i]);
            }

            // Set data
            if (orgPoints != null)
                DA.SetDataList(0, orgPoints);
            if (vonMises != null)
                DA.SetDataList(1, vonMises);

            if (stressXX != null)
                DA.SetDataList(2, stressXX);
            if (stressYY != null)
                DA.SetDataList(3, stressYY);
            if (stressZZ != null)
                DA.SetDataList(4, stressZZ);
            if (stressXY != null)
                DA.SetDataList(5, stressXY);
            if (stressXZ != null)
                DA.SetDataList(6, stressXZ);
            if (stressYZ != null)
                DA.SetDataList(7, stressYZ);

            if (strainXX != null)
                DA.SetDataList(8, strainXX);
            if (strainYY != null)
                DA.SetDataList(9, strainYY);
            if (strainZZ != null)
                DA.SetDataList(10, strainZZ);
            if (strainXY != null)
                DA.SetDataList(11, strainXY);
            if (strainXZ != null)
                DA.SetDataList(12, strainXZ);
            if (strainYZ != null)
                DA.SetDataList(13, strainYZ);

            if (dir1 != null)
                DA.SetDataList(14, dir1);
            if (dir2 != null)
                DA.SetDataList(15, dir2);
            if (dir3 != null)
                DA.SetDataList(16, dir3);

            if (stress1 != null)
                DA.SetDataList(17, stress1);
            if (stress2 != null)
                DA.SetDataList(18, stress2);
            if (stress3 != null)
                DA.SetDataList(19, stress3);

            if (curlSum != null)
                DA.SetDataList (20, curlSum);
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
            get { return new Guid("9720FCF9-9282-4F56-B55C-B0F72498B5BE"); }
        }
    }
}