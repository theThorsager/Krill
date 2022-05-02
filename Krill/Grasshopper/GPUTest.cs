using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

using GPUCompute;

namespace Krill.Grasshopper
{
    public class GPUTest : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GPUTest class.
        /// </summary>
        public GPUTest()
          : base("GPUTest", "GPUTest",
              "Description",
              "Krill", "TestComponents")
        {
        }
        public override void AddedToDocument(GH_Document document)
        {
            wrapper = new SilkWrapper();
            string errors = wrapper.Init();

            if (!(errors is null))
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, errors);

            base.AddedToDocument(document);
        }
        public override void RemovedFromDocument(GH_Document document)
        {
            wrapper.Finilize();
            base.RemovedFromDocument(document);
        }
        SilkWrapper wrapper;
        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("n", "n", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("disp", "disp", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int n = 256;
            int xi_n = 3;

            int N = n * n * n;
            int xi_N = (xi_n * 2 + 1);
            xi_N = xi_N * xi_N * xi_N;

            var dsp = new Vector3d[N];
            var mask = new int[N];
            var dens = new Vector3d[N];
            var bload = new Vector3d[N];
            var stiff = new Vector3d[N];
            var velo = new Vector3d[N];
            var forces = new Vector3d[N];

            float[] xi = new float[xi_N * 4];

            var disp = SetValues(dsp, mask);
            var densities = SetValues(dens);
            var bodyload = SetValues(bload);
            var stiffness = SetValues(stiff);
            var vel = SetValues(velo);
            var force = SetValues(forces);

            for (int i = 0; i < xi_N; i++)
            {
                // ...
            }

            wrapper.AssignBuffers(disp, vel, force, densities, bodyload, stiffness, xi, xi_n, n);

            wrapper.EnqueueKernel(n);

            wrapper.ReadBuffers(disp, n);

            GetValues(disp, ref dsp);

            DA.SetData(0, dsp);
        }
        void GetValues(float[] floats, ref Vector3d[] res)
        {
            for (int i = 0; i < res.Length; i++)
            {
                res[i].X = floats[i * 4 + 0];
                res[i].Y = floats[i * 4 + 1];
                res[i].Z = floats[i * 4 + 2];

            }
        }
        float[] SetValues(Vector3d[] vec)
        {
            var arr = new float[vec.Length * 4];
            for (int i = 0; i < vec.Length; i++)
            {
                arr[i * 4 + 0] = (float)vec[i].X;
                arr[i * 4 + 1] = (float)vec[i].Y;
                arr[i * 4 + 2] = (float)vec[i].Z;
            }
            return arr;
        }
        float[] SetValues(Vector3d[] vec, int[] mask)
        {
            double maxvolume = 1.0; // Fix this
            var arr = new float[vec.Length * 4];
            for (int i = 0; i < vec.Length; i++)
            {
                arr[i * 4 + 0] = (float)vec[i].X;
                arr[i * 4 + 1] = (float)vec[i].Y;
                arr[i * 4 + 2] = (float)vec[i].Z;
                arr[i * 4 + 3] = (float)((mask[i] >> 20) / maxvolume); // do volume conversion
            }
            return arr;
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
            get { return new Guid("69C32323-30BC-4C83-9358-A0668CA20569"); }
        }
    }
}