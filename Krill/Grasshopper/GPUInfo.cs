using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;

using Grasshopper.Kernel;
using Rhino.Geometry;

using GPUCompute;
using System.Reflection;

namespace Krill.Grasshopper
{
    public class GPUInfo : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GPUTest class.
        /// </summary>
        public GPUInfo()
          : base("GPUInfo", "GPUInfo",
              "Description",
              "Krill", "TestComponents")
        {
        }
        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("DeviceId", "id", "Which device index to use in the output list devices. -1 does not modify the current setting.", GH_ParamAccess.item, -1);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("devices", "devices", "Avalible devices", GH_ParamAccess.list);
            pManager.AddTextParameter("device", "device", "Selected device", GH_ParamAccess.item);
            pManager.AddTextParameter("log", "log", "OpenCL errors found.", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int device_id = -1;
            DA.GetData(0, ref device_id);

            if (device_id >= 0)
            {
                // write to file
                string path = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "Grasshopper/Libraries/Krill/Device_Index.txt");

                string text = device_id.ToString();
                File.WriteAllText(path, text);
            }

            SilkWrapper wrapper;
            wrapper = new SilkWrapper();
            string errors = wrapper.Init();

            var errorList = new List<string>(wrapper.errorList);
            if (errors != null)
                errorList = errorList.Concat(errors.Split('\n')).ToList();

            if (errorList.Count > 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "OpenCL errors found. See log for details.");

            DA.SetDataList(0, wrapper.QueryDevices());
            DA.SetData(1, wrapper.platformDevice);
            DA.SetDataList(2, errorList);

            wrapper.Finilize();
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