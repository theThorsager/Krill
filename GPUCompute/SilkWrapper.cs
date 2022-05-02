using System;
using System.Collections.Generic;

using Silk;
using Silk.NET;
using Silk.NET.OpenCL;

using System.Linq;
using System.Runtime.InteropServices;


namespace GPUCompute
{
    public unsafe class SilkWrapper
    {
        CL api;

        nint platform;
        nint device;
        nint context;
        nint program;
        nint queue;

        // Kernels
        nint kernel_forcesA;
        nint kernel_forcesB;
        nint kernel_dispA;
        nint kernel_dispB;

        // Buffers
        nint dispA;
        nint dispB;

        nint bodyload;
        nint stiffness;

        nint force;

        nint xi;

        nint velocityA;
        nint velocityB;
        
        nint densities;

        // 
        bool even = true;

        public string Init()
        {
            api = CL.GetApi();

            int err;


            // Platform
            err = api.GetPlatformIDs(1, out platform, null);
            if (err < 0)
                return "No Platform";

            // Device
            err = api.GetDeviceIDs(platform, CLEnum.DeviceTypeGpu, 1, out device, null);
            if (err == (nint)CLEnum.DeviceNotFound)
            {
                err = api.GetDeviceIDs(platform, CLEnum.DeviceTypeCpu, 1, out device, null);
            }
            if (err < 0)
                return "No device could be found";

            // Context
            context = api.CreateContext(null, 1, in device, null, null, out err);
            if (err < 0)
                return "No Context";


            {
                // Read source from file
                string path = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "Grasshopper/Libraries/peridynamics.cl");
                string source = System.IO.File.ReadAllText(path);

                string[] sources = new string[] { source };

                nuint sourceLength = (nuint)source.Length;
                program = api.CreateProgramWithSource(context, 1, sources, in sourceLength, out err);
                if (err < 0)
                    return "No Program Creation";

                err = api.BuildProgram(program, 0, null, (byte*)null, null, null);
                if (err < 0)
                {
                    api.GetProgramBuildInfo(program, device, (uint)CLEnum.ProgramBuildLog, 0, null, out nuint logsize);

                    var programlog = new byte[logsize]; // note that C# char is 2 bytes vs 1 byte as assumed in OpenCL;
                    GCHandle handle = GCHandle.Alloc(programlog, GCHandleType.Pinned);
                    void* ptr = handle.AddrOfPinnedObject().ToPointer();

                    api.GetProgramBuildInfo(program, device, (uint)CLEnum.ProgramBuildLog, logsize, ptr, null);
                    handle.Free();

                    return "Build Error:" + new string(programlog.Select(x => (char)x).ToArray());
                }
            }


            queue = api.CreateCommandQueue(context, device, 0, out err);
            if (err < 0)
                return "No Queue";

            string kernel_name = "update_force";
            kernel_forcesA = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_forcesB = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";

            kernel_name = "update_displacement";
            kernel_dispA = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_dispB = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";

            return null;
        }

        public void AssignBuffers(float[] disp, float[] vel, float[] force, 
            float[] densities, float[] bodyload, float[] stiffness, 
            float[] xi, int xi_n, int n_i)
        {
            nuint n = (nuint)n_i;
            int err;

            ReadOnlySpan<uint> image_format = new uint[] { (uint)CLEnum.Rgba, (uint)CLEnum.Float };

            {
                GCHandle dispH = GCHandle.Alloc(disp, GCHandleType.Pinned);
                void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
                dispA = api.CreateImage3D(context, CLEnum.MemReadWrite, image_format, n, n, n, 0, 0, dispPtr, &err);
                dispB = api.CreateImage3D(context, CLEnum.MemReadWrite, image_format, n, n, n, 0, 0, dispPtr, &err);
                dispH.Free();
            }
            {
                GCHandle forceH = GCHandle.Alloc(force, GCHandleType.Pinned);
                void* forcePtr = forceH.AddrOfPinnedObject().ToPointer();
                this.force = api.CreateImage3D(context, CLEnum.MemReadWrite, image_format, n, n, n, 0, 0, forcePtr, &err);
                forceH.Free();
            }
            {
                GCHandle xiH = GCHandle.Alloc(xi, GCHandleType.Pinned);
                void* xiPtr = xiH.AddrOfPinnedObject().ToPointer();
                this.xi = api.CreateImage3D(context, CLEnum.MemReadOnly, image_format, (nuint)xi_n, (nuint)xi_n, (nuint)xi_n, 0, 0, xiPtr, &err);
                xiH.Free();
            }
            {
                GCHandle velH = GCHandle.Alloc(vel, GCHandleType.Pinned);
                void* velPtr = velH.AddrOfPinnedObject().ToPointer();
                this.velocityA = api.CreateImage3D(context, CLEnum.MemReadWrite, image_format, n, n, n, 0, 0, velPtr, &err);
                this.velocityB = api.CreateImage3D(context, CLEnum.MemReadWrite, image_format, n, n, n, 0, 0, velPtr, &err);
                velH.Free();
            }
            {
                GCHandle densH = GCHandle.Alloc(densities, GCHandleType.Pinned);
                void* densPtr = densH.AddrOfPinnedObject().ToPointer();
                this.densities = api.CreateImage3D(context, CLEnum.MemReadOnly, image_format, n, n, n, 0, 0, densPtr, &err);
                densH.Free();
            }
            {
                GCHandle bodyH = GCHandle.Alloc(bodyload, GCHandleType.Pinned);
                void* bodyPtr = bodyH.AddrOfPinnedObject().ToPointer();
                this.bodyload = api.CreateImage3D(context, CLEnum.MemReadOnly, image_format, n, n, n, 0, 0, bodyPtr, &err);
                bodyH.Free();
            }
            {
                GCHandle stiffH = GCHandle.Alloc(stiffness, GCHandleType.Pinned);
                void* stiffPtr = stiffH.AddrOfPinnedObject().ToPointer();
                this.stiffness = api.CreateImage3D(context, CLEnum.MemReadOnly, image_format, n, n, n, 0, 0, stiffPtr, &err);
                stiffH.Free();
            }
        }

        public void AssignBuffers2d(float[] disp, float[] vel, float[] force,
            float[] stiffness,
            float[] xi, int xi_n, int n_i)
        {
            nuint n = (nuint)n_i;
            int err;

            ReadOnlySpan<uint> image_format = new uint[] { (uint)CLEnum.Rgba, (uint)CLEnum.Float };

            {
                GCHandle dispH = GCHandle.Alloc(disp, GCHandleType.Pinned);
                void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
                dispA = api.CreateImage2D(context, CLEnum.MemReadWrite, image_format, n, n, 0, dispPtr, &err);
                dispB = api.CreateImage2D(context, CLEnum.MemReadWrite, image_format, n, n, 0, dispPtr, &err);
                dispH.Free();
            }
            {
                GCHandle forceH = GCHandle.Alloc(force, GCHandleType.Pinned);
                void* forcePtr = forceH.AddrOfPinnedObject().ToPointer();
                this.force = api.CreateImage2D(context, CLEnum.MemReadWrite, image_format, n, n, 0, forcePtr, &err);
                forceH.Free();
            }
            {
                GCHandle xiH = GCHandle.Alloc(xi, GCHandleType.Pinned);
                void* xiPtr = xiH.AddrOfPinnedObject().ToPointer();
                this.xi = api.CreateImage2D(context, CLEnum.MemReadOnly, image_format, (nuint)xi_n, (nuint)xi_n, 0, xiPtr, &err);
                xiH.Free();
            }
            {
                GCHandle velH = GCHandle.Alloc(vel, GCHandleType.Pinned);
                void* velPtr = velH.AddrOfPinnedObject().ToPointer();
                this.velocityA = api.CreateImage2D(context, CLEnum.MemReadWrite, image_format, n, n, 0, velPtr, &err);
                this.velocityB = api.CreateImage2D(context, CLEnum.MemReadWrite, image_format, n, n, 0, velPtr, &err);
                velH.Free();
            }
            {
                GCHandle stiffH = GCHandle.Alloc(stiffness, GCHandleType.Pinned);
                void* stiffPtr = stiffH.AddrOfPinnedObject().ToPointer();
                this.stiffness = api.CreateImage2D(context, CLEnum.MemReadOnly, image_format, n, n, 0, stiffPtr, &err);
                stiffH.Free();
            }
        }

        public void SetKernelArguments(int nbonds, float bond_constant)
        {
            api.SetKernelArg(kernel_forcesA, 0, (nuint)sizeof(nint), dispA);
            api.SetKernelArg(kernel_forcesA, 1, (nuint)sizeof(nint), bodyload);
            api.SetKernelArg(kernel_forcesA, 2, (nuint)sizeof(nint), stiffness);
            api.SetKernelArg(kernel_forcesA, 3, (nuint)sizeof(nint), force);
            api.SetKernelArg(kernel_forcesA, 4, (nuint)sizeof(nint), xi);
            api.SetKernelArg(kernel_forcesA, 5, (nuint)sizeof(nint), nbonds);
            api.SetKernelArg(kernel_forcesA, 6, (nuint)sizeof(float), bond_constant);

            api.SetKernelArg(kernel_forcesB, 0, (nuint)sizeof(nint), dispB);
            api.SetKernelArg(kernel_forcesB, 1, (nuint)sizeof(nint), bodyload);
            api.SetKernelArg(kernel_forcesB, 2, (nuint)sizeof(nint), stiffness);
            api.SetKernelArg(kernel_forcesB, 3, (nuint)sizeof(nint), force);
            api.SetKernelArg(kernel_forcesB, 4, (nuint)sizeof(nint), xi);
            api.SetKernelArg(kernel_forcesB, 5, (nuint)sizeof(nint), nbonds);
            api.SetKernelArg(kernel_forcesB, 6, (nuint)sizeof(float), bond_constant);

            api.SetKernelArg(kernel_dispA, 0, (nuint)sizeof(nint), dispA);
            api.SetKernelArg(kernel_dispA, 1, (nuint)sizeof(nint), dispB);
            api.SetKernelArg(kernel_dispA, 2, (nuint)sizeof(nint), velocityA);
            api.SetKernelArg(kernel_dispA, 3, (nuint)sizeof(nint), velocityB);
            api.SetKernelArg(kernel_dispA, 4, (nuint)sizeof(nint), force);
            api.SetKernelArg(kernel_dispA, 5, (nuint)sizeof(nint), densities);
            api.SetKernelArg(kernel_dispA, 6, (nuint)sizeof(float), 0.0f);

            api.SetKernelArg(kernel_dispB, 0, (nuint)sizeof(nint), dispB);
            api.SetKernelArg(kernel_dispB, 1, (nuint)sizeof(nint), dispA);
            api.SetKernelArg(kernel_dispB, 2, (nuint)sizeof(nint), velocityB);
            api.SetKernelArg(kernel_dispB, 3, (nuint)sizeof(nint), velocityA);
            api.SetKernelArg(kernel_dispB, 4, (nuint)sizeof(nint), force);
            api.SetKernelArg(kernel_dispB, 5, (nuint)sizeof(nint), densities);
            api.SetKernelArg(kernel_dispB, 6, (nuint)sizeof(float), 0.0f);
        }

        public void EnqueueKernel(int n)
        {
            int lsize = 256;
            ReadOnlySpan<nuint> global_size = new nuint[] { (nuint)n, (nuint)n, (nuint)n };
            ReadOnlySpan<nuint> local_size = new nuint[] { (nuint)lsize, (nuint)lsize, (nuint)lsize };

            //ReadOnlySpan<nuint> global_size = new nuint[] { (nuint)n, (nuint)n };
            //ReadOnlySpan<nuint> local_size = new nuint[] { (nuint)lsize, (nuint)lsize };

            int err = api.EnqueueNdrangeKernel(queue, even ? kernel_forcesA : kernel_forcesB, 3, (nuint*)null, global_size, local_size, 0, (nint*)null, (nint*)null);
            // Find dampening ...
            err = api.EnqueueNdrangeKernel(queue, even ? kernel_dispA : kernel_dispB, 3, (nuint*)null, global_size, local_size, 0, (nint*)null, (nint*)null);
            even = !even;
        }

        public void CheckResidual(float tol)
        {
            //...
        }

        public void ReadBuffers(float[] disp, int n)
        {
            ReadOnlySpan<nuint> origin = new nuint[] { 0, 0, 0 };
            ReadOnlySpan<nuint> region = new nuint[] { (nuint)n, (nuint)n, (nuint)n };

            GCHandle dispH = GCHandle.Alloc(disp, GCHandleType.Pinned);
            void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
            api.EnqueueReadImage(queue, even ? dispA : dispB, true, origin, region, 0, 0, dispPtr, 0, (nint*)null, (nint*)null);
            dispH.Free();
        }

        public void Finilize()
        {
            api.ReleaseKernel(kernel_forcesA);
            api.ReleaseKernel(kernel_forcesB);
            api.ReleaseKernel(kernel_dispA);
            api.ReleaseKernel(kernel_dispB);

            // Release Buffers
            

            api.ReleaseCommandQueue(queue);
            api.ReleaseProgram(program);
            api.ReleaseContext(context);
        }

    }
}