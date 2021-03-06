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

        public string platformDevice = "";

        // Kernels
        nint kernel_forcesA;
        nint kernel_forcesB;
        nint kernel_dispA;
        nint kernel_dispB;

        nint kernel_residuals;
        nint kernel_reduce_resid;

        nint kernel_c_dampA;
        nint kernel_c_dampB;
        nint kernel_reduce_dampA;
        nint kernel_reduce_dampB;
        nint kernel_set_damp;

        nint kernel_test;

        // Buffers
        nint dispA;
        nint dispB;

        nint bodyload;
        nint stiffness;

        nint forceA;
        nint forceB;

        nint xi;

        nint velocityA;
        nint velocityB;
        
        nint densities;

        nint buffer_residual_inner;
        nint buffer_residual;

        nint buffer_dampeningA;
        nint buffer_dampeningB;
        nint buffer_dampeningC;

        // Constants
        const int reduction_factor = 512;
        const int local_x = 8;
        const int local_y = 4;
        const int local_z = 8;
        const int local_linear = local_x * local_y * local_z;

        // 
        bool even = true;

        public List<string> errorList = new List<string>();

        public string Init()
        {
            api = CL.GetApi();
            int err;

            // Platform
            err = api.GetPlatformIDs(0, null, out uint num_platforms);
            if (err < 0)
                return "No Platform";

            int device_index = 0;
            string pathD = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "Grasshopper/Libraries/Krill/Device_Index.txt");
            if (System.IO.File.Exists(pathD))
            {
                string number = System.IO.File.ReadAllText(pathD);
                device_index = int.Parse(number);
            }

            device = 0;
            int device_counter = 0;
            Span<nint> platfroms = new nint[num_platforms];
            err = api.GetPlatformIDs(num_platforms, platfroms, (uint*)null);
            for (int i = 0; i < num_platforms; i++)
            {
                // Device
                uint num_devices = 0;
                err = api.GetDeviceIDs(platfroms[i], CLEnum.DeviceTypeAll, 0, null, &num_devices);
                Span<nint> devices = new nint[num_devices];
                err = api.GetDeviceIDs(platfroms[i], CLEnum.DeviceTypeAll, num_devices, devices, (uint*)null);

                for (int j = 0; j < num_devices; j++)
                {
                    if (ValidDevice(devices[j]))
                    {
                        platform = platfroms[i];
                        device = devices[j];
                        if (device_index == device_counter)
                        {
                            goto endLoop;
                        }

                        device_counter++;
                    }
                }
            }
            if (device == 0)
                return "No device could be found";

            endLoop:
            {
                api.GetPlatformInfo(platform, (uint)CLEnum.PlatformName, 0, null, out nuint stringlength);
                Span<byte> bytearrPlatform = new byte[stringlength];
                api.GetPlatformInfo(platform, (uint)CLEnum.PlatformName, stringlength, bytearrPlatform, (nuint*)null);

                api.GetDeviceInfo(device, (uint)CLEnum.DeviceName, 0, null, out stringlength);
                Span<byte> bytearrDevice = new byte[stringlength];
                api.GetDeviceInfo(device, (uint)CLEnum.DeviceName, stringlength, bytearrDevice, (nuint*)null);

                platformDevice = new string(bytearrPlatform.ToArray().Select(x => (char)x).ToArray());
                platformDevice += ": ";
                platformDevice += new string(bytearrDevice.ToArray().Select(x => (char)x).ToArray());
            }

            // Context
            context = api.CreateContext(null, 1, in device, null, null, out err);
            if (err < 0)
                return "No Context";

            {
                // Read source from file
                string path = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "Grasshopper/Libraries/Krill/peridynamics.cl");
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

                    return "Build Error: \n" + new string(programlog.Select(x => (char)x).ToArray());
                }
            }


            queue = api.CreateCommandQueue(context, device, 0, out err);
            if (err < 0)
                return "No Queue";

            // Force
            string kernel_name = "update_force";
            kernel_forcesA = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_forcesB = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";

            // Displacement
            kernel_name = "update_displacement";
            kernel_dispA = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_dispB = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";

            // Residual
            kernel_name = "compute_residual";
            kernel_residuals = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_name = "reduce";
            kernel_reduce_resid = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";

            // Dampening
            kernel_name = "compute_dampening";
            kernel_c_dampA = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_c_dampB = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_name = "reduce_dampening";
            kernel_reduce_dampA = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_reduce_dampB = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";
            kernel_name = "set_dampining";
            kernel_set_damp = api.CreateKernel(program, kernel_name, out err);
            if (err < 0)
                return $"Could not create kernel: {kernel_name}";

            // Test
            kernel_name = "test";
            kernel_test = api.CreateKernel(program, kernel_name, out err);
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

            even = true;

            // Images
            ReadOnlySpan<uint> image_format = new uint[] { (uint)CLEnum.Rgba, (uint)CLEnum.Float };
            {
                GCHandle dispH = GCHandle.Alloc(disp, GCHandleType.Pinned);
                void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
                dispA = api.CreateImage3D(context, CLEnum.MemReadWrite | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, dispPtr, &err);
                dispB = api.CreateImage3D(context, CLEnum.MemReadWrite | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, dispPtr, &err);
                dispH.Free();
            }
            {
                GCHandle forceH = GCHandle.Alloc(force, GCHandleType.Pinned);
                void* forcePtr = forceH.AddrOfPinnedObject().ToPointer();
                this.forceA = api.CreateImage3D(context, CLEnum.MemReadWrite | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, forcePtr, &err);
                this.forceB = api.CreateImage3D(context, CLEnum.MemReadWrite | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, forcePtr, &err);
                forceH.Free();
            }
            {
                GCHandle xiH = GCHandle.Alloc(xi, GCHandleType.Pinned);
                void* xiPtr = xiH.AddrOfPinnedObject().ToPointer();
                this.xi = api.CreateImage3D(context, CLEnum.MemReadOnly | CLEnum.MemCopyHostPtr, image_format, (nuint)xi_n, (nuint)xi_n, (nuint)xi_n, 0, 0, xiPtr, &err);
                xiH.Free();
            }
            {
                GCHandle velH = GCHandle.Alloc(vel, GCHandleType.Pinned);
                void* velPtr = velH.AddrOfPinnedObject().ToPointer();
                this.velocityA = api.CreateImage3D(context, CLEnum.MemReadWrite | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, velPtr, &err);
                this.velocityB = api.CreateImage3D(context, CLEnum.MemReadWrite | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, velPtr, &err);
                velH.Free();
            }
            {
                GCHandle densH = GCHandle.Alloc(densities, GCHandleType.Pinned);
                void* densPtr = densH.AddrOfPinnedObject().ToPointer();
                this.densities = api.CreateImage3D(context, CLEnum.MemReadOnly | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, densPtr, &err);
                densH.Free();
            }
            {
                GCHandle bodyH = GCHandle.Alloc(bodyload, GCHandleType.Pinned);
                void* bodyPtr = bodyH.AddrOfPinnedObject().ToPointer();
                this.bodyload = api.CreateImage3D(context, CLEnum.MemReadOnly | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, bodyPtr, &err);
                bodyH.Free();
            }
            {
                GCHandle stiffH = GCHandle.Alloc(stiffness, GCHandleType.Pinned);
                void* stiffPtr = stiffH.AddrOfPinnedObject().ToPointer();
                this.stiffness = api.CreateImage3D(context, CLEnum.MemReadOnly | CLEnum.MemCopyHostPtr, image_format, n, n, n, 0, 0, stiffPtr, &err);
                stiffH.Free();
            }

            // Buffers
            // Residuals
            buffer_residual_inner = api.CreateBuffer(context, CLEnum.MemReadWrite, n * n * n * sizeof(float), null, &err);
            int size = (int)Math.Ceiling(n * n * n / (double)(local_linear * reduction_factor));
            buffer_residual = api.CreateBuffer(context, CLEnum.MemWriteOnly, (nuint)(size * sizeof(float)), null, &err);

            // Dampening
            buffer_dampeningA = api.CreateBuffer(context, CLEnum.MemReadWrite, 2 * n * n * n * sizeof(float), null, &err);
            buffer_dampeningB = api.CreateBuffer(context, CLEnum.MemReadWrite, (nuint)(2 * size * sizeof(float)), null, &err);
            buffer_dampeningC = api.CreateBuffer(context, CLEnum.MemReadWrite, sizeof(float), null, &err);
        }

        public void SetKernelArguments(int nbonds, float bond_constant, int n)
        {
            int nb = nbonds;
            // Update Force
            api.SetKernelArg(kernel_forcesA, 0, (nuint)sizeof(nint), dispA);
            api.SetKernelArg(kernel_forcesA, 1, (nuint)sizeof(nint), bodyload);
            api.SetKernelArg(kernel_forcesA, 2, (nuint)sizeof(nint), stiffness);
            api.SetKernelArg(kernel_forcesA, 3, (nuint)sizeof(nint), forceA);
            api.SetKernelArg(kernel_forcesA, 4, (nuint)sizeof(nint), xi);
            api.SetKernelArg(kernel_forcesA, 5, (nuint)sizeof(int), &nb);
            api.SetKernelArg(kernel_forcesA, 6, (nuint)sizeof(float), bond_constant);


            api.SetKernelArg(kernel_forcesB, 0, (nuint)sizeof(nint), dispB);
            api.SetKernelArg(kernel_forcesB, 1, (nuint)sizeof(nint), bodyload);
            api.SetKernelArg(kernel_forcesB, 2, (nuint)sizeof(nint), stiffness);
            api.SetKernelArg(kernel_forcesB, 3, (nuint)sizeof(nint), forceB);
            api.SetKernelArg(kernel_forcesB, 4, (nuint)sizeof(nint), xi);
            api.SetKernelArg(kernel_forcesB, 5, (nuint)sizeof(int), &nb);
            api.SetKernelArg(kernel_forcesB, 6, (nuint)sizeof(float), bond_constant);

            // Update Displacement
            api.SetKernelArg(kernel_dispA, 0, (nuint)sizeof(nint), dispA);
            api.SetKernelArg(kernel_dispA, 1, (nuint)sizeof(nint), dispB);
            api.SetKernelArg(kernel_dispA, 2, (nuint)sizeof(nint), velocityA);
            api.SetKernelArg(kernel_dispA, 3, (nuint)sizeof(nint), velocityB);
            api.SetKernelArg(kernel_dispA, 4, (nuint)sizeof(nint), forceA);
            api.SetKernelArg(kernel_dispA, 5, (nuint)sizeof(nint), densities);
            api.SetKernelArg(kernel_dispA, 6, (nuint)sizeof(nint), buffer_dampeningC);

            api.SetKernelArg(kernel_dispB, 0, (nuint)sizeof(nint), dispB);
            api.SetKernelArg(kernel_dispB, 1, (nuint)sizeof(nint), dispA);
            api.SetKernelArg(kernel_dispB, 2, (nuint)sizeof(nint), velocityB);
            api.SetKernelArg(kernel_dispB, 3, (nuint)sizeof(nint), velocityA);
            api.SetKernelArg(kernel_dispB, 4, (nuint)sizeof(nint), forceB);
            api.SetKernelArg(kernel_dispB, 5, (nuint)sizeof(nint), densities);
            api.SetKernelArg(kernel_dispB, 6, (nuint)sizeof(nint), buffer_dampeningC);

            // Compute Residual
            api.SetKernelArg(kernel_residuals, 0, (nuint)sizeof(nint), forceA);
            api.SetKernelArg(kernel_residuals, 1, (nuint)sizeof(nint), buffer_residual_inner);

            api.SetKernelArg(kernel_reduce_resid, 0, (nuint)sizeof(nint), buffer_residual_inner);
            api.SetKernelArg(kernel_reduce_resid, 1, (nuint)sizeof(nint), buffer_residual);
            api.SetKernelArg(kernel_reduce_resid, 2, (nuint)(local_linear * sizeof(float)), null);
            int size = reduction_factor;
            int num = n * n * n;
            api.SetKernelArg(kernel_reduce_resid, 3, (nuint)sizeof(int), &num);
            api.SetKernelArg(kernel_reduce_resid, 4, (nuint)sizeof(int), &size);

            // Compute Dampening
            api.SetKernelArg(kernel_c_dampA, 0, (nuint)sizeof(nint), forceA);
            api.SetKernelArg(kernel_c_dampA, 1, (nuint)sizeof(nint), forceB);
            api.SetKernelArg(kernel_c_dampA, 2, (nuint)sizeof(nint), velocityA);
            api.SetKernelArg(kernel_c_dampA, 3, (nuint)sizeof(nint), densities);
            api.SetKernelArg(kernel_c_dampA, 4, (nuint)sizeof(nint), dispA);
            api.SetKernelArg(kernel_c_dampA, 5, (nuint)sizeof(nint), buffer_dampeningA);

            api.SetKernelArg(kernel_c_dampB, 0, (nuint)sizeof(nint), forceB);
            api.SetKernelArg(kernel_c_dampB, 1, (nuint)sizeof(nint), forceA);
            api.SetKernelArg(kernel_c_dampB, 2, (nuint)sizeof(nint), velocityB);
            api.SetKernelArg(kernel_c_dampB, 3, (nuint)sizeof(nint), densities);
            api.SetKernelArg(kernel_c_dampB, 4, (nuint)sizeof(nint), dispB);
            api.SetKernelArg(kernel_c_dampB, 5, (nuint)sizeof(nint), buffer_dampeningA);

            api.SetKernelArg(kernel_reduce_dampA, 0, (nuint)sizeof(nint), buffer_dampeningA);
            api.SetKernelArg(kernel_reduce_dampA, 1, (nuint)sizeof(nint), buffer_dampeningB);
            api.SetKernelArg(kernel_reduce_dampA, 2, (nuint)(2 * local_linear * sizeof(float)), null);
            api.SetKernelArg(kernel_reduce_dampA, 3, (nuint)sizeof(int), &num);
            api.SetKernelArg(kernel_reduce_dampA, 4, (nuint)sizeof(int), &size);

            api.SetKernelArg(kernel_reduce_dampB, 0, (nuint)sizeof(nint), buffer_dampeningB);
            api.SetKernelArg(kernel_reduce_dampB, 1, (nuint)sizeof(nint), buffer_dampeningA);
            api.SetKernelArg(kernel_reduce_dampB, 2, (nuint)(2 * local_linear * sizeof(float)), null);
            int reducednum = (int)Math.Ceiling(num / (double)(size * local_linear));
            api.SetKernelArg(kernel_reduce_dampB, 3, (nuint)sizeof(int), &reducednum);
            api.SetKernelArg(kernel_reduce_dampB, 4, (nuint)sizeof(int), &size);

            api.SetKernelArg(kernel_set_damp, 0, (nuint)sizeof(nint), buffer_dampeningA);
            api.SetKernelArg(kernel_set_damp, 1, (nuint)sizeof(nint), buffer_dampeningC);
        }

        public void EnqueueKernel(int n)
        {
            ReadOnlySpan<nuint> global_size = new nuint[] { (nuint)n, (nuint)n, (nuint)n };
            ReadOnlySpan<nuint> local_size = new nuint[] { (nuint)local_x, (nuint)local_y, (nuint)local_z };
            ReadOnlySpan<nuint> offset = new nuint[] { 0, 0, 0 };

            int err = api.EnqueueNdrangeKernel(queue, even ? kernel_forcesA : kernel_forcesB, 3, offset, global_size, local_size, 0, (nint*)null, (nint*)null);
            UpdateDampening(n);
            err = api.EnqueueNdrangeKernel(queue, even ? kernel_dispA : kernel_dispB, 3, offset, global_size, local_size, 0, (nint*)null, (nint*)null);
            api.Finish(queue);
            even = !even;
        }

        void UpdateDampening(int n)
        {
            ReadOnlySpan<nuint> global_size = new nuint[] { (nuint)n, (nuint)n, (nuint)n };
            ReadOnlySpan<nuint> local_size = new nuint[] { (nuint)local_x, (nuint)local_y, (nuint)local_z };
            ReadOnlySpan<nuint> offset = new nuint[] { 0, 0, 0 };

            // compute dampening
            int err = api.EnqueueNdrangeKernel(queue, even ? kernel_c_dampA : kernel_c_dampB, 3, offset, global_size, local_size, 0, (nint*)null, (nint*)null);

            // reduce loop
            int n_output = n * n * n;
            // n_output >= (512 * 256 * 512 * 256))
            // would give problems, but it would also already have overflowed

            int g_size;
            int l_size = local_linear;
            
            n_output = (int)Math.Ceiling(n_output / (double)(l_size * reduction_factor));
            g_size = n_output * l_size;

            err = api.EnqueueNdrangeKernel(queue, kernel_reduce_dampA, 1, 0, (nuint)g_size, (nuint)l_size, 0, (nint*)null, (nint*)null);

            n_output = (int)Math.Ceiling(n_output / (double)(l_size * reduction_factor));
            g_size = n_output * l_size;

            err = api.EnqueueNdrangeKernel(queue, kernel_reduce_dampB, 1, 0, (nuint)g_size, (nuint)l_size, 0, (nint*)null, (nint*)null);

            // set_dampening
            err = api.EnqueueNdrangeKernel(queue, kernel_set_damp, 1, 0, 1, 1, 0, (nint*)null, (nint*)null);
        }

        public double CheckResidual(int n, double F, int n_particles)
        {
            int threads = (n * n * n) / reduction_factor;
            // compute_residual
            ReadOnlySpan<nuint> global_size = new nuint[] { (nuint)n, (nuint)n, (nuint)n };
            ReadOnlySpan<nuint> local_size = new nuint[] { (nuint)local_x, (nuint)local_y, (nuint)local_z };
            ReadOnlySpan<nuint> offset = new nuint[] { 0, 0, 0 };

            int err = api.EnqueueNdrangeKernel(queue, kernel_residuals, 3, offset, global_size, local_size, 0, (nint*)null, (nint*)null);
            // reduce
            threads = (int)Math.Ceiling(threads / (double)local_linear) * local_linear;
            err = api.EnqueueNdrangeKernel(queue, kernel_reduce_resid, 1, 0, (nuint)(threads), (nuint)local_linear, 0, (nint*)null, (nint*)null);

            // read_buffer
            int size = (int)Math.Ceiling(n * n * n / (double)(local_linear * reduction_factor));
            var results = new float[size];
            GCHandle handle = GCHandle.Alloc(results, GCHandleType.Pinned);
            void* ptr = handle.AddrOfPinnedObject().ToPointer();
            api.EnqueueReadBuffer(queue, buffer_residual, true, 0, (nuint)(size * sizeof(float)), ptr, 0, (nint*)null, (nint*)null);
            handle.Free();

            return results.Sum() / (F * n_particles);
        }

        public void ReadBuffers(float[] disp, float[] vel, float[] force, int n)
        {
            ReadOnlySpan<nuint> origin = new nuint[] { 0, 0, 0 };
            ReadOnlySpan<nuint> region = new nuint[] { (nuint)n, (nuint)n, (nuint)n };
            {
                GCHandle dispH = GCHandle.Alloc(disp, GCHandleType.Pinned);
                void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
                api.EnqueueReadImage(queue, even ? dispA : dispB, true, origin, region, 0, 0, dispPtr, 0, (nint*)null, (nint*)null);
                dispH.Free();
            }
            {
                GCHandle dispH = GCHandle.Alloc(vel, GCHandleType.Pinned);
                void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
                api.EnqueueReadImage(queue, even ? velocityA : velocityB, true, origin, region, 0, 0, dispPtr, 0, (nint*)null, (nint*)null);
                dispH.Free();
            }
            {
                GCHandle dispH = GCHandle.Alloc(force, GCHandleType.Pinned);
                void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
                api.EnqueueReadImage(queue, even ? this.forceA : this.forceB, true, origin, region, 0, 0, dispPtr, 0, (nint*)null, (nint*)null);
                dispH.Free();
            }
        }

        public void ReadDisp(float[] disp, int n)
        {
            ReadOnlySpan<nuint> origin = new nuint[] { 0, 0, 0 };
            ReadOnlySpan<nuint> region = new nuint[] { (nuint)n, (nuint)n, (nuint)n };
            {
                GCHandle dispH = GCHandle.Alloc(disp, GCHandleType.Pinned);
                void* dispPtr = dispH.AddrOfPinnedObject().ToPointer();
                api.EnqueueReadImage(queue, even ? dispA : dispB, true, origin, region, 0, 0, dispPtr, 0, (nint*)null, (nint*)null);
                dispH.Free();
            }
        }

        public void ReleaseBuffers()
        {
            // Release Buffers
            api.ReleaseMemObject(dispA);
            api.ReleaseMemObject(dispB);

            api.ReleaseMemObject(forceA);
            api.ReleaseMemObject(forceB);

            api.ReleaseMemObject(xi);
            api.ReleaseMemObject(velocityA);
            api.ReleaseMemObject(velocityB);

            api.ReleaseMemObject(densities);
            api.ReleaseMemObject(bodyload);
            api.ReleaseMemObject(stiffness);

            api.ReleaseMemObject(buffer_residual);
            api.ReleaseMemObject(buffer_residual_inner);

            api.ReleaseMemObject(buffer_dampeningA);
            api.ReleaseMemObject(buffer_dampeningB);
            api.ReleaseMemObject(buffer_dampeningC);
        }

        public void Finilize()
        {
            api.ReleaseKernel(kernel_forcesA);
            api.ReleaseKernel(kernel_forcesB);
            api.ReleaseKernel(kernel_dispA);
            api.ReleaseKernel(kernel_dispB);

            api.ReleaseKernel(kernel_residuals);
            api.ReleaseKernel(kernel_reduce_resid);

            api.ReleaseKernel(kernel_c_dampA);
            api.ReleaseKernel(kernel_c_dampB);
            api.ReleaseKernel(kernel_reduce_dampA);
            api.ReleaseKernel(kernel_reduce_dampB);
            api.ReleaseKernel(kernel_set_damp);

            api.ReleaseCommandQueue(queue);
            api.ReleaseProgram(program);
            api.ReleaseContext(context);

            api.Dispose();
        }

        public float TestKernel()
        {
            nuint size = 4;
            ReadOnlySpan<uint> image_format = new uint[] { (uint)CLEnum.Rgba, (uint)CLEnum.Float };

            int err;

            nuint rowpitch = 0; // 4 * sizeof(float) * size;
            nuint slicepitch = 0; // rowpitch * size;
            nint image = api.CreateImage3D(context, CLEnum.MemReadWrite, image_format, size, size, size, rowpitch, slicepitch, null, &err);


            api.SetKernelArg(kernel_test, 0, (nuint)sizeof(nint), image);

            ReadOnlySpan<nuint> global_size = new nuint[] { (nuint)size, (nuint)size, (nuint)size };
            ReadOnlySpan<nuint> local_size = new nuint[] { (nuint)size, (nuint)size, (nuint)size };
            ReadOnlySpan<nuint> offset = new nuint[] { 0, 0, 0 };

            api.EnqueueNdrangeKernel(queue, kernel_test, 3, offset, global_size, local_size, 0, (nint*)null, (nint*)null);


            float[] values = new float[size * size * size * 4 * 4];
            GCHandle handle = GCHandle.Alloc(values, GCHandleType.Pinned);
            void* ptr = handle.AddrOfPinnedObject().ToPointer();

            float* temp = (float*)ptr;
            temp += (int)(size * size * size * 4);
            api.Finish(queue);

            api.EnqueueReadImage(queue, image, true, offset, global_size, rowpitch, slicepitch, temp, 0, (nint*)null, (nint*)null);

            handle.Free();

            return values.Sum();
        }

        public float TestReduction()
        {
            int n = 8 * 32;
            int err;
            int num = n * n * n;
            int size = reduction_factor;

            int threads = num / reduction_factor;

            // create buffers
            buffer_dampeningA = api.CreateBuffer(context, CLEnum.MemReadWrite, (nuint)(2 * n * n * n * sizeof(float)), null, &err);
            buffer_dampeningB = api.CreateBuffer(context, CLEnum.MemReadWrite, (nuint)(2 * size * sizeof(float)), null, &err);
            buffer_dampeningC = api.CreateBuffer(context, CLEnum.MemReadWrite, sizeof(float), null, &err);

            // Set arguments
            api.SetKernelArg(kernel_reduce_dampA, 0, (nuint)sizeof(nint), buffer_dampeningA);
            api.SetKernelArg(kernel_reduce_dampA, 1, (nuint)sizeof(nint), buffer_dampeningB);
            api.SetKernelArg(kernel_reduce_dampA, 2, (nuint)(2 * local_linear * sizeof(float)), null);
            api.SetKernelArg(kernel_reduce_dampA, 3, (nuint)sizeof(int), &num);
            api.SetKernelArg(kernel_reduce_dampA, 4, (nuint)sizeof(int), &size);

            api.SetKernelArg(kernel_reduce_dampB, 0, (nuint)sizeof(nint), buffer_dampeningB);
            api.SetKernelArg(kernel_reduce_dampB, 1, (nuint)sizeof(nint), buffer_dampeningA);
            api.SetKernelArg(kernel_reduce_dampB, 2, (nuint)(2 * local_linear * sizeof(float)), null);
            int reducednum = (int)Math.Ceiling(num / (double)(size * local_linear));
            api.SetKernelArg(kernel_reduce_dampB, 3, (nuint)sizeof(int), &reducednum);
            api.SetKernelArg(kernel_reduce_dampB, 4, (nuint)sizeof(int), &size);

            // write to Buffer
            // ...
            float[] valuesA = new float[num];
            valuesA = valuesA.SelectMany(x => new float[] { 1.0f, -1.0f }).ToArray();
            GCHandle handle = GCHandle.Alloc(valuesA, GCHandleType.Pinned);
            void* ptr = handle.AddrOfPinnedObject().ToPointer();
            api.EnqueueWriteBuffer(queue, buffer_dampeningA, true, 0, (nuint)(2 * num * sizeof(float)), ptr, 0, (nint*)null, (nint*)null);
            handle.Free();
            api.Finish(queue);

            // Do the thing
            // reduce loop
            int n_output = n * n * n;
            // n_output >= (512 * 256 * 512 * 256))
            // would give problems, but it would also already have overflowed

            int g_size;
            int l_size = local_linear;

            n_output = (int)Math.Ceiling(n_output / (double)(l_size * reduction_factor));
            g_size = n_output * l_size;

            err = api.EnqueueNdrangeKernel(queue, kernel_reduce_dampA, 1, 0, (nuint)g_size, (nuint)l_size, 0, (nint*)null, (nint*)null);
            api.Finish(queue);

            n_output = (int)Math.Ceiling(n_output / (double)(l_size * reduction_factor));
            g_size = n_output * l_size;

            err = api.EnqueueNdrangeKernel(queue, kernel_reduce_dampB, 1, 0, (nuint)g_size, (nuint)l_size, 0, (nint*)null, (nint*)null);
            api.Finish(queue);



            // read from Buffer
            float[] values = new float[2];
            handle = GCHandle.Alloc(values, GCHandleType.Pinned);
            ptr = handle.AddrOfPinnedObject().ToPointer();
            api.EnqueueReadBuffer(queue, buffer_dampeningA, true, 0, (nuint)(2 * sizeof(float)), ptr, 0, (nint*)null, (nint*)null);
            handle.Free();

            api.ReleaseMemObject(buffer_dampeningA);
            api.ReleaseMemObject(buffer_dampeningB);
            api.ReleaseMemObject(buffer_dampeningC);

            return values.Sum();
        }

        public List<string> QueryDevices()
        {
            var results = new List<string>();

            int err;
            // Platform
            err = api.GetPlatformIDs(0, null, out uint num_platforms);
            if (CheckError(err))
                return results;

            Span<nint> platfroms = new nint[num_platforms];
            err = api.GetPlatformIDs(num_platforms, platfroms, (uint*)null);
            for (int i = 0; i < num_platforms; i++)
            {
                api.GetPlatformInfo(platfroms[i], (uint)CLEnum.PlatformName, 0, null, out nuint res_size);
                var platform_info = new byte[res_size]; // note that C# char is 2 bytes vs 1 byte as assumed in OpenCL;
                GCHandle phandle = GCHandle.Alloc(platform_info, GCHandleType.Pinned);
                void* ptr = phandle.AddrOfPinnedObject().ToPointer();

                api.GetPlatformInfo(platfroms[i], (uint)CLEnum.PlatformName, res_size, ptr, null);
                phandle.Free();

                string pres = new string(platform_info.Select(x => (char)x).ToArray());
                // Device
                uint num_devices = 0;
                err = api.GetDeviceIDs(platfroms[i], CLEnum.DeviceTypeAll, 0, null, &num_devices);
                if (CheckError(err))
                    continue;

                Span<nint> devices = new nint[num_devices];
                err = api.GetDeviceIDs(platfroms[i], CLEnum.DeviceTypeAll, num_devices, devices, (uint*)null);

                for (int j = 0; j < num_devices; j++)
                {
                    if (ValidDevice(devices[j]))
                    {
                        api.GetDeviceInfo(devices[j], (uint)CLEnum.DeviceName, 0, null, out res_size);
                        var device_info = new byte[res_size]; // note that C# char is 2 bytes vs 1 byte as assumed in OpenCL;
                        GCHandle dhandle = GCHandle.Alloc(device_info, GCHandleType.Pinned);
                        ptr = dhandle.AddrOfPinnedObject().ToPointer();

                        api.GetDeviceInfo(devices[j], (uint)CLEnum.DeviceName, res_size, ptr, null);
                        dhandle.Free();

                        string dres = new string(device_info.Select(x => (char)x).ToArray());
                        results.Add(pres + ": " + dres);
                    }
                }
            }

            return results;
        }

        bool CheckError(int err)
        {
            if (err >= 0)
                return false;

            errorList.Add(EnumCodes.Query(err));
            return true;
        }

        bool ValidDevice(nint device)
        {
            nuint res_size;
            GCHandle handle;
            // Write Image 3d
            // CL_DEVICE_EXTENSIONS
            api.GetDeviceInfo(device, (uint)CLEnum.DeviceExtensions, 0, null, &res_size);
            var char_arr = new byte[res_size]; // note that C# char is 2 bytes vs 1 byte as assumed in OpenCL;
            handle = GCHandle.Alloc(char_arr, GCHandleType.Pinned);
            var ptr = handle.AddrOfPinnedObject().ToPointer();
            api.GetDeviceInfo(device, (uint)CLEnum.DeviceExtensions, res_size, ptr, null);
            handle.Free();

            string extensions = new string(char_arr.Select(x => (char)x).ToArray());
            bool test = extensions.Split(' ').Contains("cl_khr_3d_image_writes");
            if (!test)
                return false;

            // CL_DEVICE_IMAGE_SUPPORT
            api.GetDeviceInfo(device, (uint)CLEnum.DeviceImageSupport, 0, null, &res_size);
            Span<byte> result = new byte[res_size];
            api.GetDeviceInfo(device, (uint)CLEnum.DeviceImageSupport, res_size, result, (nuint*)null);
            if (result[0] == 0)
                return false;

            // CL_DRIVER_VERSION
            api.GetDeviceInfo(device, (uint)CLEnum.DeviceVersion, 0, null, &res_size);
            char_arr = new byte[res_size]; // note that C# char is 2 bytes vs 1 byte as assumed in OpenCL;
            handle = GCHandle.Alloc(char_arr, GCHandleType.Pinned);
            ptr = handle.AddrOfPinnedObject().ToPointer();
            api.GetDeviceInfo(device, (uint)CLEnum.DeviceVersion, res_size, ptr, null);
            handle.Free();

            string driver_version = new string(char_arr.Select(x => (char)x).ToArray());
            var version = driver_version.Split(' ')[1].Split('.');
            bool valid = (int.Parse(version[0]) >= 1 && int.Parse(version[1]) >= 2) ||
                         (int.Parse(version[0]) > 1);
            if (!valid) 
                return false;

            return true;
        }
    }
}