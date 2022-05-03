#pragma OPENCL EXTENSION cl_khr_3d_image_writes : enable

const sampler_t imagesampler = 
        CLK_FILTER_NEAREST | 
        CLK_ADDRESS_CLAMP | 
        CLK_NORMALIZED_COORDS_FALSE;

// Only displacment needs to be an image ?
// Bodyload is assumed to already be normilized
// bondstiffness should include volume
__kernel void update_force(
    read_only image3d_t disp_im, 
    read_only image3d_t bodyload_im, 
    read_only image3d_t stiffness_im, 
    write_only image3d_t force_im,
    read_only image3d_t xi_im,
    const int nbonds, 
    const float bond_constant)
{
	int4 coord = (int4)(get_global_id(0), get_global_id(1), get_global_id(2), 0);

    float4 disp = read_imagef(disp_im, imagesampler, coord);
    // Check if real disp
    if (disp.w > 0.f)
    {
        int i,j,k;
        float3 res = 0;
        for (k = -nbonds; k <= nbonds; ++k)
        {
            for (j = -nbonds; j <= nbonds; ++j)
            {
                for (i = -nbonds; i <= nbonds; ++i)
                {
                    int4 othercoord = (int4)(coord.x + i, coord.y + j, coord.z + k, 0);
                    int4 localcoord = (int4)(nbonds + i, nbonds + j, nbonds + k, 0);
                    float4 othervalue = read_imagef(disp_im, imagesampler, othercoord);
                    float4 xi = read_imagef(xi_im, imagesampler, localcoord);
                    if (xi.w > 0.f)
                    {
                        float use = (float)(othervalue.w > 0.f);
                        // get volume factor
                        float factor = bond_constant * use / (disp.w + othervalue.w);

                    
                        float3 xi_eta = othervalue.xyz - disp.xyz + xi.xyz;
                        float y = length(xi_eta);
                      
                        float s = factor * (y - xi.w) / (xi.w * y);
                        res += s * xi_eta;
                    }
                }
            }
        }

        res += read_imagef(bodyload_im, imagesampler, coord).xyz;
        res += read_imagef(stiffness_im, imagesampler, coord).xyz * disp.xyz;

        float4 realres = (float4)(res, 1);
        write_imagef(force_im, coord, realres);
    }
}

__kernel void compute_dampening()
{

}

__kernel void update_displacement(
    read_only image3d_t dispold_im,
    write_only image3d_t disp_im,
    read_only image3d_t vel_old_im,
    write_only image3d_t vel_im,
    read_only image3d_t force_im,
    read_only image3d_t densities_im,
    const float c
)
{
    int4 coord = (int4)(get_global_id(0), get_global_id(1), get_global_id(2), 0);
    float3 force = read_imagef(force_im, imagesampler, coord).xyz;
    float4 disp = read_imagef(dispold_im, imagesampler, coord);

    if (disp.w > 0.f)
    {
        float3 densitiy = read_imagef(densities_im, imagesampler, coord).xyz;
        float3 vel_old = read_imagef(vel_old_im, imagesampler, coord).xyz;

        force /= densitiy;
        force -= c * vel_old;

        float3 newvel = vel_old + force;
        write_imagef(vel_im, coord, (float4)(newvel, 0));
        write_imagef(disp_im, coord, (float4)(disp.xyz + newvel, disp.w));
    }
}

__kernel void test(write_only image3d_t result)
{
    int4 i = (int4)(get_global_id(0), get_global_id(1), get_global_id(2), 0);

    int val = get_global_size(0);

    float4 write = (float4)((float)(i.x+1), (float)(i.y+1), (float)(i.z+1), (float)(val+1));

    write_imagef(result, i, write);
}