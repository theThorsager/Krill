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
	int4 coord = (get_global_id(0), get_global_id(1), get_global_id(2), 1);

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
                    int4 othercoord = (coord.x + i, coord.y + j, coord.z + k, 1);
                    int4 localcoord = (nbonds + i, nbonds + j, nbonds + k, 1);
                    float4 othervalue = read_imagef(disp_im, imagesampler, othercoord);
                    float4 xi = read_imagef(xi_im, imagesampler, localcoord);
                    float use = (float)(xi.w > 0 && othervalue.w > 0);
                    // get volume factor
                    float factor = bond_constant * use / (disp.w + othervalue.w);

                    
                    float3 xi_eta = othervalue.xyz - disp.xyz + xi.xyz;
                    float y = length(xi_eta);
                      
                    float s = factor * (y - xi.w) / (xi.w * y);
                    res += s * xi_eta;
                }
            }
        }

        res += read_imagef(bodyload_im, imagesampler, coord).xyz;
        res += read_imagef(stiffness_im, imagesampler, coord).xyz * disp.xyz;

        write_imagef(force_im, coord, (res, 1));
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
    int4 coord = (get_global_id(0), get_global_id(1), get_global_id(2), 1);
    float3 force = read_imagef(force_im, imagesampler, coord).xyz;
    float4 disp = read_imagef(dispold_im, imagesampler, coord);

    if (disp.w > 0.f)
    {
        float3 densitiy = read_imagef(densities_im, imagesampler, coord).xyz;
        float3 vel_old = read_imagef(vel_old_im, imagesampler, coord).xyz;

        force /= densitiy;
        force -= c * vel_old;

        float3 newvel = vel_old + force;
        write_imagef(vel_im, coord, (newvel, 0));
        write_imagef(disp_im, coord, (disp.xyz + newvel, disp.w));
    }
}