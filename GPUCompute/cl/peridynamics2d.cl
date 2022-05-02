const sampler_t imagesampler = 
        CLK_FILTER_NEAREST | 
        CLK_ADDRESS_CLAMP | 
        CLK_NORMALIZED_COORDS_FALSE;

// Only displacment needs to be an image ?
// Bodyload is assumed to already be normilized
// bondstiffness should include volume
__kernel void update_force(
    read_only image2d_t disp_im, 
    read_only image2d_t bcs_im, 
    write_only image2d_t force_im,
    read_only image2d_t xi_im,
    const int nbonds, 
    const float bond_constant)
{
	int2 coord = (get_global_id(0), get_global_id(1));

    float4 disp = read_imagef(disp_im, imagesampler, coord);
    // Check if real disp
    if (disp.w > 0.f)
    {
        int i,j;
        float2 res = 0;
        for (j = -nbonds; j <= nbonds; ++j)
        {
            for (i = -nbonds; i <= nbonds; ++i)
            {
                int2 othercoord = (coord.x + i, coord.y + j);
                int2 localcoord = (nbonds + i, nbonds + j);
                float4 othervalue = read_imagef(disp_im, imagesampler, othercoord);
                float4 xi = read_imagef(xi_im, imagesampler, localcoord);
                float use = (float)(xi.w > 0 && othervalue.w > 0);
                // get volume factor
                float factor = bond_constant * use / (disp.w + othervalue.w);
             
                float2 xi_eta = othervalue.xy - disp.xy + xi.xy;
                float y = length(xi_eta);
                  
                float s = factor * (y - xi.w) / (xi.w * y);
                res += s * xi_eta;
            }
        }

        float4 bcs = read_imagef(bcs_im, imagesampler, coord);
        res += bcs.xy;
        res += bcs.zw * disp.xy;

        write_imagef(force_im, coord, (res, 1, 1));
    }
}

__kernel void compute_dampening(
    read_only image2d_t forceold_im,
    read_only image2d_t force_im,
    read_only image2d_t vel_im,
    read_only image2d_t disp_im,
    local float* interres,
    global float* workgroupres)
{
	int2 coord = (get_global_id(0), get_global_id(1));

    float2 fold = read_imagef(forceold_im, imagesampler, coord).xy;
    float2 f = read_imagef(force_im, imagesampler, coord).xy;
    float4 veldens = read_imagef(vel_im, imagesampler, coord);
    float4 disp = read_imagef(disp_im, imagesampler, coord);

    // oldforce - force
    float2 K = fold - f;

    veldens.x = veldens.x < 0.0001 ? veldens.x > -0.0001 ? 0.0001 : 0.0001 : veldens.x;
    veldens.y = veldens.y < 0.0001 ? veldens.y > -0.0001 ? 0.0001 : 0.0001 : veldens.y;

    K /= veldens.xy;
    K /= veldens.zw;

    float2 u = disp.xy * disp.xy;
    float2 n = u * K;

    int l_id = get_local_id(0) + get_local_size(0) * get_local_id(1);
    interres[l_id * 2] = n.x + n.y;
    interres[l_id * 2 + 1] = u.x + u.y;

    barrier(CLK_LOCAL_MEM_FENCE);
    if (!l_id)
    {
        float2 result = 0;
        int local_size = get_local_size(0) * get_local_size(1);
        for (int i = 0; i < local_size; ++i)
            result += (interres[2*i], interres[2*i+1]);

        int id = get_group_id(0) + get_num_groups(0) * get_group_id(1);
        workgroupres[id * 2] = result.x;
        workgroupres[id * 2+1] = result.y;
    }
}

__kernel void update_displacement(
    read_only image2d_t dispold_im,
    write_only image2d_t disp_im,
    read_only image2d_t vel_old_im,
    write_only image2d_t vel_im,
    read_only image2d_t force_im,
    const float c
)
{
    int2 coord = (get_global_id(0), get_global_id(1));
    float2 force = read_imagef(force_im, imagesampler, coord).xy;
    float4 disp = read_imagef(dispold_im, imagesampler, coord);

    if (disp.w > 0.f)
    {
        float4 vel_old = read_imagef(vel_old_im, imagesampler, coord);

        force /= vel_old.zw;
        force -= c * vel_old.xy;

        float2 test2 = vel_old.zw;

        float2 newvel = vel_old.xy + force;
        write_imagef(vel_im, coord, (newvel, vel_old.z, vel_old.w));
        write_imagef(disp_im, coord, (disp.xy + newvel, 1, disp.w));
    }
}