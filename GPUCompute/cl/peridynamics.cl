#pragma OPENCL EXTENSION cl_khr_3d_image_writes : enable

#define TOL 1e-12f

const sampler_t imagesampler = 
        CLK_FILTER_NEAREST | 
        CLK_ADDRESS_CLAMP | 
        CLK_NORMALIZED_COORDS_FALSE;

// Only displacment needs to be an image ?
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


__kernel void compute_dampening(
    read_only image3d_t forceA_im,
    read_only image3d_t forceB_im,
    read_only image3d_t vel_im,
    read_only image3d_t densities_im,
    read_only image3d_t disp_im,
    global float* results
)
{
    int4 coord = (int4)(get_global_id(0), get_global_id(1), get_global_id(2), 0);
    float4 disp = read_imagef(disp_im, imagesampler, coord);

    if (disp.w > 0.f)
    {
        int n = get_image_width(forceA_im);
        float3 fA = read_imagef(forceA_im, imagesampler, coord).xyz;
        float3 fB = read_imagef(forceB_im, imagesampler, coord).xyz;
        float3 dens = read_imagef(densities_im, imagesampler, coord).xyz;
        float3 vel = read_imagef(vel_im, imagesampler, coord).xyz;

        float3 K = fA - fB;
        
        vel *= dens;
        vel.x = vel.x > TOL ? vel.x : vel.x < -TOL ? vel.x : vel.x < 0 ? -TOL : TOL;
        vel.y = vel.y > TOL ? vel.y : vel.y < -TOL ? vel.y : vel.y < 0 ? -TOL : TOL;
        vel.z = vel.z > TOL ? vel.z : vel.z < -TOL ? vel.z : vel.z < 0 ? -TOL : TOL;

        K /= vel;

        int I = 2*(coord.x + n * coord.y + n * n * coord.z);
        results[I] =   disp.x * disp.x * K.x  +  disp.y * disp.y * K.y  +  disp.z * disp.z * K.z;
        results[I+1] = disp.x * disp.x +         disp.y * disp.y  +        disp.z * disp.z;
    }
}

__kernel void set_dampining(
    global float* input, 
    global float* result)
{
    float nom = input[0];
    float den = input[1];

    den = den > TOL ? den : TOL;
    nom = fabs(nom);
    float res = 2.0f * sqrt(nom / den);

    result[0] = res < 1.0f ? res : 1.0f;
}

__kernel void update_displacement(
    read_only image3d_t dispold_im,
    write_only image3d_t disp_im,
    read_only image3d_t vel_old_im,
    write_only image3d_t vel_im,
    read_only image3d_t force_im,
    read_only image3d_t densities_im,
    global float* c
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
        force -= c[0] * vel_old;

        float3 newvel = vel_old + force;
        write_imagef(vel_im, coord, (float4)(newvel, 0));
        write_imagef(disp_im, coord, (float4)(disp.xyz + newvel, disp.w));
    }
}

__kernel void compute_residual(
    read_only image3d_t force_im, 
    global float* results)
{
    int4 coord = (int4)(get_global_id(0), get_global_id(1), get_global_id(2), 0);
    float3 f = read_imagef(force_im, imagesampler, coord).xyz;
    int n = get_image_width(force_im);

    int I = coord.x + n * coord.y + n * n * coord.z;
    results[I] = fabs(f.x) + fabs(f.y) +  fabs(f.z);
}

__kernel void reduce(
    global float* input, 
    global float* output, 
    local float* intermidiate, 
    const int n, const int size)
{
    int gid = get_global_id(0) * size;

    int m = gid + size;
    m = m < n ? m : n;
    float res = 0.f;
    for ( ; gid < m; gid++)
        res += input[gid];

    int lid = get_local_id(0);
    intermidiate[lid] = res;

    barrier(CLK_LOCAL_MEM_FENCE);
    if (!lid)
    {
        res = 0.f;
        for (int i = 0; i < get_local_size(0); i++)
            res += intermidiate[i];
    
        output[get_group_id(0)] = res;
    }
}
__kernel void reduce_dampening(
    global float* input, 
    global float* output, 
    local float* intermidiate, 
    const int n, const int size)
{
    int gid = get_global_id(0) * 2 * size;

    int m = gid + size * 2;
    m = m < n * 2 ? m : n * 2;
    float resa = 0.f;
    float resb = 0.f;
    for ( ; gid < m; gid+=2)
    {
        resa += input[gid];
        resb += input[gid+1];
    }
    int lid = get_local_id(0) * 2;
    intermidiate[lid] = resa;
    intermidiate[lid+1] = resb;

    barrier(CLK_LOCAL_MEM_FENCE);
    if (!lid)
    {
        resa = 0.f;
        resb = 0.f;
        for (int i = 0; i < get_local_size(0) * 2; i += 2)
        {
            resa += intermidiate[i];
            resb += intermidiate[i+1];
        }
        int group_id = get_group_id(0) * 2;
        output[group_id] = resa;
        output[group_id+1] = resb;
    }
}

__kernel void test(write_only image3d_t result)
{
    int4 i = (int4)(get_global_id(0), get_global_id(1), get_global_id(2), 0);

    int val = get_global_size(0);

    float4 write = (float4)((float)(i.x+1), (float)(i.y+1), (float)(i.z+1), (float)(val+1));

    write_imagef(result, i, write);
}