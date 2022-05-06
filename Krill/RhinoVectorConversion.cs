using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino.Geometry;

namespace Krill
{
    internal static class RhinoVectorConversion
    {

        public static void GetValues(float[] floats, ref Vector3d[] res)
        {
            for (int i = 0; i < res.Length; i++)
            {
                res[i].X = floats[i * 4 + 0];
                res[i].Y = floats[i * 4 + 1];
                res[i].Z = floats[i * 4 + 2];

            }
        }
        public static float[] SetValues(Vector3d[] vec)
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
        public static float[] SetValues(Vector3d[] vec, int[] mask, int maxvolume)
        {
            var arr = new float[vec.Length * 4];
            for (int i = 0; i < vec.Length; i++)
            {
                arr[i * 4 + 0] = (float)vec[i].X;
                arr[i * 4 + 1] = (float)vec[i].Y;
                arr[i * 4 + 2] = (float)vec[i].Z;
                arr[i * 4 + 3] = (float)((mask[i] >> 20) / (double)maxvolume); // do volume conversion
            }
            return arr;
        }
    }
}
