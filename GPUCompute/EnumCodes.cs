using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Silk.NET.OpenCL;

namespace GPUCompute
{
    internal static class EnumCodes
    {
        public static string Query(int input)
        {
            string result = "";
            foreach (var item in Enum.GetValues(typeof(CLEnum)))
            {
                if ((int)item == input)
                {
                    result += item.ToString();
                    result += Environment.NewLine;
                }
            }

            return result;
        }
    }
}
