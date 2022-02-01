using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Krill
{
    internal class BBperi
    {
        public double bond_stiffness;
        public Point3d[] startPos;
        public Point3d[] u;     // Current displacements
        public int[,] nlist;    // Neighbour list
        public double[] vol;   // Volumes of the particles
        public Vector3d[] force;
        public int[,] fbc_types;    // Boundary force type
        public Vector3d[] fbc;      // Boundary force
        public double fbc_scale;
        public List<Vector3d> body_force = new List<Vector3d>();

        public BBperi(Point3d[] OrgPos, Point3d[] currentDisp, double Bond_stiffness, int[,] neighbour_list, double[] volumes, Vector3d[] Force, int[,] ForceBC_type, Vector3d[] ForceBC, double ForceBC_scale)
        {
            startPos = OrgPos;
            u = currentDisp;
            bond_stiffness = Bond_stiffness;
            nlist = neighbour_list;
            vol = volumes;
            force = Force;
            fbc_types = ForceBC_type;   // Behövs den här just nu?
            fbc = ForceBC;
            fbc_scale = ForceBC_scale;

            Vector3d[] local_cache = new Vector3d[startPos.Length];

            // Assuming the bonds never break, otherwise need a new if-statement
            for (int i = 0; i < startPos.Length; i++)
            {
                int noBonds = nlist.GetLength(i);   // Number of bonds for that particle

                for (int j = 0; j < noBonds; j++)
                {
                    Vector3d xi_vec;
                    Vector3d xi_eta;

                    xi_vec = startPos[j] - startPos[i];
                    xi_eta = u[j] - u[i] + xi_vec;

                    double xi = xi_vec.Length;
                    double y = xi_eta.Length;
                    double s = (y - xi) / xi;

                    double f = s * bond_stiffness * vol[j];
                    local_cache[i] += f * xi_eta / y;    // Node force (?)

                }

                body_force.Add(local_cache[i]);

                force[i] = body_force[i] + fbc_scale * fbc[i];

            }

        }

    }
}
