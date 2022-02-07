using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using System.Numerics;

namespace Krill
{
    static internal class AnalyticalSolutions
    {
        /// <summary>
        /// Displacement of a infinite solid with a spherical cavity subject to a stress along the z-axis.
        /// </summary>
        /// <param name="x">Initial point in the solid, must be on or outside of the sphere.</param>
        /// <param name="a">Radii of the sphere</param>
        /// <param name="sigma">Stress infinitely far away along the Z-axis</param>
        /// <param name="E">Youngs modulus</param>
        /// <param name="nu">Poissons number</param>
        /// <returns>displacments vector</returns>
        public static Vector3d SphericalCavity(Point3d x, double a, double sigma, double E, double nu)
        {
            double R = x.DistanceTo(Point3d.Origin);
            if (R < a - 1e-6)
                return Vector3d.Unset;

            double a3 = a * a * a;
            double a5 = a3 * a * a;
            double R3 = R * R * R;
            double R5 = R3 * R * R;

            double zdisp = (
                2.0 +
                (5.0 * (5.0 - 4.0 * nu) / (7.0 - 5.0 * nu)) * a3 / R3 +
                6.0 / (7.0 - 5.0 * nu) * a5 / R5
                ) * x.Z;

            double generalldisp = -2.0 * nu / (1.0 + nu) +
                (5.0 * nu - 6.0) / (7.0 - 5.0 * nu) * a3 / R3 +
                3.0 / (7.0 - 5.0 * nu) * a5 / R5 * (1.0 - 5.0 * x.Z * x.Z / (R * R));

            double ux = generalldisp * x.X;
            double uy = generalldisp * x.Y;
            double uz = generalldisp * x.Z + zdisp;


            Vector3d u = new Vector3d(ux, uy, uz);
            u *= (1.0 + nu) * sigma / (2.0 * E);

            return u;
        }

        /// <summary>
        /// Displacment of a point in a elastic halfspace subject to a cylindrical punch
        /// </summary>
        /// <param name="x">Initial position of a point. Must be equal to or less than 0 in the Z-axis</param>
        /// <param name="a">Radii of cylinder</param>
        /// <param name="h">Displacement of cylinder</param>
        /// <param name="nu">Poissons number for material</param>
        /// <returns>displacement vector</returns>
        public static Vector3d CylinderPunch(Point3d x, double a, double h, double nu)
        {
            if (x.Z > 0)
                return Vector3d.Unset;

            x.Z *= -1;

            Complex i = Complex.ImaginaryOne;
            Complex thing = x.Z + a * i;
            Complex t = thing * thing;
            Complex R = Complex.Sqrt(x.X * x.X + x.Y * x.Y + t);

            Complex zpart = 2.0 * (1.0 - nu) * Complex.Log(R + thing) - x.Z / R;
            Complex notz = 1.0 / (R + thing) * (1.0 - 2.0 * nu - x.Z / R);

            double ux = (notz * x.X).Imaginary;
            double uy = (notz * x.Y).Imaginary;
            double uz = zpart.Imaginary;

            Vector3d u = new Vector3d(ux, uy, uz);
            u *= h / (Math.PI * (1.0 - nu));

            return u;
        }
    }
}
