using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Krill
{
    struct Coord
    {
        public int X;
        public int Y;
        public int Z;
        public Coord(int x, int y, int z)
        {
            X = x; Y = y; Z = z;
        }

        //public static Coord operator -(Coord a, Coord b)
        //{
        //    return new Coord(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        //}
    }
}
