using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Krill.Containers
{
    public class SettingsSTM
    {
        // Mean elastic modulus of concrete [Pa]
        public double Ec { get; set; } = 35e9;
        // Characteristic compressive strength of concrete [Pa]
        public double fck { get; set; } = 40e6;
        // Design compressive strength of concrete [Pa]
        public double fcd { get; set; } = 26.67e6;
        // Elastic modulus of reinforcing steel [Pa]
        public double Es { get; set; } = 200e9;
        // Design yield strength of reinforcing steel [Pa]
        public double fyd { get; set; } = 435e6;
    }
}
