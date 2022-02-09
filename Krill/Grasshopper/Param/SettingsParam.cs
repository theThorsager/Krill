using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;

namespace Krill.Grasshopper.Param
{
    class SettingsParam : GH_Param<SettingsGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("2508D227-6A6C-4006-8DC9-2B2DFBA66018"); }
        }

        public SettingsParam() : 
            base("settings", "settings", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class SettingsGoo : GH_Goo<Krill.Containers.Settings>
    {
        public SettingsGoo()
        {
            this.Value = null;
        }
        public SettingsGoo(Krill.Containers.Settings settings)
        {
            this.Value = settings;
        }
        public SettingsGoo(SettingsGoo settings)
        {
            this.Value = settings.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "Settings";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new SettingsGoo(this);
        }

        public override string ToString()
        {
            return "A Krill settings object";
        }
    }
}
