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
    class SettingsSTMParam : GH_Param<SettingsSTMGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("5A0C1AA2-7B0D-4C17-AC69-D115D4E29D89"); }
        }

        public SettingsSTMParam() : 
            base("settingsSTM", "settingsSTM", "", "Krill", "Params", GH_ParamAccess.item) 
        { }

    }

    public class SettingsSTMGoo : GH_Goo<Krill.Containers.SettingsSTM>
    {
        public SettingsSTMGoo()
        {
            this.Value = null;
        }
        public SettingsSTMGoo(Krill.Containers.SettingsSTM settings)
        {
            this.Value = settings;
        }
        public SettingsSTMGoo(SettingsSTMGoo settings)
        {
            this.Value = settings.Value;
        }

        public override bool IsValid => this.Value is null;

        public override string TypeName => "SettingsSTM";

        public override string TypeDescription => "";

        public override IGH_Goo Duplicate()
        {
            return new SettingsSTMGoo(this);
        }

        public override string ToString()
        {
            return "A Krill STM settings object";
        }
    }
}
