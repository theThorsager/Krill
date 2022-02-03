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
    public class SettingsParam : GH_PersistentParam<SettingsGoo>
    {
        public override Guid ComponentGuid
        {
            get { return new Guid("2508D227-6A6C-4006-8DC9-2B2DFBA66018"); }
        }

        public SettingsParam() : 
            base("settings", "settings", "", "Krill", "Params") {}

        protected override GH_GetterResult Prompt_Singular(ref SettingsGoo value)
        {
            Rhino.Input.Custom.GetOption go = new Rhino.Input.Custom.GetOption();
            go.SetCommandPrompt("Settings value");
            go.AcceptNothing(true);
            go.AddOption("True");
            go.AddOption("False");
            go.AddOption("Unknown");

            switch (go.Get())
            {
                case Rhino.Input.GetResult.Option:
                    return GH_GetterResult.success;

                case Rhino.Input.GetResult.Nothing:
                    return GH_GetterResult.accept;

                default:
                    return GH_GetterResult.cancel;
            }
        }

        protected override GH_GetterResult Prompt_Plural(ref List<SettingsGoo> values)
        {
            values = new List<SettingsGoo>();

            while (true)
            {
                SettingsGoo val = null;
                switch (Prompt_Singular(ref val))
                { 
                case GH_GetterResult.success:
                    values.Add(val);
                    break;

                case GH_GetterResult.accept:
                    return GH_GetterResult.success;

                case GH_GetterResult.cancel:
                    values = null;
                    return GH_GetterResult.cancel;
                }
            }
        }
    }

    public class SettingsGoo : GH_Goo<Krill.Settings>
    {
        public SettingsGoo()
        {
            this.Value = null;
        }
        public SettingsGoo(Krill.Settings settings)
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
