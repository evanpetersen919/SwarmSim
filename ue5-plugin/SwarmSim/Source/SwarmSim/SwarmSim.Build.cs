using System.IO;
using UnrealBuildTool;

public class SwarmSim : ModuleRules
{
    public SwarmSim(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new string[] { "Projects" });

        // core/format is compiled into this module as sources. scripts/link_ue.ps1 creates
        // the junction Private/swarm_format -> <repo>/core/format. It must stay free of
        // Eigen, yaml-cpp and wall-clock calls, and must compile as C++20.
        string FormatDir = Path.Combine(ModuleDirectory, "Private", "swarm_format");
        PrivateIncludePaths.Add(Path.Combine(FormatDir, "include"));
    }
}
