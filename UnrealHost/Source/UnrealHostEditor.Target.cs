using UnrealBuildTool;

public class UnrealHostEditorTarget : TargetRules
{
    public UnrealHostEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
        ExtraModuleNames.Add("UnrealHost");
    }
}
