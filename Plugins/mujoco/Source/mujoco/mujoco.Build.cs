using UnrealBuildTool;
using System;
using System.IO;

public class mujoco : ModuleRules
{
    public mujoco(ReadOnlyTargetRules Target) : base(Target)
    {
        string ThirdPartyPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "ThirdParty/mujocoLibrary"));
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicIncludePaths.AddRange(
            new string[] {
                Path.Combine(ModuleDirectory, "Public"),
                Path.Combine(ThirdPartyPath, "include") // MuJoCo headers

                // ... add public include paths required here ...
            }
            );
        
        PrivateIncludePaths.AddRange(
            new string[] {
                Path.Combine(ModuleDirectory, "Private"),
                Path.Combine(ThirdPartyPath, "include") // MuJoCo headers
            }
            );


        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine",
                "Projects" // Required for IPluginManager
                // ... add other public dependencies that you statically link with here ...
            }
            );
        
        // Tell Unreal where to find the .lib
        PublicAdditionalLibraries.Add(Path.Combine(ThirdPartyPath, "lib", "mujoco.lib")); // Or whatever the .lib file is named

        // Tell Unreal how to find the .dll at runtime (copy it to the binaries folder)
        string DllPath = Path.Combine(ThirdPartyPath, "bin", "mujoco.dll"); // Or whatever the .dll file is named
        RuntimeDependencies.Add(DllPath);

        // Delay-load the DLL (optional, but often a good idea)
        PublicDelayLoadDLLs.Add("mujoco.dll"); // Or whatever the .dll file is named

        // Add a definition to the compile environment, so we can use it in our code to conditionally compile
        PublicDefinitions.Add("WITH_MUJOCO=1");
    }
}
