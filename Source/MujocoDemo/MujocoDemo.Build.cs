// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MujocoDemo : ModuleRules
{
	public MujocoDemo(ReadOnlyTargetRules Target) : base(Target)
	{
		PrivateDependencyModuleNames.AddRange(new string[] { "GeometryFramework" });
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "mujoco", "GeometryCore", "GeometryFramework" });
	}
}
