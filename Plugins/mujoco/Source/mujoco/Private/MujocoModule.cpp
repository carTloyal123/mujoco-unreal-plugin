#include "MujocoModule.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "mujoco/mujoco.h"

// Define the static variables initial data
void* FMujocoModule::MujocoLibraryHandle = nullptr;
FMujocoModule::Mj_Version_T FMujocoModule::Mj_Version_Func = nullptr;

/**
 * Initializes the MuJoCo module by loading the shared library DLL.
 */
void FMujocoModule::StartupModule()
{
#ifdef WITH_MUJOCO
	// Get the desired DLL path, only Windows for now :)
	FString PluginDir = FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("mujoco"));
	const FString DllPath = FPaths::Combine(PluginDir, TEXT("ThirdParty/mujocoLibrary/bin/mujoco.dll"));

	// Load the MuJoCo DLL
	MujocoLibraryHandle = FPlatformProcess::GetDllHandle(*DllPath);
	if (MujocoLibraryHandle)
	{
		// Retrieve the function pointer for mj_version to print current version
		Mj_Version_Func = static_cast<Mj_Version_T>(FPlatformProcess::GetDllExport(MujocoLibraryHandle, TEXT("mj_version")));
		if (Mj_Version_Func)
		{
			const int Version = Mj_Version_Func();
			// Do recommended version check
			if (constexpr int VersionHeader = mjVERSION_HEADER; VersionHeader != Version)
			{
				UE_LOG(LogTemp, Warning, TEXT("MuJoCo version: %d, DOES NOT match installed headers: %d"), Version, VersionHeader);
			} else
			{
				UE_LOG(LogTemp, Log, TEXT("MuJoCo version: %d"), Version);
			}
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to load mj_version function"));
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to load MuJoCo DLL: %s"), *DllPath);
	}
#endif
}

/**
 * Shuts down the MuJoCo module by unloading the shared library.
 */
void FMujocoModule::ShutdownModule()
{
#ifdef WITH_MUJOCO
	if (MujocoLibraryHandle)
	{
		FPlatformProcess::FreeDllHandle(MujocoLibraryHandle);
		MujocoLibraryHandle = nullptr;
		Mj_Version_Func = nullptr;
	}
#endif
}

// Implements the module for use in Unreal Engine.
IMPLEMENT_MODULE(FMujocoModule, mujoco)
