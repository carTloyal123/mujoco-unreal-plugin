// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "Modules/ModuleManager.h"


/**
 * FMujocoModule - A module interface for integrating MuJoCo with Unreal Engine.
 * This class manages the loading and unloading of the MuJoCo library DLLs.
 */
class FMujocoModule final : public IModuleInterface
{
public:
	/**
	 * Called when the module attempts to load into memory.
	 * Used to initialize the MuJoCo library and perform any necessary setup.
	 */
	virtual void StartupModule() override;
    
	/**
	 * Called when the module is unloaded from memory.
	 * Used to release any allocated resources and clean up before shutdown.
	 */
	virtual void ShutdownModule() override;

private:
	/** Handle to the dynamically loaded MuJoCo library. */
	static void* MujocoLibraryHandle;

	/** Function pointer type for retrieving the MuJoCo version.
	 *  This is mainly for making sure we actually loaded the library.
	 */
	typedef int (*Mj_Version_T)();
    
	/** Function pointer to the MuJoCo version function */
	static Mj_Version_T Mj_Version_Func;
};
