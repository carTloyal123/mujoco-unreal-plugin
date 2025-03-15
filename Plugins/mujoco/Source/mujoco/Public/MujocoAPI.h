#pragma once

#include "CoreMinimal.h"
#include "mujoco/mujoco.h"

DECLARE_LOG_CATEGORY_EXTERN(LogMujocoAPI, Log, All);

/**
 * @class FMujocoAPI
 * @brief A wrapper class for interfacing with the MuJoCo physics engine.
 *
 * This class provides a high-level C++ interface for dynamically loading the MuJoCo library,
 * managing models and simulation data, and executing physics simulation steps.
 */
class MUJOCO_API FMujocoAPI {
public:
    /**
     * @brief Constructs a MujocoAPI object but does NOT load MuJoCo.
     */
    FMujocoAPI();

    /**
     * @brief Destructor that ensures MuJoCo is unloaded if still loaded.
     */
    ~FMujocoAPI();

    /**
     * @brief Loads the MuJoCo shared library dynamically.
     * @return True if successfully loaded, false otherwise.
     */
    bool LoadMuJoCo();

    /**
     * @brief Unloads the MuJoCo shared library and clears function pointers.
     */
    void UnloadMuJoCo();

    /**
     * @brief Retrieves the MuJoCo version as an integer.
     * @return MuJoCo version number, or -1 if not available.
     */
    int GetVersion() const;

    /**
     * @brief Retrieves the MuJoCo version as a string.
     * @return MuJoCo version as FString, or "Unknown" if not available.
     */
    FString GetVersionString() const;

    // Parsing & Compilation

    /**
     * @brief Loads a MuJoCo model from an XML file.
     * @param Filename The path to the XML file.
     * @return Pointer to the loaded mjModel, or nullptr on failure.
     */
    mjModel* LoadModelFromXML(const FString& Filename) const;

    /**
     * @brief Parses an XML string into a MuJoCo specification object.
     * @param XMLContent XML string containing the MuJoCo model.
     * @return Pointer to the parsed mjSpec, or nullptr on failure.
     */
    mjSpec* ParseXMLString(const FString& XMLContent) const;

    /**
     * @brief Compiles a MuJoCo specification into a model.
     * @param Spec Pointer to the MuJoCo specification.
     * @return Pointer to the compiled mjModel, or nullptr on failure.
     */
    mjModel* CompileSpec(mjSpec* Spec) const;

    /**
     * @brief Frees a MuJoCo specification object.
     * @param Spec Pointer to the mjSpec to be freed.
     */
    void FreeSpec(mjSpec* Spec) const;

    // Simulation Functions

    /**
     * @brief Advances the simulation by one time step using the given model and data.
     * @param Model Pointer to the MuJoCo model.
     * @param Data Pointer to the simulation data.
     */
    void Step(const mjModel* Model, mjData* Data) const;

    /**
     * @brief Computes forward dynamics without advancing simulation.
     * @param Model Pointer to the MuJoCo model.
     * @param Data Pointer to the simulation data.
     */
    void Forward(const mjModel* Model, mjData* Data) const;

    /**
     * @brief Resets the simulation data for a given model.
     * @param Model Pointer to the MuJoCo model.
     * @param Data Pointer to the simulation data.
     */
    void ResetData(const mjModel* Model, mjData* Data) const;

    // Data & Model Management

    /**
     * @brief Creates and returns a new mjData structure for a given model.
     * @param Model Pointer to the MuJoCo model.
     * @return Pointer to the allocated mjData structure.
     */
    mjData* CreateData(const mjModel* Model) const;

    /**
     * @brief Frees the allocated mjData structure.
     * @param Data Pointer to the mjData to be freed.
     */
    void FreeData(mjData* Data) const;

    /**
     * @brief Frees the allocated MuJoCo model.
     * @param Model Pointer to the mjModel to be freed.
     */
    void FreeModel(mjModel* Model) const;

private:
    /** Handle to the dynamically loaded MuJoCo shared library. */
    void* MuJoCoHandle;

    /** Function pointer types for MuJoCo API functions. */
    typedef int (*Mj_VersionFunc)();
    typedef const char* (*Mj_VersionStringFunc)();
    typedef mjModel* (*Mj_LoadXMLFunc)(const char*, const mjVFS*, char*, int);
    typedef mjSpec* (*Mj_ParseXMLStringFunc)(const char*, const mjVFS*, char*, int);
    typedef mjModel* (*Mj_CompileFunc)(mjSpec*, const mjVFS*);
    typedef void (*Mj_DeleteSpecFunc)(mjSpec*);
    typedef void (*Mj_StepFunc)(const mjModel*, mjData*);
    typedef void (*Mj_ForwardFunc)(const mjModel*, mjData*);
    typedef void (*Mj_ResetDataFunc)(const mjModel*, mjData*);
    typedef mjData* (*Mj_MakeDataFunc)(const mjModel*);
    typedef void (*Mj_DeleteDataFunc)(mjData*);
    typedef void (*Mj_DeleteModelFunc)(mjModel*);

    /** Function pointers for MuJoCo API calls. */
    Mj_VersionFunc Mj_Version;
    Mj_VersionStringFunc Mj_VersionString;
    Mj_LoadXMLFunc Mj_LoadXML;
    Mj_ParseXMLStringFunc Mj_ParseXMLString;
    Mj_CompileFunc Mj_Compile;
    Mj_DeleteSpecFunc Mj_DeleteSpec;
    Mj_StepFunc Mj_Step;
    Mj_ForwardFunc Mj_Forward;
    Mj_ResetDataFunc Mj_ResetData;
    Mj_MakeDataFunc Mj_MakeData;
    Mj_DeleteDataFunc Mj_DeleteData;
    Mj_DeleteModelFunc Mj_DeleteModel;
};
