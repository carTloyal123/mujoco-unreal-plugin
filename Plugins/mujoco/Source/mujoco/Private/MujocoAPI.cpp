// ReSharper disable CppPrintfBadFormat
#include "MujocoAPI.h"

#include "HAL/PlatformProcess.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

DEFINE_LOG_CATEGORY(LogMujocoAPI);

FMujocoAPI::FMujocoAPI()
    : MuJoCoHandle(nullptr),
      Mj_Version(nullptr),
      Mj_VersionString(nullptr),
      Mj_LoadXML(nullptr),
      Mj_ParseXMLString(nullptr),
      Mj_Compile(nullptr),
      Mj_DeleteSpec(nullptr),
      Mj_Step(nullptr),
      Mj_Forward(nullptr),
      Mj_ResetData(nullptr),
      Mj_MakeData(nullptr),
      Mj_DeleteData(nullptr),
      Mj_DeleteModel(nullptr) {}

FMujocoAPI::~FMujocoAPI() {
  UnloadMuJoCo();
  UE_LOG(LogMujocoAPI, Log, TEXT("MujocoAPI Destructor called"));
}

bool FMujocoAPI::LoadMuJoCo() {
  if (MuJoCoHandle) {
    UE_LOG(LogMujocoAPI, Warning, TEXT("MuJoCo is already loaded."));
    return true;
  }

  const FString DLL_Path = FPaths::Combine(
      FPaths::ProjectDir(),
      TEXT("Plugins/mujoco/Source/mujoco/ThirdParty/mujocoLibrary/bin/"
           "mujoco.dll"));
  MuJoCoHandle = FPlatformProcess::GetDllHandle(*DLL_Path);

  if (!MuJoCoHandle) {
    UE_LOG(LogMujocoAPI, Error, TEXT("Failed to load MuJoCo DLL from: %s"),
           *DLL_Path);
    return false;
  }

  // Load function pointers
  Mj_Version =
      static_cast<Mj_VersionFunc>(FPlatformProcess::GetDllExport(
        MuJoCoHandle, TEXT("mj_version")));
  Mj_VersionString =
      static_cast<Mj_VersionStringFunc>(FPlatformProcess::GetDllExport(
        MuJoCoHandle, TEXT("mj_versionString")));
  Mj_LoadXML = static_cast<Mj_LoadXMLFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_loadXML")));
  Mj_ParseXMLString =
      static_cast<Mj_ParseXMLStringFunc>(FPlatformProcess::GetDllExport(
        MuJoCoHandle, TEXT("mj_parseXMLString")));
  Mj_Compile = static_cast<Mj_CompileFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_compile")));
  Mj_DeleteSpec = static_cast<Mj_DeleteSpecFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_deleteSpec")));
  Mj_Step = static_cast<Mj_StepFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_step")));
  Mj_Forward = static_cast<Mj_ForwardFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_forward")));
  Mj_ResetData = static_cast<Mj_ResetDataFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_resetData")));
  Mj_MakeData = static_cast<Mj_MakeDataFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_makeData")));
  Mj_DeleteData = static_cast<Mj_DeleteDataFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_deleteData")));
  Mj_DeleteModel = static_cast<Mj_DeleteModelFunc>(FPlatformProcess::GetDllExport(
    MuJoCoHandle, TEXT("mj_deleteModel")));

  if (!Mj_Version || !Mj_VersionString || !Mj_ParseXMLString || !Mj_Compile ||
      !Mj_DeleteSpec || !Mj_Step || !Mj_Forward || !Mj_ResetData ||
      !Mj_MakeData || !Mj_DeleteData || !Mj_DeleteModel || !Mj_LoadXML) {
    UE_LOG(LogMujocoAPI, Error, TEXT("Failed to bind MuJoCo functions."));
    UnloadMuJoCo();
    return false;
  }

  UE_LOG(LogMujocoAPI, Log, TEXT("Successfully loaded MuJoCo."));
  return true;
}

void FMujocoAPI::UnloadMuJoCo() {
  if (MuJoCoHandle) {
    FPlatformProcess::FreeDllHandle(MuJoCoHandle);
    MuJoCoHandle = nullptr;
    Mj_Version = nullptr;
    Mj_VersionString = nullptr;
    Mj_LoadXML = nullptr;
    Mj_ParseXMLString = nullptr;
    Mj_Compile = nullptr;
    Mj_DeleteSpec = nullptr;
    Mj_Step = nullptr;
    Mj_Forward = nullptr;
    Mj_ResetData = nullptr;
    Mj_MakeData = nullptr;
    Mj_DeleteData = nullptr;
    Mj_DeleteModel = nullptr;

    UE_LOG(LogMujocoAPI, Log, TEXT("MuJoCo successfully unloaded."));
  }
}

int FMujocoAPI::GetVersion() const {
  return Mj_Version ? Mj_Version() : -1;
}

FString FMujocoAPI::GetVersionString() const {
  return Mj_VersionString ? FString(ANSI_TO_TCHAR(Mj_VersionString()))
                          : FString("Unknown");
}

mjModel* FMujocoAPI::LoadModelFromXML(const FString& Filename) const
{
  if (!Mj_LoadXML) {
    UE_LOG(LogMujocoAPI, Error,
           TEXT("MuJoCo function pointer 'mj_loadXML' is null."));
    return nullptr;
  }

  const std::string XmlStr = TCHAR_TO_UTF8(*Filename);
  char Error[1024] = "";
  mjModel* Model = Mj_LoadXML(XmlStr.c_str(), nullptr, Error, sizeof(Error));

  if (!Model) {
    UE_LOG(LogMujocoAPI, Error, TEXT("Failed to load MuJoCo XML: %s"),
           UTF8_TO_TCHAR(Error));
  }

  return Model;
}

mjSpec* FMujocoAPI::ParseXMLString(const FString& XMLContent) const
{
  if (!Mj_ParseXMLString) {
    UE_LOG(LogMujocoAPI, Error,
           TEXT("MuJoCo function pointer 'mj_parseXMLString' is null."));
    return nullptr;
  }

  const std::string XmlStr = TCHAR_TO_UTF8(*XMLContent);
  char Error[1024] = "";
  mjSpec* Spec =
      Mj_ParseXMLString(XmlStr.c_str(), nullptr, Error, sizeof(Error));

  if (!Spec) {
    UE_LOG(LogMujocoAPI, Error, TEXT("Failed to parse MuJoCo XML: %s"),
           UTF8_TO_TCHAR(Error));
  }

  return Spec;
}

mjModel* FMujocoAPI::CompileSpec(mjSpec* Spec) const
{
  if (!Mj_Compile) {
    UE_LOG(LogMujocoAPI, Error,
           TEXT("MuJoCo function pointer 'mj_compile' is null."));
    return nullptr;
  }

  return Mj_Compile(Spec, nullptr);
}

void FMujocoAPI::FreeSpec(mjSpec* Spec) const
{
  if (Mj_DeleteSpec && Spec) {
    Mj_DeleteSpec(Spec);
  }
}

mjData* FMujocoAPI::CreateData(const mjModel* Model) const
{
  if (!Mj_MakeData) {
    UE_LOG(LogMujocoAPI, Error,
           TEXT("MuJoCo function pointer 'mj_makeData' is null."));
    return nullptr;
  }

  return Mj_MakeData(Model);
}

void FMujocoAPI::FreeData(mjData* Data) const
{
  if (Mj_DeleteData && Data) {
    Mj_DeleteData(Data);
  }
}

void FMujocoAPI::FreeModel(mjModel* Model) const
{
  if (Mj_DeleteModel && Model) {
    Mj_DeleteModel(Model);
  }
}

void FMujocoAPI::Step(const mjModel* Model, mjData* Data) const
{
  if (Mj_Step && Model && Data) {
    Mj_Step(Model, Data);
  }
}

void FMujocoAPI::Forward(const mjModel* Model, mjData* Data) const
{
  if (Mj_Forward && Model && Data) {
    Mj_Forward(Model, Data);
  }
}

void FMujocoAPI::ResetData(const mjModel* Model, mjData* Data) const
{
  if (Mj_ResetData && Model && Data) {
    Mj_ResetData(Model, Data);
  }
}
