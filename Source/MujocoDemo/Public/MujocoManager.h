// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <mujoco/mjmodel.h>
#include <mujoco/mjdata.h>

#include "CoreMinimal.h"
#include "MujocoAPI.h"
#include "GameFramework/Actor.h"
#include "MujocoManager.generated.h"


DECLARE_LOG_CATEGORY_EXTERN(LogMujocoManager, Log, All);


UCLASS()
class MUJOCODEMO_API AMujocoManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMujocoManager();

	/** Set the XML file path for MuJoCo model */
	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void SetMuJoCoXMLPath(const FString& FilePath);

	/** Load the MuJoCo model from the XML file */
	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	bool LoadModel();

	/** Step the simulation */
	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void StepSimulation();

	/** Reset the simulation */
	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void ResetSimulation() const;

	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void PrintBodyPosition() const;

	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void SpawnMuJoCoObjects(); // Creates Unreal objects from the model

	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void HandleStaticMeshObject(const int ModelNum, const FVector& Position, const FVector& Size, const FQuat& Rotation, UStaticMesh* Mesh);

	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void HandleDynamicMeshObject(const int ModelNum, const FVector& Position, const FVector& Size, const FQuat& Rotation);

	UFUNCTION(BlueprintCallable, Category="MuJoCo")
	void UpdateMuJoCoObjects(); // Syncs objects with simulation

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
	/** Path to the MuJoCo XML model */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	FString MuJoCoXMLPath;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	bool bStepSimulation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	bool bApplyControl;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	bool bLogStats;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	float InputControl = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	double TransformScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	double PositionScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	double SizeScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MuJoCo")
	double VertexScale = 1000.0;
	/** MuJoCo Model */
	mjModel* MjModel;

	/** MuJoCo Data */
	mjData* MjData;

	// MujocoAPI that makes sure the library is loaded through Unreal
	std::shared_ptr<FMujocoAPI> MujocoApi;

private:
	bool bLogStateChange = true;

	float AccumulatedTime = 0.0f;  // Keeps track of accumulated time
	const float FixedTimeStep = 1.0f / 60.0f;  // MuJoCo step time (60 Hz)

	// Store objects in map from unreal <-> mujoco
	UPROPERTY()
	TMap<int32, UMeshComponent*> SpawnedMeshes;
};
