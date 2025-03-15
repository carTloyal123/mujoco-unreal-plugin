// Fill out your copyright notice in the Description page of Project Settings.


#include "RobotPawn.h"
#include "Logging/LogMacros.h"
#include "MujocoAPI.h"

DEFINE_LOG_CATEGORY(LogMujoco);

// Sets default values
ARobotPawn::ARobotPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	UE_LOG(LogMujoco, Warning, TEXT("This is Robot Pawn constructor!!"));
}

ARobotPawn::~ARobotPawn()
{
	UE_LOG(LogMujoco, Warning, TEXT("This is Robot Pawn DESTRUCTOR!!"));
}

// Called when the game starts or when spawned
void ARobotPawn::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogMujoco, Warning, TEXT("This is Robot Pawn BeginPlay!"));
	auto mujocoAPI = std::make_shared<FMujocoAPI>();

	if (mujocoAPI->LoadMuJoCo())
	{
		int VersionNumber = mujocoAPI->GetVersion();
		FString VersionString = mujocoAPI->GetVersionString();

		UE_LOG(LogTemp, Log, TEXT("MuJoCo Version: %s"), *VersionString);
	}
}

// Called every frame
void ARobotPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (this->m_tickLogs > 0)
	{
		UE_LOG(LogMujoco, Warning, TEXT("This is Robot Pawn tick!!"));
		this->m_tickLogs--;
	}
}

// Called to bind functionality to input
void ARobotPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

