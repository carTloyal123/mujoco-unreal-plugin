// Fill out your copyright notice in the Description page of Project Settings.
#include "MujocoManager.h"

#include <mujoco/mujoco.h>

#include "MaterialDomain.h"
#include "Components/DynamicMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"


DEFINE_LOG_CATEGORY(LogMujocoManager);


// Sets default values
AMujocoManager::AMujocoManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	MjModel = nullptr;
	MjData = nullptr;
	bStepSimulation = false;
	bLogStats = false;
	bApplyControl = false;
	
	MujocoApi = std::make_shared<FMujocoAPI>();
	if (MujocoApi->LoadMuJoCo())
	{
		const FString APIVersion = MujocoApi->GetVersionString();
		UE_LOG(LogMujocoManager, Display, TEXT("MujocoAPI Version: %s"), *APIVersion);
	} else
	{
		UE_LOG(LogMujocoManager, Error, TEXT("Failed to load MujocoAPI!"));
	}
	
	UE_LOG(LogMujocoManager, Log, TEXT("AMujocoManager constructor"));
}

void AMujocoManager::SetMuJoCoXMLPath(const FString& FilePath)
{
	UE_LOG(LogMujocoManager, Log, TEXT("SetMuJoCoXMLPath to %s"), *FilePath);
	if (FPaths::FileExists(FilePath))
	{
		MuJoCoXMLPath = FilePath;
	}
	else
	{
		UE_LOG(LogMujocoManager, Error, TEXT("MuJoCo XML file not found: %s"), *FilePath);
	}
}

bool AMujocoManager::LoadModel()
{
	UE_LOG(LogMujocoManager, Log, TEXT("Loading model from raw XML..."));

	if (MuJoCoXMLPath.IsEmpty())
	{
		UE_LOG(LogMujocoManager, Error, TEXT("MuJoCo XML path is not set."));
		return false;
	}

	if (!FPaths::FileExists(MuJoCoXMLPath))
	{
		UE_LOG(LogMujocoManager, Error, TEXT("MuJoCo XML path does not exist: %s"), *MuJoCoXMLPath);
		return false;
	}
	
	MjModel = MujocoApi->LoadModelFromXML(MuJoCoXMLPath);

	if (!MjModel)
	{
		UE_LOG(LogMujocoManager, Error, TEXT("Failed to compile MuJoCo model."));
		return false;
	}

	// Allocate simulation data
	MjData = MujocoApi->CreateData(MjModel);
	if (!MjData)
	{
		UE_LOG(LogMujocoManager, Error, TEXT("Failed to allocate mjData."));
		MujocoApi->FreeModel(MjModel);
		MjModel = nullptr;
		return false;
	}

	MujocoApi->Forward(MjModel, MjData);
	SpawnMuJoCoObjects();

	UE_LOG(LogMujocoManager, Log, TEXT("Successfully loaded MuJoCo model from raw XML."));
	return true;
}

void AMujocoManager::StepSimulation()
{
	if (MjModel && MjData)
	{
		UE_LOG(LogMujocoManager, VeryVerbose, TEXT("StepSimulation"));

		// Apply forward control input to first actuator
		if (bApplyControl)
		{
			if (MjModel->nu > 0)
			{
				MjData->ctrl[0] = static_cast<mjtNum>(InputControl);
			}
		}

		if (MjData->warning[mjWARN_BADQPOS].number > 0)
		{
			UE_LOG(LogMujocoManager, Error, TEXT("Simulation Diverged: Bad qpos detected!"));
			return;
		}

		// Step simulation one time
		MujocoApi->Step(MjModel, MjData);
		MujocoApi->Forward(MjModel, MjData);
		bLogStateChange = true;
	}
	else
	{
		if (bStepSimulation)
		{
			UE_LOG(LogMujocoManager, Warning, TEXT("Cannot step simulation. Model or data is missing."));
		}
		else
		{
			if (bLogStateChange)
			{
				UE_LOG(LogMujocoManager, Warning, TEXT("Simulation Step Manually Disabled!"));
				bLogStateChange = false;
			}
		}
	}
}

void AMujocoManager::SpawnMuJoCoObjects()
{
    if (!MjModel)
    {
        UE_LOG(LogMujocoManager, Error, TEXT("Cannot spawn objects. Model is not loaded."));
        return;
    }

	const int ModelCount = MjModel->ngeom;
    UE_LOG(LogMujocoManager, Log, TEXT("Spawning %i objects from MuJoCo model..."), ModelCount);

    for (int i = 0; i < ModelCount; i++)
    {
        const FVector Position(
            MjModel->geom_pos[i * 3] * PositionScale, 
            MjModel->geom_pos[i * 3 + 1] * PositionScale, 
            MjModel->geom_pos[i * 3 + 2] * PositionScale
        );

        const FVector Size(
            MjModel->geom_size[i * 3] * SizeScale, // MuJoCo uses half-sizes
            MjModel->geom_size[i * 3 + 1] * SizeScale,
            MjModel->geom_size[i * 3 + 2] * SizeScale
        );

        FQuat Rotation(
            MjModel->geom_quat[i * 4 + 3],  // w
            MjModel->geom_quat[i * 4],      // x
            MjModel->geom_quat[i * 4 + 1],  // y
            MjModel->geom_quat[i * 4 + 2]   // z
        );

        // Determine the geometry type
        UStaticMesh* MeshAsset;
        switch (const int ModelType = MjModel->geom_type[i])
        {
        case mjGEOM_BOX:
            MeshAsset = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
        	HandleStaticMeshObject(i, Position, Size, Rotation, MeshAsset);
        	break;
        case mjGEOM_SPHERE:
            MeshAsset = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
        	HandleStaticMeshObject(i, Position, Size, Rotation, MeshAsset);
            break;
        case mjGEOM_PLANE:
            MeshAsset = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Plane.Plane"));
        	HandleStaticMeshObject(i, Position, Size, Rotation, MeshAsset);
            break;
		case mjGEOM_CAPSULE:
			{
				UE_LOG(LogMujocoManager, Log, TEXT("Loading Capsule"));

				// Load capsule asset (Ensure you have a valid capsule mesh in Unreal)
				MeshAsset = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/StarterContent/Shapes/Shape_NarrowCapsule.Shape_NarrowCapsule"));

				const int Body_ID = MjModel->geom_bodyid[i];

				// Extract body orientation quaternion
				const FQuat BodyRotation(
					MjModel->body_quat[Body_ID * 4 + 3],  // w
					MjModel->body_quat[Body_ID * 4],      // x
					MjModel->body_quat[Body_ID * 4 + 1],  // y
					MjModel->body_quat[Body_ID * 4 + 2]   // z
				);

				Rotation = BodyRotation;
				HandleStaticMeshObject(i, Position, Size, Rotation, MeshAsset);
			}
        	break;
		case mjGEOM_MESH:
			{
				HandleDynamicMeshObject(i, Position, Size, Rotation);
			}
        	break;
        default:
            UE_LOG(LogMujocoManager, Warning, TEXT("Unsupported geometry type. Skipping. (%i)"), ModelType);
            continue;
        }
    }
}

void AMujocoManager::HandleStaticMeshObject(const int ModelNum, const FVector& Position, const FVector& Size, const FQuat& Rotation, UStaticMesh* Mesh)
{
	if (!Mesh)
    {
        UE_LOG(LogMujocoManager, Warning, TEXT("Failed to load mesh asset for geom type %d"), MjModel->geom_type[ModelNum]);
        return;
    }

    // Extract RGBA color from MuJoCo model
    const FLinearColor ObjectColor(
		MjModel->geom_rgba[ModelNum * 4],     // Red
		MjModel->geom_rgba[ModelNum * 4 + 1], // Green
		MjModel->geom_rgba[ModelNum * 4 + 2], // Blue
		MjModel->geom_rgba[ModelNum * 4 + 3]  // Alpha
	);
	
    // Load a base material (we are using our own basic material in Unreal with a color parameter)
    UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/StarterContent/Materials/BaseDemoMat.BaseDemoMat"));
    if (!BaseMaterial)
    {
    	UE_LOG(LogMujocoManager, Warning, TEXT("Failed to load base material. Using default Unreal material."));
    	BaseMaterial = UMaterial::GetDefaultMaterial(MD_Surface);
    }

    // Create a dynamic material instance and apply color
    UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, this);
    if (DynamicMaterial)
    {
    	DynamicMaterial->SetVectorParameterValue("MainColor", ObjectColor);
    	UE_LOG(LogMujocoManager, Verbose, TEXT("Object Color: %.3f, %.3f, %.3f, %.3f"), ObjectColor.R, ObjectColor.G, ObjectColor.B, ObjectColor.A);
    }

    // Create a new mesh component
    UStaticMeshComponent* NewMesh = NewObject<UStaticMeshComponent>(this);
	if (!NewMesh) return;

    NewMesh->SetMaterial(0, DynamicMaterial);
    NewMesh->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepWorldTransform);
    NewMesh->SetStaticMesh(Mesh);
    NewMesh->SetWorldLocation(Position);
    NewMesh->SetWorldRotation(Rotation);
    NewMesh->SetWorldScale3D(Size);
    NewMesh->RegisterComponent();
    
    AddInstanceComponent(NewMesh);
    SpawnedMeshes.Add(ModelNum, NewMesh);

    UE_LOG(LogMujocoManager, Verbose, TEXT("%d at (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f)"),
    	ModelNum, Position.X, Position.Y, Position.Z,
    	Rotation.X, Rotation.Y, Rotation.Z,
    	Size.X, Size.Y, Size.Z);
}

void AMujocoManager::HandleDynamicMeshObject(const int ModelNum, const FVector& Position, const FVector& Size, const FQuat& Rotation) {
	FDynamicMesh3 DynamicMesh = FDynamicMesh3();
	
	const int BodyMeshId = MjModel->geom_dataid[ModelNum];
	const int TotalNumberOfMeshVerts = MjModel->mesh_vertnum[BodyMeshId];
	const int TotalNumOfFaces = MjModel->mesh_facenum[BodyMeshId];
	const int TotalNumOfNormals = MjModel->mesh_normalnum[BodyMeshId];
	
	UE_LOG(
	  LogMujocoManager, Log,
	  TEXT("Loading mesh %i, verts (%i), norms: %i, faces: %i"), BodyMeshId,
	  TotalNumberOfMeshVerts, TotalNumOfNormals, TotalNumOfFaces);
	
	// Get pointers to the start of the data for this mesh
	const float* Vertices = MjModel->mesh_vert + MjModel->mesh_vertadr[BodyMeshId];
	const float* Normals = MjModel->mesh_normal + MjModel->mesh_normaladr[BodyMeshId];
	const int* Faces = MjModel->mesh_face + MjModel->mesh_faceadr[BodyMeshId];
	
	// Log array addresses to verify they are valid
	UE_LOG(LogMujocoManager, Log, TEXT("Vertex array address: %p"), Vertices);
	UE_LOG(LogMujocoManager, Log, TEXT("Normal array address: %p"), Normals);
	UE_LOG(LogMujocoManager, Log, TEXT("Face array address: %p"), Faces);
	
	for (int i = 0; i < TotalNumberOfMeshVerts; ++i) {
		const double VertOne = Vertices[i * 3] * VertexScale;
		const double VertTwo = Vertices[i * 3 + 1] * VertexScale;
		const double VertThree = Vertices[i * 3 + 2] * VertexScale;
		FVector3d CurrentVertex = FVector3d(VertOne, VertTwo, VertThree);
		DynamicMesh.AppendVertex(CurrentVertex);
		
		// Log vertex data
		UE_LOG(LogMujocoManager, Verbose,
			   TEXT("Vertex %i: %.3f, %.3f, %.3f"), i, VertOne, VertTwo, VertThree);
	}
	
	// Add Triangles (Faces)
	for (int i = 0; i < TotalNumOfFaces; ++i) {
		const int IndexOne = Faces[i * 3];
		const int IndexTwo = Faces[i * 3 + 1];
		const int IndexThree = Faces[i * 3 + 2];
		
		// Log face indices
		UE_LOG(LogMujocoManager, Verbose,
			   TEXT("Face %i: %i, %i, %i"), i, IndexOne, IndexTwo, IndexThree);
		
		// Check for valid vertex indices
		if (IndexOne < 0 || IndexOne >= DynamicMesh.VertexCount() ||
			IndexTwo < 0 || IndexTwo >= DynamicMesh.VertexCount() ||
			IndexThree < 0 || IndexThree >= DynamicMesh.VertexCount()) {
		  UE_LOG(LogMujocoManager, Error,
				 TEXT("Invalid vertex index in face %d: %d, %d, %d"), i, IndexOne,
				 IndexTwo, IndexThree);
		  continue; // Skip this triangle
		}
		
		UE::Geometry::FIndex3i CurrentTriangle(IndexOne, IndexTwo, IndexThree);
		DynamicMesh.AppendTriangle(CurrentTriangle);
	}
	
	// Add Normals (assuming one normal per vertex - adjust if needed)
	if (TotalNumOfNormals == TotalNumberOfMeshVerts) {
		for (int i = 0; i < TotalNumOfNormals; ++i) {
		  FVector3f CurrentNormal = FVector3f(Normals[i * 3], Normals[i * 3 + 1],
												Normals[i * 3 + 2]);
		  DynamicMesh.SetVertexNormal(i, CurrentNormal);
		
		  // Log normal data
		  UE_LOG(LogMujocoManager, Log,
				 TEXT("Normal %i: %.3f, %.3f, %.3f"), i, Normals[i * 3],
				 Normals[i * 3 + 1], Normals[i * 3 + 2]);
		}
	} else {
		UE_LOG(LogMujocoManager, Log,
			   TEXT("Number of normals does not match number of vertices.  "
					"Normals may not be assigned correctly."));
		// Handle the case where normals are not one-to-one with vertices.
		// You might need to calculate normals per-triangle or use a different
		// approach.
	}
	
	UE_LOG(LogMujocoManager, Log, TEXT("Setup mesh with %i verts and %i tris"),
		 DynamicMesh.VertexCount(), DynamicMesh.TriangleCount());
	
	// Extract RGBA color from MuJoCo model
	const FLinearColor ObjectColor(MjModel->geom_rgba[ModelNum * 4], // Red
							MjModel->geom_rgba[ModelNum * 4 + 1], // Green
							MjModel->geom_rgba[ModelNum * 4 + 2], // Blue
							MjModel->geom_rgba[ModelNum * 4 + 3]  // Alpha
	);
	
	// Load a base material (ensure you have a basic material in Unreal with a
	// color parameter)
	UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(
	  nullptr, TEXT("/Game/StarterContent/Materials/BaseDemoMat.BaseDemoMat"));
	if (!BaseMaterial) {
		UE_LOG(LogMujocoManager, Warning,
			   TEXT("Failed to load base material. Using default Unreal material."));
		BaseMaterial = UMaterial::GetDefaultMaterial(MD_Surface);
	}
	
	// Create a dynamic material instance and apply color
	UMaterialInstanceDynamic* DynamicMaterial =
	  UMaterialInstanceDynamic::Create(BaseMaterial, this);
	if (DynamicMaterial) {
		DynamicMaterial->SetVectorParameterValue("MainColor", ObjectColor);
		UE_LOG(LogMujocoManager, Log,
			   TEXT("Object Color: %.3f, %.3f, %.3f, %.3f"), ObjectColor.R,
			   ObjectColor.G, ObjectColor.B, ObjectColor.A);
	}
	UDynamicMeshComponent* DynamicMeshComponent =
	  NewObject<UDynamicMeshComponent>(this);
	if (!DynamicMeshComponent) return;
	
	// Check mesh validity
	if (DynamicMesh.CheckValidity()) {
		UE_LOG(LogMujocoManager, Log, TEXT("INVALID MESH AHHHHHH"));
	}
	
	// Add mesh to root and scene
	UDynamicMesh* DynamicMeshContainer = NewObject<UDynamicMesh>(this);
	DynamicMeshContainer->SetMesh(DynamicMesh);
	UE_LOG(LogMujocoManager, Log, TEXT("Dynamic Mesh Tri Count: %i"),
		 DynamicMeshContainer->GetTriangleCount());
	
	DynamicMeshComponent->SetDynamicMesh(DynamicMeshContainer);
	DynamicMeshComponent->NotifyMeshModified();
	
	DynamicMeshComponent->SetMaterial(0, DynamicMaterial);
	DynamicMeshComponent->AttachToComponent(
	  RootComponent, FAttachmentTransformRules::KeepWorldTransform);
	DynamicMeshComponent->SetWorldLocation(Position);
	DynamicMeshComponent->SetWorldRotation(Rotation);
	DynamicMeshComponent->SetWorldScale3D(Size);
	DynamicMeshComponent->RegisterComponent();
	DynamicMeshComponent->SetVisibility(true); // Ensure visibility
	
	AddInstanceComponent(DynamicMeshComponent);
	SpawnedMeshes.Add(ModelNum, DynamicMeshComponent);
	UE_LOG(LogMujocoManager, Log, TEXT("Added full mesh, woo!"));
}

// void AMujocoManager::HandleDynamicMeshObject(int ModelNum, FVector Position, FVector Size, FQuat Rotation)
// {
// 	FDynamicMesh3 DynamicMesh = FDynamicMesh3();
//
// 	const int BodyMeshId = MjModel->geom_dataid[ModelNum];
// 	const int TotalNumOfVertsInModel = MjModel->nmeshvert;
// 	const int TotalNumberOfMeshVerts = MjModel->mesh_vertnum[BodyMeshId];
// 	const int TotalNumOfFaces = MjModel->mesh_facenum[BodyMeshId];
// 	const int TotalNumOfNormals = MjModel->mesh_normalnum[BodyMeshId];
// 	UE_LOG(LogMujocoManager, Log, TEXT("Loading mesh %i, verts (%i / %i), norms: %i, faces: %i"),
// 		BodyMeshId, TotalNumberOfMeshVerts, TotalNumOfVertsInModel, TotalNumOfFaces, TotalNumOfNormals);
//
// 	// Process Vertices
// 	const int* FirstMeshVertex = &MjModel->mesh_vertadr[BodyMeshId];
// 	const int* FirstMeshNormal = &MjModel->mesh_normaladr[BodyMeshId];
// 	const int* FirstMeshFace = &MjModel->mesh_faceadr[BodyMeshId];
// 	for (int i = 0; i < TotalNumberOfMeshVerts; i++)
// 	{
// 		double VertOne = FirstMeshVertex[i * 3];
// 		double VertTwo = FirstMeshVertex[i * 3 + 1];
// 		double VertThree = FirstMeshVertex[i * 3 + 2];
// 		FVector3d CurrentVertex = FVector3d(VertOne, VertTwo, VertThree);
// 		FVector3f CurrentNormal = FVector3f(FirstMeshNormal[i * 3],
// 											FirstMeshNormal[i * 3 + 1],
// 											FirstMeshNormal[i * 3 + 2]);
// 		UE::Geometry::FIndex3i CurrentTriangle = UE::Geometry::FIndex3i(
// 			FirstMeshFace[i * 3],
// 			FirstMeshFace[i * 3 + 1],
// 			FirstMeshFace[i * 3 + 2]);
// 		
// 		// we need faces or something?
// 		DynamicMesh.AppendVertex(CurrentVertex);
// 		DynamicMesh.AppendTriangle(CurrentTriangle);
// 		DynamicMesh.SetVertexNormal(i, CurrentNormal);
// 	}
// 	
// 	UE_LOG(LogMujocoManager, Log, TEXT("Setup mesh with mesh with %i verts"), DynamicMesh.VertexCount());
// 	
// 	// Extract RGBA color from MuJoCo model
// 	FLinearColor ObjectColor(
// 		MjModel->geom_rgba[ModelNum * 4],     // Red
// 		MjModel->geom_rgba[ModelNum * 4 + 1], // Green
// 		MjModel->geom_rgba[ModelNum * 4 + 2], // Blue
// 		MjModel->geom_rgba[ModelNum * 4 + 3]  // Alpha
// 	);
// 	
// 	// Load a base material (ensure you have a basic material in Unreal with a color parameter)
// 	UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/StarterContent/Materials/BaseDemoMat.BaseDemoMat"));
// 	if (!BaseMaterial)
// 	{
// 		UE_LOG(LogMujocoManager, Warning, TEXT("Failed to load base material. Using default Unreal material."));
// 		BaseMaterial = UMaterial::GetDefaultMaterial(MD_Surface);
// 	}
//
// 	// Create a dynamic material instance and apply color
// 	UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, this);
// 	if (DynamicMaterial)
// 	{
// 		DynamicMaterial->SetVectorParameterValue("MainColor", ObjectColor);
// 		UE_LOG(LogMujocoManager, Verbose, TEXT("Object Color: %.3f, %.3f, %.3f, %.3f"), ObjectColor.R, ObjectColor.G, ObjectColor.B, ObjectColor.A);
// 	}
//
//
// 	// Check mesh validity
// 	if (bool IsMeshValid = DynamicMesh.CheckValidity())
// 	{
// 		UE_LOG(LogMujocoManager, Log, TEXT("MESH IS VALID"));
//
// 	} else
// 	{
// 		UE_LOG(LogMujocoManager, Log, TEXT("INVALID MESH AHHHHHH"));
// 	}
// 	
// 	// Add mesh to root and scene
// 	UDynamicMesh* DynamicMeshContainer = NewObject<UDynamicMesh>(this);
// 	DynamicMeshContainer->SetMesh(DynamicMesh);
// 	UE_LOG(LogMujocoManager, Log, TEXT("Dynamic Mesh Tri Count: %i"), DynamicMeshContainer->GetTriangleCount());
//
// 	UDynamicMeshComponent* DynamicMeshComponent = NewObject<UDynamicMeshComponent>(this);
// 	if (!DynamicMeshComponent) return;
// 	DynamicMeshComponent->RegisterComponent();
//
// 	DynamicMeshComponent->SetDynamicMesh(DynamicMeshContainer);
// 	DynamicMeshComponent->SetMesh(FDynamicMesh3(DynamicMesh));
// 	DynamicMeshComponent->NotifyMeshModified();
// 	
// 	DynamicMeshComponent->SetMaterial(0, DynamicMaterial);
// 	DynamicMeshComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepWorldTransform);
// 	DynamicMeshComponent->SetWorldLocation(Position);
// 	DynamicMeshComponent->SetWorldRotation(Rotation);
// 	DynamicMeshComponent->SetWorldScale3D(Size);
//     
// 	AddInstanceComponent(DynamicMeshComponent);
// 	UE_LOG(LogMujocoManager, Log, TEXT("Added full mesh, woo!"));
// 	SpawnedDynamicMeshes.Add(ModelNum, DynamicMeshComponent);
// }

void AMujocoManager::UpdateMuJoCoObjects()
{
	if (!MjModel || !MjData)
	{
		UE_LOG(LogMujocoManager, Warning, TEXT("Cannot update objects. Model or data is missing."));
		return;
	}

	MujocoApi->Forward(MjModel, MjData); // Update kinematics

	for (const auto& MeshPair : SpawnedMeshes)
	{
		const int32 BodyIndex = MeshPair.Key;
		// This will break probably with the dynamic meshes lol
		UMeshComponent* Mesh = MeshPair.Value;

		if (!Mesh) continue;

		// Convert MuJoCo position to Unreal world position
		const FVector Position(
			MjData->geom_xpos[BodyIndex * 3] * PositionScale, 
			MjData->geom_xpos[BodyIndex * 3 + 1] * PositionScale, 
			MjData->geom_xpos[BodyIndex * 3 + 2] * PositionScale
		);
		
		// convert MjData->geom_xmat to quaternion
		FMatrix MujocoMatrix = FMatrix(
			FVector(MjData->geom_xmat[BodyIndex*9 + 0], MjData->geom_xmat[BodyIndex*9 + 3], MjData->geom_xmat[BodyIndex*9 + 6]),  // X-axis
			FVector(MjData->geom_xmat[BodyIndex*9 + 1], MjData->geom_xmat[BodyIndex*9 + 4], MjData->geom_xmat[BodyIndex*9 + 7]),  // Y-axis
			FVector(MjData->geom_xmat[BodyIndex*9 + 2], MjData->geom_xmat[BodyIndex*9 + 5], MjData->geom_xmat[BodyIndex*9 + 8]),  // Z-axis
			FVector::ZeroVector
		);

		FQuat Rotation = MujocoMatrix.ToQuat();
		// Apply position and rotation updates
		Mesh->SetWorldLocation(Position);
		Mesh->SetWorldRotation(Rotation);

		if (bLogStats)
		{
			UE_LOG(LogMujocoManager, Log, TEXT("%i: (%.5f, %.5f, %.5f), (%.5f, %.5f, %.5f)"), BodyIndex, Position.X, Position.Y, Position.Z, Rotation.X, Rotation.Y, Rotation.Z);
		}
	}
}

void AMujocoManager::PrintBodyPosition() const
{
	if (!MjModel || !MjData)
	{
		UE_LOG(LogMujocoManager, Warning, TEXT("Cannot print body position, model or data is missing."));
		return;
	}

	// Ensure positions are updated before reading xpos
	MujocoApi->Forward(MjModel, MjData);  // Updates all kinematic data

	// Print world position of the first body (ignoring world body at index 0)
	if (MjModel->nbody > 1)
	{
		constexpr int BodyIndex = 1; // First movable body
		const FVector Position(
			MjData->xpos[BodyIndex * 3],     // X position
			MjData->xpos[BodyIndex * 3 + 1], // Y position
			MjData->xpos[BodyIndex * 3 + 2]  // Z position
		);
		
		UE_LOG(LogMujocoManager, Log, TEXT("Body Position: X=%.3f, Y=%.3f, Z=%.3f"), Position.X, Position.Y, Position.Z);
	}
}

void AMujocoManager::ResetSimulation() const
{
	UE_LOG(LogMujocoManager, Log, TEXT("ResetSimulation"));
	if (MjModel && MjData)
	{
		MujocoApi->ResetData(MjModel, MjData);
		UE_LOG(LogMujocoManager, Log, TEXT("Simulation reset."));
	}
}

// Called when the game starts or when spawned
void AMujocoManager::BeginPlay()
{
	UE_LOG(LogMujocoManager, Log, TEXT("BeginPlay"));
	Super::BeginPlay();
	if (!MuJoCoXMLPath.IsEmpty())
	{
		LoadModel();
	} else
	{
		UE_LOG(LogMujocoManager, Warning, TEXT("Model Path is empty :("));
	}
}

// Called every frame
void AMujocoManager::Tick(const float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (MjModel && MjData && bStepSimulation)
	{
		AccumulatedTime += DeltaTime;

		while (AccumulatedTime >= FixedTimeStep)
		{
			StepSimulation();
			AccumulatedTime -= FixedTimeStep;
		}
		// Sync Unreal Objects with Mujoco data
		UpdateMuJoCoObjects(); 
	}
}


void AMujocoManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	UE_LOG(LogMujocoManager, Log, TEXT("EndPlay"));
	if (MjData)
	{
		MujocoApi->FreeData(MjData);
		MjData = nullptr;
	}

	if (MjModel)
	{
		MujocoApi->FreeModel(MjModel);
		MjModel = nullptr;
	}

	Super::EndPlay(EndPlayReason);
}

