// AssignmentManager.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Pathfinder.h"
#include "AssignmentManager.generated.h"

UCLASS()
class RACINGGAME_API AAssignmentManager : public AActor
{
	GENERATED_BODY()

public:
	AAssignmentManager();

protected:
	virtual void BeginPlay() override;

public:
	// Reference to your existing Pathfinder (handles A*)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding")
	APathfinder* PathfinderRef;

	// Blueprint classes for pawns and targets
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assignment")
	TSubclassOf<APawn> PawnClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assignment")
	TSubclassOf<AActor> TargetClass;

	// Entry point: assign best targets to all pawns
	UFUNCTION(BlueprintCallable, Category = "Assignment")
	void AssignPawnsToTargets(bool bUseAStarDistance = false);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool Debug = false;

private:
	UPROPERTY()
	TArray<APawn*> Pawns;

	UPROPERTY()
	TArray<AActor*> Targets;

	// Remember last assignments between runs
	TMap<APawn*, AActor*> LastAssignments;

	// Build NxN cost matrix (pawns vs targets)
	TArray<TArray<float>> BuildCostMatrix(bool bUseAStarDistance);

	// Hungarian algorithm solver
	TArray<int32> HungarianSolve(const TArray<TArray<float>>& CostMatrix);

	// Utility to compute A* path length
	float ComputeAStarDistance(FVector Start, FVector End);
};
