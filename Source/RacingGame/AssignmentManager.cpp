// AssignmentManager.cpp
#include "AssignmentManager.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Engine/World.h"

AAssignmentManager::AAssignmentManager()
    : PathfinderRef(nullptr)
{
    PrimaryActorTick.bCanEverTick = false;
}

void AAssignmentManager::BeginPlay()
{
    Super::BeginPlay();
}

void AAssignmentManager::AssignPawnsToTargets(bool bUseAStarDistance)
{
    // Collect Pawns & Targets
    TArray<AActor*> FoundPawns;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), PawnClass, FoundPawns);
    Pawns.Empty();

    for (AActor* Actor : FoundPawns)
    {
        APawn* Pawn = Cast<APawn>(Actor);
        if (Pawn)
            Pawns.Add(Pawn);
    }

    UGameplayStatics::GetAllActorsOfClass(GetWorld(), TargetClass, Targets);
    UE_LOG(LogTemp, Display, TEXT("Found %d targets."), Targets.Num());
    for (AActor* Target : Targets)
    {
        UE_LOG(LogTemp, Display, TEXT("Target found: %s at %s"),
            *Target->GetName(),
            *Target->GetActorLocation().ToString());
    }

    if (Pawns.Num() == 0 || Targets.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("No pawns or targets found."));
        return;
    }

    int32 MinCount = FMath::Min(Pawns.Num(), Targets.Num());

    if (!PathfinderRef)
    {
        UE_LOG(LogTemp, Error, TEXT("Pathfinder reference not set in AssignmentManager!"));
        return;
    }

    TArray<TArray<float>> CostMatrix = BuildCostMatrix(bUseAStarDistance);
    // After HungarianSolve
    TArray<int32> Assignments = HungarianSolve(CostMatrix);

    int32 NumAssignments = FMath::Min(Pawns.Num(), Targets.Num());

    for (int32 i = 0; i < NumAssignments; i++)
    {
        int32 TargetIndex = Assignments[i];
        if (!Pawns.IsValidIndex(i) || !Targets.IsValidIndex(TargetIndex))
            continue;
    
        APawn* Pawn = Pawns[i];
        AActor* Target = Targets[TargetIndex];

        FVector Start = Pawn->GetActorLocation();
        FVector End = Target->GetActorLocation();

        FVector NextStep = PathfinderRef->FindPath(Start, End);

        // Call Assign_Target_Actor in BP_Item
        UFunction* SetTargetFunc = Pawn->FindFunction(TEXT("Assign_Target_Actor"));
        if (SetTargetFunc)
        {
            UE_LOG(LogTemp, Display, TEXT("Found Assign_Target_Actor on %s"), *Pawn->GetName());

            struct FSetTargetParams { AActor* NewTarget; };
            FSetTargetParams Params;
            Params.NewTarget = Target;
            Pawn->ProcessEvent(SetTargetFunc, &Params);
            LastAssignments.Add(Pawn, Target);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Assign_Target_Actor not found on %s"), *Pawn->GetName());
        }

        // Pawn->SetActorLocation(NextStep);

        UE_LOG(LogTemp, Display, TEXT("Pawn %d assigned to Target %d (Distance: %.2f)"),
            i, TargetIndex, CostMatrix[i][TargetIndex]);

        if (Debug)
        {
            FLinearColor LineColor = FLinearColor::MakeFromHSV8((i * 40) % 255, 255, 255);
            UKismetSystemLibrary::DrawDebugLine(this, Start, End, LineColor, 10.0f, 3.0f);
            UKismetSystemLibrary::DrawDebugSphere(this, Start, 30.f, 8, FLinearColor::Green, 10.0f, 1.5f);
            UKismetSystemLibrary::DrawDebugSphere(this, End, 30.f, 8, FLinearColor::Red, 10.0f, 1.5f);
            FString Label = FString::Printf(TEXT("P%d -> T%d"), i, TargetIndex);
            UKismetSystemLibrary::DrawDebugString(this, (Start + End) / 2, Label, nullptr, LineColor, 10.0f);
        }
    }

}

TArray<TArray<float>> AAssignmentManager::BuildCostMatrix(bool bUseAStarDistance)
{
    int32 NumPawns = Pawns.Num();
    int32 NumTargets = Targets.Num();
    int32 N = FMath::Max(NumPawns, NumTargets); // make square

    TArray<TArray<float>> Matrix;
    Matrix.SetNum(N);

    for (int32 i = 0; i < N; i++)
    {
        Matrix[i].SetNum(N);
        for (int32 j = 0; j < N; j++)
        {
            if (i < NumPawns && j < NumTargets)
            {
                FVector Start = Pawns[i]->GetActorLocation();
                FVector End = Targets[j]->GetActorLocation();
                float Cost = bUseAStarDistance ? ComputeAStarDistance(Start, End)
                               : FVector::Dist(Start, End);

                // --- Stability Bias ---
                if (LastAssignments.Contains(Pawns[i]) && LastAssignments[Pawns[i]] == Targets[j])
                {
                    // compute relative weight based on how big the cost is
                    float BiasScale = FMath::Clamp(200.f / Cost, 0.02f, 0.1f); 
                    Cost *= (1.f - BiasScale); 
                }


                Matrix[i][j] = Cost;
            }
            else
            {
                // padding for non-existing pawn/target
                Matrix[i][j] = 999999.f;
            }
        }
    }

    return Matrix;
}

float AAssignmentManager::ComputeAStarDistance(FVector Start, FVector End)
{
    if (!PathfinderRef)
        return FVector::Dist(Start, End);

    TArray<FVector> PathPoints = PathfinderRef->FindFullPath(Start, End);

    // If no path found, return large penalty
    if (PathPoints.Num() == 0)
        return 999999.f;

    float TotalDistance = 0.f;
    FVector Prev = Start;
    for (const FVector& Point : PathPoints)
    {
        TotalDistance += FVector::Dist(Prev, Point);
        Prev = Point;
    }

    return TotalDistance;
}

TArray<int32> AAssignmentManager::HungarianSolve(const TArray<TArray<float>>& CostMatrix)
{
    int32 N = CostMatrix.Num();
    TArray<int32> Assignment;
    Assignment.Init(-1, N);

    TArray<float> u, v;
    TArray<int32> p, way;
    u.Init(0.f, N + 1);
    v.Init(0.f, N + 1);
    p.Init(0, N + 1);
    way.Init(0, N + 1);

    for (int32 i = 1; i <= N; i++)
    {
        p[0] = i;
        int32 j0 = 0;
        TArray<float> minv;
        TArray<bool> used;
        minv.Init(FLT_MAX, N + 1);
        used.Init(false, N + 1);

        do
        {
            used[j0] = true;
            int32 i0 = p[j0];
            float delta = FLT_MAX;
            int32 j1 = 0;

            for (int32 j = 1; j <= N; j++)
            {
                if (!used[j])
                {
                    float cur = CostMatrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j])
                    {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta)
                    {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (int32 j = 0; j <= N; j++)
            {
                if (used[j])
                {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else
                {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do
        {
            int32 j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    for (int32 j = 1; j <= N; j++)
    {
        if (p[j] != 0)
            Assignment[p[j] - 1] = j - 1;
    }

    return Assignment;
}
