#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RoadAStar.generated.h"

USTRUCT(BlueprintType)
struct ROADS_API FRoadPath
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    TArray<FVector> Points;

    UPROPERTY(BlueprintReadWrite)
    float Fitness;

    UPROPERTY(BlueprintReadWrite)
    int32 Layer;
};

USTRUCT(BlueprintType)
struct ROADS_API FConnectionPoint
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    FVector Position;

    UPROPERTY(BlueprintReadWrite)
    int32 Layer;

    UPROPERTY(BlueprintReadWrite)
    int32 PointIndex;

    UPROPERTY(BlueprintReadWrite)
    FVector Tangent;

    FConnectionPoint()
        : Position(FVector::ZeroVector)
        , Layer(0)
        , PointIndex(0)
        , Tangent(FVector::ForwardVector)
    {}
};

USTRUCT(BlueprintType)
struct ROADS_API FConnectionPair
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    FConnectionPoint Lower;

    UPROPERTY(BlueprintReadWrite)
    FConnectionPoint Upper;

    UPROPERTY(BlueprintReadWrite)
    float Distance2D;

    UPROPERTY(BlueprintReadWrite)
    float HeightDiff;

    UPROPERTY(BlueprintReadWrite)
    float EstimatedRampLength;

    FConnectionPair()
        : Distance2D(0.f)
        , HeightDiff(0.f)
        , EstimatedRampLength(0.f)
    {}
};

struct FAStarNode
{
    FIntVector GridPos;
    FVector WorldPos;
    float GCost;
    float HCost;
    float FCost;
    FAStarNode* Parent;

    FAStarNode()
        : GridPos(FIntVector::ZeroValue)
        , WorldPos(FVector::ZeroVector)
        , GCost(0.f)
        , HCost(0.f)
        , FCost(0.f)
        , Parent(nullptr)
    {}

    FAStarNode(FIntVector InGridPos, FVector InWorldPos)
        : GridPos(InGridPos)
        , WorldPos(InWorldPos)
        , GCost(0.f)
        , HCost(0.f)
        , FCost(0.f)
        , Parent(nullptr)
    {}

    bool operator==(const FAStarNode& Other) const
    {
        return GridPos == Other.GridPos;
    }

    friend uint32 GetTypeHash(const FAStarNode& Node)
    {
        return HashCombine(HashCombine(
            GetTypeHash(Node.GridPos.X),
            GetTypeHash(Node.GridPos.Y)),
            GetTypeHash(Node.GridPos.Z));
    }
};

UCLASS()
class ROADS_API URoadGenLib : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "Procedural Roads")
    static FRoadPath GenerateSingleRoadFromVertices(
        int32 Seed,
        const TArray<FVector>& TerrainVertices,
        bool bSouthToNorth,
        int32 PopulationSize,
        int32 Generations,
        int32 ControlPoints,
        float NoiseScale,
        float HeightScale,
        float MaxSlope,
        int32 Layer,
        int32 NoiseOffsetX,
        int32 NoiseOffsetY
    );

    UFUNCTION(BlueprintCallable, Category = "Road Generation")
    static TArray<FConnectionPair> FindConnectionPairs(
        const FRoadPath& LowerRoad,
        const FRoadPath& UpperRoad,
        int32 NumConnectionPoints = 2,
        float MaxSlope = 0.15f
    );

    UFUNCTION(BlueprintCallable, Category = "Road Generation")
    static FVector CalculateTangentAtPoint(
        const FRoadPath& Road,
        int32 PointIndex
    );

    UFUNCTION(BlueprintCallable, Category = "Road Generation")
    static FVector FindNearestPointOnRoad(
        const FVector& SourcePoint,
        const FRoadPath& TargetRoad,
        int32& OutPointIndex
    );

    UFUNCTION(BlueprintCallable, Category = "Road Generation")
    static FRoadPath GenerateRampWithAStar(
        const FConnectionPair& Connection,
        float GridSize = 50.f,
        float MaxSlope = 0.15f,
        float NoiseScale = 0.1f,
        float HeightScale = 500.f,
        float MaxRampLength = 5000.f,
        int32 NoiseOffsetX = 0,
        int32 NoiseOffsetY = 0
    );

private:
    static void ComputeWorldBounds(
        const TArray<FVector>& Vertices,
        FVector& OutMin,
        FVector& OutMax
    );

    static float EvaluateFitness(
        const TArray<FVector>& Path,
        float NoiseScale,
        float HeightScale,
        float MaxSlope,
        int32 Layer,
        float TargetZ,
        int32 NoiseOffsetX,
        int32 NoiseOffsetY
    );

    static void MutatePath(
        TArray<FVector>& Path,
        FRandomStream& RNG,
        const FVector& WorldMin,
        const FVector& WorldMax,
        int32 Layer,
        float TargetZ,
        float NoiseScale,
        float HeightScale,
        int32 NoiseOffsetX,
        int32 NoiseOffsetY
    );

    static void SubdividePath(
        const TArray<FVector>& InPoints,
        float SampleSpacing,
        TArray<FVector>& OutPoints,
        int32 Layer,
        float TargetZ,
        float NoiseScale,
        float HeightScale,
        int32 NoiseOffsetX,
        int32 NoiseOffsetY
    );

    static void ChaikinSmoothPath(
        const TArray<FVector>& InPoints,
        int32 Iterations,
        TArray<FVector>& OutPoints,
        int32 Layer,
        float TargetZ,
        float NoiseScale,
        float HeightScale,
        int32 NoiseOffsetX,
        int32 NoiseOffsetY
    );

    static FIntVector WorldToGrid(const FVector& WorldPos, float GridSize);

    static FVector GridToWorld(const FIntVector& GridPos, float GridSize);

    static float CalculateAStarHeuristic(
        const FVector& Current,
        const FVector& Goal,
        float MaxSlope
    );

    static float CalculateMoveCost(
        const FVector& From,
        const FVector& To,
        float MaxSlope,
        int32 Layer,
        float NoiseScale,
        float HeightScale,
        int32 NoiseOffsetX,
        int32 NoiseOffsetY
    );

    static TArray<FIntVector> GetNeighbors(const FIntVector& GridPos);

    static void ReconstructPath(
        FAStarNode* EndNode,
        TArray<FVector>& OutPath
    );

    static void ChaikinSmoothRamp(
        const TArray<FVector>& InPoints,
        int32 Iterations,
        TArray<FVector>& OutPoints
    );

    static void SubdivideRampPath(
        const TArray<FVector>& InPoints,
        float SampleSpacing,
        TArray<FVector>& OutPoints
    );

    static void CatmullRomSmoothRamp(
        const TArray<FVector>& InPoints,
        float Alpha,
        TArray<FVector>& OutPoints
    );

    static void SimplifyPath(
        const TArray<FVector>& InPoints,
        TArray<FVector>& OutPoints,
        float MaxSlope,
        float NoiseScale,
        float HeightScale,
        int32 NoiseOffsetX,
        int32 NoiseOffsetY
    );
};