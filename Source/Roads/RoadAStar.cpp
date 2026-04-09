#include "RoadAStar.h"
#include "PerlinNoise2D.h"

void URoadGenLib::ComputeWorldBounds(
    const TArray<FVector>& Vertices,
    FVector& OutMin,
    FVector& OutMax
)
{
    check(Vertices.Num() > 0);

    OutMin = Vertices[0];
    OutMax = Vertices[0];

    for (const FVector& V : Vertices)
    {
        OutMin.X = FMath::Min(OutMin.X, V.X);
        OutMin.Y = FMath::Min(OutMin.Y, V.Y);

        OutMax.X = FMath::Max(OutMax.X, V.X);
        OutMax.Y = FMath::Max(OutMax.Y, V.Y);
    }
}

float URoadGenLib::EvaluateFitness(
    const TArray<FVector>& Path,
    float NoiseScale,
    float HeightScale,
    float MaxSlope,
    int32 Layer,
    float TargetZ,
    int32 NoiseOffsetX,
    int32 NoiseOffsetY
)
{
    float Cost = 0.f;

    for (int32 i = 0; i < Path.Num() - 1; i++)
    {
        FVector A = Path[i];
        FVector B = Path[i + 1];

        // Wysokosc terenu
        float TerrainH1 = UPerlinNoise2D::UPerlinNoise2D::GetHeight(A.X, A.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);
        float TerrainH2 = UPerlinNoise2D::GetHeight(B.X, B.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);

        float RoadH1, RoadH2;

        if (Layer == 0)
        {
            RoadH1 = TerrainH1;
            RoadH2 = TerrainH2;
        }
        else
        {
            RoadH1 = TargetZ;
            RoadH2 = TargetZ;

            float ClearanceMargin = 100.f;

            if (TerrainH1 > (RoadH1 - ClearanceMargin))
            {
                float Penetration = TerrainH1 - (RoadH1 - ClearanceMargin);
                Cost += 50000.f * (1.f + Penetration / 100.f);
            }

            if (TerrainH2 > (RoadH2 - ClearanceMargin))
            {
                float Penetration = TerrainH2 - (RoadH2 - ClearanceMargin);
                Cost += 50000.f * (1.f + Penetration / 100.f);
            }
        }

        float Dist = FVector::Dist(FVector(A.X, A.Y, RoadH1), FVector(B.X, B.Y, RoadH2));

        float HeightDiff = FMath::Abs(RoadH2 - RoadH1);
        float Slope = HeightDiff / FMath::Max(Dist, 1.f);

        Cost += Dist;

        if (Layer == 0)
        {
            Cost += FMath::Abs(RoadH1) * 0.5f;
        }

        if (Slope > MaxSlope)
            Cost += 10000.f;
    }

    return Cost;
}

void URoadGenLib::MutatePath(
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
)
{
    int32 Index = RNG.RandRange(1, Path.Num() - 2);
    FVector& P = Path[Index];

    P.X += RNG.FRandRange(-500.f, 500.f);
    P.Y += RNG.FRandRange(-500.f, 500.f);

    P.X = FMath::Clamp(P.X, WorldMin.X, WorldMax.X);
    P.Y = FMath::Clamp(P.Y, WorldMin.Y, WorldMax.Y);

    if (Layer > 0)
    {
        P.Z = TargetZ;
    }
    else
    {
        P.Z = UPerlinNoise2D::GetHeight(P.X, P.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);
    }
}

FRoadPath URoadGenLib::GenerateSingleRoadFromVertices(
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
)
{
    FVector WorldMin, WorldMax;
    ComputeWorldBounds(TerrainVertices, WorldMin, WorldMax);

    FRandomStream RNG(Seed);
    TArray<FRoadPath> Population;

    float LayerHeightStep = 200.0f; 
    float TargetZ = Layer * LayerHeightStep;

    for (int32 i = 0; i < PopulationSize; i++)
    {
        FRoadPath Path;
        FVector Start, End;

        auto GetInitialZ = [&](float X, float Y) -> float {
            if (Layer == 0) {
                return UPerlinNoise2D::GetHeight(X, Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);
            }
            return TargetZ;
            };

        float StartX, StartY, EndX, EndY;

        if (bSouthToNorth)
        {
            StartX = RNG.FRandRange(WorldMin.X, WorldMax.X);
            StartY = WorldMin.Y;
            EndX = RNG.FRandRange(WorldMin.X, WorldMax.X);
            EndY = WorldMax.Y;
        }
        else
        {
            StartX = WorldMin.X;
            StartY = RNG.FRandRange(WorldMin.Y, WorldMax.Y);
            EndX = WorldMax.X;
            EndY = RNG.FRandRange(WorldMin.Y, WorldMax.Y);
        }

        Start = FVector(StartX, StartY, GetInitialZ(StartX, StartY));
        End = FVector(EndX, EndY, GetInitialZ(EndX, EndY));

        Path.Points.Add(Start);

        for (int32 p = 0; p < ControlPoints; p++)
        {
            float CX = RNG.FRandRange(WorldMin.X, WorldMax.X);
            float CY = RNG.FRandRange(WorldMin.Y, WorldMax.Y);
            FVector CP(CX, CY, GetInitialZ(CX, CY));
            Path.Points.Add(CP);
        }

        Path.Points.Add(End);

        Path.Fitness = EvaluateFitness(Path.Points, NoiseScale, HeightScale, MaxSlope, Layer, TargetZ, NoiseOffsetX, NoiseOffsetY);

        Population.Add(Path);
    }

    for (int32 G = 0; G < Generations; G++)
    {
        Population.Sort([](const FRoadPath& A, const FRoadPath& B) {
            return A.Fitness < B.Fitness;
            });

        int32 Survivors = PopulationSize / 4;

        for (int32 i = Survivors; i < PopulationSize; i++)
        {
            Population[i] = Population[RNG.RandRange(0, Survivors - 1)];

            MutatePath(
                Population[i].Points,
                RNG,
                WorldMin,
                WorldMax,
                Layer,
                TargetZ,
                NoiseScale,
                HeightScale,
                NoiseOffsetX,
                NoiseOffsetY
            );

            Population[i].Fitness = EvaluateFitness(
                Population[i].Points,
                NoiseScale,
                HeightScale,
                MaxSlope,
                Layer,
                TargetZ,
                NoiseOffsetX,
                NoiseOffsetY
            );
        }
    }

    Population.Sort([](const FRoadPath& A, const FRoadPath& B) {
        return A.Fitness < B.Fitness;
        });

    FRoadPath Path = Population[0];
    Path.Layer = Layer;

    FRoadPath SubdividedPath;
    SubdividedPath.Layer = Layer;

    SubdividePath(Path.Points, 300.f, SubdividedPath.Points, Layer, TargetZ, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);

    FRoadPath SmoothPath;
    SmoothPath.Layer = Layer;

    ChaikinSmoothPath(SubdividedPath.Points, 2, SmoothPath.Points, Layer, TargetZ, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);

    UE_LOG(LogTemp, Warning, TEXT("=== Road Generation ==="));
    UE_LOG(LogTemp, Warning, TEXT("Layer: %d, TargetZ: %.2f"), Layer, TargetZ);
    UE_LOG(LogTemp, Warning, TEXT("NoiseScale: %.4f, HeightScale: %.2f"), NoiseScale, HeightScale);
    UE_LOG(LogTemp, Warning, TEXT("WorldMin: %s, WorldMax: %s"), *WorldMin.ToString(), *WorldMax.ToString());

    return SmoothPath;
}


void URoadGenLib::SubdividePath(
    const TArray<FVector>& InPoints,
    float SampleSpacing,
    TArray<FVector>& OutPoints,
    int32 Layer,
    float TargetZ,
    float NoiseScale,
    float HeightScale,
    int32 NoiseOffsetX,
    int32 NoiseOffsetY
)
{
    OutPoints.Empty();

    if (InPoints.Num() < 2) return;

    for (int32 i = 0; i < InPoints.Num() - 1; i++)
    {
        const FVector& P0 = InPoints[i];
        const FVector& P1 = InPoints[i + 1];

        OutPoints.Add(P0);

        FVector Delta = P1 - P0;
        float DistanceXY = FVector(Delta.X, Delta.Y, 0.f).Size();
        int32 NumSteps = FMath::CeilToInt(DistanceXY / SampleSpacing);

        for (int32 Step = 1; Step < NumSteps; Step++)
        {
            float T = static_cast<float>(Step) / NumSteps;

            FVector NewPoint = FMath::Lerp(P0, P1, T);

            if (Layer == 0)
            {
                NewPoint.Z = UPerlinNoise2D::GetHeight(NewPoint.X, NewPoint.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);
            }
            else
            {
                NewPoint.Z = TargetZ;
            }

            OutPoints.Add(NewPoint);
        }
    }

    OutPoints.Add(InPoints.Last());
}

void URoadGenLib::ChaikinSmoothPath(
    const TArray<FVector>& InPoints,
    int32 Iterations,
    TArray<FVector>& OutPoints,
    int32 Layer,
    float TargetZ,
    float NoiseScale,
    float HeightScale,
    int32 NoiseOffsetX,
    int32 NoiseOffsetY
)
{
    OutPoints = InPoints;

    for (int32 It = 0; It < Iterations; It++)
    {
        if (OutPoints.Num() < 3) return;

        TArray<FVector> Temp;
        Temp.Add(OutPoints[0]);

        for (int32 i = 0; i < OutPoints.Num() - 1; i++)
        {
            const FVector& P0 = OutPoints[i];
            const FVector& P1 = OutPoints[i + 1];

            FVector Q = FMath::Lerp(P0, P1, 0.25f);
            FVector R = FMath::Lerp(P0, P1, 0.75f);

            if (Layer == 0)
            {
                Q.Z = UPerlinNoise2D::GetHeight(Q.X, Q.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);
                R.Z = UPerlinNoise2D::GetHeight(R.X, R.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);
            }
            else
            {
                Q.Z = TargetZ;
                R.Z = TargetZ;
            }

            Temp.Add(Q);
            Temp.Add(R);
        }

        Temp.Add(OutPoints.Last());
        OutPoints = Temp;
    }
}

FVector URoadGenLib::CalculateTangentAtPoint(
    const FRoadPath& Road,
    int32 PointIndex
)
{
    if (Road.Points.Num() < 2)
    {
        return FVector::ForwardVector;
    }

    PointIndex = FMath::Clamp(PointIndex, 0, Road.Points.Num() - 1);

    FVector Tangent;

    if (PointIndex == 0)
    {
        Tangent = Road.Points[1] - Road.Points[0];
    }
    else if (PointIndex == Road.Points.Num() - 1)
    {
        Tangent = Road.Points[PointIndex] - Road.Points[PointIndex - 1];
    }
    else
    {
        FVector TangentForward = Road.Points[PointIndex + 1] - Road.Points[PointIndex];
        FVector TangentBackward = Road.Points[PointIndex] - Road.Points[PointIndex - 1];
        Tangent = (TangentForward + TangentBackward) * 0.5f;
    }

    FVector TangentXY(Tangent.X, Tangent.Y, 0.f);
    TangentXY.Normalize();

    return TangentXY;
}

FVector URoadGenLib::FindNearestPointOnRoad(
    const FVector& SourcePoint,
    const FRoadPath& TargetRoad,
    int32& OutPointIndex
)
{
    if (TargetRoad.Points.Num() == 0)
    {
        OutPointIndex = -1;
        return FVector::ZeroVector;
    }

    float MinDistance = FLT_MAX;
    int32 BestIndex = 0;

    for (int32 i = 0; i < TargetRoad.Points.Num(); i++)
    {
        float Dist2D = FVector::Dist2D(SourcePoint, TargetRoad.Points[i]);

        if (Dist2D < MinDistance)
        {
            MinDistance = Dist2D;
            BestIndex = i;
        }
    }

    OutPointIndex = BestIndex;
    return TargetRoad.Points[BestIndex];
}

TArray<FConnectionPair> URoadGenLib::FindConnectionPairs(
    const FRoadPath& LowerRoad,
    const FRoadPath& UpperRoad,
    int32 NumConnectionPoints,
    float MaxSlope
)
{
    TArray<FConnectionPair> Pairs;

    if (LowerRoad.Points.Num() < 2 || UpperRoad.Points.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("FindConnectionPairs: Drogi maja za malo punktow!"));
        return Pairs;
    }

    if (NumConnectionPoints < 1)
    {
        UE_LOG(LogTemp, Warning, TEXT("FindConnectionPairs: NumConnectionPoints musi byc >= 1"));
        return Pairs;
    }

    int32 NumPoints = LowerRoad.Points.Num();

    for (int32 i = 0; i < NumConnectionPoints; i++)
    {
        float T = (float)(i + 1) / (float)(NumConnectionPoints + 1);

        int32 LowerIndex = FMath::RoundToInt(T * (NumPoints - 1));
        LowerIndex = FMath::Clamp(LowerIndex, 0, NumPoints - 1);

        FConnectionPoint LowerPoint;
        LowerPoint.Position = LowerRoad.Points[LowerIndex];
        LowerPoint.Layer = LowerRoad.Layer;
        LowerPoint.PointIndex = LowerIndex;
        LowerPoint.Tangent = CalculateTangentAtPoint(LowerRoad, LowerIndex);

        int32 UpperIndex;
        FVector UpperPos = FindNearestPointOnRoad(
            LowerPoint.Position,
            UpperRoad,
            UpperIndex
        );

        FConnectionPoint UpperPoint;
        UpperPoint.Position = UpperPos;
        UpperPoint.Layer = UpperRoad.Layer;
        UpperPoint.PointIndex = UpperIndex;
        UpperPoint.Tangent = CalculateTangentAtPoint(UpperRoad, UpperIndex);

        float Dist2D = FVector::Dist2D(LowerPoint.Position, UpperPoint.Position);
        float HeightDiff = FMath::Abs(UpperPoint.Position.Z - LowerPoint.Position.Z);

        float MinHorizontalDist = HeightDiff / MaxSlope;

        float EstimatedLength = FMath::Sqrt(MinHorizontalDist * MinHorizontalDist + HeightDiff * HeightDiff);

        FConnectionPair Pair;
        Pair.Lower = LowerPoint;
        Pair.Upper = UpperPoint;
        Pair.Distance2D = Dist2D;
        Pair.HeightDiff = HeightDiff;
        Pair.EstimatedRampLength = EstimatedLength;

        Pairs.Add(Pair);

        UE_LOG(LogTemp, Log, TEXT("=== Connection Pair %d ==="), i);
        UE_LOG(LogTemp, Log, TEXT("Lower: Layer %d, Index %d, Pos %s"),
            LowerPoint.Layer, LowerPoint.PointIndex, *LowerPoint.Position.ToString());
        UE_LOG(LogTemp, Log, TEXT("Upper: Layer %d, Index %d, Pos %s"),
            UpperPoint.Layer, UpperPoint.PointIndex, *UpperPoint.Position.ToString());
        UE_LOG(LogTemp, Log, TEXT("Distance2D: %.2f, HeightDiff: %.2f"), Dist2D, HeightDiff);
        UE_LOG(LogTemp, Log, TEXT("Estimated Ramp Length: %.2f"), EstimatedLength);

        if (Dist2D < MinHorizontalDist)
        {
            UE_LOG(LogTemp, Warning, TEXT("WARNING: Dystans 2D (%.2f) < Minimalny dystans (%.2f) dla MaxSlope %.2f!"),
                Dist2D, MinHorizontalDist, MaxSlope);
            UE_LOG(LogTemp, Warning, TEXT("Rampa bedzie za stroma! Rozwaz zwiekszenie MaxSlope lub przesuniecie punktow."));
        }
    }

    return Pairs;
}

FIntVector URoadGenLib::WorldToGrid(const FVector& WorldPos, float GridSize)
{
    return FIntVector(
        FMath::RoundToInt(WorldPos.X / GridSize),
        FMath::RoundToInt(WorldPos.Y / GridSize),
        FMath::RoundToInt(WorldPos.Z / GridSize)
    );
}

FVector URoadGenLib::GridToWorld(const FIntVector& GridPos, float GridSize)
{
    return FVector(
        GridPos.X * GridSize,
        GridPos.Y * GridSize,
        GridPos.Z * GridSize
    );
}

float URoadGenLib::CalculateAStarHeuristic(
    const FVector& Current,
    const FVector& Goal,
    float MaxSlope
)
{
    float DirectDist3D = FVector::Dist(Current, Goal);

    float HeightDiff = FMath::Abs(Goal.Z - Current.Z);

    float Dist2D = FVector::Dist2D(Current, Goal);

    float MinHorizontalDist = HeightDiff / FMath::Max(MaxSlope, 0.01f);

    if (Dist2D < MinHorizontalDist)
    {
        float SlopePenalty = (MinHorizontalDist - Dist2D) * 2.0f;
        return DirectDist3D + SlopePenalty + HeightDiff * 0.5f;
    }

    return DirectDist3D + HeightDiff * 0.3f;
}

float URoadGenLib::CalculateMoveCost(
    const FVector& From,
    const FVector& To,
    float MaxSlope,
    int32 Layer,
    float NoiseScale,
    float HeightScale,
    int32 NoiseOffsetX,
    int32 NoiseOffsetY
)
{
    float Distance = FVector::Dist(From, To);
    float Cost = Distance;

    float HorizontalDist = FVector::Dist2D(From, To);
    float VerticalDist = FMath::Abs(To.Z - From.Z);

    if (HorizontalDist > 0.01f)
    {
        float VerticalRatio = VerticalDist / (HorizontalDist + VerticalDist);
        Cost += Distance * VerticalRatio * 1.5f;
    }

    float HeightDiff = FMath::Abs(To.Z - From.Z);
    HorizontalDist = FVector::Dist2D(From, To);

    if (HorizontalDist > 0.01f)
    {
        float Slope = HeightDiff / HorizontalDist;

        if (Slope > MaxSlope)
        {
            Cost += Distance * 10.0f * (Slope / MaxSlope);
        }
        else
        {
            float SlopeRatio = Slope / MaxSlope;
            Cost += Distance * SlopeRatio * 2.0f;
        }
    }

    if (From.Z > 100.f || To.Z > 100.f)
    {
        float TerrainHeight1 = UPerlinNoise2D::GetHeight(From.X, From.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);
        float TerrainHeight2 = UPerlinNoise2D::GetHeight(To.X, To.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);

        float ClearanceMargin = 100.f;

        if (From.Z < (TerrainHeight1 + ClearanceMargin))
        {
            float Penetration = (TerrainHeight1 + ClearanceMargin) - From.Z;
            Cost += 1000.f * (Penetration / 100.f);
        }

        if (To.Z < (TerrainHeight2 + ClearanceMargin))
        {
            float Penetration = (TerrainHeight2 + ClearanceMargin) - To.Z;
            Cost += 1000.f * (Penetration / 100.f);
        }
    }

    return Cost;
}

TArray<FIntVector> URoadGenLib::GetNeighbors(const FIntVector& GridPos)
{
    TArray<FIntVector> Neighbors;
    Neighbors.Reserve(26);

    for (int32 dx = -1; dx <= 1; dx++)
    {
        for (int32 dy = -1; dy <= 1; dy++)
        {
            for (int32 dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dy == 0 && dz == 0) continue;

                if (dx == 0 && dy == 0) continue;

                Neighbors.Add(GridPos + FIntVector(dx, dy, dz));
            }
        }
    }
    return Neighbors;
}

void URoadGenLib::ReconstructPath(
    FAStarNode* EndNode,
    TArray<FVector>& OutPath
)
{
    OutPath.Empty();

    FAStarNode* Current = EndNode;

    while (Current != nullptr)
    {
        OutPath.Add(Current->WorldPos);
        Current = Current->Parent;
    }

    Algo::Reverse(OutPath);
}

void URoadGenLib::SubdivideRampPath(
    const TArray<FVector>& InPoints,
    float SampleSpacing,
    TArray<FVector>& OutPoints
)
{
    OutPoints.Empty();

    if (InPoints.Num() < 2) return;

    for (int32 i = 0; i < InPoints.Num() - 1; i++)
    {
        const FVector& P0 = InPoints[i];
        const FVector& P1 = InPoints[i + 1];

        OutPoints.Add(P0);

        float Distance3D = FVector::Dist(P0, P1);
        int32 NumSteps = FMath::CeilToInt(Distance3D / SampleSpacing);

        for (int32 Step = 1; Step < NumSteps; Step++)
        {
            float T = static_cast<float>(Step) / NumSteps;

            FVector NewPoint = FMath::Lerp(P0, P1, T);

            OutPoints.Add(NewPoint);
        }
    }

    OutPoints.Add(InPoints.Last());
}

void URoadGenLib::CatmullRomSmoothRamp(
    const TArray<FVector>& InPoints,
    float Alpha,
    TArray<FVector>& OutPoints
)
{
    OutPoints.Empty();

    if (InPoints.Num() < 4)
    {
        OutPoints = InPoints;
        return;
    }

    for (int32 i = 0; i < InPoints.Num() - 1; i++)
    {
        FVector P0 = (i == 0) ? InPoints[i] : InPoints[i - 1];
        FVector P1 = InPoints[i];
        FVector P2 = InPoints[i + 1];
        FVector P3 = (i + 2 < InPoints.Num()) ? InPoints[i + 2] : InPoints[i + 1];

        int32 Samples = 10;

        for (int32 s = 0; s < Samples; s++)
        {
            float t = static_cast<float>(s) / Samples;

            float t2 = t * t;
            float t3 = t2 * t;

            FVector Point =
                P1 +
                0.5f * (
                    (-P0 + P2) * t +
                    (2.f * P0 - 5.f * P1 + 4.f * P2 - P3) * t2 +
                    (-P0 + 3.f * P1 - 3.f * P2 + P3) * t3
                    );

            OutPoints.Add(Point);
        }
    }

    OutPoints.Add(InPoints.Last());
}

FRoadPath URoadGenLib::GenerateRampWithAStar(
    const FConnectionPair& Connection,
    float GridSize,
    float MaxSlope,
    float NoiseScale,
    float HeightScale,
    float MaxRampLength,
    int32 NoiseOffsetX,
    int32 NoiseOffsetY
)
{
    FRoadPath ResultPath;

    FVector StartPos = Connection.Lower.Position;
    FVector GoalPos = Connection.Upper.Position;

    int32 Layer = FMath::Max(Connection.Lower.Layer, Connection.Upper.Layer);
    ResultPath.Layer = Layer;

    UE_LOG(LogTemp, Log, TEXT("=== A* Ramp Generation ==="));
    UE_LOG(LogTemp, Log, TEXT("Start: %s (Layer %d)"), *StartPos.ToString(), Connection.Lower.Layer);
    UE_LOG(LogTemp, Log, TEXT("Goal: %s (Layer %d)"), *GoalPos.ToString(), Connection.Upper.Layer);
    UE_LOG(LogTemp, Log, TEXT("GridSize: %.2f, MaxSlope: %.2f"), GridSize, MaxSlope);

    TMap<FIntVector, FAStarNode*> OpenSet;

    TSet<FIntVector> ClosedSet;

    FIntVector StartGrid = WorldToGrid(StartPos, GridSize);
    FAStarNode* StartNode = new FAStarNode(StartGrid, StartPos);
    StartNode->GCost = 0.f;
    StartNode->HCost = CalculateAStarHeuristic(StartPos, GoalPos, MaxSlope);
    StartNode->FCost = StartNode->GCost + StartNode->HCost;

    OpenSet.Add(StartGrid, StartNode);

    FIntVector GoalGrid = WorldToGrid(GoalPos, GridSize);

    FAStarNode* GoalNode = nullptr;
    int32 Iterations = 0;
    int32 MaxIterations = 100000;

    while (OpenSet.Num() > 0 && Iterations < MaxIterations)
    {
        Iterations++;

        FAStarNode* Current = nullptr;
        float LowestF = FLT_MAX;
        FIntVector CurrentGridPos;

        for (auto& Pair : OpenSet)
        {
            if (Pair.Value->FCost < LowestF)
            {
                LowestF = Pair.Value->FCost;
                Current = Pair.Value;
                CurrentGridPos = Pair.Key;
            }
        }

        if (Current == nullptr)
            break;

        if (Current->GridPos == GoalGrid)
        {
            GoalNode = Current;
            UE_LOG(LogTemp, Log, TEXT("A* SUCCESS! Found path in %d iterations"), Iterations);
            break;
        }

        if (Current->GCost > MaxRampLength)
        {
            UE_LOG(LogTemp, Warning, TEXT("A* reached max ramp length (%.2f)"), MaxRampLength);
            break;
        }


        OpenSet.Remove(CurrentGridPos);
        ClosedSet.Add(CurrentGridPos);

        TArray<FIntVector> Neighbors = GetNeighbors(Current->GridPos);

        for (const FIntVector& NeighborGrid : Neighbors)
        {
            if (ClosedSet.Contains(NeighborGrid))
                continue;

            FVector NeighborWorld = GridToWorld(NeighborGrid, GridSize);

            float MoveCost = CalculateMoveCost(
                Current->WorldPos,
                NeighborWorld,
                MaxSlope,
                Layer,
                NoiseScale,
                HeightScale,
                NoiseOffsetX,
                NoiseOffsetY
            );

            float TentativeG = Current->GCost + MoveCost;

            FAStarNode** ExistingPtr = OpenSet.Find(NeighborGrid);

            if (ExistingPtr == nullptr)
            {
                FAStarNode* NewNode = new FAStarNode(NeighborGrid, NeighborWorld);
                NewNode->GCost = TentativeG;
                NewNode->HCost = CalculateAStarHeuristic(NeighborWorld, GoalPos, MaxSlope);
                NewNode->FCost = NewNode->GCost + NewNode->HCost;
                NewNode->Parent = Current;

                OpenSet.Add(NeighborGrid, NewNode);
            }
            else
            {
                FAStarNode* ExistingNode = *ExistingPtr;
                if (TentativeG < ExistingNode->GCost)
                {
                    ExistingNode->GCost = TentativeG;
                    ExistingNode->FCost = ExistingNode->GCost + ExistingNode->HCost;
                    ExistingNode->Parent = Current;
                }
            }
        }
    }

    if (GoalNode != nullptr)
    {
        TArray<FVector> RawPath;
        ReconstructPath(GoalNode, RawPath);

        UE_LOG(LogTemp, Log, TEXT("Raw path points: %d"), RawPath.Num());

        TArray<FVector> SimplifiedPath;
        SimplifyPath(RawPath, SimplifiedPath, MaxSlope, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);

        UE_LOG(LogTemp, Log, TEXT("Simplified path points: %d"), SimplifiedPath.Num());

        TArray<FVector> SubdividedPath;
        SubdivideRampPath(SimplifiedPath, 50.f, SubdividedPath);

        UE_LOG(LogTemp, Log, TEXT("Subdivided path points: %d"), SubdividedPath.Num());

        CatmullRomSmoothRamp(SubdividedPath, 0.5f, ResultPath.Points);

        ResultPath.Fitness = GoalNode->GCost;

        UE_LOG(LogTemp, Log, TEXT("Final smoothed points: %d"), ResultPath.Points.Num());
        UE_LOG(LogTemp, Log, TEXT("Total ramp length: %.2f"), ResultPath.Fitness);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("A* FAILED to find path after %d iterations!"), Iterations);

        ResultPath.Points.Empty();
        ResultPath.Fitness = FLT_MAX;
    }

    for (auto& Pair : OpenSet)
    {
        delete Pair.Value;
    }

    return ResultPath;
}

void URoadGenLib::SimplifyPath(
    const TArray<FVector>& InPoints,
    TArray<FVector>& OutPoints,
    float MaxSlope,
    float NoiseScale,    
    float HeightScale,
    int32 NoiseOffsetX,
    int32 NoiseOffsetY
)
{
    if (InPoints.Num() < 3)
    {
        OutPoints = InPoints;
        return;
    }

    OutPoints.Empty();
    OutPoints.Add(InPoints[0]);

    int32 CurrentIdx = 0;

    while (CurrentIdx < InPoints.Num() - 1)
    {
        int32 NextIdx = CurrentIdx + 1;

        for (int32 LookAhead = CurrentIdx + 2; LookAhead < InPoints.Num(); LookAhead++)
        {
            FVector Start = InPoints[CurrentIdx];
            FVector End = InPoints[LookAhead];

            float Dist2D = FVector::Dist2D(Start, End);
            float HeightDiff = FMath::Abs(End.Z - Start.Z);

            if (Dist2D < 1.0f) continue;

            float Slope = HeightDiff / Dist2D;

            if (Slope > MaxSlope) break;

            FVector MidPoint = (Start + End) * 0.5f;
            float TerrainH = UPerlinNoise2D::GetHeight(MidPoint.X, MidPoint.Y, NoiseScale, HeightScale, NoiseOffsetX, NoiseOffsetY);

            if (MidPoint.Z < TerrainH + 50.0f && Start.Z > TerrainH && End.Z > TerrainH)
            {
                break;
            }

            NextIdx = LookAhead;
        }

        OutPoints.Add(InPoints[NextIdx]);
        CurrentIdx = NextIdx;
    }
}