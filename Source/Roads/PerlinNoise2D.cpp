// Fill out your copyright notice in the Description page of Project Settings.


#include "PerlinNoise2D.h"
#include "Math/UnrealMathUtility.h"

float  UPerlinNoise2D::GetHeight(float X, float Y, float NoiseScale, float HeightScale, int32 NoiseOffsetX, int32 NoiseOffsetY)
{
    float SampleX = (X + NoiseOffsetX) * NoiseScale;
    float SampleY = (Y + NoiseOffsetY) * NoiseScale;

    float N = FMath::PerlinNoise2D(FVector2D(SampleX, SampleY));
    return N * HeightScale;
}