// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PerlinNoise2D.generated.h"

UCLASS()
class ROADS_API UPerlinNoise2D : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
	public:
		UFUNCTION(BlueprintPure, Category = "Math|Noise")
		static float GetHeight(float X, float Y, float NoiseScale, float HeightScale, int32 NoiseOffsetX, int32 NoiseOffsetY);
};
