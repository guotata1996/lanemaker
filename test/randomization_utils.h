#pragma once

#include "xodr/road.h"
#include "xodr/junction.h"

#include "spdlog/spdlog.h"

// Inclusive
int RandomIntBetween(int low, int hi);

// Sorted, Containing low and hi
std::vector<int> RandomSortedVector(int low, int hi, uint32_t count);

RoadRunner::RoadProfile GenerateConfig(int seed, uint32_t length);

