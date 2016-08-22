#pragma once

#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

const double pi = std::acos(-1);

inline float deg2rad(float angle)
{
	return angle * 180 / pi;
}
