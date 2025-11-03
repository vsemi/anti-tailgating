/**
 * Copyright (C) 2025 Visionary Semiconductor Inc.
 *
 * @defgroup lib
 * @brief people count
 *
 * @{
 */
#ifndef PEOPLE_COUNTING_H
#define PEOPLE_COUNTING_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/imgproc.hpp>

#include "ToFImage.hpp"
#include "o3d.h"

size_t read(std::string path, float *background);
void write(std::string path, float *background);
void train(float *data, float *background);
void train_background(
	ToFImage* tofImage, float *background, int &wings, float &wing_angle
);

void people_count(
	ToFImage* tofImage, 
	float* background,
	float floor_height,
	bool with_wing,
	int wing_init_bars,
	int wing_init_angle,
	bool &presented,
	float &wing_position,
	std::vector<SpatialObject> &tracked_objects,
	int &detected, int &tracked,
	int &persons,
	int &state
);

#endif // PEOPLE_COUNTING_H

/** @} */