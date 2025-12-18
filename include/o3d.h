/**
 * Copyright (C) 2025 Visionary Semiconductor Inc.
 *
 * @defgroup SpatialObject
 * @brief SpatialObject
 *
 * @{
 */
#ifndef OBJECT_3D_H
#define OBJECT_3D_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point>::Ptr PointCloud;

class Layer {
public:
	int id;

	int n_points;

	float min_x;
	float min_y;
	float max_x;
	float max_y;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	Layer()
	{
		cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		min_x = 7.5;
		min_y = 7.5;
		max_x = 0.0;
		max_y = 0.0;

		n_points = 0;
	}
};

class Cloud {
public:
	pcl::PointXYZRGB peak;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

class SpatialObject {
public:
	int id;
	int partKey;

	float min_x;
	float min_y;
	float min_z;
	float max_x;
	float max_y;
	float max_z;
	float cx;
	float cy;
	float cz;
	float dx;
	float dy;
	float dz;

	float max_dx;
	float max_dy;

	int lost_period;
	float confidence;
	bool validated;
	float distance;
	pcl::PointXYZRGB peak;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	Point origin;

	bool tracked;

	double movement;

	bool is_at_edge;

	int age_tracked;
	float height;

	SpatialObject();
	SpatialObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);
	void update(SpatialObject o);
	void copy(SpatialObject o);
	void recal();
};

#endif // OBJECT_3D_H

/** @} */