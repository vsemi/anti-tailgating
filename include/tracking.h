#pragma once

#include <stdint.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/io.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point>::Ptr PointCloud;

static int next_object_id = 0;

const float PART_MIN_SIZE = 0.05;
const float PART_MIN_SIZE_Y = 0.05;

const float OBJECT_VALID_SIZE = 0.085;

const float OBJECT_MIN_SIZE = 0.12;
const float OBJECT_MIN_SIZE_Y = 0.05;

const float OBJECT_NORM_SIZE = 0.4;
const float OBJECT_NORM_SIZE_Y = 0.27;

const float OBJECT_MIN_DIS = 0.25;

const float Y_EDGE_ANGLE   = 0.15; // tg 8.5 degrees
const float Y_CENTER_EDGE_ANGLE = 0.113935608; // tg 6.5 degrees //0.087488664; // tg 5 degrees

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
	bool from_split;
	bool validated;
	float distance;
	pcl::PointXYZRGB peak;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	Point origin;

	bool tracked;

	double movement;

	bool is_at_edge;

	bool is_new;

	SpatialObject();
	SpatialObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);
	void update(SpatialObject o);
	void copy(SpatialObject o);
	void recal();
};

void refersh_track_objects(std::vector<SpatialObject> &tracked_objects);

void find_and_track_object(std::vector<SpatialObject> &tracked_objects, std::vector<SpatialObject> tracked_candidates, SpatialObject &o, int queque_for_track, bool &swap, int &swapKey);

void audit_track_objects(std::vector<SpatialObject> &tracked_objects, int &in, int &out);
