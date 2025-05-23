#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <utils.h>
#include "tracking.h"

size_t read(std::string path, float *background);
void write(std::string path, float *background);
void train(float *data, float *background);
int detect(
	Cloud cloud_source, 
	std::vector<SpatialObject> &parts_validated
);

int track(
	std::vector<SpatialObject> parts_validated, 
	int &in, int &out, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_detected,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_centers,	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_debug,
	std::vector<SpatialObject> &tracked_objects,
	float floor_height
);