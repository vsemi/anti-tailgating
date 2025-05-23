#pragma once

#include <stdint.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//const float SPLIT_MIN_DIS = 0.06;
//const float SPLIT_MIN_DIS = 0.035;
const float SPLIT_MIN_DIS = 0.0375;

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

int clustering_cloud(
	Cloud input_cloud, 
	std::vector<Cloud> &cloud_clusters,
	float dis, int min_size, int max_size
);

void split_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int axis, float value, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2);
void clustering_split(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
	float peak_z,
	float split_z,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &split_parts
);
void filter_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int meanK);
void filterAndDownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, Cloud &filteredCloud, float *data_background);
