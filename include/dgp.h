#ifndef DGP_H
#define DGP_H

#include <string>

#include <opencv2/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void read_frame(std::string filename, cv::Mat* depth_map, cv::Mat* grayscale);
void to_point_cloud(cv::Mat depth_mat, cv::Mat grayscale_mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &depth_bgr, cv::Mat_<cv::Point3f> &point3f_mat_, int pointCloudColor, float *data_background);

#endif
