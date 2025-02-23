/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup ToFImage
 * @brief ToFImage obtain from ToF sensor
 * @ingroup driver
 *
 * @{
 */
#ifndef VSEMI_TOF_IMAGE_H
#define VSEMI_TOF_IMAGE_H

#include <stdint.h>

//! ToFImage
/*!
 * This struct defines ToF image obtained from ToF sensor, which contains data includes raw distance, colored 2D and 3D points array.
 * It can be obtained by subscribing to the signal sig_tof_image_received in ToF_Camera_Driver.
 */
typedef struct ToFImage
{
	int width,      /*! Image width.    */
	    height,     /*! Image height.   */
	    n_points;   /*! Total points of the image. */

	/*!
	* @brief Pointer of an arry float[n_points], raw distance data in unit of meters.
	*
	* Example of reinterpreting to Opencv Mat:
	*
	* Mat depth_mat = Mat(tof_image->height, tof_image->width, CV_32F, tof_image->data_depth);
	*/
	float* data_depth;

	/*!
	* @brief Pointer of an arry uint8_t[n_points], grayscale data.
	*
	* Example of reinterpreting to Opencv Mat:
	*
	* Mat grayscale_mat = Mat(tof_image->height, tof_image->width, CV_8UC1, tof_image->data_grayscale);
	*/
	uint8_t* data_grayscale;

	/*!
	* @brief Pointer of an arry float[n_points], amplitude data.
	*
	* Example of reinterpreting to Opencv Mat:
	*
	* Mat amplitude_mat = Mat(tof_image->height, tof_image->width, CV_32F, tof_image->data_amplitude);
	*/
	float* data_amplitude;

	/*!
	* @brief Pointer of an arry float[n_points * 8], 3D points in XYZRGB format:
	* float x
	* float y
	* float z
	* float 0
	* *reinterpret_cast<float*>(&rgb), which rgb is type of uint32_t
	* *reinterpret_cast<float*>(&r),   which r   is type of uint8_t
	* *reinterpret_cast<float*>(&g),   which g   is type of uint8_t
	* *reinterpret_cast<float*>(&b),   which b   is type of uint8_t
	*
	* Example of reinterpreting to pcl PointCloud:
	*
	* pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	*
	* pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tof_image->data_3d_xyz_rgb);
	* std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tof_image->n_points);
	* point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());
	*
	* point_cloud_ptr->resize(tof_image->n_points);
	* point_cloud_ptr->width = tof_image->n_points;
	* point_cloud_ptr->height = 1;
	* point_cloud_ptr->is_dense = false;
	*/
	float* data_3d_xyz_rgb;

	/*!
	* @brief Pointer of an arry uint8_t[n_points * 3], 3-channel image in BGR format:
	* uint8_t b
	* uint8_t g
	* uint8_t r
	*
	* Example of reinterpreting to Opencv Mat:
	*
	* Mat depth_bgr = Mat(tof_image->height, tof_image->width, CV_8UC3, tof_image->data_2d_bgr);
	*/
	uint8_t* data_2d_bgr;

	/*!
	* @brief Mask of saturated depth map, use this to segment or remove saturated 2D points from depth map if needed.
	*
	* Only avaiable if ignoreSaturatedPoints in Settings set to false.
	*
	* Example of reinterpreting to OpenCV Mat:
	*
	* Mat saturated_mask = Mat(tof_image->height, tof_image->width, CV_8UC1, tof_image->saturated_mask);
	*/
	uint8_t* saturated_mask;

	/*!
	* @param w: image width
	* @param h: image height
	*/
	ToFImage(int w, int h);

	~ToFImage();

} tof_image_t;

#endif // VSEMI_TOF_IMAGE_H

/** @} */
