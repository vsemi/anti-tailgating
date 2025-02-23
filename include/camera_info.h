/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup CameraInfo
 * @brief CameraInfo of the ToF sensor
 * @ingroup driver
 *
 * @{
 */
#ifndef TOF_CAMERA_INFO_H_
#define TOF_CAMERA_INFO_H_

#include <stdint.h>

//! CameraInfo
/*!
 * This struct holds basic CameraInfo obtained from ToF sensor.
 */

typedef double  _D_type[8];
typedef double  _K_type[9];

/**
* CameraInfo
*/
typedef struct CameraInfo {

	uint32_t height;
	uint32_t width;

	/**
	* The distortion coefficient vector: (k1, k2, p1, p2, k3, k4, k5, k6), 
	*where k1 to k6 are for radial distortion, and the ps are for tangential distortion.
	* 
	* Example: -0.397059, 0.143377, 0, 0, 0, 0, 0, 0
	* 
	*/
	_D_type D;

	/**
	* Intrinsic camera matrix for the raw (distorted) images.
	*     [fx  0 cx]
	* K = [ 0 fy cy]
	*     [ 0  0  1]
	* Projects 3D points in the camera coordinate frame to 2D pixel
	* coordinates using the focal lengths (fx, fy) and principal point
	* (cx, cy)
	* 
	* Example:
	* 
	* 181.296 0         80
	* 0       181.079   30
	* 0       0         1
	* 
	*/
	_K_type K;

} camera_info_t;

#endif /* TOF_CAMERA_INFO_H_ */

/** @} */
