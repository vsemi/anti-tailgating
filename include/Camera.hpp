/**
 * Copyright (C) 2021 Visionary Semiconductor Inc.
 *
 * @defgroup Camera
 * @brief camera interface
 * @ingroup driver
 *
 * @{
 */
#ifndef TOF_CAMERA_H_
#define TOF_CAMERA_H_

#include <string>
#include <memory>

#include <ToFImage.hpp>
#include "camera_info.h"
#include "common.h"

class Camera
{
public:
	static Camera* usb_tof_camera_160(std::string port);

	virtual ~Camera() throw ();

	virtual bool open() = 0;

	virtual void startStream() = 0;
	virtual ErrorNumber_e stopStream() = 0;

	virtual ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId) = 0;
	virtual ErrorNumber_e getIdentification(unsigned int &device, unsigned int &version) = 0;
	virtual ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor) = 0;

	virtual ErrorNumber_e getTemperature(int16_t &temperature) = 0;

	virtual ErrorNumber_e getDistance(ToFImage &tofImage) = 0;
	virtual ErrorNumber_e getDistanceGrayscale(ToFImage &tofImage) = 0;
	virtual ErrorNumber_e getDistanceAmplitude(ToFImage &tofImage) = 0;

	virtual ErrorNumber_e getLensCalibrationData(CameraInfo &cameraInfo) = 0;

	virtual ErrorNumber_e setOperationMode(OperationMode_e mode) = 0;
	virtual void setAcquisitionMode(AcquisitionMode_e mode) = 0;
	virtual ErrorNumber_e setOffset(const int offset) = 0;
	virtual ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime) = 0;
	virtual ErrorNumber_e setAutoIntegrationTime3d() = 0;

	virtual ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime) = 0;
	virtual ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency) = 0;
	virtual ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor) = 0;
	virtual ErrorNumber_e setFilterSpot(const unsigned int threshold, const unsigned int factor) = 0;
	virtual ErrorNumber_e setDcsFilter(const bool enabled) = 0;
	virtual ErrorNumber_e setGaussianFilter(const bool enabled) = 0;
	virtual ErrorNumber_e setCalibrationMode(const bool enabled) = 0;

	virtual ErrorNumber_e setMinimalAmplitude(const unsigned index, const unsigned int amplitude) = 0;
	virtual ErrorNumber_e setBinning(const int binning) = 0;
	virtual ErrorNumber_e setFrameRate(const unsigned int FrameTime) = 0;
	virtual ErrorNumber_e setHdr(HDR_e hdr) = 0;
	virtual ErrorNumber_e setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax) = 0;

	virtual ErrorNumber_e setModulationChannel(const bool autoChannelEnabled, const int channel) = 0;

	virtual void setRange(int start, int stop) = 0;

	uint16_t getChipId();
	uint16_t getWaferId();
	uint32_t getID();
	int getWidth();
	int getHeight();
	virtual void setFoV(double angle_x, double angle_y) = 0;
	
	virtual void setBackground(float *background, bool removeBackground) = 0;

protected:
	double _angle_x;
	double _angle_y;

	void setWidth(int width, int height);
	void setID(uint16_t chipId, uint16_t waferId);

private:
	uint16_t _chipId;
	uint16_t _waferId;
	uint32_t _sensorId;

	int _width;
	int _height;
};

#endif /* TOF_CAMERA_H_ */

/** @} */
