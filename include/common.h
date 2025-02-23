/**
 * Copyright (C) 2021 Visionary Semiconductor Inc.
 *
 * @defgroup Camera160
 * @brief common enums
 * @ingroup driver
 *
 * @{
 */
#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

enum ModulationFrequency_e
{
	MODULATION_FREQUENCY_10MHZ = 0,
	MODULATION_FREQUENCY_20MHZ = 1
};

enum OperationMode_e
{
	MODE_BEAM_A = 0,                                        ///<Normal operation with illumination beam A, Wide Field
	MODE_BEAM_B = 1                                         ///<Normal operation with illumination beam B, Narrow Field
};

enum AcquisitionMode_e
{
	SINGLE = 0,
	AUTO_REPEAT = 1,
	STREAM = 2
};

enum HDR_e
{
	HDR_OFF = 0,
	HDR_SPATIAL = 1,
	HDR_TEMPORAL = 2
};

enum ErrorNumber_e
{
	ERROR_NUMMBER_NO_ERROR = 0,
	ERROR_NUMBER_INVALID_PARAMETER = 32770,
	ERROR_NUMBER_IO_ERROR = 32771,
	ERROR_NUMBER_INVALID_DATA = 32772
};

#endif /* INCLUDE_COMMON_H_ */
