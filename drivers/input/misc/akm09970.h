/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * Copyright (c) 2014-2020, Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef _UAPI_MISC_AKM09970_H
#define _UAPI_MISC_AKM09970_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define AKM_INPUT_DEVICE_NAME	"hall_sensor"
#define AKM_MISCDEV_NAME		"akm09970_dev"

/* Device specific constant values */
#define AK09970_REG_WIA                 0x00


#define AK09970_REG_WORD_ST             0x10
#define AK09970_REG_ST_XYZ              0x17

#define AK09970_REG_CNTL1               0x20
#define AK09970_REG_CNTL2               0x21
#define AK09970_REG_THRESHOLD           0x22
#define AK09970_REG_TH2X                0x23
#define AK09970_REG_TH1Y                0x24
#define AK09970_REG_TH2Y                0x25
#define AK09970_REG_TH1Z                0x26
#define AK09970_REG_TH2Z                0x27
#define AK09970_REG_RESET               0x30

#define AK09970_RESET_DATA              0x01

#define AK09970_WIA1_VALUE              0x48
#define AK09970_WIA2_VALUE              0xC0

#define AK09970_MODE_POWERDOWN          0x00
#define AK09970_MODE_SINGLE_MEASUREMENT 0x01 /* Single Mode */
#define AK09970_MODE_CONTINUOUS_1HZ     0x06 /* 1Hz */
#define AK09970_MODE_CONTINUOUS_10HZ    0x08 /* 10Hz */
#define AK09970_MODE_CONTINUOUS_20HZ    0x0A /* 20Hz */
#define AK09970_MODE_CONTINUOUS_50HZ    0x0C /* 50Hz */
#define AK09970_MODE_CONTINUOUS_100HZ   0x0E /* 100Hz */

#define AKM_SENSOR_INFO_SIZE        2
#define AKM_SENSOR_CONF_SIZE        3
#define AKM_SENSOR_DATA_SIZE        8

#define AKM_SENSOR_THRESHOLD_SIZE   24

#define AK09970_MODE_POS        0
#define AK09970_MODE_MSK        0x0F
#define AK09970_MODE_REG        AK09970_REG_CNTL2

#define AK09970_SDR_MODE_POS    4
#define AK09970_SDR_MODE_MSK    0x10
#define AK09970_SDR_MODE_REG    AK09970_REG_CNTL2

#define AK09970_SMR_MODE_POS    5
#define AK09970_SMR_MODE_MSK    0x20
#define AK09970_SMR_MODE_REG    AK09970_REG_CNTL2


#define AKM_DRDY_IS_HIGH(x)         ((x) & 0x01)
#define AKM_DOR_IS_HIGH(x)          ((x) & 0x02)
#define AKM_ERRADC_IS_HIGH(x)       ((x) & 0x01)
#define AKM_ERRXY_IS_HIGH(x)        ((x) & 0x80)

#define AK09970_SENS_Q16    ((int32_t)(72090))  /* 1.1uT in Q16 format */

struct akm09970_threshold {
	union {
		struct {
			uint16_t bop1x, brp1x;
			uint16_t bop2x, brp2x;
			uint16_t bop1y, brp1y;
			uint16_t bop2y, brp2y;
			uint16_t bop1z, brp1z;
			uint16_t bop2z, brp2z;
		};
		uint8_t reg[24];
	};
};

/* ioctl numbers */
#define AKM_SENSOR 0xD4
/* AFU devices */
#define AKM_IOC_SET_ACTIVE			_IOW(AKM_SENSOR, 0x00, uint8_t)
#define AKM_IOC_SET_MODE			_IOW(AKM_SENSOR, 0x01, uint8_t)
#define AKM_IOC_SET_PRESS			_IOW(AKM_SENSOR, 0x02, uint8_t)
#define AKM_IOC_GET_SENSEDATA		_IOR(AKM_SENSOR, 0x10, int[4])
#define AKM_IOC_GET_SENSSMR			_IOR(AKM_SENSOR, 0x11, uint8_t)
#define AKM_IOC_SET_THRESHOLD    	_IOW(AKM_SENSOR, 0x12, struct akm09970_threshold)
#define AKM_IOC_GET_THRESHOLD    	_IOW(AKM_SENSOR, 0x15, struct akm09970_threshold)

#endif /* _UAPI_MISC_AKM09970_H */
