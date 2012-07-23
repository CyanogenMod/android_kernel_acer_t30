#ifndef ___YUV_SENSOR_H__
#define ___YUV_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

/*-------------------------------------------Important---------------------------------------
 * for changing the SENSOR_NAME, you must need to change the owner of the device. For example
 * Please add /dev/mt9d115 0600 media camera in below file
 * ./device/nvidia/ventana/ueventd.ventana.rc
 * Otherwise, ioctl will get permission deny
 * -------------------------------------------Important--------------------------------------
*/

#define SENSOR_NAME     "mt9d115"
#define DEV(x)          "/dev/"x
#define SENSOR_PATH     DEV(SENSOR_NAME)
#define LOG_NAME(x)     "ImagerODM-"x
#define LOG_TAG         LOG_NAME(SENSOR_NAME)

#define SENSOR_WAIT_MS       0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END     1 /* special number to indicate this is end of table */
#define SENSOR_MAX_RETRIES   3 /* max counter for retry I2C access */

#define SENSOR_IOCTL_SET_MODE           _IOW('o', 1, struct sensor_mode)
#define SENSOR_IOCTL_GET_STATUS         _IOR('o', 2, __u8)
#define SENSOR_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, int)
#define SENSOR_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, int)
#define SENSOR_IOCTL_SET_SCENE_MODE     _IOW('o', 5, __u8)
#define SENSOR_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define SENSOR_IOCTL_GET_AF_STATUS      _IOW('o', 7, __u8)
#define SENSOR_IOCTL_SET_EXPOSURE       _IOW('o', 8, int)
#define SENSOR_IOCTL_GET_EXPOSURE_TIME  _IOW('o', 9, unsigned int)

// must match with YUVCustomInfoEnum in nvomxcamerasettingsparser.h
enum {
	// start from 1 in case atoi() encounters a string
	// with no numerical sequence and returns 0
	YUV_WhiteBalance = 2,
	YUV_ColorEffect,
	YUV_Exposure,
};

// must match with NvOmxCameraUserWhitebalanceEnum in nvomxcamerasettingsparser.h
enum {
	YUV_Whitebalance_Auto =1,
	YUV_Whitebalance_Incandescent = 2,
	YUV_Whitebalance_Fluorescent =3,
	YUV_Whitebalance_Daylight =5,
	YUV_Whitebalance_CloudyDaylight =6,
};

// must match with NvOmxCameraUserColorEffect in nvomxcamerasettingsparser.h
enum {
	YUV_ColorEffect_Mono     = 3,
	YUV_ColorEffect_Negative = 4,
	YUV_ColorEffect_None     = 5,
	YUV_ColorEffect_Sepia    = 7,
	YUV_ColorEffect_Solarize = 8,
};

// must match with the exposure value in programExposureYUV() in nvomxcamerasettings.cpp
enum {
	YUV_Exposure_Minus_Two,
	YUV_Exposure_Minus_One,
	YUV_Exposure_Zero,
	YUV_Exposure_Plus_One,
	YUV_Exposure_Plus_Two,
};

struct sensor_mode {
	int xres;
	int yres;
};

#ifdef __KERNEL__
struct yuv_sensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __YUV_SENSOR_H__ */
