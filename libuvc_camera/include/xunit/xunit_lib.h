#ifndef XUNIT_LIB_H
#define XUNIT_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xunit/xunit_tab_lib.h"
#include <libudev.h>


/* Report Numbers */
#define SET_TRUE        0x01
#define SET_FAIL        0x02
#define SET_SUCCESS	    0x01
#define OUT_OF_RANGE    0x03

#define GENERIC_I2C_OPERATION 0x07
#define I2C_READ_LEVEL 0x02 
#define I2C_WRITE_LEVEL 0x01


#define PASS 0
#define FAIL 1
#define SAME_MODE 2
#define HIGH_SPEED 3

#define BUFFER_LENGTH       80
#define TIMEOUT	            2000

#define SET_FAIL_TARA           0x00
#define SET_SUCCESS_TARA        0x01
#define GET_FAIL                0x00
#define GET_SUCCESS			    0x01

#define SUCCESS     1
#define FAILURE     0	

/* Commands */
#define READ_FIRMWARE_VERSION   0x40
#define GET_CAMERA_UNIQUE_ID    0x41

#define MASTER_MODE             0X50
#define TRIGGER_MODE            0X51
#define CROPPED_VGA_MODE        0X52

#define WHILL_CAMERA_CONTROL    0X91
#define CAMERA_CONTROL_STEREO   0x78

#define GET_TRIGGER_MODE        0x01
#define FRAME_RATE_CONTROL      0x02

#define GET_MINIMUM_FRAME_RATE	0x01
#define GET_MAXIMUM_FRAME_RATE	0x02
#define GET_DEFAULT_FRAME_RATE	0x03
#define GET_FRAME_RATE          0x04
#define SET_FRAME_RATE          0x05

/* TARA Specific Commands */
#define GET_EXPOSURE_VALUE			0x01
#define SET_EXPOSURE_VALUE			0x02
#define SET_AUTO_EXPOSURE			0x02
#define SET_HDR_MODE_STEREO     0x0E
#define GET_HDR_MODE_STEREO     0x0F

/* EXPOSURE CONTROL */
#define SEE3CAM_STEREO_EXPOSURE_AUTO	(1)
#define SEE3CAM_STEREO_EXPOSURE_MIN		(10)
#define SEE3CAM_STEREO_EXPOSURE_MAX		(1000000)
#define SEE3CAM_STEREO_EXPOSURE_DEF		(8000)

/* General Value */
#define NULL_HANDLE 0

//uint32_t hid_fd;

unsigned char g_out_packet_buf[BUFFER_LENGTH];
unsigned char g_in_packet_buf[BUFFER_LENGTH];

const char	*hid_device;

/* Function Declarations */

BOOL CalculateCRC();

const char *bus_str(int);

unsigned int GetTickCount();

int find_hid_device(const char *);

#if XUNIT_DEBUGGING
#include <libgen.h>
#define XUNIT_DEBUG(format, ...) fprintf(stderr, "[%s:%d/%s] " format "\n", basename(__FILE__), __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define XUNIT_DEBUG(format, ...)
#endif

#ifdef __cplusplus
}
#endif
#endif
