#ifndef XUNIT_LIB_H
#define XUNIT_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xunit/xunit_tab_lib.h"
#include <libudev.h>


/* Report Numbers */

#define SET_FAIL		0x02
#define SET_TRUE		0x01
#define SET_SUCCESS		0x01

#define GENERIC_I2C_OPERATION 0x07
#define I2C_READ_LEVEL 0x02 
#define I2C_WRITE_LEVEL 0x01


#define PASS 0
#define FAIL 1
#define SAME_MODE 2
#define HIGH_SPEED 3

#define BUFFER_LENGTH		80 //41
#define TIMEOUT			2000


#define SUCCESS			1
#define FAILURE			0	

#define READFIRMWAREVERSION	0x40
#define GETCAMERA_UNIQUEID	0x41
#define MASTERMODE		0X50
#define TRIGGERMODE		0X51
#define GETTRIGGERMODE		0X91
#define CROPPEDVGAMODE		0X52

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
