#ifndef XUNIT_TAB_LIB_H
#define XUNIT_TAB_LIB_H

#include <stdbool.h>
#include <stdint.h>

typedef bool                BOOL;
typedef int8_t              INT8;
typedef int16_t             INT16;
typedef int32_t             INT32;
typedef unsigned char       UINT8;
typedef unsigned short int  UINT16;
typedef unsigned int        UINT32;

#define FALSE 0
#define TRUE 1

/* VID and PID of SEE3CAM Products */
#define SEE3CAM_USB_VID			        0x2560
#define SEE3CAM_1MSTEREO_MONO_PID    	0xC110
#define SEE3CAM_1MSTEREO_COLOUR_PID 	0xC111
#define SEE3CAM_STEREO_TARA_PID         0xC114


#define UVC_RC_UNDEFINED                                0x00
#define UVC_SET_CUR                                     0x01
#define UVC_GET_CUR                                     0x81
#define UVC_GET_MIN                                     0x82
#define UVC_GET_MAX                                     0x83
#define UVC_GET_LEN                                     0x85
#define UVC_GET_INFO                                    0x86
#define EXTENSION_UNIT_ID                               3

extern void UpdatePanTilt();
bool g_flash_flag;
bool g_external_trigger_flag;

enum see3cam_device_index
{
  SEE3CAM_1MSTEREO_MONO = 1,
  SEE3CAM_1MSTEREO_COLOUR,
};

typedef struct g_FWVER
{
  uint8_t pMajorVersion;
  uint8_t pMinorVersion1;
  uint16_t pMinorVersion2;
  uint16_t pMinorVersion3;
} g_FWver_t;


typedef struct g_sensor_temp
{
  uint8_t integer_num;
  uint8_t fractional_num;
}g_rst_t;



typedef enum g_Mode
{
  ENABLE = 1,
  DISABLE
} g_Mode_t;

typedef struct g_uvcBulkStreamStats {    
  unsigned int glStatPauseCnt;
  unsigned int glStatCommitFailCnt;
  unsigned int hidlnTokenFlushes;
} g_uvcBulkStreamStats_t;
/* Function Declarations */

typedef struct g_CPLDversion
{
  uint32_t idcode;
  uint32_t usercode;
} g_CPLDversion_t;

uint32_t InitExtensionUnit(const char *);

int DeinitExtensionUnit(uint32_t *);

bool SendOSCode();

bool SendCaptureComplete();

int ReadFirmwareVersion (uint32_t *, g_FWver_t *);

bool GetCameraUniqueID (char *);

int DeinitExtensionUnit(uint32_t *g_Handle);

int Master_Mode (uint32_t *g_Handle);

int Trigger_Mode (uint32_t *g_Handle);

int Get_Trigger_Mode (uint32_t *g_Handle, int *mode , int *exposure);

int GetFrameRateValue (uint32_t *g_Handle, uint32_t *Value);

int SetFrameRateValue (uint32_t *g_Handle, uint32_t Value);

int GetMaximumFrameRateValue (uint32_t *g_Handle, uint32_t *Value);

int GetMinimumFrameRateValue (uint32_t *g_Handle, uint32_t *Value);

int GetDefaultFrameRateValue (uint32_t *g_Handle, uint32_t *Value);

int ReadBaseBoardSerialNumber(uint32_t *g_Handle, char *pSN, int maxBufLen);

bool SetHDRModeStereo (uint32_t *g_Handle, uint32_t HDRMode);				//Enables/Disables the HDR mode

bool GetHDRModeStereo (uint32_t *g_Handle, uint32_t *HDRMode);			//Reads the status of the HDR mode

bool SetManualExposureStereo(uint32_t *g_handle, int32_t ExposureValue);
/* Special event handlers Declarations */

#endif
