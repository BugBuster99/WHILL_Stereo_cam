#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>      
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <math.h>
#include "xunit/xunit_lib.h"

#include "xunit/xunit_tab_lib.h"
uint8_t curr_see3cam_dev=0;

char g_szBaseBoardSerialNumber[20];

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	ReadFirmwareVersion								    *
 *  Parameter1	:	uint32_t *g_Handle							    *
 *  Parameter2	:	g_FWver_t *g_UvcFwVer					    *
 *  Returns	:	int (PASS or FAIL)								    *
 *  Description	:       sends the extension unit command for reading firmware version to the UVC device     *
 *			and then device sends back the firmware version will be stored in the variables	    *	
  **********************************************************************************************************
*/

int ReadFirmwareVersion (uint32_t *g_Handle, g_FWver_t *g_UvcFwVer)
{
  bool timeout = true;
  int ret = 0;
  unsigned int start, end = 0;
  unsigned short int sdk_ver=0, svn_ver=0;
  if(*g_Handle <= 0) {
      return FAIL;
  }
        
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  
  //Set the Report Number
  g_out_packet_buf[1] = READ_FIRMWARE_VERSION; 	/* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    XUNIT_DEBUG("%s(): wrote %d bytes", __func__,ret);
  }
  /* Read the Firmware Version from the device */
  start = GetTickCount();
  while(timeout) 
  {	
    /* Get a report from the device */
    ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
    if (ret < 0) {
      // perror("read");
    } else {
      XUNIT_DEBUG("%s(): read %d bytes:", __func__,ret);
      if(g_in_packet_buf[0] == READ_FIRMWARE_VERSION) {
        sdk_ver = (g_in_packet_buf[3]<<8)+g_in_packet_buf[4];
        svn_ver = (g_in_packet_buf[5]<<8)+g_in_packet_buf[6];
  
        g_UvcFwVer->pMajorVersion = g_in_packet_buf[1];
        g_UvcFwVer->pMinorVersion1 = g_in_packet_buf[2];
        g_UvcFwVer->pMinorVersion2 = sdk_ver;
        g_UvcFwVer->pMinorVersion3 = svn_ver;
  
        timeout = false;
      }
    }
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      XUNIT_DEBUG("%s(): Timeout occurred", __func__);
      timeout = false;
      return FAIL;
    }		
  }
  return PASS;
  
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	GetCameraUniqueID								    *
 *  Parameter1	:	char *		(UniqueID)							    *
 *  Returns	:	bool (true or false)								    *
 *  Description	:       sends the extension unit command for reading serial number to the UVC device	    *
 *			and then device sends back the unique ID which will be stored in UniqueID	    *	
  **********************************************************************************************************
*/
int ReadBaseBoardSerialNumber(uint32_t *g_Handle, char *pSN, int maxBufLen)
{
  bool timeout = true;
  int ret = 0;
  unsigned int start, end = 0;
  uint32_t iBaseBoardSerialNumber;
  
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  memset(pSN, 0x00, maxBufLen);
  
  //Set the Report Number
  g_out_packet_buf[1] = GET_CAMERA_UNIQUE_ID; 	/* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    XUNIT_DEBUG("%s(): wrote %d bytes", __func__,ret);
  }
  /* Read the Serial Number from the device */
  start = GetTickCount();
  while(timeout) 
  {	
    /* Get a report from the device */
    ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
    if (ret < 0) {
      //perror("read");
    } else {
      if(g_in_packet_buf[0] == GET_CAMERA_UNIQUE_ID) {
#if 0
      XUNIT_DEBUG("%s(): read %d bytes:", __func__,ret);
      XUNIT_DEBUG("g_in_packet_buf %x %x %x %x %x %x %x %x",g_in_packet_buf[0],g_in_packet_buf[1],g_in_packet_buf[2],
              g_in_packet_buf[3],g_in_packet_buf[4],g_in_packet_buf[5],g_in_packet_buf[6],g_in_packet_buf[7]);
#endif
        iBaseBoardSerialNumber = 0;
        iBaseBoardSerialNumber = ( ((uint32_t)g_in_packet_buf[4]) 
                | ( ((uint32_t)g_in_packet_buf[3]) << 8)
                | ( ((uint32_t)g_in_packet_buf[2]) << 16) 
                | ( ((uint32_t)g_in_packet_buf[1]) << 24) ); 

        XUNIT_DEBUG("%d", iBaseBoardSerialNumber);
        strcat(pSN, "\0");
        XUNIT_DEBUG("%s", pSN);
        timeout = false;
      }
    }
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      XUNIT_DEBUG("%s(): Timeout occurred", __func__);
      timeout = false;
      return FAIL;
    }		
  }
  return PASS;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	UninitExtensionUnit								    *
 *  Parameter1	:	uint32_t *g_Handle							    *
 *  Returns	:	int (PASS or FAIL)								    *
 *  Description	:    to release all the extension unit objects and other internal library objects
 ***********************************************************************************************************
*/


int DeinitExtensionUnit(uint32_t *g_Handle)
{
	int ret=0;
	/* Close the hid fd */
	if(*g_Handle > 0)
	{
		ret=close(*g_Handle);
	}
	if(ret<0)
		return FAIL;
	else
		return PASS;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	bus_str										    *
 *  Parameter1	:	int										    *
 *  Parameter2	:											    *
 *  Returns	:	const char *									    *
 *  Description	:       to convert integer bus type to string					 	    *	
  **********************************************************************************************************
*/
const char *bus_str(int bus)
{
	switch (bus) {
	case BUS_USB:
		return "USB";
		break;
	case BUS_HIL:
		return "HIL";
		break;
	case BUS_BLUETOOTH:
		return "Bluetooth";
		break;
	case BUS_VIRTUAL:
		return "Virtual";
		break;
	default:
		return "Other";
		break;
	}
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	Internal API 									    *
 *  Name	:	GetTicketCount									    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	unsigned int									    *
 *  Description	:       to return current time in the milli seconds				 	    *	
  **********************************************************************************************************
*/

unsigned int GetTickCount()
{
  struct timeval tv;
  if(gettimeofday(&tv, NULL) != 0) {
    return 0;
  }
  
  return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	find_hid_device									    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	int (true or false)							    *
 *  Description	:       to find the first hid device connected to the linux pc		 	    *	
  **********************************************************************************************************
*/


int find_hid_device(const char *serial)
{
  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices, *dev_list_entry;
  struct udev_device *dev, *pdev;
  int ret = false;
  char buf[256];
  int g_Handle;
  
    /* Create the udev object */
  udev = udev_new();
  if (!udev) {
    XUNIT_DEBUG("Can't create udev");
    return PASS;
  }
  
  /* Create a list of the devices in the 'hidraw' subsystem. */
  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "hidraw");
  udev_enumerate_scan_devices(enumerate);
  devices = udev_enumerate_get_list_entry(enumerate);
  
  /* For each item enumerated, print out its information. udev_list_entry_foreach is a macro which expands to a loop. The loop will be executed for each member in  devices, setting dev_list_entry to a list entry which contains the device's path in /sys. */
  udev_list_entry_foreach(dev_list_entry, devices) {
    const char *path;
    
    /* Get the filename of the /sys entry for the device and create a udev_device object (dev) representing it */
    path = udev_list_entry_get_name(dev_list_entry);
    dev = udev_device_new_from_syspath(udev, path);
  
    /* usb_device_get_devnode() returns the path to the device node itself in /dev. */
    XUNIT_DEBUG("Device Node Path: %s", udev_device_get_devnode(dev));
    
    /* The device pointed to by dev contains information about the hidraw device. In order to get information about the USB device, get the parent device with the subsystem/devtype pair of "usb"/"usb_device". This will be several levels up the tree, but the function will find it.*/
    pdev = udev_device_get_parent_with_subsystem_devtype(
            dev,
            "usb",
            "usb_device");
    if (!pdev) {
      XUNIT_DEBUG("Unable to find parent usb device.");
      return PASS;
    }

    if(!strncmp(udev_device_get_sysattr_value(pdev,"idVendor"), "2560", 4)) {
      if(!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c110", 4) ||
         !strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c111", 4) ||
         !strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c114", 4)) {
        if(!strncmp(udev_device_get_sysattr_value(pdev,"serial"), serial, 8)) {
          XUNIT_DEBUG("    idVendor:  %s", udev_device_get_sysattr_value(pdev, "idVendor"));
          XUNIT_DEBUG("    idProduct: %s", udev_device_get_sysattr_value(pdev, "idProduct"));
          XUNIT_DEBUG("    Serial:    %s = %s", udev_device_get_sysattr_value(pdev, "serial"), serial);
          hid_device = udev_device_get_devnode(dev);
          udev_device_unref(pdev);
          return PASS;
        }
      }
    }
  }
  /* Free the enumerator object */
  udev_enumerate_unref(enumerate);
  
  udev_unref(udev);
  
  return ret;
}


/*
 ************************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	Master mode									    *
 *  Parameter1	:   uint32_t *g_Handle									    *
 *  Returns	:	int (PASS or FAIL)								    *
 *  Description	:   This function is used to SET MASTER MODE				    *
 ************************************************************************************************************
*/
int Master_Mode (uint32_t *g_Handle)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

    if(*g_Handle <= 0) {
        return FAIL;
    }
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = MASTER_MODE; /* Report Number */

	/* Send a Report to the Device */
	ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FAIL;
	} else {
		XUNIT_DEBUG("%s(): wrote %d bytes", __func__,ret);
	}
    
	/* Read the Master Mode status from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
      XUNIT_DEBUG("%s(): read %d bytes:", __func__,ret);
      if(g_in_packet_buf[0] == MASTER_MODE ){
        if(g_in_packet_buf[1] == 0x00) {
          return FAIL;
        } else if(g_in_packet_buf[1]== SET_TRUE) {
          timeout = false;
        }else if(g_in_packet_buf[1]== SAME_MODE) {
          timeout = false;
        }
      }
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			XUNIT_DEBUG("%s(): Timeout occurred", __func__);
			timeout = false;
			return FAIL;
		}		
	}
	if(g_in_packet_buf[1]== SET_TRUE)
		return PASS;
	else if(g_in_packet_buf[1]== SAME_MODE)
		return SAME_MODE;
}

/*
 ************************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	Get trigger mode									    *
 *  Parameter1	:   uint32_t *g_Handle									    *
 *  Returns	:	int (PASS or FAIL)								    *
 *  Description	:   This function is used to SET TRIGGER MODE				    *
 ************************************************************************************************************
*/
int Get_Trigger_Mode (uint32_t *g_Handle, int *mode , int *exposure)
{
  bool timeout = true;
  int ret = 0;
  unsigned int start, end = 0;
  
  if(*g_Handle <= 0) {
    return FAIL;
  }
  
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  
  //Set the Report Number
  g_out_packet_buf[0] = WHILL_CAMERA_CONTROL; /* Report Number */
  g_out_packet_buf[1] = GET_TRIGGER_MODE; /* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    XUNIT_DEBUG("%s(): wrote %d bytes", __func__,ret);
  }
    
  /* Read the Trigger Mode status from the device */
  start = GetTickCount();
  while(timeout) 
  {	
    /* Get a report from the device */
    ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
    if (ret < 0) {
      // perror("read");
    } else {
      XUNIT_DEBUG("%s(): read %d bytes:", __func__,ret);
      if(g_in_packet_buf[0] == WHILL_CAMERA_CONTROL &&
         g_in_packet_buf[1] == GET_TRIGGER_MODE ){
        if(g_in_packet_buf[4] == FAILURE) {
          return FAIL;
        } else {
          *mode = g_in_packet_buf[2];
          *exposure = g_in_packet_buf[3];
          timeout = false;
        } 
      }
    }
  
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      XUNIT_DEBUG("%s(): Timeout occurred", __func__);
      timeout = false;
      return FAIL;
    }		
  }
  
  return PASS;
}


/*
 ************************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	trigger mode									    *
 *  Parameter1	:   uint32_t *g_Handle									    *
 *  Returns	:	int (PASS or FAIL)								    *
 *  Description	:   This function is used to SET TRIGGER MODE				    *
 ************************************************************************************************************
*/
int Trigger_Mode (uint32_t *g_Handle)
{
  bool timeout = true;
  int ret = 0;
  unsigned int start, end = 0;
  
  if(*g_Handle <= 0) {
    return FAIL;
  }
  
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  
  //Set the Report Number
  g_out_packet_buf[1] = TRIGGER_MODE; /* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    XUNIT_DEBUG("%s(): wrote %d bytes", __func__,ret);
  }
    
  /* Read the Trigger Mode status from the device */
  start = GetTickCount();
  while(timeout) 
  {	
    /* Get a report from the device */
    ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
    if (ret < 0) {
      // perror("read");
    } else {
      XUNIT_DEBUG("%s(): read %d bytes:", __func__,ret);
      if(g_in_packet_buf[0] == TRIGGER_MODE ){
        if(g_in_packet_buf[1] == 0x00) {
          return FAIL;
        } else if(g_in_packet_buf[1]== SET_TRUE) {
          timeout = false;
        } else if(g_in_packet_buf[1]== SAME_MODE) {
          timeout = false;
        } else if(g_in_packet_buf[1]== HIGH_SPEED) {
          timeout = false;
        }
      }
    }
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      XUNIT_DEBUG("%s(): Timeout occurred", __func__);
      timeout = false;
      return FAIL;
    }		
  }
  if(g_in_packet_buf[1]== SET_TRUE)
    return PASS;
  else if(g_in_packet_buf[1]== SAME_MODE)
    return SAME_MODE;
  else if(g_in_packet_buf[1]== HIGH_SPEED)
    return HIGH_SPEED;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	    :	ReadFrameRateValue								    *
 *  Parameter1	:	uint32_t *g_Handle								    *
 *  Parameter2	:	uint8_t *Value									    *
 *  Returns	    :	int (PASS or FAIL)								    *
 *  Description	:   This function is used to get the FrameRate value of the camera.
  **********************************************************************************************************
*/
int GetFrameRateValue (uint32_t *g_Handle, uint32_t *Value)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

    if(*g_Handle <= 0) {
        return FAIL;
    }
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHILL_CAMERA_CONTROL; /* Report Number */
	g_out_packet_buf[2] = FRAME_RATE_CONTROL;
	g_out_packet_buf[3] = GET_FRAME_RATE; 	/* Report Number */

	/* Send a Report to the Device */
	ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FAIL;
	} else {
		XUNIT_DEBUG("%s(): wrote %d bytes\n", __func__,ret);
	}
    
	/* Read the Trigger Mode status from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			XUNIT_DEBUG("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == WHILL_CAMERA_CONTROL &&
			    g_in_packet_buf[1] == FRAME_RATE_CONTROL &&
			    g_in_packet_buf[2] == GET_FRAME_RATE ) {
					if(g_in_packet_buf[4] == SET_FAIL) {
						return FAIL;
					} else if(g_in_packet_buf[4]== SET_TRUE) {
						*Value = g_in_packet_buf[3];
						timeout = false;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			XUNIT_DEBUG("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return FAIL;
		}		
	}
	return PASS;
	
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	    :	WriteFrameRateValue								    *
 *  Parameter1	:	uint32_t *g_Handle								    *
 *  Parameter2	:	uint8_t Value									    *
 *  Returns	    :	int (PASS or FAIL)								    *
 *  Description	:   This function is used to get the frame rate  value of the camera.			    *
  **********************************************************************************************************
*/
int SetFrameRateValue (uint32_t *g_Handle, uint32_t Value)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

  if(*g_Handle <= 0) {
      return FAIL;
  }
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHILL_CAMERA_CONTROL; /* Report Number */
	g_out_packet_buf[2] = FRAME_RATE_CONTROL;
	g_out_packet_buf[3] = SET_FRAME_RATE; 	/* Report Number */
	g_out_packet_buf[4] = Value;

	/* Send a Report to the Device */
	ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FAIL;
	} else {
		XUNIT_DEBUG("%s(): wrote %d bytes", __func__,ret);
	}
    
	/* Read the Trigger Mode status from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			XUNIT_DEBUG("%s(): read %d bytes:  Framerate = %d", __func__,ret,Value);
			if(g_in_packet_buf[0] == WHILL_CAMERA_CONTROL &&
			    g_in_packet_buf[1] == FRAME_RATE_CONTROL &&
			    g_in_packet_buf[2] == SET_FRAME_RATE) {

					if(g_in_packet_buf[4] == SET_FAIL) {
						return FAIL;
					} else if(g_in_packet_buf[4]== SET_TRUE) {
						timeout = false;
					}else if(g_in_packet_buf[4]== OUT_OF_RANGE) {
						timeout = false;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			XUNIT_DEBUG("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return FAIL;
		}		
	}
	if(g_in_packet_buf[4]== SET_TRUE)
		return PASS;
	else if(g_in_packet_buf[4]== OUT_OF_RANGE)
		return OUT_OF_RANGE;
}



/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	    :	ReadFrameRateValue								    *
 *  Parameter1	:	uint32_t *g_Handle								    *
 *  Parameter2	:	uint8_t *Value									    *
 *  Returns	    :	int (PASS or FAIL)								    *
 *  Description	:   This function is used to get the Maximum FrameRate value of the camera.
  **********************************************************************************************************
*/
int GetMaximumFrameRateValue (uint32_t *g_Handle, uint32_t *Value)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

    if(*g_Handle <= 0) {
        return FAIL;
    }
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHILL_CAMERA_CONTROL; /* Report Number */
	g_out_packet_buf[2] = FRAME_RATE_CONTROL;
	g_out_packet_buf[3] = GET_MAXIMUM_FRAME_RATE; 	/* Report Number */

	/* Send a Report to the Device */
	ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FAIL;
	} else {
		XUNIT_DEBUG("%s(): wrote %d bytes\n", __func__,ret);
	}
    
	/* Read the Trigger Mode status from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			XUNIT_DEBUG("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == WHILL_CAMERA_CONTROL &&
			    g_in_packet_buf[1] == FRAME_RATE_CONTROL &&
			    g_in_packet_buf[2] == GET_MAXIMUM_FRAME_RATE ) {
					if(g_in_packet_buf[4] == SET_FAIL) {
						return FAIL;
					} else if(g_in_packet_buf[4]== SET_TRUE) {
						*Value = g_in_packet_buf[3];
						timeout = false;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			XUNIT_DEBUG("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return FAIL;
		}		
	}
	return PASS;
	
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	    :	ReadFrameRateValue								    *
 *  Parameter1	:	uint32_t *g_Handle								    *
 *  Parameter2	:	uint8_t *Value									    *
 *  Returns	    :	int (PASS or FAIL)								    *
 *  Description	:   This function is used to get the Minimum FrameRate value of the camera.
  **********************************************************************************************************
*/
int GetMinimumFrameRateValue (uint32_t *g_Handle, uint32_t *Value)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

    if(*g_Handle <= 0) {
        return FAIL;
    }
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHILL_CAMERA_CONTROL; /* Report Number */
	g_out_packet_buf[2] = FRAME_RATE_CONTROL;
	g_out_packet_buf[3] = GET_MINIMUM_FRAME_RATE; 	/* Report Number */

	/* Send a Report to the Device */
	ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FAIL;
	} else {
		XUNIT_DEBUG("%s(): wrote %d bytes\n", __func__,ret);
	}
    
	/* Read the Trigger Mode status from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			XUNIT_DEBUG("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == WHILL_CAMERA_CONTROL &&
			    g_in_packet_buf[1] == FRAME_RATE_CONTROL &&
			    g_in_packet_buf[2] == GET_MINIMUM_FRAME_RATE ) {
					if(g_in_packet_buf[4] == SET_FAIL) {
						return FAIL;
					} else if(g_in_packet_buf[4]== SET_TRUE) {
						*Value = g_in_packet_buf[3];
						timeout = false;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			XUNIT_DEBUG("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return FAIL;
		}		
	}
	return PASS;
	
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	    :	ReadFrameRateValue								    *
 *  Parameter1	:	uint32_t *g_Handle								    *
 *  Parameter2	:	uint8_t *Value									    *
 *  Returns	    :	int (PASS or FAIL)								    *
 *  Description	:   This function is used to get the DEFAULT FrameRate value of the camera.
  **********************************************************************************************************
*/
int GetDefaultFrameRateValue (uint32_t *g_Handle, uint32_t *Value)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

    if(*g_Handle <= 0) {
        return FAIL;
    }
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = WHILL_CAMERA_CONTROL; /* Report Number */
	g_out_packet_buf[2] = FRAME_RATE_CONTROL;
	g_out_packet_buf[3] = GET_DEFAULT_FRAME_RATE; 	/* Report Number */

	/* Send a Report to the Device */
	ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FAIL;
	} else {
		XUNIT_DEBUG("%s(): wrote %d bytes\n", __func__,ret);
	}
    
	/* Read the Trigger Mode status from the device */
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			XUNIT_DEBUG("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == WHILL_CAMERA_CONTROL &&
			    g_in_packet_buf[1] == FRAME_RATE_CONTROL &&
			    g_in_packet_buf[2] == GET_DEFAULT_FRAME_RATE ) {
					if(g_in_packet_buf[4] == SET_FAIL) {
						return FAIL;
					} else if(g_in_packet_buf[4]== SET_TRUE) {
						*Value = g_in_packet_buf[3];
						timeout = false;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			XUNIT_DEBUG("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return FAIL;
		}		
	}
	return PASS;
	
}
/*
 **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	InitExtensionUnit								    *
 *  Parameter1	:	char *USBInstanceID										    *
 *  Parameter2	:											    *
 *  Returns	:uint32_t	 (HANDLE or NULL)								    *
 *  Description	:       finds see3cam device based on the VID, PID and USBInstanceID and initialize the HID	    *
			device. Application should call this function before calling any other function.    *
  **********************************************************************************************************
*/

uint32_t InitExtensionUnit(const char *serial)
{
  int i, ret, desc_size = 0;
  uint32_t g_Handle;
  char buf[256];
  struct hidraw_devinfo info;
  struct hidraw_report_descriptor rpt_desc;
  
  ret = find_hid_device(serial);
  if(ret < 0)
  {
    XUNIT_DEBUG("%s(): Not able to find the rambus device", __func__);
    return NULL_HANDLE;
  }
  XUNIT_DEBUG("Selected HID Device : %s",hid_device);
  
  /* Open the Device with non-blocking reads. In real life,
      don't use a hard coded path; use libudev instead. */
  g_Handle = open(hid_device, O_RDWR|O_NONBLOCK);
  
  if (g_Handle < 0) {
    perror("Unable to open device");
    return NULL_HANDLE;
  }
  else{
    XUNIT_DEBUG("Handle : %u", g_Handle);
  }
  
  memset(&rpt_desc, 0x0, sizeof(rpt_desc));
  memset(&info, 0x0, sizeof(info));
  memset(buf, 0x0, sizeof(buf));
  
  /* Get Report Descriptor Size */
  ret = ioctl(g_Handle, HIDIOCGRDESCSIZE, &desc_size);
  if (ret < 0) {
    perror("HIDIOCGRDESCSIZE");
    return NULL_HANDLE;
  }
  else
    XUNIT_DEBUG("Report Descriptor Size: %d", desc_size);
  
  /* Get Report Descriptor */
  rpt_desc.size = desc_size;
  ret = ioctl(g_Handle, HIDIOCGRDESC, &rpt_desc);
  if (ret < 0) {
    perror("HIDIOCGRDESC");
    return NULL_HANDLE;
  } else {
    #if XUNIT_DEBUGGING
    XUNIT_DEBUG("Report Descriptors:");
    for (i = 0; i < rpt_desc.size; i++)
      XUNIT_DEBUG("%hhx ", rpt_desc.value[i]);
    puts("\n");
    #endif
  }
  
  /* Get Raw Name */
  ret = ioctl(g_Handle, HIDIOCGRAWNAME(256), buf);
  if (ret < 0) {
    perror("HIDIOCGRAWNAME");
    return NULL_HANDLE;
  }
  else
    XUNIT_DEBUG("Raw Name: %s", buf);
  
  /* Get Physical Location */
  ret = ioctl(g_Handle, HIDIOCGRAWPHYS(256), buf);
  if (ret < 0) {
    perror("HIDIOCGRAWPHYS");
    return NULL_HANDLE;
  }
  else
    XUNIT_DEBUG("Raw Phys: %s", buf);
  
  /* Get Raw Info */
  ret = ioctl(g_Handle, HIDIOCGRAWINFO, &info);
  if (ret < 0) {
    perror("HIDIOCGRAWINFO");
    return NULL_HANDLE;
  } else {
    XUNIT_DEBUG("Raw Info:");
    XUNIT_DEBUG("\tbustype: %d (%s)", info.bustype, bus_str(info.bustype));
    XUNIT_DEBUG("\tvendor: 0x%04hx", info.vendor);
    XUNIT_DEBUG("\tproduct: 0x%04hx", info.product);
  }
  
  return g_Handle;
}


/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 			*
 *  Name	:	SetHDRModeStereo		*
 *  Parameter1	:	uint32_t (HDRMode)		*
 *  Returns	:	bool (true or false)		*
 *  Description	:   	Sends the extension unit command to set a particular HDR mode. *
  **********************************************************************************************************
*/
bool SetHDRModeStereo(uint32_t *g_handle, uint32_t HDRMode)
{
    bool timeout = true;
    int ret = 0;
    unsigned int start, end = 0;

    //Initialize the buffer	
    memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

    //Set the Report Number
    g_out_packet_buf[1] = CAMERA_CONTROL_STEREO; 	/* Report Number */
    g_out_packet_buf[2] = SET_HDR_MODE_STEREO;
    g_out_packet_buf[3] = HDRMode;

    ret = write(*g_handle, g_out_packet_buf, BUFFER_LENGTH);
    if (ret < 0)
    {
        perror("xunit-GetHDRMode : write failed");
        return false;
    }
    else
    {
        XUNIT_DEBUG("%s(): wrote %d bytes", __func__,ret);
    }

    start = GetTickCount();
    while (timeout)
    {
        /* Get a report from the device */
        ret = read(*g_handle, g_in_packet_buf, BUFFER_LENGTH);
        
        if (ret < 0)
        {
            //perror("read");
        }
        else
        {
            XUNIT_DEBUG("%s(): read %d bytes:", __func__,ret);
            for(int ii; ii < ret; ++ii)
            {
                printf("%x, ", g_in_packet_buf[ii]);
            }
            printf("\n");
            if(g_in_packet_buf[0] == CAMERA_CONTROL_STEREO && g_in_packet_buf[1] == SET_HDR_MODE_STEREO)
            {
                if (  g_in_packet_buf[4] == SET_SUCCESS_TARA )
                {
                    timeout = false;
                }
                else
                {
                    if ( g_in_packet_buf[4] == SET_FAIL_TARA )
                    {
                        return false;
                    }
                }
            }
        }
        end = GetTickCount();
        if(end - start > TIMEOUT)
        {
            printf("%s(): Timeout occurred\n", __func__);
            timeout = false;
            return false;
        }
    }
    return true;
}


/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 				*
 *  Name	:	GetHDRModeStereo			*
 *  Parameter1	:	uint32_t ( *HDRMode)			*
 *  Returns	:	bool (true or false)			*
 *  Description	:   	Sends the extension unit command to read the HDR mode in which the camera is set. *
  **********************************************************************************************************
*/
bool GetHDRModeStereo(uint32_t *g_handle, uint32_t *HDRMode)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

	//Initialize the buffer	
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
	
	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_STEREO; 	/* Report Number */
	g_out_packet_buf[2] = GET_HDR_MODE_STEREO;
	
	ret = write(*g_handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0)
	{
		perror("xunit-GetHDRMode : write failed");
		return false;
	}
	else
	{
		//printf("%s(): wrote %d bytes\n", __func__,ret);
	}
	
	start = GetTickCount();
	while (timeout)
	{
		/* Get a report from the device */
		ret = read(*g_handle, g_in_packet_buf, BUFFER_LENGTH);
		
		if (ret < 0)
		{
			//perror("read");
		}
		else
		{
			//printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == CAMERA_CONTROL_STEREO && g_in_packet_buf[1] == GET_HDR_MODE_STEREO)
			{
				if (  g_in_packet_buf[4] == GET_SUCCESS )
				{
					*HDRMode = g_in_packet_buf[2];
					timeout = false;
				}
				else
				{
					if ( g_in_packet_buf[4] == GET_FAIL )
					{
						return false;
					}
				}
			}
	 	}
	 	end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return false;
		}
	}
	return true;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 						*
 *  Name	:	SetManualExposureStereo					*
 *  Parameter1	:	int32_t	(ExposureValue)				    	*
 *  Returns	:	bool (true or false)					*
 *  Description	:   	Sends the extension unit command to set the manual exposure value to the camera   *
 *			The exposure value ranges from 1 to 1000,000	  			  	  *
  **********************************************************************************************************
*/
bool SetManualExposureStereo(uint32_t *g_handle, int32_t ExposureValue)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;

	if((ExposureValue > SEE3CAM_STEREO_EXPOSURE_MAX) || (ExposureValue < SEE3CAM_STEREO_EXPOSURE_MIN))
	{
		printf("Set Manual Exposure failed : Input out of bounds\n");
		return false;
	}

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_STEREO; 	/* Report Number */
	g_out_packet_buf[2] = SET_EXPOSURE_VALUE; 	/* Report Number */

	g_out_packet_buf[3] = (uint8_t)((ExposureValue >> 24) & 0xFF);
	g_out_packet_buf[4] = (uint8_t)((ExposureValue >> 16) & 0xFF);
	g_out_packet_buf[5] = (uint8_t)((ExposureValue >> 8) & 0xFF);
	g_out_packet_buf[6] = (uint8_t)(ExposureValue & 0xFF);

	/* Send a Report to the Device */
	ret = write(*g_handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("xunit-SetManualExposureValue_Stereo : write failed");
		return false;
	} else {
		//printf("%s(): wrote %d bytes\n", __func__,ret);
	}

	/* Read the status from the device */
	start = GetTickCount();
	while(timeout)
	{
		/* Get a report from the device */
		ret = read(*g_handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			//printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == CAMERA_CONTROL_STEREO &&
							g_in_packet_buf[1] == SET_EXPOSURE_VALUE){
					if(g_in_packet_buf[10] == SET_SUCCESS) {
						timeout = false;
					} else if(g_in_packet_buf[10] == SET_FAIL) {
						return false;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return false;
		}
	}
	return true;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 				*
 *  Name	:	SetAutoExposureStereo			*
 *  Returns	:	bool (true or false)			*
 *  Description	:   	Sends the extension unit command to set the camera to auto exposure.   *
  **********************************************************************************************************
*/
bool SetAutoExposureStereo(uint32_t *g_handle)
{
	bool timeout = true;
	int ret = 0;
	unsigned int start, end = 0;
	int32_t ExposureValue = 1;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = CAMERA_CONTROL_STEREO; 	/* Report Number */
	g_out_packet_buf[2] = SET_AUTO_EXPOSURE; 	/* Report Number */

	g_out_packet_buf[3] = (uint8_t)((ExposureValue >> 24) & 0xFF);
	g_out_packet_buf[4] = (uint8_t)((ExposureValue >> 16) & 0xFF);
	g_out_packet_buf[5] = (uint8_t)((ExposureValue >> 8) & 0xFF);
	g_out_packet_buf[6] = (uint8_t)(ExposureValue & 0xFF);

	/* Send a Report to the Device */
	ret = write(*g_handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("xunit-SetAutoExposureStereo : write failed");
		return false;
	} else {
		//printf("%s(): wrote %d bytes\n", __func__,ret);
	}

	/* Read the status from the device */
	start = GetTickCount();
	while(timeout)
	{
		/* Get a report from the device */
		ret = read(*g_handle, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			//printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == CAMERA_CONTROL_STEREO &&
							g_in_packet_buf[1] == SET_AUTO_EXPOSURE){
					if(g_in_packet_buf[10] == SET_SUCCESS) {
						timeout = false;
					} else if(g_in_packet_buf[10] == SET_FAIL) {
						return false;
					}
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = false;
			return false;
		}
	}
	return true;
}