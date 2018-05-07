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
UINT8 curr_see3cam_dev=0;

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
  BOOL timeout = TRUE;
  int ret = 0;
  unsigned int start, end = 0;
  unsigned short int sdk_ver=0, svn_ver=0;
  
  if(*g_Handle <= 0) {
      return FAIL;
  }
        
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  
  //Set the Report Number
  g_out_packet_buf[1] = READFIRMWAREVERSION; 	/* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    printf("%s(): wrote %d bytes\n", __func__,ret);
  }
  /* Read the Firmware Version from the device */
  start = GetTickCount();
  while(timeout) 
  {	
    /* Get a report from the device */
    ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
    if (ret < 0) {
      //perror("read");
    } else {
      if(g_in_packet_buf[0] == READFIRMWAREVERSION) {
        sdk_ver = (g_in_packet_buf[3]<<8)+g_in_packet_buf[4];
        svn_ver = (g_in_packet_buf[5]<<8)+g_in_packet_buf[6];
  
        g_UvcFwVer->pMajorVersion = g_in_packet_buf[1];
        g_UvcFwVer->pMinorVersion1 = g_in_packet_buf[2];
        g_UvcFwVer->pMinorVersion2 = sdk_ver;
        g_UvcFwVer->pMinorVersion3 = svn_ver;
  
        timeout = FALSE;
      }
    }
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      printf("%s(): Timeout occurred\n", __func__);
      timeout = FALSE;
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
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command for reading serial number to the UVC device	    *
 *			and then device sends back the unique ID which will be stored in UniqueID	    *	
  **********************************************************************************************************
*/
int ReadBaseBoardSerialNumber(uint32_t *g_Handle, char *pSN, int maxBufLen)
{
  BOOL timeout = TRUE;
  int ret = 0;
  unsigned int start, end = 0;
  UINT32 iBaseBoardSerialNumber;
  
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  memset(pSN, 0x00, maxBufLen);
  
  //Set the Report Number
  g_out_packet_buf[1] = GETCAMERA_UNIQUEID; 	/* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    printf("%s(): wrote %d bytes\n", __func__,ret);
  }
  /* Read the Serial Number from the device */
  start = GetTickCount();
  while(timeout) 
  {	
    /* Get a report from the device */
    ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
    if (ret < 0) {
      //perror("read");
    } else {
      if(g_in_packet_buf[0] == GETCAMERA_UNIQUEID) {
#if 0
      printf("%s(): read %d bytes:\n", __func__,ret);
      printf("g_in_packet_buf %x %x %x %x %x %x %x %x\n",g_in_packet_buf[0],g_in_packet_buf[1],g_in_packet_buf[2],
              g_in_packet_buf[3],g_in_packet_buf[4],g_in_packet_buf[5],g_in_packet_buf[6],g_in_packet_buf[7]);
#endif
        iBaseBoardSerialNumber = 0;
        iBaseBoardSerialNumber = ( ((UINT32)g_in_packet_buf[4]) 
                | ( ((UINT32)g_in_packet_buf[3]) << 8)
                | ( ((UINT32)g_in_packet_buf[2]) << 16) 
                | ( ((UINT32)g_in_packet_buf[1]) << 24) ); 

        sprintf(pSN, "%d", iBaseBoardSerialNumber);
        strcat(pSN, "\0");
        timeout = FALSE;
      }
    }
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      printf("%s(): Timeout occurred\n", __func__);
      timeout = FALSE;
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
 *  Returns	:	int (TRUE or FALSE)							    *
 *  Description	:       to find the first hid device connected to the linux pc		 	    *	
  **********************************************************************************************************
*/


int find_hid_device(const char *serial)
{
  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices, *dev_list_entry;
  struct udev_device *dev, *pdev;
  int ret = FALSE;
  char buf[256];
  int g_Handle;
  
    /* Create the udev object */
  udev = udev_new();
  if (!udev) {
    printf("Can't create udev\n");
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
    printf("Device Node Path: %s\n", udev_device_get_devnode(dev));
    
    /* The device pointed to by dev contains information about the hidraw device. In order to get information about the USB device, get the parent device with the subsystem/devtype pair of "usb"/"usb_device". This will be several levels up the tree, but the function will find it.*/
    pdev = udev_device_get_parent_with_subsystem_devtype(
            dev,
            "usb",
            "usb_device");
    if (!pdev) {
      printf("Unable to find parent usb device.\n");
      return PASS;
    }

    if(!strncmp(udev_device_get_sysattr_value(pdev,"idVendor"), "2560", 4)) {
      if(!strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c110", 4) ||
         !strncmp(udev_device_get_sysattr_value(pdev, "idProduct"), "c111", 4)) {
        if(!strncmp(udev_device_get_sysattr_value(pdev,"serial"), serial, 8)) {
          printf("    idVendor:  %s\n", udev_device_get_sysattr_value(pdev, "idVendor"));
          printf("    idProduct: %s\n", udev_device_get_sysattr_value(pdev, "idProduct"));
          printf("    Serial:    %s = %s\n", udev_device_get_sysattr_value(pdev, "serial"), serial);
          hid_device = udev_device_get_devnode(dev);
          udev_device_unref(pdev);
          ret = TRUE;
        }
      }
    }
  
    //Open each hid device and Check for bus name here
    g_Handle = open(hid_device, O_RDWR|O_NONBLOCK);
    //printf("%d\n", g_Handle);
  
    if (g_Handle < 0) {
      perror("Unable to open device");
      continue;
    }else {
      memset(buf, 0x00, sizeof(buf));
    }
  
    /* Get Physical Location */
    ret = ioctl(g_Handle, HIDIOCGRAWPHYS(256), buf);
    if (ret < 0) {
      perror("HIDIOCGRAWPHYS");
    }
  
    /* Close the hid fd */
    if(g_Handle > 0)
    {
      if(close(g_Handle) < 0) {
        printf("\nFailed to close %s\n",hid_device);
      }
    }
    if(ret == TRUE) {
      return PASS;
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
	BOOL timeout = TRUE;
	int ret = 0;
	unsigned int start, end = 0;

    if(*g_Handle <= 0) {
        return FAIL;
    }
	
	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = MASTERMODE; /* Report Number */

	/* Send a Report to the Device */
	ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FAIL;
	} else {
		printf("%s(): wrote %d bytes\n", __func__,ret);
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
      printf("%s(): read %d bytes:\n", __func__,ret);
      if(g_in_packet_buf[0] == MASTERMODE ){
        if(g_in_packet_buf[1] == 0x00) {
          return FAIL;
        } else if(g_in_packet_buf[1]== SET_TRUE) {
          timeout = FALSE;
        }else if(g_in_packet_buf[1]== SAME_MODE) {
          timeout = FALSE;
        }
      }
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
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
  BOOL timeout = TRUE;
  int ret = 0;
  unsigned int start, end = 0;
  
  if(*g_Handle <= 0) {
    return FAIL;
  }
  
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  
  //Set the Report Number
  g_out_packet_buf[1] = GETTRIGGERMODE; /* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    printf("%s(): wrote %d bytes\n", __func__,ret);
  }
    
  /* Read the Trigger Mode status from the device */
  start = GetTickCount();
  while(timeout) 
  {	
    /* Get a report from the device */
    ret = read(*g_Handle, g_in_packet_buf, BUFFER_LENGTH);
    if (ret < 0) {
      perror("read");
    } else {
      printf("%s(): read %d bytes:\n", __func__,ret);
      if(g_in_packet_buf[0] == GETTRIGGERMODE ){
        if(g_in_packet_buf[3] == FAILURE) {
          return FAIL;
        } else {
          *mode = g_in_packet_buf[1];
          *exposure = g_in_packet_buf[2];
          timeout = FALSE;
        } 
      }
    }
  
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      printf("%s(): Timeout occurred\n", __func__);
      timeout = FALSE;
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
  BOOL timeout = TRUE;
  int ret = 0;
  unsigned int start, end = 0;
  
  if(*g_Handle <= 0) {
    return FAIL;
  }
  
  //Initialize the buffer
  memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
  
  //Set the Report Number
  g_out_packet_buf[1] = TRIGGERMODE; /* Report Number */
  
  /* Send a Report to the Device */
  ret = write(*g_Handle, g_out_packet_buf, BUFFER_LENGTH);
  if (ret < 0) {
    perror("write");
    return FAIL;
  } else {
    printf("%s(): wrote %d bytes\n", __func__,ret);
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
      printf("%s(): read %d bytes:\n", __func__,ret);
      if(g_in_packet_buf[0] == TRIGGERMODE ){
        if(g_in_packet_buf[1] == 0x00) {
          return FAIL;
        } else if(g_in_packet_buf[1]== SET_TRUE) {
          timeout = FALSE;
        } else if(g_in_packet_buf[1]== SAME_MODE) {
          timeout = FALSE;
        } else if(g_in_packet_buf[1]== HIGH_SPEED) {
          timeout = FALSE;
        }
      }
    }
    end = GetTickCount();
    if(end - start > TIMEOUT)
    {
      printf("%s(): Timeout occurred\n", __func__);
      timeout = FALSE;
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
 *  Name	:	InitExtensionUnit								    *
 *  Parameter1	:	char *USBInstanceID										    *
 *  Parameter2	:											    *
 *  Returns	:uint32_t	 (HANDLE or NULL)								    *
 *  Description	:       finds see3cam device based on the VID, PID and USBInstanceID and initialize the HID	    *
			device. Application should call this function before calling any other function.    *
  **********************************************************************************************************
*/

//uint32_t InitExtensionUnit(char *USBInstanceID)
uint32_t InitExtensionUnit(const char *serial)
{
  int i, ret, desc_size = 0;
  UINT32 g_Handle;
  char buf[256];
  struct hidraw_devinfo info;
  struct hidraw_report_descriptor rpt_desc;
  
  //ret = find_hid_device((char*)USBInstanceID);
  ret = find_hid_device(serial);
  if(ret < 0)
  {
    printf("%s(): Not able to find the rambus device\n", __func__);
    return NULL_HANDLE;
  }
  printf("Selected HID Device : %s\n",hid_device);
  
  /* Open the Device with non-blocking reads. In real life,
      don't use a hard coded path; use libudev instead. */
  g_Handle = open(hid_device, O_RDWR|O_NONBLOCK);
  
  if (g_Handle < 0) {
    perror("Unable to open device");
    return NULL_HANDLE;
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
    printf("Report Descriptor Size: %d\n", desc_size);
  
  /* Get Report Descriptor */
  rpt_desc.size = desc_size;
  ret = ioctl(g_Handle, HIDIOCGRDESC, &rpt_desc);
  if (ret < 0) {
    perror("HIDIOCGRDESC");
    return NULL_HANDLE;
  } else {
    printf("Report Descriptors:\n");
    for (i = 0; i < rpt_desc.size; i++)
      printf("%hhx ", rpt_desc.value[i]);
    puts("\n");
  }
  
  /* Get Raw Name */
  ret = ioctl(g_Handle, HIDIOCGRAWNAME(256), buf);
  if (ret < 0) {
    perror("HIDIOCGRAWNAME");
    return NULL_HANDLE;
  }
  else
    printf("Raw Name: %s\n", buf);
  
  /* Get Physical Location */
  ret = ioctl(g_Handle, HIDIOCGRAWPHYS(256), buf);
  if (ret < 0) {
    perror("HIDIOCGRAWPHYS");
    return NULL_HANDLE;
  }
  else
    printf("Raw Phys: %s\n", buf);
  
  /* Get Raw Info */
  ret = ioctl(g_Handle, HIDIOCGRAWINFO, &info);
  if (ret < 0) {
    perror("HIDIOCGRAWINFO");
    return NULL_HANDLE;
  } else {
    printf("Raw Info:\n");
    printf("\tbustype: %d (%s)\n", info.bustype, bus_str(info.bustype));
    printf("\tvendor: 0x%04hx\n", info.vendor);
    printf("\tproduct: 0x%04hx\n", info.product);
  }
  
  return g_Handle;
}
