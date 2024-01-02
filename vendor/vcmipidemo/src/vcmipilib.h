// This was pulled out of vcmipidemo so we can made use of its (very useful) helper functions.

#ifndef _VC_MIPI_LIB_H_
#define _VC_MIPI_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#include <linux/videodev2.h>

#include "vclib-excerpt.h"
#include "vcimgnet.h"


/*--*STRUCT*----------------------------------------------------------*/
/**
*  @brief  Capture Queue Slot.
*
*    This structure represents a capture queue slot
*    through which data of captured images is accessible.
*/
typedef struct
{
	char              **st;           /*!<   multiple allocated image regions due to hardware */
	struct v4l2_plane  *plane;
	size_t              planeCount;   /*!<   Number of regions          */
} QBuf;
#define  NULL_QBuf { NULL, NULL, 0 }


/*--*STRUCT*----------------------------------------------------------*/
/**
*  @brief  Sensor Access and Attributes, Image Capture Queue Slots.
*
*    This structure holds the file descriptor of the sensor for access
*    as well as sensor specific information, like width and height of
*    the pixel image. Also available are the capture queue slots to
*    be able to access the recorded images.
*/
typedef struct
{
	int      fd;  /*!<  File Descriptor of the opened Sensor Device.  */

	QBuf    *qbuf; /*!<  Queue Buffers where Images are recorded to.  */
	U32      qbufCount; /*!<  Number of Queue Buffers available.      */

	struct v4l2_format  format; /*!<  Sensor Attributes.            */

} VCMipiSenCfg;
#define NULL_VCMipiSenCfg  { -1, NULL,0, {0} }

/*--*ENUM*------------------------------------------------------------*/
/**
*  @brief  Sensor White Balance Mode
*
*    This enum encodes the Whitebalancer mode to be used.
*    See function @ref process_whitebalance() for usage instructions.
*/
typedef enum
{
	WBMODE_INACTIVE = 0,
	WBMODE_MEASURE  = 1,
	WBMODE_APPLY    = 2
} VCWhiteBalMode;
/*--*STRUCT*----------------------------------------------------------*/
/**
*  @brief  Sensor White Balance Parameters for colored images.
*
*    Run a white balance calculation for an image of type IMAGE_RGB.
*    See function @ref process_whitebalance() for usage instructions.
*/
typedef struct
{
	VCWhiteBalMode  mode;     /*!<  White balance mode  */

	int             ampRed;   /*!<  Red   channel amplifier [1 .. 255], for example 121.  */
	int             ampGreen; /*!<  Green channel amplifier [1 .. 255], for example 198.  */
	int             ampBlue;  /*!<  Blue  channel amplifier [1 .. 255], for example 125.  */
} VCWhiteBalCfg;
#define NULL_VCWhiteBalCfg  { WBMODE_INACTIVE, 1, 1, 1 }


void  sig_handler(int signo);

int  change_options_by_commandline(int argc, char *argv[], int *shutter, float *gain, int *maxCaptures, int *fbOutIff1, char *pcFramebufferDev, int *stdOutIff1, int *fileOutIff1, int *bufCount, int *videoDevId, int *width, int *height, int *x0, int *y0, int *fps, VCWhiteBalCfg *cfgWB);
int  media_set_roi(char *pcVideoDev, int optX0, int optY0, int optWidth, int optHeight);
int  sensor_open(char *dev_video_device, VCMipiSenCfg *sen, unsigned int qBufCount);
int  sensor_close(VCMipiSenCfg *sen);
int  sensor_set_shutter_gain(VCMipiSenCfg  *sen, int newGain, int newShutter);
int  sensor_set_cropping_roi(VCMipiSenCfg  *sen, int newX0, int newY0, int newWidth, int newHeight);
int  sensor_set_fps(VCMipiSenCfg *sen, I32 fps);
int  sensor_streaming_start(VCMipiSenCfg *sen);
int  sensor_streaming_stop(VCMipiSenCfg *sen);
int  capture_buffer_enqueue(I32 bufIdx, VCMipiSenCfg *sen);
int  capture_buffer_dequeue(I32 bufIdx, VCMipiSenCfg *sen);
int  capture_buffer_dequeue_ts(I32 bufIdx, VCMipiSenCfg *sen, struct timeval *timestamp);
int  sleep_for_next_capture(VCMipiSenCfg  *sen, int timeoutUS);
int  imgnet_connect(VCImgNetCfg *imgnetCfg, U32 pixelformat, int dx, int dy);
int  imgnet_disconnect(VCImgNetCfg *imgnetCfg);
int  process_capture(unsigned int pixelformat, char *st, int dx, int dy, int pitch, int stdOutIff1, int netSrvOutIff1, VCImgNetCfg *imgnetCfg, int fbOutIff1, int fileOutIff1, int frameNr, char *pcFramebufferDev, VCWhiteBalCfg *cfgWB);
I32  process_whitebalance(image *img, VCWhiteBalCfg *cfgWB);
I32  copy_grey_to_image(image *imgOut, char *bufIn, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  convert_raw10_to_image(image *imgOut, char *bufIn, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  convert_raw12_to_image(image *imgOut, char *bufIn, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  convert_raw14_to_image(image *imgOut, char *bufIn, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  convert_raw_and_debayer_image(image *imgOut, char *bufIn, U32 pixelformat, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  convert_16bit_to_image(image *imgOut, char *bufIn, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 maxBits);
I32  convert_srggb_and_debayer_image(image *imgOut, char *bufIn, U32 pixelformat, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 maxBits);
I32  convert_yuyv_to_image(image *imgOut, char *bufIn, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  simple_debayer_to_image(image *imgOut, char *bufIn, unsigned int pixelformat, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
int  copy_image(image *in, image *out);
int  copy_image_to_framebuffer(char *pcFramebufferDev, const void *pvDataGREY_OR_R, const void *pvDataGREY_OR_G, const void *pvDataGREY_OR_B, U32 dy, U32 pitch);
I32  write_image_as_pnm(char *path, image *img);
I32  fill_image_with_pattern(image *imgOut, image *imgPat);
I32  fill_image_with_hourglasses(image *imgOut);
I32  fill_framebuffer_with_hourglasses(char *pcFramebufferDev, I32 dx, I32 dy);
void print_image_to_stdout(image *img, int stp, int goUpIff1);
void timemeasurement_start(struct  timeval *timer);
void timemeasurement_stop(struct  timeval *timer, I64 *s, I64 *us);
#ifdef __cplusplus
#ifdef WITH_OPENCV
	I32  openCV_example(image *imgIn, image *imgOut);
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif
