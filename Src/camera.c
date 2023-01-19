/*
 * camera.c
 *
 *  Created on: Nov 3, 2018
 *      Author: Graham Thoms
 * 	-- Kevin Dong modified on 2022-05-18
 * 		serve as base project for ENGG4420 & ENGG6550
 *
 */

#include "camera.h"
#include "arducam.h"
#include "nanojpeg.h"

#include "picojpeg.h"

#include "stlogo.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_cdc_if.h"
#ifndef min
#define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif
static uint32_t captureStart;
extern int myprintf(const char *format, ...);
uint8_t *pjpeg_load_from_file(FILE *g_pInFile, int *x, int *y, int *comps, pjpeg_scan_type_t *pScan_type, int reduce);
unsigned char pjpeg_need_bytes_callback(unsigned char* pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data);
static void camera_get_image();
BaseType_t write_fifo_to_buffer(uint32_t length);
unsigned char cameraReady;

static FILE *g_pInFile;
static uint g_nInFileSize;
static uint g_nInFileOfs;

void camera_setup(){

	cameraReady = pdFALSE;
	/**
	 * Detect and initialize the Arduchip interface.
	 * Ensure that the OV5642 is powered on.
	 * Detect and initialize the OV5642 sensor chip.
	 */
	if (   arduchip_detect()
		&& arducam_exit_standby()
		&& ov5642_detect()
	) {

		osDelay(100);

		if (!ov5642_configure()) {
			myprintf("camera_task: ov5642 configure failed\n\r");
			return;
		} else {
			myprintf("camera: setup complete\n\r");
			cameraReady = pdTRUE;
			osDelay(100);
		}
	} else {
		myprintf("camera: setup failed\n\r");
	}
}

/**
 * Capture an image from the camera.
 */
void camera_initiate_capture(){

	uint8_t done = 0;

	myprintf("camera: initiate capture\n\r");

	if (!cameraReady) {
		myprintf("camera: set up camera before capture\n\r");
	}

	/* Initiate an image capture. */
	if (!arduchip_start_capture()) {
		myprintf("camera: initiate capture failed\n\r");
		return;
	}

	/* wait for capture to be done */
	captureStart = (uint32_t)xTaskGetTickCount();
	while(!arduchip_capture_done(&done) || !done){

		if ((xTaskGetTickCount() - captureStart) >= CAPTURE_TIMEOUT) {
			myprintf("camera: capture timeout\n\r");
			return;
		}
	}

	myprintf("camera: capture complete\n\r");

	camera_get_image();

	return;

}

void camera_get_image(){

	/* Determine the FIFO buffer length. */
	uint32_t length = 0;
	if (arduchip_fifo_length(&length) == pdTRUE) {
		myprintf("camera: captured jpeg image -> %lu bytes\n\r", length);
		write_fifo_to_buffer(length);
	} else {
		myprintf("camera: get fifo length failed\n\r");
	}

	return;
}

uint8_t fifoBuffer[BURST_READ_LENGTH];
#define MAX_PIC_SIZE 64000
#define BITMAP_SIZE 3600
uint8_t pic_buffer[MAX_PIC_SIZE];
uint8_t bitmap[BITMAP_SIZE];

int pic_index = 0;


static void get_pixel(int* pDst, const uint8_t *pSrc, int luma_only, int num_comps)
{
   int r, g, b;
   if (num_comps == 1)
   {
      r = g = b = pSrc[0];
   }
   else if (luma_only)
   {
      const int YR = 19595, YG = 38470, YB = 7471;
      r = g = b = (pSrc[0] * YR + pSrc[1] * YG + pSrc[2] * YB + 32768) / 65536;
   }
   else
   {
      r = pSrc[0]; g = pSrc[1]; b = pSrc[2];
   }
   pDst[0] = r; pDst[1] = g; pDst[2] = b;
}


// https://helloacm.com
inline float
BilinearInterpolation(float q11, float q12, float q21, float q22, float x1, float x2, float y1, float y2, float x, float y)
{
    float x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 - x1;
    y2y1 = y2 - y1;
    x2x = x2 - x;
    y2y = y2 - y;
    yy1 = y - y1;
    xx1 = x - x1;
    return 1.0 / (x2x1 * y2y1) * (
        q11 * x2x * y2y +
        q21 * xx1 * y2y +
        q12 * x2x * yy1 +
        q22 * xx1 * yy1
    );
}

static void display_bitmap(const uint8_t * image, int width, int height, int comps, int start_x, int start_y, int scale)
{

	int a[3];

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			get_pixel(a, image + (y * width + x) * comps, 0, comps);
			uint32_t argb = (0xFF << 24) | (a[0] << 16) | (a[1] << 8) | (a[0]);
			for (int yy = 0; yy < scale; yy++) {
				for (int xx = 0; xx < scale; xx++) {
					BSP_LCD_DrawPixel(start_x + x * scale + xx, start_y + y * scale + yy, argb);
				}
			}
		}
	}
	uint32_t old_col = BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawRect(start_x, start_y, width * scale, height * scale);
	BSP_LCD_SetTextColor(old_col);
}

BaseType_t
write_fifo_to_buffer(uint32_t length)
{
#if 1	// display captured image
	/* Write the FIFO contents to disk. */
	uint16_t chunk = 0;

//	free(ptr_picture);
//	// jpeg pic size
//	unsigned int jpeg_size = length*sizeof(uint8_t);
//	// allocate memory to store jpeg picture
//	if((ptr_picture = malloc(jpeg_size)) == NULL){
//		myprintf("camera: ran out of memory\n\r");
//	}else{
//		myprintf("camera: allocated %d bytes of memory for picture\n\r", malloc_usable_size(ptr_picture));
//	}
	FILE *file=fmemopen(NULL,length+1,"ab+");
	for (uint16_t i = 0; length > 0; ++i) {

		chunk = MIN(length, BURST_READ_LENGTH);
		arduchip_burst_read(fifoBuffer, chunk);
		length -= chunk;
		fwrite(fifoBuffer,1,chunk,file);

		// maybe send the fifo buffer to LabVIEW for displaying ....

	}
	int x,y,comps;
	pjpeg_scan_type_t pScan_type;
	uint8_t *image=pjpeg_load_from_file(file,&x,&y,&comps,&pScan_type,1);
	display_bitmap((uint8_t*)image, x, y, comps, 60, 80, 2 );
#else
	// test image: make sure to build the project with -Og to show this static .bmp image
	// 		project properties -> C/C++ Build -> Settings -> Optimization | Optimize for debugging (-Og)
    BSP_LCD_DrawBitmap(80, 180, (uint8_t *)stlogo);
#endif
    osDelay(500);

	return pdTRUE;
}
uint8_t *pjpeg_load_from_file(FILE *file, int *x, int *y, int *comps, pjpeg_scan_type_t *pScan_type, int reduce)
{
   pjpeg_image_info_t image_info;
   int mcu_x = 0;
   int mcu_y = 0;
   uint row_pitch;
   uint8_t *pImage;
   uint8_t status;
   uint decoded_width, decoded_height;
   uint row_blocks_per_mcu, col_blocks_per_mcu;
   g_pInFile=file;
   *x = 0;
   *y = 0;
   *comps = 0;
   if (pScan_type) *pScan_type = PJPG_GRAYSCALE;

   if (!g_pInFile)
      return NULL;

   g_nInFileOfs = 0;

   fseek(g_pInFile, 0, SEEK_END);
   g_nInFileSize = ftell(g_pInFile);
   fseek(g_pInFile, 0, SEEK_SET);

   status = pjpeg_decode_init(&image_info, pjpeg_need_bytes_callback, NULL, (unsigned char)reduce);

   if (status)
   {
      printf("pjpeg_decode_init() failed with status %u\n", status);
      if (status == PJPG_UNSUPPORTED_MODE)
      {
         printf("Progressive JPEG files are not supported.\n");
      }

      fclose(g_pInFile);
      return NULL;
   }

   if (pScan_type)
      *pScan_type = image_info.m_scanType;

   // In reduce mode output 1 pixel per 8x8 block.
   decoded_width = reduce ? (image_info.m_MCUSPerRow * image_info.m_MCUWidth) / 8 : image_info.m_width;
   decoded_height = reduce ? (image_info.m_MCUSPerCol * image_info.m_MCUHeight) / 8 : image_info.m_height;

   row_pitch = decoded_width * image_info.m_comps;
   pImage = (uint8_t *)malloc(row_pitch * decoded_height);
   if (!pImage)
   {
      fclose(g_pInFile);
      return NULL;
   }

   row_blocks_per_mcu = image_info.m_MCUWidth >> 3;
   col_blocks_per_mcu = image_info.m_MCUHeight >> 3;

   for ( ; ; )
   {
      int y, x;
      uint8_t *pDst_row;

      status = pjpeg_decode_mcu();

      if (status)
      {
         if (status != PJPG_NO_MORE_BLOCKS)
         {
            printf("pjpeg_decode_mcu() failed with status %u\n", status);

            free(pImage);
            fclose(g_pInFile);
            return NULL;
         }

         break;
      }

      if (mcu_y >= image_info.m_MCUSPerCol)
      {
         free(pImage);
         fclose(g_pInFile);
         return NULL;
      }

      if (reduce)
      {
         // In reduce mode, only the first pixel of each 8x8 block is valid.
         pDst_row = pImage + mcu_y * col_blocks_per_mcu * row_pitch + mcu_x * row_blocks_per_mcu * image_info.m_comps;
         if (image_info.m_scanType == PJPG_GRAYSCALE)
         {
            *pDst_row = image_info.m_pMCUBufR[0];
         }
         else
         {
            uint y, x;
            for (y = 0; y < col_blocks_per_mcu; y++)
            {
               uint src_ofs = (y * 128U);
               for (x = 0; x < row_blocks_per_mcu; x++)
               {
                  pDst_row[0] = image_info.m_pMCUBufR[src_ofs];
                  pDst_row[1] = image_info.m_pMCUBufG[src_ofs];
                  pDst_row[2] = image_info.m_pMCUBufB[src_ofs];
                  pDst_row += 3;
                  src_ofs += 64;
               }

               pDst_row += row_pitch - 3 * row_blocks_per_mcu;
            }
         }
      }
      else
      {
         // Copy MCU's pixel blocks into the destination bitmap.
         pDst_row = pImage + (mcu_y * image_info.m_MCUHeight) * row_pitch + (mcu_x * image_info.m_MCUWidth * image_info.m_comps);

         for (y = 0; y < image_info.m_MCUHeight; y += 8)
         {
            const int by_limit = min(8, image_info.m_height - (mcu_y * image_info.m_MCUHeight + y));

            for (x = 0; x < image_info.m_MCUWidth; x += 8)
            {
               uint8_t *pDst_block = pDst_row + x * image_info.m_comps;

               // Compute source byte offset of the block in the decoder's MCU buffer.
               uint src_ofs = (x * 8U) + (y * 16U);
               const uint8_t *pSrcR = image_info.m_pMCUBufR + src_ofs;
               const uint8_t *pSrcG = image_info.m_pMCUBufG + src_ofs;
               const uint8_t *pSrcB = image_info.m_pMCUBufB + src_ofs;

               const int bx_limit = min(8, image_info.m_width - (mcu_x * image_info.m_MCUWidth + x));

               if (image_info.m_scanType == PJPG_GRAYSCALE)
               {
                  int bx, by;
                  for (by = 0; by < by_limit; by++)
                  {
                     uint8_t *pDst = pDst_block;

                     for (bx = 0; bx < bx_limit; bx++)
                        *pDst++ = *pSrcR++;

                     pSrcR += (8 - bx_limit);

                     pDst_block += row_pitch;
                  }
               }
               else
               {
                  int bx, by;
                  for (by = 0; by < by_limit; by++)
                  {
                     uint8_t *pDst = pDst_block;

                     for (bx = 0; bx < bx_limit; bx++)
                     {
                        pDst[0] = *pSrcR++;
                        pDst[1] = *pSrcG++;
                        pDst[2] = *pSrcB++;
                        pDst += 3;
                     }

                     pSrcR += (8 - bx_limit);
                     pSrcG += (8 - bx_limit);
                     pSrcB += (8 - bx_limit);

                     pDst_block += row_pitch;
                  }
               }
            }

            pDst_row += (row_pitch * 8);
         }
      }

      mcu_x++;
      if (mcu_x == image_info.m_MCUSPerRow)
      {
         mcu_x = 0;
         mcu_y++;
      }
   }

   fclose(g_pInFile);

   *x = decoded_width;
   *y = decoded_height;
   *comps = image_info.m_comps;

   return pImage;
}

unsigned char pjpeg_need_bytes_callback(unsigned char* pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data)
{
   uint n;
   pCallback_data;

   n = min(g_nInFileSize - g_nInFileOfs, buf_size);
   if (n && (fread(pBuf, 1, n, g_pInFile) != n))
      return PJPG_STREAM_READ_ERROR;
   *pBytes_actually_read = (unsigned char)(n);
   g_nInFileOfs += n;
   return 0;
}

