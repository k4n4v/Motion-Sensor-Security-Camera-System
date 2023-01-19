/**
 * Simple SPI/I2C interface to the ArduCAM 5MP module.
 */

#ifndef _ARDUCAM_H_
#define _ARDUCAM_H_

#include <stm32f4xx.h>
#include <stm32f4xx_hal_conf.h>

#include <spi.h>
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Device addresses */
#define OV5642_ADDRESS_W         0x78
#define OV5642_ADDRESS_R         0x79

/* Register addresses */
#define ARDUCHIP_TEST            0x00
#define ARDUCHIP_CCR             0x01
#define ARDUCHIP_SITR            0x03
#define ARDUCHIP_FIFO_CR         0x04
#define ARDUCHIP_GPIO			 0x06
#define ARDUCHIP_VERSION         0x40
#define ARDUCHIP_STATUS          0x41
#define ARDUCHIP_BURST_READ      0x3C
#define ARDUCHIP_SINGLE_READ     0x3D
#define ARDUCHIP_FIFO_WRITE_0    0x42
#define ARDUCHIP_FIFO_WRITE_1    0x43
#define ARDUCHIP_FIFO_WRITE_2    0x44

#define OV5642_CHIP_ID_HIGH_BYTE 0x300A
#define OV5642_CHIP_ID_LOW_BYTE  0x300B

/* Register constants */
#define ARDUCHIP_5MP             0x41
#define ARDUCHIP_5MP_PLUS        0x62
#define ARDUCHIP_5MP_PLUS_REV    0x73

#define OV5642_CHIP_ID           0x5642

/* Register masks */
#define SITR_VSYNC_MASK          0x02
#define SITR_FIFO_MASK           0x10
#define SITR_POWER_MASK          0x40
#define GPIO_PWDN_MASK		     0x02
#define FIFO_CLEAR_MASK    		 0x01
#define FIFO_START_MASK    		 0x02
#define FIFO_RDPTR_RST_MASK      0x10
#define FIFO_WRPTR_RST_MASK      0x20
#define STATUS_FIFO_DONE_MASK    0x08

/* Register access macros */
#define CCR_FRAMES(n) (n & 0x3)

/* Arduchip API */
uint8_t arduchip_burst_read(uint8_t* buffer, uint16_t length);
uint8_t arduchip_detect();
uint8_t arduchip_start_capture();
uint8_t arduchip_capture_done(uint8_t* done);
BaseType_t arduchip_fifo_length(uint32_t* length);
uint8_t arducam_enter_standby();
uint8_t arducam_exit_standby();

/* OV5642 API */
uint8_t ov5642_detect();
uint8_t ov5642_configure();

/* OV5642 register configuration arrays */
extern RegisterTuple16_8 ov5642_dvp_fmt_global_init[];
extern RegisterTuple16_8 ov5642_dvp_fmt_jpeg_qvga[];
extern RegisterTuple16_8 ov5642_res_720P[];
extern RegisterTuple16_8 ov5642_res_1080P[];

extern RegisterTuple16_8 sensor_reg_ov5642_RAW[];
extern RegisterTuple16_8 OV5642_QVGA_Preview[];




/////////////////////////////////// Functions for camera task ///////////////////////////////////////

/* Function prototypes. ----------------------------------------------------- */
//osTimerId xCameraTimer;
//osTimerId xCameraTimerPhoto;
//
//uint8_t camera_task_create();
////void camera_timer_elapsed(TimerHandle_t xTimer __attribute__((unused)));
////void camera_timer_photo(TimerHandle_t xTimer __attribute__((unused)));
//void camera_timer_elapsed();
//void camera_timer_photo();
//void camera_task_message_loop();
//void camera_tell(uint16_t message, uint32_t param1);
//
//#define CAMERA_TASK_NAME "CAMR"
//#define CAMERA_TASK_STACK_SIZE 1024
//typedef struct _Msg {
//	uint16_t message;
//	uint32_t param1;
//} Msg;

#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )

/**
 * Camera task messages --------------------------------------------------------
 */
#define MSG_CAMERA_BASE                0x400
#define MSG_CAMERA_SETUP               (MSG_CAMERA_BASE + 1)
#define MSG_CAMERA_MOUNT_SD            (MSG_CAMERA_BASE + 2)
#define MSG_CAMERA_CLEAN_SD            (MSG_CAMERA_BASE + 3)
#define MSG_CAMERA_ENTER_STANDBY       (MSG_CAMERA_BASE + 4)
#define MSG_CAMERA_EXIT_STANDBY        (MSG_CAMERA_BASE + 5)

#define MSG_CAMERA_CAPTURE_TIMER_START (MSG_CAMERA_BASE + 7)
#define MSG_CAMERA_CAPTURE_TIMER_STOP  (MSG_CAMERA_BASE + 8)
#define MSG_CAMERA_CAPTURE_INITIATE    (MSG_CAMERA_BASE + 9)
#define MSG_CAMERA_CAPTURE_AWAIT_DONE  (MSG_CAMERA_BASE + 10)
#define MSG_CAMERA_CAPTURE_WRITE_DISK  (MSG_CAMERA_BASE + 11)

#define MSG_CAMERA_WRITE_SAMPLES       (MSG_CAMERA_BASE + 12)
#define MSG_CAMERA_ROTATE_LOG          (MSG_CAMERA_BASE + 13)

/**
 * Sensor task messages --------------------------------------------------------
 */
#define MSG_SENSOR_BASE                0x500
#define MSG_SENSOR_SETUP               (MSG_SENSOR_BASE + 1)
#define MSG_SENSOR_SAMPLE			   (MSG_SENSOR_BASE + 2)
#define MSG_SENSOR_ROTATE_LOG		   (MSG_SENSOR_BASE + 3)
#define MSG_SENSOR_TIMER_START		   (MSG_SENSOR_BASE + 4)
#define MSG_SENSOR_TIMER_STOP  		   (MSG_SENSOR_BASE + 5)


#define ERROR_ON_NULL(retVal) if ((retVal) == NULL)   { goto error; }
#define ERROR_ON_FAIL(retVal) if ((retVal) == pdFAIL) { goto error; }


/**
 * Phases of recovery from standby mode.
 */
#define RECOVER_POWER    0
#define RECOVER_CONFIG   1
#define RECOVER_READY    2

/**
 * Recovery times when sensor is exiting standby in ticks.
 */
#define STANDBY_POWER_STABILIZE    1000
#define STANDBY_EXPOSURE_STABILIZE 2000

/**
 * Interval on which to capture photos in ticks.
 */
#define CAMERA_PHOTO_INTERVAL 60000

/**
 * Effective interval for camera timer which includes recovery from standby.
 */
#define CAMERA_TIMER_INTERVAL \
	(CAMERA_PHOTO_INTERVAL - STANDBY_POWER_STABILIZE - STANDBY_EXPOSURE_STABILIZE)

/**
 * Timeout when polling capture complete in ticks.
 */
#define CAPTURE_TIMEOUT 5000

/**
 * Number of consecutive capture failures before task is flagged unhealthy.
 */
#define CAPTURE_FAILED_THRESHOLD 10

/**
 * Buffer size for burst read operation in bytes.
 */
#define BURST_READ_LENGTH 512u

/* Externals and IPC -------------------------------------------------------- */

/**
 * Message queue for the camera task.
 */
extern osMessageQId xCameraQueue;

/* Function prototypes. ----------------------------------------------------- */
uint8_t camera_task_create();
void camera_tell(uint16_t message, uint32_t param1);




#ifdef __cplusplus
}
#endif

#endif /* _ARDUCAM_H_ */

