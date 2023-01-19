#include "arducam.h"
#include <spi.h>
#include <i2c.h>
extern int myprintf(const char *format, ...);

#define ARDUCAM_CS_SELECT  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define ARDUCAM_CS_RELEASE HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

/* Arduchip ----------------------------------------------------------------- */

/**
 * Read an 8-bit register value from the ArduCAM.
 */
uint8_t
arduchip_read_reg(uint8_t reg, uint8_t *ptr) {
	ARDUCAM_CS_SELECT;
	if (spi_read8(&hspi4, reg, ptr, 1) != DEVICES_OK) {
		return 0;
	}
	ARDUCAM_CS_RELEASE;
	return 1;
}

/**
 * Write an 8-bit register value to the ArduCAM.
 */
uint8_t
arduchip_write_reg(uint8_t reg, uint8_t value) {
	ARDUCAM_CS_SELECT;
	if (spi_write8_8(&hspi4, reg, value) != DEVICES_OK) {
		return 0;
	}
	ARDUCAM_CS_RELEASE;
	return 1;
}

/**
 * Initiate a burst read operation.
 */
uint8_t
arduchip_burst_read(uint8_t* buffer, uint16_t length) {
	/**
	 * Send a command byte addressing the burst read register.
	 * The response contains the first byte of data.
	 */
	ARDUCAM_CS_SELECT;
	if (spi_read8(&hspi4, ARDUCHIP_BURST_READ, buffer, 1) != DEVICES_OK) {
		ARDUCAM_CS_RELEASE;
		return 0;
	}

	/* Read the remaining bytes of data. */
	if (HAL_SPI_Receive(&hspi4, buffer + 1, length - 1, 1024) != HAL_OK) {
		ARDUCAM_CS_RELEASE;
		return 0;
	}

	ARDUCAM_CS_RELEASE;
	return 1;
}


/**
 * Read the ArduChip version.
 */
uint8_t
arduchip_chip(uint8_t* chipId) {
	/* Emit a few clock cycles so the ArduChip can get ready. */
	ARDUCAM_CS_RELEASE;
	spi_read8(&hspi4, 0x0, chipId, 1);
	spi_read8(&hspi4, 0x0, chipId, 1);
	spi_read8(&hspi4, 0x0, chipId, 1);
	spi_read8(&hspi4, 0x0, chipId, 1);

	/* Read the ArduChip version register. */
	*chipId = 0;
	if (!arduchip_read_reg(ARDUCHIP_VERSION, chipId)) {
		return 0;
	}
	return 1;
}

/**
 * Initialize the Arduchip.
 */
uint8_t
arduchip_detect() {
	/**
	 * Read the chip ID if present.
	 * Expect a supported chip ID.
	 */
	uint8_t chipId = 0;
	if (   !arduchip_chip(&chipId)
		|| (chipId != ARDUCHIP_5MP && chipId != ARDUCHIP_5MP_PLUS && chipId != ARDUCHIP_5MP_PLUS_REV )
	) {
		myprintf("camera/arduchip: not present\n");
		return 0;
	}

	/* Explicitly enable FIFO mode on the 5MP. */
	uint8_t flags = 0;
	if (chipId == ARDUCHIP_5MP || chipId == ARDUCHIP_5MP_PLUS_REV) {
		flags |= SITR_FIFO_MASK;
	}

	/**
	 * Configure the chip to use active-low VSYNC.
	 * Configure the chip to capture one frame.
	 */
	flags |= SITR_VSYNC_MASK;
	if (!arduchip_write_reg(ARDUCHIP_SITR, flags) ||
		!arduchip_write_reg(ARDUCHIP_CCR, CCR_FRAMES(1))
	) {
		myprintf("camera/arduchip: not ready\n\r");
		return 0;
	}

	myprintf("camera/arduchip: ready (%x)\n\r", chipId);
	return 1;
}

/**
 * Initiate a frame capture.
 */
uint8_t
arduchip_start_capture() {
	/**
	 * Clear the FIFO write done flag.
	 * Reset the FIFO read pointer.
	 * Reset the FIFO write pointer.
	 * Initiate a capture.
	 */
	if (   !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_CLEAR_MASK)
		|| !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_RDPTR_RST_MASK)
		|| !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_WRPTR_RST_MASK)
		|| !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_START_MASK)
	) {
		return 0;
	}

	return 1;
}

/**
 * Poll the capture complete flag.
 */
uint8_t
arduchip_capture_done(uint8_t* done) {
	uint8_t status = 0;
	if (arduchip_read_reg(ARDUCHIP_STATUS, &status)) {
		*done = (status & STATUS_FIFO_DONE_MASK) > 0;
		return 1;
	}
	return 0;
}

/**
 * Get the FIFO buffer length.
 */
BaseType_t
arduchip_fifo_length(uint32_t* length) {
	uint8_t a = 0, b = 0, c = 0;
	if (   !arduchip_read_reg(ARDUCHIP_FIFO_WRITE_0, &a)
		|| !arduchip_read_reg(ARDUCHIP_FIFO_WRITE_1, &b)
		|| !arduchip_read_reg(ARDUCHIP_FIFO_WRITE_2, &c)
	) {
		return pdFALSE;
	}
	*length = a | (b << 8) | ((c & 0x7) << 16);
	return pdTRUE;
}

/* OV5642 Sensor ------------------------------------------------------------ */

/**
 * Initialize the OV5642.
 */
uint8_t
ov5642_detect() {
	/**
	 * Read the chip ID from the sensor.
	 * Expect the OV5642.
	 */
	uint8_t high_byte, low_byte;
	if (   i2c_read16(OV5642_ADDRESS_W, OV5642_CHIP_ID_HIGH_BYTE, &high_byte, 1) != DEVICES_OK
		&& i2c_read16(OV5642_ADDRESS_W, OV5642_CHIP_ID_LOW_BYTE, &low_byte, 1)   != DEVICES_OK
		&& ((high_byte << 8) | low_byte) != OV5642_CHIP_ID
	) {
		myprintf("camera/ov5642: not present\n\r");
		return 0;
	}

	myprintf("camera/ov5642: ready\n\r");
	return 1;
}

/**
 * Configure the OV5642 for JPEG capture.
 */
uint8_t
ov5642_configure() {
	/* Perform a software reset of the camera sensor. */
	//if (i2c_write16_8(OV5642_ADDRESS_W, 0x3008, 0x80) != DEVICES_OK) {
	//	goto error;
	//}

	/* This delay appears to be important. */
	//vTaskDelay(100);

	/**
	 * TODO Writing the same register twice in a short period seems to have no
	 * affect. It would be better to merge the arrays into one array with all
	 * desired values.
	 */

	/* Initialize the registers of the camera sensor. */
	if (i2c_array16_8(OV5642_ADDRESS_W, ov5642_dvp_fmt_global_init) != DEVICES_OK) {
		goto error;
	}

	/* This delay appears to be important. */
	vTaskDelay(50);

	/* Configure the format and resolution. */
	if (   i2c_array16_8(OV5642_ADDRESS_W, ov5642_dvp_fmt_jpeg_qvga)          != DEVICES_OK
		&& i2c_write16_8(OV5642_ADDRESS_W, 0x4407, 0x0C)             != DEVICES_OK
	) {
		goto error;
	}

	return 1;

error:
	myprintf("camera/ov5642: configure failed\n\r");
	return 0;
}

/* Arducam module ----------------------------------------------------------- */

/**
 * Have the Arducam power down the OV5642.
 * This will reset all registers in the OV5642.
 */
uint8_t
arducam_enter_standby() {
	uint8_t gpio;
	if (   !arduchip_read_reg(ARDUCHIP_GPIO, &gpio)
		&& !arduchip_write_reg(ARDUCHIP_GPIO, gpio | GPIO_PWDN_MASK)
	) {
		return 0;
	}

	return 1;
}

/**
 * Have the Arducam power up the OV5642. Note that the sensor requires a few
 * milliseconds to stabilize, and the auto-exposure algorithm also requires
 * a few seconds as well before images are acceptable.
 */
uint8_t
arducam_exit_standby() {
	uint8_t gpio;
	if (   !arduchip_read_reg(ARDUCHIP_GPIO, &gpio)
		&& !arduchip_write_reg(ARDUCHIP_GPIO, gpio & ~GPIO_PWDN_MASK)
	) {
		return 0;
	}

	return 1;
}

