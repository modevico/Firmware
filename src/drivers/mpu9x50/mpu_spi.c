/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <dev_fs_lib.h>
#include <dev_fs_lib_spi.h>
#include <dspal_errno.h>
#include "mpu_spi.h"

#define FARF_MEDIUM 1
#include <HAP_farf.h>

#define SPI_BUF_LEN   50

static int fd = -1;
static uint8_t spiTxBuf[SPI_BUF_LEN];
static uint8_t spiRxBuf[SPI_BUF_LEN];
static bool spi_dev_is_open = false;

int mpu_spi_open(const char *dev)
{
	fd = sys_open(dev, 0);

	if (fd > 0) {
		spi_dev_is_open = true;
		return SUCCESS;
	}

	return ERROR;
}

int mpu_spi_close()
{
	int ret;

	spi_dev_is_open = false;
	ret = sys_close(fd);
	fd = -1;

	return ret;
}

int mpu_spi_set_reg(int reg, uint8_t val)
{
	int retVal = 0;
	int bytes_written;

	FARF(ALWAYS, "mpu_spi_set_reg %d=%u", reg, val);

	if (!spi_dev_is_open) {
		FARF(HIGH, "mpu_spi_set_reg called withtout opening dev");
		return ERROR;
	}

	retVal = mpu_spi_configure_speed(MPU_SPI_FREQUENCY_1MHZ); // 1 MHz

	if (retVal != SUCCESS) {
		return retVal;
	}

	spiTxBuf[0] = reg; //register high bit=0 for write
	spiTxBuf[1] = val; // while we read the result, we need to clock another byte (0) into the sensor

	bytes_written = sys_write(fd, (char *)spiTxBuf, 2);

	if (bytes_written != 2) {
		FARF(HIGH, "error sys_write: ret = %d", bytes_written);
		return ERROR;
	}

	return SUCCESS;
}

int mpu_spi_configure_speed(enum MPU_SPI_FREQUENCY freq)
{
	struct dspal_spi_ioctl_set_bus_frequency bus_freq;

	if (!spi_dev_is_open || fd <= 0) {
		FARF(HIGH, "mpu_spi_configure_speed called without opening spi device");
		return ERROR;
	}

	bus_freq.bus_frequency_in_hz = freq;

	return sys_ioctl(fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq);
}

int mpu_spi_get_reg(int reg, uint8_t *val)
{
	int retVal;
	struct dspal_spi_ioctl_read_write read_write;

	if (!spi_dev_is_open || fd <= 0) {
		FARF(HIGH, "mpu_spi_get_reg called without opening spi device");
		return ERROR;
	}

	retVal = mpu_spi_configure_speed(MPU_SPI_FREQUENCY_1MHZ);

	if (retVal != SUCCESS) {
		FARF(HIGH, "mpu_spi_get_reg: error configuring speed %d", retVal);
		return retVal;
	}

	spiTxBuf[0] = reg | 0x80; //register high bit=1 for read

	read_write.read_buffer = spiRxBuf;
	read_write.read_buffer_length = 2;
	read_write.write_buffer = spiTxBuf;
	read_write.write_buffer_length = 2;
	retVal = sys_ioctl(fd, SPI_IOCTL_RDWR, &read_write);

	if (retVal != 2) {
		FARF(HIGH, "mpu_spi_get_reg error read/write ioctl: %d", retVal);
		return retVal;
	}

	*val = spiRxBuf[1];

	FARF(MEDIUM, "mpu_spi_get_reg %d=%d", reg, *val);

	return SUCCESS;
}

int mpu_spi_set_reg_verified(int reg, uint8_t val, uint8_t mask)
{
	int retVal;
	uint8_t b;
	int retry = 5;
	bool err_seen;

	while (retry) {
		err_seen = FALSE;
		--retry;
		retVal = mpu_spi_set_reg(reg, val);

		if (retVal != SUCCESS) {
			err_seen = TRUE;
			continue;
		}

		retVal = mpu_spi_get_reg(reg, &b);

		if (retVal != SUCCESS) {
			err_seen = TRUE;
			continue;
		}

		if ((b & mask) != val) {
			continue;

		} else {
			FARF(MEDIUM, "mpu_spi_set_reg_verified succ for reg %d=%d", reg, val);
			return SUCCESS;
		}
	}

	if (err_seen) {
		FARF(HIGH, "mpu_spi_set_reg_verified failed for reg %d. Error %d.",
		     reg, retVal);

	} else {
		FARF(HIGH, "mpu_spi_set_reg_verified failed for reg %d. %d!=%d",
		     reg, val, b);
	}

	return retVal;
}

int mpu_spi_bulk_read(int reg, uint8_t length, uint8_t *buf)
{
	int retVal = SUCCESS;
	int transfer_bytes = 1 + length; // first byte is address
	struct dspal_spi_ioctl_read_write read_write;

	if (transfer_bytes > SPI_BUF_LEN) {
		FARF(HIGH, "SPI Bytes requested (%u) exceed buffer size (%u)",
		     length, SPI_BUF_LEN - 1);
		return ERROR;
	}

	// mpu9250 SPI supports up to 20 MHz for reading registers
	// Note: 20MHz SPI read sometimes gets 0 register value, 5MHz seems stable
	retVal = mpu_spi_configure_speed(MPU_SPI_FREQUENCY_15MHZ);

	if (retVal != SUCCESS) {
		FARF(HIGH, "mpu_spi_bulk_read: error configuring speed %d", retVal);
		return retVal;
	}

	spiTxBuf[0] = reg | 0x80; //register high bit=1 for read

	read_write.read_buffer = spiRxBuf;
	read_write.read_buffer_length = transfer_bytes;
	read_write.write_buffer = spiTxBuf;
	read_write.write_buffer_length = transfer_bytes;
	retVal = sys_ioctl(fd, SPI_IOCTL_RDWR, &read_write);

	if (retVal != transfer_bytes) {
		FARF(HIGH, "mpu_spi_bulk_read error read/write ioctl: %d", retVal);
		return retVal;
	}

	memcpy(buf, spiRxBuf + 1, transfer_bytes - 1);

	return SUCCESS;
}

int mpu_spi_configure_gpio_interrupt(int gpio_int_dev_id,
				     void(*callback)(void *),
				     void *user_context)
{
	if (!spi_dev_is_open || fd <= 0) {
		FARF(HIGH, "mpu_spi_configure_gpio_interrupt() called without opening spi device");
		return ERROR;
	}

	struct dspal_spi_ioctl_set_gpio_interrupt gpio_int = {
		.enable_gpio_interrupt = true,
		.gpio_int_dev_id = gpio_int_dev_id,
		.gpio_int_callback = (spi_gpio_int_func_ptr_t)callback,
		.user_context = user_context,
	};

	return sys_ioctl(fd, SPI_IOCTL_SET_GPIO_INTERRUPT, &gpio_int);
}

int mpu_spi_disable_gpio_interrupt()
{
	if (!spi_dev_is_open || fd <= 0) {
		FARF(HIGH, "mpu_spi_configure_gpio_interrupt() called without opening spi device");
		return ERROR;
	}

	struct dspal_spi_ioctl_set_gpio_interrupt gpio_int = {
		.enable_gpio_interrupt = false,
		.gpio_int_dev_id = -1,
		.gpio_int_callback = NULL,
	};

	return sys_ioctl(fd, SPI_IOCTL_SET_GPIO_INTERRUPT, &gpio_int);
}
