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

#ifndef MPU_SPI_H
#define MPU_SPI_H

/**
 * @brief
 * This file defines the interfaces to carry out SPI I/O operations with
 * mpu9x50 using dspal interfaces.
 *
 * Dspal abstracts the physical device as a file such as /dev/spi-8 and
 * provides posix APIs for I/O operations on the file.
 *
 * The interfaces defined here are used to read from or write
 * to certain register of MPU9x50 through SPI. MPU9x50 defines the packet
 * format for SPI communication. The first byte is the register address with
 * MSB representing read/write. The second byte is the value to write to the
 * specified register in write mode, or a byte placeholder in read mode.
 */

#include <stdint.h>

/**
 * Supported SPI frequency to talk to MPU9x50 slave device
 */
enum MPU_SPI_FREQUENCY {
	MPU_SPI_FREQUENCY_1MHZ = 1000000UL,
	MPU_SPI_FREQUENCY_5MHZ = 5000000UL,
	MPU_SPI_FREQUENCY_15MHZ = 15000000UL,
};

/**
 * Open the MPU SPI device specified with dev using dspal.
 *
 * @param[in]   dev     full path of the SPI device. E.g. /dev/spi-8
 * @return      0 on success, negative value on error
 */
int mpu_spi_open(const char *dev);

/**
 * Close the MPU SPI device. This must be called after mpu_spi_open()
 * succeeded and the device file is still open
 *
 * @return      0 on success, negative value on error
 */
int mpu_spi_close();

/**
 * Set register value
 *
 * @param[in]   reg    register address to set value on
 * @param[in]   val    the value that needs to be set to register
 * @return      0 on success, negative value  on error
 */
int mpu_spi_set_reg(int reg, uint8_t val);

/**
 * Set register value and verify the value is successfully set
 * This works only for R/W registers.
 *
 * @param[in]   reg    register address to set value on
 * @param[in]   val    the value that needs to be set to register
 * @param[in]   mask   bit mask to indicate the bits of val which needs to be
 *                     written and verified
 * @return      0 on success, negative value on error
 */
int mpu_spi_set_reg_verified(int reg, uint8_t val, uint8_t mask);

/**
 * Get register value
 *
 * @param[in]   reg    register address to set value on
 * @param[out]  val    the address to store the register value.
 * @return      0 on success, and negative value on error
 */
int mpu_spi_get_reg(int reg, uint8_t *val);

/**
 * Do bulk reading from certain starting register address.
 *
 * @param[in]   reg    the starting register address to read from
 * @param[in]   length total bytes to read starting from reg
 * @param[out]  buf    the address to store the read bytes
 * @return      0 on success, and negative value on error
 */
int mpu_spi_bulk_read(int reg, uint8_t length, uint8_t *buf);

/**
 * Configure the frequency in HZ to talk to the SPI slave device
 *
 * @param[in]   freq   frequency in HZ
 * @return      0 on success, and negative value on error
 */
int mpu_spi_configure_speed(enum MPU_SPI_FREQUENCY freq);

/**
 * Configure the GPIO interrupt for data ready interrupt mode in MPU9x50
 *
 * @param[in]  gpio_int_dev_id   GPIO interrupt device ID
 * @param[in]  callback          interrupt callback function pointer to register
 * @return     0 on success, and negative value on error
 */
int mpu_spi_configure_gpio_interrupt(int gpio_int_dev_id,
				     void(*callback)(void *),
				     void *user_context);

/**
 * Disable the GPIO interrupt
 *
 * @return     0 on success, and negative value on error
 */
int mpu_spi_disable_gpio_interrupt();

#endif // MPU_SPI_H
