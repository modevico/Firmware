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

/**
 * @file
 * This file declares the data structures and APIs for users to access mpu9x50
 * driver. This includes configuring the MPU9x50 chipset and obtaining the
 * sensor readings in poll mode or data ready interrupt mode.
 *
 * @par Configuring MPU9x50
 * mpu9x50 driver supporting configuring the sample rate, full scale range
 * and low pass filter for accelerometer, gyro and magnetometer.
 * For the details of mpu9x50 configuration and regsiter map, please see the
 * spec document.
 *
 * @par Reading MPU9x50 Sensor Data
 * MPU9x50 supports both I2C and SPI mechanism for users to read the sensor
 * data. In current driver implementation, only SPI mode is supported.
 *
 * @par
 * Sample source code for getting MPU9x50 sensor data is available at
 * @include mpu9x50_test.c
 */
#include <stdbool.h>
#include <stdint.h>
#include <mathlib.h>
/**
* @brief
* Utility macro functions to do unit conversion
*/
#define DEG_TO_RAD(x) ((x) * (M_PI_F) / 180.0)
#define G_TO_MS2(x)   ((x) * 9.80665)

/**
 * MPU9250 Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_REG_ADDR {
	MPU9250_REG_SMPLRT_DIV     = 25,
	MPU9250_REG_CONFIG         = 26,
	MPU9250_REG_GYRO_CONFIG    = 27,
	MPU9250_REG_ACCEL_CONFIG   = 28,
	MPU9250_REG_ACCEL_CONFIG2  = 29,
	MPU9250_REG_I2C_MST_CTRL   = 36,
	MPU9250_REG_I2C_SLV0_ADDR  = 37,
	MPU9250_REG_I2C_SLV0_REG   = 38,
	MPU9250_REG_I2C_SLV0_CTRL  = 39,
	MPU9250_REG_I2C_SLV4_ADDR  = 49,
	MPU9250_REG_I2C_SLV4_REG   = 50,
	MPU9250_REG_I2C_SLV4_DO    = 51,
	MPU9250_REG_I2C_SLV4_CTRL  = 52,
	MPU9250_REG_I2C_SLV4_DI    = 53,
	MPU9250_REG_I2C_MST_STATUS = 54,
	MPU9250_REG_INT_EN         = 56,
	MPU9250_REG_INT_STATUS     = 58,
	MPU9250_REG_I2C_MST_DELAY_CTRL = 103,
	MPU9250_REG_USER_CTRL      = 106,
	MPU9250_REG_PWR_MGMT1      = 107,
	MPU9250_REG_WHOAMI         = 117,
};

/**
 * MPU9250 Compass Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_COMPASS_REG_ADDR {
	MPU9250_COMP_REG_WIA       = 0x00,
	MPU9250_COMP_REG_CNTL1     = 0x0a,
	MPU9250_COMP_REG_ASAX      = 0x10,
	MPU9250_COMP_REG_ASAY      = 0x11,
	MPU9250_COMP_REG_ASAZ      = 0x12,
};

/**
 * Gyro Low Pass Filter Enum
 */
enum gyro_lpf_e {
	MPU9X50_GYRO_LPF_250HZ = 0,
	MPU9X50_GYRO_LPF_184HZ,
	MPU9X50_GYRO_LPF_92HZ,
	MPU9X50_GYRO_LPF_41HZ,
	MPU9X50_GYRO_LPF_20HZ,
	MPU9X50_GYRO_LPF_10HZ,
	MPU9X50_GYRO_LPF_5HZ,
	MPU9X50_GYRO_LPF_3600HZ_NOLPF,
	NUM_MPU9X50_GYRO_LPF
};

/**
 * Accelerometer Low Pass Filter Enum
 */
enum acc_lpf_e {
	MPU9X50_ACC_LPF_460HZ = 0,
	MPU9X50_ACC_LPF_184HZ,
	MPU9X50_ACC_LPF_92HZ,
	MPU9X50_ACC_LPF_41HZ,
	MPU9X50_ACC_LPF_20HZ,
	MPU9X50_ACC_LPF_10HZ,
	MPU9X50_ACC_LPF_5HZ,
	MPU9X50_ACC_LPF_460HZ_NOLPF,
	NUM_MPU9X50_ACC_LPF
};

/**
 * Gyro Full Scale Range Enum
 */
enum gyro_fsr_e {
	MPU9X50_GYRO_FSR_250DPS = 0,
	MPU9X50_GYRO_FSR_500DPS,
	MPU9X50_GYRO_FSR_1000DPS,
	MPU9X50_GYRO_FSR_2000DPS,
	NUM_MPU9X50_GYRO_FSR
};

/**
 * Accelerometor Full Scale Range Enum
 */
enum acc_fsr_e {
	MPU9X50_ACC_FSR_2G = 0,
	MPU9X50_ACC_FSR_4G,
	MPU9X50_ACC_FSR_8G,
	MPU9X50_ACC_FSR_16G,
	NUM_MPU9X50_ACC_FSR
};

#define MPU9X50_INTERNAL_SAMPLE_RATE_HZ    1000

/**
 * Sample rate for gyro and accelerometer. Here we only define the sample rate
 * supported in current driver. The compass sample rate is fixed 100Hz.
 */
enum gyro_sample_rate_e {
	MPU9x50_SAMPLE_RATE_100HZ = 0,
	MPU9x50_SAMPLE_RATE_200HZ,
	MPU9x50_SAMPLE_RATE_500HZ,
	MPU9x50_SAMPLE_RATE_1000HZ,
	NUM_MPU9X50_SAMPLE_RATE
};

/**
 * Sample rate for compass. The maximum compass sample rate in mpu9250 is 100Hz
 */
enum compass_sample_rate_e {
	MPU9x50_COMPASS_SAMPLE_RATE_100HZ = 0,
	NUM_MPU9X50_COMPASS_SAMPLE_RATE
};

/**
 * Full Scale Range of the magnetometer chip AK89xx in MPU9250
 */
#define MPU9250_AK89xx_FSR  4915 // from invensense doc

#define MPU9250_AKM_DEV_ID  0x48  // compass device ID

/**
 * Structure used to store the MPU9x50 configuration. The default
 * configuration is defined.
 */
struct mpu9x50_config {
	enum gyro_lpf_e            gyro_lpf;
	enum acc_lpf_e             acc_lpf;
	enum gyro_fsr_e            gyro_fsr;
	enum acc_fsr_e             acc_fsr;
	enum gyro_sample_rate_e    gyro_sample_rate;
	bool                       compass_enabled;
	enum compass_sample_rate_e compass_sample_rate;
	const char                *spi_dev_path;
};

struct mpu9x50_data {
	uint64_t timestamp;
	int16_t  temperature_raw;
	float    temperature;
	int16_t  accel_raw[3];
	float    accel_scaling;
	float    accel_range_m_s2;
	int16_t  gyro_raw[3];
	float    gyro_range_rad_s;
	float    gyro_scaling;
	bool     mag_data_ready;  /*< mag sample at 100HZ, not every mpu9x50 sample contains valid mag sample */
	int16_t  mag_raw[3];
	float    mag_range_ga;
	float    mag_scaling;
};

/**
 * @brief
 * Opens the mpu9250 device and configure the sensor using specified config
 * settings. The device path is part of the mpu9x50_config structure.
 * On success, this function returns a file descriptor for the mpu device.
 * The file descriptor should be used for all mpu driver operations.
 *
 * @param   config[in]   struct mpu9x50_config which stores the mpu9x50
 *                       configurations
 * @return  0 on success, negaitve integer value on error
 */
int mpu9x50_initialize(struct mpu9x50_config *config);

/**
 * @brief
 * Close the mpu9250 device.  This must be called after a successful call of
 * mpu9x50_initialize()
 *
 * @return  0 on success, negaitve integer value on error
 */
int mpu9x50_close();

/**
 * @brief
 * Validate the configuration.
 *
 * @param   config[in]   struct mpu9x50_config object which needs to be
 *                       validated
 * @return  0 if validation pass, negaitve integer value on error
 */
int mpu9x50_validate_configuration(struct mpu9x50_config *config);

/**
 * @brief
 * Detect the presence of mpu9x50 gyro/accelerometer by reading the whoami
 * register
 *
 * @return  0 on success, negative integer on error
 */
int mpu9x50_detect_gyro();

/**
 * @brief
 * Register Data Ready Interrupt in the following mode
 * active high, push-pull, pulse, FSYNC does not create INT.
 * @param   gpio_id[in]         gpio id used for data ready interrupt pin
 * @param   data_ready_isr[in]  function pointer to be called when interrupt fires
 *                              data stores the parsed sensor data, including
 *                              raw and scaled values. user_context is provided
 *                              by the user.
 * @param   context[in]         address of the context data provided by user
 * @return  0 on success, negative integer on error
 */
int mpu9x50_register_interrupt(int gpio_id,
			       void (*data_ready_isr)(struct mpu9x50_data *data, void *user_context),
			       void *context);

/**
 * @brief
 * Unregister the Data Ready Interrupt. This can ben called explicitly to
 * disable interrupt. Alternatively, if mpu9x50_close() is called when the
 * interrupt is still enabled, this will be called first.
 *
 * @return  0 on success or mpu device is not yet open,
 *          negative integer on error
 */
int mpu9x50_unregister_interrupt();

/**
 * Get Gyro LPF value in HZ, and store the value in gyro_lpf
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_gyro_lpf(int *gyro_lpf);

/**
 * Get Accel LPF value in HZ, and store the value in accel_lpf
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_accel_lpf(int *accel_lpf);

/**
 * Get Gyro/Accel sample rate in Hz, and store in sample_Rate
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_gyro_sample_rate(int *sample_rate);

/**
 * Get compass sample rate in Hz, and store in compass_sample_Rate
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_compass_sample_rate(int *compass_sample_rate);

/**
 * Get Gyro FSR, and store in gyro_fsr in DPS
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_gyro_fsr(int *gyro_fsr);

/**
 * Get Accel FSR in G, and store in accel_fsr
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_accel_fsr(int *accel_fsr);

/**
 * Get compass FSR, and store in compass_fsr
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_compass_fsr(int *compass_fsr);


/**
 * Get INT status, and store in int_status
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_int_status(uint8_t *int_status);
