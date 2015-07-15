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

#include <unistd.h>
#include <dspal_errno.h>
#include <dspal_time.h>
#include <dev_fs_lib.h>
#include <dev_fs_lib_spi.h>
#include "mpu9x50.h"
#include "mpu_spi.h"

#define FARF_MEDIUM 1
#include <HAP_farf.h>

//===========================================================================
// Static Variable Declaration
//===========================================================================
static bool is_mpu9x50_open = false;  /*< indicate if mpu9x50 device is open */
static bool is_mpu9x50_dri_enabled = false; /*< indicate if mpu9x50 data ready interrupt is enabled */
static float compass_sens_adj[3];
void (*user_data_ready_isr)(struct mpu9x50_data *data, void *user_context) = NULL;
struct mpu9x50_data sensor_data;
static void mpu9x50_process_temperature(uint8_t *data);
static void mpu9x50_process_accel(uint8_t *data);
static void mpu9x50_process_gyro(uint8_t *data);
static int mpu9x50_process_compass(uint8_t *data);

//===========================================================================
// Static Function Declaration
//
// The functions declared here are for internal use only. mpu9x50 driver users
// shall not directly use these functions.
//===========================================================================

/**
 * @brief
 * Initialize the gyro module in mpu9x50 with the specified configuraitons
 *
 * @param   config[in]   struct mpu9x50_config object which needs to be
 *                       validated
 * @return  0 on success, negative integer on error
 */
static int mpu9x50_initialize_gyro(struct mpu9x50_config *config);

/**
 * @brief
 * Initialize the compass module in mpu9x50 with the specified configuraitons
 *
 * @param   config[in]   struct mpu9x50_config object which needs to be
 *                       validated
 * @return  0 on success, negative integer on error
 */
static int mpu9x50_initialize_compass(struct mpu9x50_config *config);

/**
 * @brief
 * Detect the compass by reading the compass whoami register. This returns error
 * if called without opening device.
 *
 * @return  0 on success, negative integer on error
 */
static int mpu9x50_detect_compass();

/**
 * @brief
 * Write to compass register through SPI interface and internal I2C channel
 *
 * @param   reg[in]    compass register address to write the data to
 * @param   val[in]    the value to be written to the register
 * @return  0 on success, negative integer on error
 */
static int compass_write_register(uint8_t reg, uint8_t val);

/**
 * @brief
 * Read from the specified compass register through SPI interface and
 * internal I2C channel
 *
 * @param   reg[in]    compass register address to read the data from
 * @param   val[out]   the address to store the reading result
 * @return  0 on success, negative integer on error
 */
static int compass_read_register(uint8_t reg, uint8_t *val);

/**
 * @brief
 *
 * Get the compass sensitivity adjustment.
 * This function reads the ASA sensitiviy adjustment value from FUSE ROM. Then
 * the adjustment measurement value is computed with the following formula and
 * the result is stored at the address pointed by compass_sens_adj.
 *
 *      compass_sens_adj = (ASA-128) * 0.5/ 128 + 1
 *
 * @param   compass_sens_adj[out]   3 element array to store the sensitivity
 *                                  measurement result.
 * @return  0 on success, negative integer on error
 */
static int mpu9x50_get_compass_senitivity_adjustment(float *compass_sens_adj);

/**
 * @brief
 * Start the continuous measurment 2 on compass
 * [NOTE]: The invense spec defines two continuous measurement modes. Not sure
 * of the difference between them. But continuous meansurement mode 2 has been
 * working well for us.
 *
 * @return  0 on success, negative integer on error
 */
static int mpu9x50_start_compass_sampling();

static void mpu9x50_data_ready_isr(void *context);

static inline int gyro_lpf_enum_to_hz(enum gyro_lpf_e lpf)
{
	switch (lpf) {
	case MPU9X50_GYRO_LPF_250HZ:
		return 250;

	case MPU9X50_GYRO_LPF_184HZ:
		return 184;

	case MPU9X50_GYRO_LPF_92HZ:
		return 92;

	case MPU9X50_GYRO_LPF_41HZ:
		return 41;

	case MPU9X50_GYRO_LPF_20HZ:
		return 20;

	case MPU9X50_GYRO_LPF_10HZ:
		return 10;

	case MPU9X50_GYRO_LPF_5HZ:
		return 5;

	default:
		return 0;
	}
}

static inline int accel_lpf_enum_to_hz(enum acc_lpf_e lpf)
{
	switch (lpf) {
	case (MPU9X50_ACC_LPF_460HZ):
		return 460;

	case (MPU9X50_ACC_LPF_184HZ):
		return 184;

	case (MPU9X50_ACC_LPF_92HZ):
		return 92;

	case (MPU9X50_ACC_LPF_41HZ):
		return 41;

	case (MPU9X50_ACC_LPF_20HZ):
		return 20;

	case (MPU9X50_ACC_LPF_10HZ):
		return 10;

	case (MPU9X50_ACC_LPF_5HZ):
		return 5;

	default:
		return 0;
	}
}

/**
 * @brief
 * Convert the gyro_sample_rate_e enum to actual integer rate in HZ
 * @return   integer rate value in HZ, -1 if rate is not supported
 */
static inline int gyro_sample_rate_enum_to_hz(enum gyro_sample_rate_e rate)
{
	switch (rate) {
	case (MPU9x50_SAMPLE_RATE_100HZ):
		return 100;

	case (MPU9x50_SAMPLE_RATE_200HZ):
		return 200;

	case (MPU9x50_SAMPLE_RATE_500HZ):
		return 500;

	case (MPU9x50_SAMPLE_RATE_1000HZ):
		return 1000;

	default:
		return -1;
	}
}

/**
 * @brief
 * Convert the compass_sample_rate_e enum to actual integer rate in HZ
 * @return   integer rate value in HZ, -1 if rate is not supported
 */
static inline int compass_sample_rate_enum_to_hz(enum compass_sample_rate_e rate)
{
	switch (rate) {
	case (MPU9x50_COMPASS_SAMPLE_RATE_100HZ):
		return 100;

	default:
		return -1;
	}
}

static inline int gyro_fsr_enum_to_dps(enum gyro_fsr_e fsr)
{
	switch (fsr) {
	case (MPU9X50_GYRO_FSR_250DPS):
		return 250;

	case (MPU9X50_GYRO_FSR_500DPS):
		return 500;

	case (MPU9X50_GYRO_FSR_1000DPS):
		return 1000;

	case (MPU9X50_GYRO_FSR_2000DPS):
		return 2000;

	default:
		return -1;
	}
}

//===========================================================================
//  Function Implementations
//===========================================================================

int mpu9x50_validate_configuration(struct mpu9x50_config *config)
{
	if (config == NULL) {
		FARF(HIGH, "config is NULL");
		return ERROR;
	}

	if (config->gyro_lpf >= NUM_MPU9X50_GYRO_LPF) {
		FARF(HIGH, "invalid gyro lpf: %u", config->gyro_lpf);
		return ERROR;
	}

	if (config->acc_lpf >= NUM_MPU9X50_ACC_LPF) {
		FARF(HIGH, "invalid acc lpf: %u", config->acc_lpf);
		return ERROR;
	}

	if (config->gyro_fsr >= NUM_MPU9X50_GYRO_FSR) {
		FARF(HIGH, "invalid gyro fsr: %u", config->gyro_fsr);
		return ERROR;
	}

	if (config->acc_fsr >= NUM_MPU9X50_ACC_FSR) {
		FARF(HIGH, "invalid acc fsr: %u", config->acc_fsr);
		return ERROR;
	}

	if (config->gyro_sample_rate >= NUM_MPU9X50_SAMPLE_RATE) {
		FARF(HIGH, "invalid sample rate: %u", config->gyro_sample_rate);
		return ERROR;
	}

	if (config->compass_enabled &&
	    config->compass_sample_rate >= NUM_MPU9X50_COMPASS_SAMPLE_RATE) {
		FARF(HIGH, "invalid compass sample rate: %u", config->compass_sample_rate);
		return ERROR;
	}

	if (config->spi_dev_path == NULL) {
		FARF(HIGH, "spi dev path is NULL");
		return ERROR;
	}

	FARF(MEDIUM, "mpu9x50 configuration validated");
	return SUCCESS;
}

int mpu9x50_initialize(struct mpu9x50_config *config)
{
	FARF(MEDIUM, "enter mpu9x50_initialize");

	// Validate the config object before carrying out the initialization
	if (mpu9x50_validate_configuration(config) < 0) {
		return ERROR;
	}

	// open mpu spi device
	if (mpu_spi_open(config->spi_dev_path) != SUCCESS) {
		return ERROR;
	}

	is_mpu9x50_open = true;

	// Initialize MPU9250, including
	// 1. reset device and select internal 20MHz oscillator as clock source
	mpu_spi_set_reg(MPU9250_REG_PWR_MGMT1, 0x80);
	usleep(100000);

	// 2. disable I2C
	mpu_spi_set_reg(MPU9250_REG_USER_CTRL, 0x10);

	// 3. take device out of sleep
	mpu_spi_set_reg(MPU9250_REG_PWR_MGMT1, 0x00);

	// 4. detect gyro
	if (mpu9x50_detect_gyro() != SUCCESS) {
		FARF(HIGH, "gyro not detected!");
		return ERROR;
	}

	// 5. initialize gyro using configurations stored in config
	if (mpu9x50_initialize_gyro(config) != SUCCESS) {
		FARF(HIGH, "error initializing gyro!");
		return ERROR;
	}

	// 6. detect and initialize compass if enabled, using configurations stored
	//    in config
	if (config->compass_enabled) {
		if (mpu9x50_initialize_compass(config) != SUCCESS) {
			FARF(HIGH, "error initializing compass!");
			return ERROR;
		}
	}

	// Disable INT
	if (mpu_spi_set_reg_verified(MPU9250_REG_INT_EN, 0x00, 0x01) != SUCCESS) {
		FARF(ALWAYS, "failed to dsiable INT");
		return ERROR;
	}

	// Read out all the configurations and initialize the sensor_data structure
	int gyro_lpf;
	int accel_lpf;
	int gyro_sample_rate;
	int compass_sample_rate;
	int gyro_fsr;
	int accel_fsr;
	int compass_fsr;
	uint8_t int_status;
	int result = 0;

	result |= mpu9x50_get_gyro_lpf(&gyro_lpf);
	result |= mpu9x50_get_accel_lpf(&accel_lpf);
	result |= mpu9x50_get_gyro_sample_rate(&gyro_sample_rate);
	result |= mpu9x50_get_compass_sample_rate(&compass_sample_rate);
	result |= mpu9x50_get_gyro_fsr(&gyro_fsr);
	result |= mpu9x50_get_accel_fsr(&accel_fsr);
	result |= mpu9x50_get_compass_fsr(&compass_fsr);
	result |= mpu9x50_get_int_status(&int_status);

	if (result != 0) {
		FARF(ALWAYS, "Error reading the mpu settings");
		mpu9x50_close();
		return ERROR;
	}

	FARF(MEDIUM, "MPU9x50 configurations: gyro_lpf %dHz, accel_lpf %dHz int_status %d",
	     gyro_lpf, accel_lpf, int_status);
	FARF(MEDIUM, "MPU9x50 configurations: gyro_sample_rate %dHz compass_sample_rate %dHz",
	     gyro_sample_rate, compass_sample_rate);
	FARF(MEDIUM, "MPU9x50 configurations: gyro_fsr %dDPS accel_fsr %dG, compass_fsr %d",
	     gyro_fsr, accel_fsr, compass_fsr);

	// Compute accel scale factors using 32768 LSB/g (16-bit register holding +ve and -ve values)
	// scale to m/s^2
	sensor_data.accel_range_m_s2 = G_TO_MS2(accel_fsr);
	sensor_data.accel_scaling = sensor_data.accel_range_m_s2 / 32768.0;

	// Compute gyro scale factors using 32768 LSB/g (16-bit register holding +ve and -ve values)
	// scale to rad/s in SI units
	sensor_data.gyro_range_rad_s = DEG_TO_RAD(gyro_fsr);
	sensor_data.gyro_scaling = sensor_data.gyro_range_rad_s / 32768.0;

	// Compute gyro scale factors using 32768 LSB / 0.15 microTesla
	// 16-bit register holding +ve and -ve values
	// Compass_fsr is in microTesla, scale to gauss units (1 gauss is 10000 tesla)
	sensor_data.mag_range_ga = 0.01 * compass_fsr;
	sensor_data.mag_scaling = sensor_data.mag_range_ga / 32768.0;

	FARF(MEDIUM, "MPU9x50 configurations: accel_scaling %f gyro_scaling %f "
	     "mag_scaling %f", sensor_data.accel_scaling, sensor_data.gyro_scaling,
	     sensor_data.mag_scaling);

	FARF(MEDIUM, "exit mpu9x50_initialize");

	return SUCCESS;
}

int mpu9x50_close()
{
	FARF(MEDIUM, "enter mpu9x50_close");

	// Disable Data Ready Interrupt if enabled
	if (is_mpu9x50_dri_enabled) {
		FARF(MEDIUM, "unregistering mpu9x50 data ready interrupt");
		mpu9x50_unregister_interrupt();
	}

	// if this is called without mpu9x50 device opened, simply return SUCCESS
	if (!is_mpu9x50_open) {
		return SUCCESS;
	}

	// TODO: clean up work and close the device
	return mpu_spi_close();
}

int mpu9x50_detect_gyro()
{
	int retry = 0; // detect gyro, retry 10 times
	uint8_t b = 0;

	// if this is called without mpu9x50 device opened, return ERROR
	if (!is_mpu9x50_open) {
		FARF(ALWAYS, "mpu9x50_detect_gyro() called without opening mpu device");
		return ERROR;
	}

	FARF(MEDIUM, "Detecting MPU9250 gyro");

	while (retry < 10) {
		// get version (expecting 0x71 for the 9250)
		mpu_spi_get_reg(MPU9250_REG_WHOAMI, &b);

		if (b == 0x71) {
			break;
		}

		retry++;
	}

	if (b != 0x71) {
		return ERROR;
	}

	FARF(MEDIUM, "MPU9250 gyro detected after %d retries", retry);

	return SUCCESS;
}

int mpu9x50_initialize_gyro(struct mpu9x50_config *config)
{
	uint8_t sample_rate_div;
	int sample_rate;

	// calculate the sample rate div
	sample_rate = gyro_sample_rate_enum_to_hz(config->gyro_sample_rate);

	if (sample_rate == -1) {
		FARF(HIGH, "unsupported sample rate %d", config->gyro_sample_rate);
		return ERROR;
	}

	// SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
	sample_rate_div = MPU9X50_INTERNAL_SAMPLE_RATE_HZ / sample_rate - 1;

	// gyro register init data
	// end list with register 0
	struct {uint8_t r; uint8_t v; uint8_t mask;} reg_init[] = {
		// Turn off DMP interrupt and Data Ready interrupt
		{MPU9250_REG_INT_EN, 0x00, 0xff},

		// SAMPLE_RATE
		{MPU9250_REG_SMPLRT_DIV, sample_rate_div, 0xff},

		// enable FSYNC sampling in TEMP_OUT_L[0];
		// Gyro LPF bandwidth
		{MPU9250_REG_CONFIG, 0x08 + config->gyro_lpf, 0x7f},

		// 20Hz Accel LPF
		// averaging; 19.8 ms delay
		// FCHOICE=1 (accel_fchoice_b=0) A_DLPFCFG=4
		{MPU9250_REG_ACCEL_CONFIG2, 0x00 + config->acc_lpf, 0x0f},

		// Accel Full Scale
		{MPU9250_REG_ACCEL_CONFIG, 0x00 + (config->acc_fsr << 3), 0xff},

		// gyro resolution
		{MPU9250_REG_GYRO_CONFIG, 0x00 + (config->gyro_fsr << 3), 0xff},

		// I2C_MST_CTRL
		// don't wait for external I2C data (WAIT_FOR_ES = 0),
		// 500 kHz I2C operation.
		// NOTE: I2C rate less than 500kHz may be problematic in DspAL context.
		// We observed that the magnetometer data ready flag may be cleared
		// when data ready interrupt fires. Not sure of the exact reason
		{MPU9250_REG_I2C_MST_CTRL, 0x09, 0xff},

		// USER_CTRL enable master I2C I/F
		{MPU9250_REG_USER_CTRL, 0x20, 0xff},

		{0x00, 0x00, 0x00} // end of list
	};

	// gyro register init loop
	int i = 0;

	while (reg_init[i].r != 0) {
		int retVal = mpu_spi_set_reg_verified(reg_init[i].r, reg_init[i].v,
						      reg_init[i].mask);

		if (retVal != SUCCESS) {
			FARF(HIGH, "error initializing gyro reg %u, error %d. Aborting!",
			     reg_init[i].r, retVal);
			return ERROR;
		}

		i++;
	}

	return SUCCESS;
}

int mpu9x50_initialize_compass(struct mpu9x50_config *config)
{
	int retVal;
	int i;
	int compass_sample_rate;
	int gyro_sample_rate;
	uint8_t i2c_mst_delay;

	if (!config->compass_enabled) {
		return ERROR;
	}

	// Validate the gyro and compass sample rate
	gyro_sample_rate = gyro_sample_rate_enum_to_hz(config->gyro_sample_rate);

	if (gyro_sample_rate == -1) {
		FARF(HIGH, "unsupported gyro sample rate %d", config->gyro_sample_rate);
		return ERROR;
	}

	compass_sample_rate = compass_sample_rate_enum_to_hz(config->compass_sample_rate);

	if (compass_sample_rate == -1) {
		FARF(ALWAYS, "unsupported compass sample rate %d",
		     config->compass_sample_rate);
		return ERROR;
	}

	// detect compass presence by reading whoami register
	if (mpu9x50_detect_compass() != SUCCESS) {
		return ERROR;
	}

	// get compass calibraion data from Fuse ROM
	if (mpu9x50_get_compass_senitivity_adjustment(compass_sens_adj) != SUCCESS) {
		return ERROR;
	}

	// I2C_MST_DLY = (gryo_sample_rate / campass_sample_rate - 1)
	i2c_mst_delay = gyro_sample_rate / compass_sample_rate - 1;
	FARF(ALWAYS, "i2c_mst_dly = %u", i2c_mst_delay);

	// configure compass register init data
	// end list with register 0
	struct {uint8_t r; uint8_t v; uint8_t mask;} reg_init[] = {
		// set I2C_MST_DLY for I2C_MST mode
		// Note: even though this field is in I2C_SLV4_CTRL register, but it
		// applies to all slaves
		{MPU9250_REG_I2C_SLV4_CTRL, i2c_mst_delay, 0xff},

		// I2C_MST_DELAY_CTRL, delay access slave 4 every (1+I2C_MST_DLY) samples
		// no delay shadowing
		{MPU9250_REG_I2C_MST_DELAY_CTRL, 0x90, 0xff},

		{0x00, 0x00, 0x00} // end of list
	};

	// gyro register init loop
	i = 0;

	while (reg_init[i].r != 0) {
		retVal = mpu_spi_set_reg_verified(reg_init[i].r, reg_init[i].v,
						  reg_init[i].mask);

		if (retVal != SUCCESS) {
			FARF(HIGH, "error initializing compass reg %u, error %d. Aborting!",
			     reg_init[i].r, retVal);
			return ERROR;
		}

		i++;
	}

	// start the compass continuous sampling
	if (mpu9x50_start_compass_sampling() != SUCCESS) {
		return ERROR;
	}

	return SUCCESS;
}


int mpu9x50_detect_compass()
{
	uint8_t b = 0;

	// get compass version ID
	int retVal = compass_read_register(MPU9250_COMP_REG_WIA, &b);

	if (retVal != SUCCESS) {
		FARF(ALWAYS, "error reading compass whoami reg: %d", retVal);
		return ERROR;
	}

	if (b != MPU9250_AKM_DEV_ID) {
		FARF(ALWAYS, "wrong compass ID %u(expected %u)", b, MPU9250_AKM_DEV_ID);
		return ERROR;
	}

	FARF(MEDIUM, "mpu9x50 compass found");

	return SUCCESS;
}

int compass_write_register(uint8_t reg, uint8_t val)
{
	int retVal = SUCCESS;
	uint8_t b = 0;

	// I2C_SLV4_ADDR
	// write operation on compass address 0x0C
	retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_ADDR, 0x0c, 0xff);

	if (retVal != SUCCESS) { return retVal; }

	// I2C_SLV4_REG
	// set the compass register address to write to
	retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);

	if (retVal != SUCCESS) { return retVal; }

	// I2C_SLV4_DO
	// set the value to write in I2C_SLV4_DO register
	retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_DO, val, 0xff);

	if (retVal != SUCCESS) { return retVal; }

	retVal = mpu_spi_get_reg(MPU9250_REG_I2C_SLV4_CTRL, &b);

	if (retVal != SUCCESS) { return retVal; }

	// set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits, which specifies the sample rate
	b |= 0x80;
	// Trigger the data transfer
	retVal = mpu_spi_set_reg(MPU9250_REG_I2C_SLV4_CTRL, b);

	if (retVal != SUCCESS) { return retVal; }

	int loop_ctrl = 1000; // wait up to 1000 * 1ms for completion

	// Continuously check I2C_MST_STATUS regsiter value for the completion
	// of I2C transfer until timeout
	retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);

	if (retVal != SUCCESS) { return retVal; }

	while (((b & 0x40) == 0x00) && (--loop_ctrl)) {
		usleep(1000);
		retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);

		if (retVal != SUCCESS) { return retVal; }
	}

	if (loop_ctrl == 0) {
		FARF(ALWAYS, "I2C transfer timed out");
		return ERROR;
	}

	FARF(MEDIUM, "Compass register %u set to %u", reg, val);

	return SUCCESS;
}

int compass_read_register(uint8_t reg, uint8_t *val)
{
	int retVal = SUCCESS;
	uint8_t b = 0;

	// I2C_SLV4_ADDR
	// write operation on compass address 0x0C
	retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_ADDR, 0x8c, 0xff);

	if (retVal != SUCCESS) { return retVal; }

	// I2C_SLV4_REG
	// set the compass register address to write to
	retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);

	if (retVal != SUCCESS) { return retVal; }

	retVal = mpu_spi_get_reg(MPU9250_REG_I2C_SLV4_CTRL, &b);

	if (retVal != SUCCESS) { return retVal; }

	// set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits, which specifies the sample rate
	b |= 0x80;
	// Trigger the data transfer
	retVal = mpu_spi_set_reg(MPU9250_REG_I2C_SLV4_CTRL, b);

	if (retVal != SUCCESS) { return retVal; }

	int loop_ctrl = 1000; // wait up to 1000 * 1 ms for completion

	// Continuously check I2C_MST_STATUS regsiter value for the completion
	// of I2C transfer until timeout
	retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);

	if (retVal != SUCCESS) { return retVal; }

	while (((b & 0x40) == 0x00) && (--loop_ctrl)) {
		usleep(2000);
		retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);

		if (retVal != SUCCESS) { return retVal; }
	}

	if (loop_ctrl == 0) {
		FARF(ALWAYS, "I2C transfer timed out");
		return ERROR;
	}

	// get value into pointer provided on call
	retVal = mpu_spi_get_reg(MPU9250_REG_I2C_SLV4_DI, val);

	if (retVal != SUCCESS) { return retVal; }

	FARF(MEDIUM, "Compass register %u read returned %u", reg, *val);

	return SUCCESS;
}

int mpu9x50_get_compass_senitivity_adjustment(float *compass_sens_adj)
{
	int i;
	uint8_t asa[3];

	// enable FUSE ROM, since the sensitivity adjustment data is stored in
	// compass registers 0x10, 0x11 and 0x12 which is only accessible in Fuse
	// access mode.
	if (compass_write_register(MPU9250_COMP_REG_CNTL1, 0x1f) != SUCCESS) {
		return ERROR;
	}

	// get compass calibration register 0x10, 0x11, 0x12
	// store into context
	for (i = 0; i < 3; i++) {
		if (compass_read_register(MPU9250_COMP_REG_ASAX + i, asa + i) != SUCCESS) {
			return ERROR;
		}

		compass_sens_adj[i] = (float)((float)asa[i] - 128.0) / 256.0 + 1.0f;
	}

	// disable FUSE ROM
	if (compass_write_register(MPU9250_COMP_REG_CNTL1, 0x00) != SUCCESS) {
		return ERROR;
	}

	FARF(MEDIUM, "compass sensitivity adjustment: %f %f %f",
	     compass_sens_adj[0], compass_sens_adj[1], compass_sens_adj[2]);

	return SUCCESS;
}

int mpu9x50_start_compass_sampling()
{
	int i = 0;
	int retVal;

	FARF(MEDIUM, "start compass measurement");

	// enable continuous measurement mode 1, with 16 bit outputs
	if (compass_write_register(MPU9250_COMP_REG_CNTL1, 0x12) != SUCCESS) {
		return ERROR;
	}

	// program 9250 to transfer data from compass into register area
	struct {uint8_t r; uint8_t v; uint8_t mask;} reg_init[] = {
		// I2C_SLV0_ADDR
		//  I2C read operation, compass I2C addres = 0x0c
		{MPU9250_REG_I2C_SLV0_ADDR, 0x8c, 0xff},

		// start reading FROM compass register 0x01 (INFO).
		// This will keep the integers aligned %2
		{MPU9250_REG_I2C_SLV0_REG, 0x01, 0xff},

		// enable reading 9 bytes at sample rate
		{MPU9250_REG_I2C_SLV0_CTRL, 0x89, 0xff},

		{0x00, 0x00, 0x00} // end of list
	};

	i = 0;

	// compass register init loop
	while (reg_init[i].r != 0) {
		retVal = mpu_spi_set_reg_verified(reg_init[i].r, reg_init[i].v,
						  reg_init[i].mask);

		if (retVal != SUCCESS) {
			FARF(HIGH, "error setting compass reg %u, error %d. Aborting!",
			     reg_init[i].r, retVal);
			return ERROR;
		}

		i++;
	} // while additional init data

	return SUCCESS;
}

int mpu9x50_register_interrupt(int gpio_id,
			       void (*data_ready_isr)(struct mpu9x50_data *data, void *user_context),
			       void *context)
{
	if (!is_mpu9x50_open) {
		FARF(ALWAYS, "mpu9x50_register_interrupt() called without opening mpu device");
		return SUCCESS;
	}

	user_data_ready_isr = data_ready_isr;

	// configure GPIO device for interrupt use
	if (mpu_spi_configure_gpio_interrupt(gpio_id, &mpu9x50_data_ready_isr,
					     context) != SUCCESS) {
		FARF(HIGH, "failed to enable GPIO interrupt");
		return ERROR;
	}

	// Enable INT
	if (mpu_spi_set_reg_verified(MPU9250_REG_INT_EN, 0x01, 0x01) != SUCCESS) {
		FARF(ALWAYS, "failed to configure mpu9x50 interrupt");
		return ERROR;
	}

	is_mpu9x50_dri_enabled = true;

	FARF(HIGH, "registered data ready ISR on GPIO %d", gpio_id);

	return SUCCESS;
}

int mpu9x50_unregister_interrupt()
{
	if (!is_mpu9x50_open) {
		FARF(ALWAYS, "mpu9x50_unregister_interrupt() called without opening mpu device");
		return SUCCESS;
	}

	// disable INT
	if (mpu_spi_set_reg_verified(MPU9250_REG_INT_EN, 0x00, 0x01) != SUCCESS) {
		FARF(ALWAYS, "error disabling mpu9x50 interrupt");
		return ERROR;
	}

	// configure GPIO device for interrupt use
	if (mpu_spi_disable_gpio_interrupt() != SUCCESS) {
		FARF(HIGH, "failed to disable GPIO interrupt");
		return ERROR;
	}

	is_mpu9x50_dri_enabled = false;

	return SUCCESS;
}

void mpu9x50_data_ready_isr(void *context)
{
	FARF(HIGH, "mpu9x50_data_ready_isr called");

	if (!user_data_ready_isr) {
		return;
	}

	sensor_data.timestamp = time(NULL);

	// read sensor registers through SPI
	// start reading the registers at starting address 0x3a
	// 0 INT status
	// 1-6 Accel (6B)
	// 7-8 Temperature (2B)
	// 9-14 Gyro (6B)
	// 15 compass INFO (1B)
	// 16 compass ST1 (1B)
	// 17-22 Compass (6B)
	// 23 compass ST2 (1B)
	uint8_t buffer[24];
	uint8_t accel_start_idx = 1;
	uint8_t temperature_start_idx = 7;
	uint8_t gyro_start_idx = 9;
	uint8_t compass_start_idx = 15;

	if (mpu_spi_bulk_read(0x3a, sizeof(buffer), buffer) != SUCCESS) {
		return;
	}

	// parse the sensor data
	mpu9x50_process_temperature(buffer + temperature_start_idx);
	mpu9x50_process_accel(buffer + accel_start_idx);
	mpu9x50_process_gyro(buffer + gyro_start_idx);
	FARF(MEDIUM, "process compass %d",
	     mpu9x50_process_compass(buffer + compass_start_idx));

	// call user callback
	(*user_data_ready_isr)(&sensor_data, context);
}

void mpu9x50_process_temperature(uint8_t *data)
{
	// according to spec, the temperature formula is
	// TEMP_degC = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21degC
	// RoomTemp_Offset and Temp_Sensitivity are borrowed from invense code.

	int16_t raw = (data[0] << 8) | data[1];
	float temp_offset = 0.0;
	float temp_sensitivity = 321.0;

	sensor_data.temperature_raw = raw;
	sensor_data.temperature = ((float)raw - temp_offset) / temp_sensitivity + 21.0;
}

void mpu9x50_process_accel(uint8_t *data)
{
	sensor_data.accel_raw[0] = (data[0] << 8) | data[1];
	sensor_data.accel_raw[1] = (data[2] << 8) | data[3];
	sensor_data.accel_raw[2] = (data[4] << 8) | data[5];
}

void mpu9x50_process_gyro(uint8_t *data)
{
	sensor_data.gyro_raw[0] = (data[0] << 8) | data[1];
	sensor_data.gyro_raw[1] = (data[2] << 8) | data[3];
	sensor_data.gyro_raw[2] = (data[4] << 8) | data[5];
}

/**
 * Process compass data and store the result in mag_rp.
 * the data buffer contains 9 bytes with the following format
 * 0 compass INFO (1B)
 * 1 compass ST1 (1B)
 * 2-7 Compass (6B)
 * 8 compass ST2 (1B)
 * @return  0 on success, -2 data ready flag not set or overrun bit set,
 *          -3 overflow or error bit set,
 */
int mpu9x50_process_compass(uint8_t *data)
{
	uint8_t status1 = data[1];
	uint8_t status2 = data[8];


	FARF(ALWAYS, "status1 %u status2 %u", status1, status2);

	sensor_data.mag_data_ready = (status1 & 0x01);

	// Data Ready flag not set or data overrun bit set
	if (!sensor_data.mag_data_ready) {
		return -2;
	}

	// magnetic sensor overflow HOFL bit set
	if (status2 & 0x08) {
		return -3;
	}

	sensor_data.mag_data_ready = true;

	sensor_data.mag_raw[0] = (data[3] << 8) | data[2];
	sensor_data.mag_raw[1] = (data[5] << 8) | data[4];
	sensor_data.mag_raw[2] = (data[7] << 8) | data[6];

	// H_adj = H * ((ASA-128)*0.5/128 + 1)
	//       = H * ((ASA-128) / 256 + 1)
	// H is the raw compass reading, ((ASA-128) / 256 + 1) has been
	// computed and stored in compass_cal_f
	sensor_data.mag_raw[0] = (int16_t)((int)sensor_data.mag_raw[0] * compass_sens_adj[0]);
	sensor_data.mag_raw[1] = (int16_t)((int)sensor_data.mag_raw[1] * compass_sens_adj[1]);
	sensor_data.mag_raw[2] = (int16_t)((int)sensor_data.mag_raw[2] * compass_sens_adj[2]);

	// swap magnetometer x and y axis
	// Magnetometer X axis = Gyro and Accel Y axis
	// Magnetometer Y axis = Gyro and Accel X axis
	// Magnetometer Z axis = - Gyro and Accel Z axis
	int16_t temp = sensor_data.mag_raw[0];
	sensor_data.mag_raw[0] = sensor_data.mag_raw[1];
	sensor_data.mag_raw[1] = temp;

	return 0;
}

int mpu9x50_get_gyro_lpf(int *gyro_lpf)
{
	uint8_t config, gyro_config;
	uint8_t fchoice;
	uint8_t dlpf_cfg;

	if (mpu_spi_get_reg(MPU9250_REG_CONFIG, &config) != SUCCESS) {
		return -1;
	}

	if (mpu_spi_get_reg(MPU9250_REG_GYRO_CONFIG, &gyro_config) != SUCCESS) {
		return -1;
	}

	fchoice = (gyro_config & 0x03) ^ 0x03;
	dlpf_cfg = config & 0x07;

	// To use DLPF_CFG, FCHOICE must be set with 0x03
	if (fchoice != 0x03) {
		*gyro_lpf = 0;

	} else {
		*gyro_lpf = gyro_lpf_enum_to_hz(dlpf_cfg);
	}

	return 0;
}

int mpu9x50_get_accel_lpf(int *accel_lpf)
{
	uint8_t accel_config_2;
	uint8_t fchoice, a_dlpfcfg;

	if (mpu_spi_get_reg(MPU9250_REG_ACCEL_CONFIG2, &accel_config_2) != SUCCESS) {
		return -1;
	}

	fchoice = ((accel_config_2 & 0x08) >> 3) ^ 0x01;
	a_dlpfcfg = accel_config_2 & 0x07;

	// To use DLPF_CFG, FCHOICE must be set with 0x03
	if (fchoice == 0) {
		*accel_lpf = 0;

	} else {
		*accel_lpf = accel_lpf_enum_to_hz(a_dlpfcfg);
	}

	return 0;
}

int mpu9x50_get_gyro_sample_rate(int *sample_rate)
{
	uint8_t smplrt_div;

	if (mpu_spi_get_reg(MPU9250_REG_SMPLRT_DIV, &smplrt_div) != SUCCESS) {
		return -1;
	}

	*sample_rate = 1000 / (1 + smplrt_div);

	return 0;
}

int mpu9x50_get_compass_sample_rate(int *compass_sample_rate)
{
	int sample_rate;
	uint8_t i2c_slv4_ctrl;
	uint8_t i2c_mst_dly;

	if (mpu9x50_get_gyro_sample_rate(&sample_rate) < 0) {
		return -1;
	}

	if (mpu_spi_get_reg(MPU9250_REG_I2C_SLV4_CTRL, &i2c_slv4_ctrl) != SUCCESS) {
		return -1;
	}

	i2c_mst_dly = i2c_slv4_ctrl & 0x1f;

	*compass_sample_rate = sample_rate / (1 + i2c_mst_dly);

	return 0;
}

int mpu9x50_get_gyro_fsr(int *gyro_fsr)
{
	uint8_t gyro_config;
	uint8_t gyro_fs_sel;

	if (mpu_spi_get_reg(MPU9250_REG_GYRO_CONFIG, &gyro_config) != SUCCESS) {
		return -1;
	}

	gyro_fs_sel = (gyro_config >> 3) & 0x03;

	*gyro_fsr = gyro_fsr_enum_to_dps(gyro_fs_sel);

	return 0;
}

int mpu9x50_get_accel_fsr(int *accel_fsr)
{
	uint8_t accel_config;
	uint8_t accel_fs_sel;

	if (mpu_spi_get_reg(28, &accel_config) != SUCCESS) {
		return -1;
	}

	accel_fs_sel = (accel_config >> 3) & 0x03;

	*accel_fsr = 1 << (accel_fs_sel + 1);

	return 0;
}

int mpu9x50_get_compass_fsr(int *compass_fsr)
{
	*compass_fsr = MPU9250_AK89xx_FSR;
	return 0;
}

int mpu9x50_get_int_status(uint8_t *int_status)
{
	if (mpu_spi_get_reg(MPU9250_REG_INT_STATUS, int_status) != SUCCESS) {
		return -1;
	}

	return 0;
}
