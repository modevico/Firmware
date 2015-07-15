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


#include <stdint.h>

#include <lib/conversion/rotation.h>
// #include <dspal_errno.h>
#include <px4_tasks.h>
#include <px4_getopt.h>

// #include "mpu9x50.h"

/** driver 'main' command */
extern "C" { __EXPORT int mpu9x50_main(int argc, char *argv[]); }

namespace mpu9x50
{

/** Rotation factor provided as argument */
enum Rotation rotation;

/** SPI device path that mpu9x50 is connected to */
const char *device = NULL;

/** flag indicating if mpu9x50 app is running */
bool is_running = false;

/** handle to the task main thread */
px4_task_t  task_handle = -1;

/** Print out the usage information */
void usage();

/** mpu9x50 start measurement */
void start(enum Rotation rotation, const char *device);

/** mpu9x50 stop measurement */
void stop();

/** task main trampoline function */
static void	task_main_trampoline(int argc, char *argv[]);

}; // namespace mpu9x50


/** mpu9x50 main entrance */
void mpu9x50::task_main_trampoline(int argc, char *argv[])
{
	warnx("task_main_trampoline");
	// // create the mpu9x50 default configuration structure
//   struct mpu9x50_config config = {
//      .gyro_lpf = MPU9X50_GYRO_LPF_20HZ,
//      .acc_lpf  = MPU9X50_ACC_LPF_20HZ,
//      .gyro_fsr = MPU9X50_GYRO_FSR_500DPS,
//      .acc_fsr  = MPU9X50_ACC_FSR_4G,
//      .gyro_sample_rate = MPU9x50_SAMPLE_RATE_500HZ,
//      .compass_enabled = true,
//      .compass_sample_rate = MPU9x50_COMPASS_SAMPLE_RATE_100HZ,
//      .spi_dev_path = mpu9x50::device,
//   };

//   // TODO-TJIN: load the mpu9x50 config parameters if any

//   if (mpu9x50_initialize(&config) != SUCCESS)

}

void mpu9x50::start(enum Rotation rotation, const char *device)
{
	ASSERT(mpu9x50::task_handle == -1);

	mpu9x50::rotation = rotation;
	mpu9x50::device = device;

	/* start the task */
	mpu9x50::task_handle = px4_task_spawn_cmd("mpu9x50_main",
			       SCHED_DEFAULT,
			       SCHED_PRIORITY_MAX,
			       1500,
			       (px4_main_t)&mpu9x50::task_main_trampoline,
			       nullptr);

	if (mpu9x50::task_handle < 0) {
		warn("task start failed");
		return;
	}

	mpu9x50::is_running = true;
}

void mpu9x50::stop()
{
	// TODO-TJIN:
	// set thread exit signal to terminate the task main thread

	mpu9x50::is_running = false;
	mpu9x50::device = NULL;
	mpu9x50::rotation = ROTATION_NONE;
	task_handle = -1;
}

void mpu9x50::usage()
{
	warnx("missing command: try 'start', 'stop', 'status'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -D device");
}

int mpu9x50_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	const char *device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:D:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		case 'D':
			device = optarg;
			break;

		default:
			mpu9x50::usage();
			exit(0);
		}
	}

	// Check on required arguments
	if (device == NULL) {
		mpu9x50::usage();
		return 1;
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (mpu9x50::is_running) {
			warnx("mpu9x50 already running");
			return 1;
		}

		mpu9x50::start(rotation, device);
	}

	if (!strcmp(verb, "stop")) {
		if (mpu9x50::is_running) {
			warnx("mpu9x50 is not running");
			return 1;
		}

		mpu9x50::stop();
	}

	if (!strcmp(verb, "status")) {
		warnx("mpu9x50 is %s", mpu9x50::is_running ? "running" : "stopped");
		return 1;
	}

	mpu9x50::usage();
	exit(1);
}
