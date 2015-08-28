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
#include <string.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <drivers/drv_hrt.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <csr_gps_api.h>
#include <csr_defs.h>
#include <csr_gps_common.h>
#ifdef __cplusplus
}
#endif

/** driver 'main' command */
extern "C" { __EXPORT int csr_gps_main(int argc, char *argv[]); }

#define MAX_LEN_DEV_PATH 32

namespace csr_gps
{
volatile bool _task_should_exit = false; // flag indicating if csr_gps task should exit
static char _device[MAX_LEN_DEV_PATH];
static bool _is_running = false;         // flag indicating if csr_gps app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread

// publications
orb_advert_t _gps_report_pub;

// topic structures

/** Print out the usage information */
static void usage();

/** csr_gps start */
static void start(const char *device);

/** csr_gps stop */
static void stop();

/** task main trampoline function */
static void task_main_trampoline(int argc, char *argv[]);

/** csr_gps thread primary entry point */
static void task_main(int argc, char *argv[]);

static int populate_vehicle_position(struct vehicle_gps_position_s *gps_report)
{
	int tries = 50;
	int result = SUCCESS;
	struct osp_geo_data geo_data;
	float heading_error_deg;
	float heading;
	float phi;
	int i;

	if (gps_report == NULL) {
		PX4_ERR("%s: Null gps_report passed", __func__);
		result = -EINVAL;
		goto cleanup;
	}

	result = csr_gps_get_geo_data(&geo_data);

	if (result != SUCCESS) {
		PX4_ERR("%s: Failed to parse geo data %d", __func__, result);
		goto cleanup;
	}

	gps_report->timestamp_time = hrt_absolute_time();
	gps_report->timestamp_position = hrt_absolute_time();
	gps_report->lat = bswap_32(geo_data.lat);
	gps_report->lon = bswap_32(geo_data.lon);

	gps_report->alt = bswap_32(geo_data.alt_from_MSL) * 10;

	// SiRFDrive only
	gps_report->timestamp_variance = hrt_absolute_time();
	gps_report->s_variance_m_s = bswap_16(geo_data.est_hor_vel_error) / 100.0f;
	heading_error_deg = bswap_16(geo_data.heading_error) / 100.0f;
	gps_report->c_variance_rad = heading_error_deg * M_PI / 180.0f;

	if (geo_data.nav_type & NAV_TYPE_4SV_OR_MORE_KF_SOLUTION ||
	    geo_data.nav_type & NAV_TYPE_3SV_KF_SOLUTION) {
		gps_report->fix_type = 3;
		// Assume velocity and heading are valid if we have a fix
		gps_report->vel_ned_valid = 1;

	} else if (geo_data.nav_type & NAV_TYPE_2SV_KF_SOLUTION) {
		gps_report->fix_type = 2;

	} else if (geo_data.nav_type & NAV_TYPE_1SV_KF_SOLUTION) {
		gps_report->fix_type = 1;

	} else {
		gps_report->fix_type = 0;
	}

	gps_report->eph = geo_data.HDOP / 5.0f;
	// TODO: Parse MID 66 for VDOP
	//gps_report->epv = ;

	// Convert velocity and heading to North East Down (NED)
	gps_report->timestamp_velocity = hrt_absolute_time();
	heading = bswap_16(geo_data.speed_over_ground) / 100.0f;
	gps_report->vel_m_s = bswap_16(geo_data.speed_over_ground) / 100.0f;

	if (heading <= 90.0) {
		phi = heading;
		gps_report->vel_n_m_s = gps_report->vel_m_s * cosf(phi * M_PI / 180.0f);
		gps_report->vel_e_m_s = gps_report->vel_m_s * sinf(phi * M_PI / 180.0f);

	} else if (heading > 90.0 && heading <= 180.0) {
		phi = 180 - heading;
		gps_report->vel_n_m_s = -1 * gps_report->vel_m_s * cosf(phi * M_PI / 180.0f);
		gps_report->vel_e_m_s = gps_report->vel_m_s * sinf(phi * M_PI / 180.0f);

	} else if (heading > 180.0 && heading <= 270.0) {
		phi = heading - 180;
		gps_report->vel_n_m_s = -1 * gps_report->vel_m_s * cosf(phi * M_PI / 180.0f);
		gps_report->vel_e_m_s = -1 * gps_report->vel_m_s * sinf(phi * M_PI / 180.0f);

	} else if (heading > 270.0) {
		phi = 360 - heading;
		gps_report->vel_n_m_s = gps_report->vel_m_s * cosf(phi * M_PI / 180.0f);
		gps_report->vel_e_m_s = -1 * gps_report->vel_m_s * sinf(phi * M_PI / 180.0f);
	}

	gps_report->vel_d_m_s = bswap_16(geo_data.climb_rate) / 100.0f;
	gps_report->cog_rad = heading * M_PI / 180.0;
	gps_report->satellites_used = geo_data.sv_in_fix;
cleanup:
	return result;
}

void task_main(int argc, char *argv[])
{
	PX4_INFO("enter task_main");

	_gps_report_pub = nullptr;

	int result = csr_gps_init(_device);

	if (result != SUCCESS) {
		PX4_ERR("Failed to initialize GPS");
		goto exit;
	}

	result = csr_gps_set_filters();

	if (result != SUCCESS) {
		PX4_ERR("Failed to initialize GPS");
		goto exit;
	}

	result = csr_gps_enable_nav_msg_only();

	if (result != SUCCESS) {
		// Not critical so continue
		PX4_ERR("Failed to disable extra messages");
	}

	struct vehicle_gps_position_s gps_report;

	// Main loop
	while (!_task_should_exit) {
		memset(&gps_report, 0, sizeof(struct vehicle_gps_position_s));
		result = populate_vehicle_position(&gps_report);

		if (result != SUCCESS) {
			continue;
		}

		PX4_WARN("_gps_report_pub %p", _gps_report_pub);

		/* Publish mixed control outputs */
		if (_gps_report_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_gps_position), _gps_report_pub, &gps_report);

		} else {
			_gps_report_pub = orb_advertise(ORB_ID(vehicle_gps_position), &gps_report);
		}
	}

exit:
	PX4_WARN("closing csr_gps");
	result = csr_gps_deinit();

	if (result != SUCCESS) {
		PX4_ERR("Failed to stop csr_gps");
	}
}

/** csr_gps main entrance */
void task_main_trampoline(int argc, char *argv[])
{
	PX4_WARN("task_main_trampoline");
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("csr_gps_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

	_is_running = true;
}

void stop()
{
	_task_should_exit = true;

	_is_running = false;
	_task_handle = -1;
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -D device");
}

}; // namespace csr_gps

int csr_gps_main(int argc, char *argv[])
{
	const char *device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "D:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'D':
			device = myoptarg;
			break;

		default:
			csr_gps::usage();
			return 1;
		}
	}

	// Check on required arguments
	if (device == NULL || strlen(device) == 0) {
		csr_gps::usage();
		return 1;
	}

	memset(csr_gps::_device, 0, MAX_LEN_DEV_PATH);
	strncpy(csr_gps::_device, device, strlen(device));

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (csr_gps::_is_running) {
			PX4_WARN("csr_gps already running");
			return 1;
		}

		csr_gps::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (csr_gps::_is_running) {
			PX4_WARN("csr_gps is not running");
			return 1;
		}

		csr_gps::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("csr_gps is %s", csr_gps::_is_running ? "running" : "stopped");
		return 0;

	} else {
		csr_gps::usage();
		return 1;
	}

	return 0;
}
