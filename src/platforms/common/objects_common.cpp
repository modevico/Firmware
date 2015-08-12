/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file objects_common.cpp
 *
 * Common object definitions without a better home.
 */

/**
 * @defgroup topics List of all uORB topics.
 */

#include <px4_config.h>

#include <drivers/drv_orb_dev.h>

#include <drivers/drv_mag.h>
ORB_DEFINE(sensor_mag, struct mag_report);

#include <drivers/drv_accel.h>
ORB_DEFINE(sensor_accel, struct accel_report);

#include <drivers/drv_gyro.h>
ORB_DEFINE(sensor_gyro, struct gyro_report);

#include <drivers/drv_baro.h>
ORB_DEFINE(sensor_baro, struct baro_report);

#include <drivers/drv_pwm_output.h>
ORB_DEFINE(output_pwm, struct pwm_output_values);

#include <drivers/drv_rc_input.h>
ORB_DEFINE(input_rc, struct rc_input_values);

#include "uORB/topics/pwm_input.h"
ORB_DEFINE(pwm_input, struct pwm_input_s);

#include "uORB/topics/vehicle_attitude.h"
ORB_DEFINE(vehicle_attitude, struct vehicle_attitude_s);

#include "uORB/topics/sensor_combined.h"
ORB_DEFINE(sensor_combined, struct sensor_combined_s);

#include "uORB/topics/hil_sensor.h"
ORB_DEFINE(hil_sensor, struct hil_sensor_s);

#include "uORB/topics/vehicle_gps_position.h"
ORB_DEFINE(vehicle_gps_position, struct vehicle_gps_position_s);

#include "uORB/topics/vehicle_land_detected.h"
ORB_DEFINE(vehicle_land_detected, struct vehicle_land_detected_s);

#include "uORB/topics/satellite_info.h"
ORB_DEFINE(satellite_info, struct satellite_info_s);

#include "uORB/topics/home_position.h"
ORB_DEFINE(home_position, struct home_position_s);

#include "uORB/topics/vehicle_status.h"
ORB_DEFINE(vehicle_status, struct vehicle_status_s);

#include "uORB/topics/vtol_vehicle_status.h"
ORB_DEFINE(vtol_vehicle_status, struct vtol_vehicle_status_s);

#include "uORB/topics/safety.h"
ORB_DEFINE(safety, struct safety_s);

#include "uORB/topics/battery_status.h"
ORB_DEFINE(battery_status, struct battery_status_s);

#include "uORB/topics/servorail_status.h"
ORB_DEFINE(servorail_status, struct servorail_status_s);

#include "uORB/topics/system_power.h"
ORB_DEFINE(system_power, struct system_power_s);

#include "uORB/topics/vehicle_global_position.h"
ORB_DEFINE(vehicle_global_position, struct vehicle_global_position_s);

#include "uORB/topics/vehicle_local_position.h"
ORB_DEFINE(vehicle_local_position, struct vehicle_local_position_s);

#include "uORB/topics/att_pos_mocap.h"
ORB_DEFINE(att_pos_mocap, struct att_pos_mocap_s);

#include "uORB/topics/vehicle_rates_setpoint.h"
ORB_DEFINE(vehicle_rates_setpoint, struct vehicle_rates_setpoint_s);
#include "uORB/topics/mc_virtual_rates_setpoint.h"
ORB_DEFINE(mc_virtual_rates_setpoint, struct mc_virtual_rates_setpoint_s);
#include "uORB/topics/fw_virtual_rates_setpoint.h"
ORB_DEFINE(fw_virtual_rates_setpoint, struct fw_virtual_rates_setpoint_s);

#include "uORB/topics/rc_channels.h"
ORB_DEFINE(rc_channels, struct rc_channels_s);

#include "uORB/topics/vehicle_command.h"
ORB_DEFINE(vehicle_command, struct vehicle_command_s);

#include "uORB/topics/vehicle_control_mode.h"
ORB_DEFINE(vehicle_control_mode, struct vehicle_control_mode_s);

#include "uORB/topics/vehicle_local_position_setpoint.h"
ORB_DEFINE(vehicle_local_position_setpoint, struct vehicle_local_position_setpoint_s);

#include "uORB/topics/position_setpoint_triplet.h"
ORB_DEFINE(position_setpoint_triplet, struct position_setpoint_triplet_s);

#include "uORB/topics/vehicle_global_velocity_setpoint.h"
ORB_DEFINE(vehicle_global_velocity_setpoint, struct vehicle_global_velocity_setpoint_s);

#include "uORB/topics/mission.h"
ORB_DEFINE(offboard_mission, struct mission_s);
ORB_DEFINE(onboard_mission, struct mission_s);

#include "uORB/topics/mission_result.h"
ORB_DEFINE(mission_result, struct mission_result_s);

#include "uORB/topics/geofence_result.h"
ORB_DEFINE(geofence_result, struct geofence_result_s);

#include "uORB/topics/fence.h"
ORB_DEFINE(fence, struct fence_s);

#include "uORB/topics/fence_vertex.h"
ORB_DEFINE(fence_vertex, struct fence_vertex_s);

#include "uORB/topics/vehicle_attitude_setpoint.h"
ORB_DEFINE(vehicle_attitude_setpoint, struct vehicle_attitude_setpoint_s);
ORB_DEFINE(mc_virtual_attitude_setpoint, struct vehicle_attitude_setpoint_s);
ORB_DEFINE(fw_virtual_attitude_setpoint, struct vehicle_attitude_setpoint_s);

#include "uORB/topics/manual_control_setpoint.h"
ORB_DEFINE(manual_control_setpoint, struct manual_control_setpoint_s);

#include "uORB/topics/offboard_control_mode.h"
ORB_DEFINE(offboard_control_mode, struct offboard_control_mode_s);

#include "uORB/topics/optical_flow.h"
ORB_DEFINE(optical_flow, struct optical_flow_s);

#include "uORB/topics/filtered_bottom_flow.h"
ORB_DEFINE(filtered_bottom_flow, struct filtered_bottom_flow_s);

#include "uORB/topics/airspeed.h"
ORB_DEFINE(airspeed, struct airspeed_s);

#include "uORB/topics/differential_pressure.h"
ORB_DEFINE(differential_pressure, struct differential_pressure_s);

#include "uORB/topics/subsystem_info.h"
ORB_DEFINE(subsystem_info, struct subsystem_info_s);

/* actuator controls, as requested by controller */
#include "uORB/topics/actuator_controls.h"
#include "uORB/topics/actuator_controls_0.h"
ORB_DEFINE(actuator_controls_0, struct actuator_controls_0_s);
#include "uORB/topics/actuator_controls_1.h"
ORB_DEFINE(actuator_controls_1, struct actuator_controls_1_s);
#include "uORB/topics/actuator_controls_2.h"
ORB_DEFINE(actuator_controls_2, struct actuator_controls_2_s);
#include "uORB/topics/actuator_controls_3.h"
ORB_DEFINE(actuator_controls_3, struct actuator_controls_3_s);
//Virtual control groups, used for VTOL operation
#include "uORB/topics/actuator_controls_virtual_mc.h"
ORB_DEFINE(actuator_controls_virtual_mc, struct actuator_controls_virtual_mc_s);
#include "uORB/topics/actuator_controls_virtual_fw.h"
ORB_DEFINE(actuator_controls_virtual_fw, struct actuator_controls_virtual_fw_s);

#include "uORB/topics/actuator_armed.h"
ORB_DEFINE(actuator_armed, struct actuator_armed_s);

#include "uORB/topics/actuator_outputs.h"
ORB_DEFINE(actuator_outputs, struct actuator_outputs_s);

#include "uORB/topics/actuator_direct.h"
ORB_DEFINE(actuator_direct, struct actuator_direct_s);

#include "uORB/topics/multirotor_motor_limits.h"
ORB_DEFINE(multirotor_motor_limits, struct multirotor_motor_limits_s);

#include "uORB/topics/telemetry_status.h"
ORB_DEFINE(telemetry_status, struct telemetry_status_s);

#include "uORB/topics/test_motor.h"
ORB_DEFINE(test_motor, struct test_motor_s);

#include "uORB/topics/debug_key_value.h"
ORB_DEFINE(debug_key_value, struct debug_key_value_s);

#include "uORB/topics/navigation_capabilities.h"
ORB_DEFINE(navigation_capabilities, struct navigation_capabilities_s);

#include "uORB/topics/esc_status.h"
ORB_DEFINE(esc_status, struct esc_status_s);

#include "uORB/topics/esc_report.h"
ORB_DEFINE(esc_report, struct esc_report_s);

#include "uORB/topics/encoders.h"
ORB_DEFINE(encoders, struct encoders_s);

#include "uORB/topics/estimator_status.h"
ORB_DEFINE(estimator_status, struct estimator_status_s);

#include "uORB/topics/vision_position_estimate.h"
ORB_DEFINE(vision_position_estimate, struct vision_position_estimate_s);

#include "uORB/topics/vehicle_force_setpoint.h"
ORB_DEFINE(vehicle_force_setpoint, struct vehicle_force_setpoint_s);

#include "uORB/topics/tecs_status.h"
ORB_DEFINE(tecs_status, struct tecs_status_s);

#include "uORB/topics/wind_estimate.h"
ORB_DEFINE(wind_estimate, struct wind_estimate_s);

#include "uORB/topics/rc_parameter_map.h"
ORB_DEFINE(rc_parameter_map, struct rc_parameter_map_s);

#include "uORB/topics/time_offset.h"
ORB_DEFINE(time_offset, struct time_offset_s);

#include "uORB/topics/mc_att_ctrl_status.h"
ORB_DEFINE(mc_att_ctrl_status, struct mc_att_ctrl_status_s);

#include "uORB/topics/distance_sensor.h"
ORB_DEFINE(distance_sensor, struct distance_sensor_s);

#include "uORB/topics/camera_trigger.h"
ORB_DEFINE(camera_trigger, struct camera_trigger_s);