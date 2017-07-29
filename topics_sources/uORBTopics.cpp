/****************************************************************************
 *
 *   Copyright (C) 2013-2015 PX4 Development Team. All rights reserved.
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

#include <uORB/uORBTopics.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_direct.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/fence_vertex.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/hil_sensor.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/output_pwm.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/qshell_req.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/task_stack_info.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/time_offset.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/uavcan_parameter_request.h>
#include <uORB/topics/uavcan_parameter_value.h>
#include <uORB/topics/ulog_stream.h>
#include <uORB/topics/ulog_stream_ack.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_force_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>



const struct orb_metadata* _uorb_topics_list[_uorb_topics_count] = { 
    ORB_ID(mc_virtual_rates_setpoint),
    ORB_ID(sensor_baro),
    ORB_ID(mission),
    ORB_ID(vehicle_status),
    ORB_ID(transponder_report),
    ORB_ID(fw_virtual_attitude_setpoint),
    ORB_ID(actuator_controls),
    ORB_ID(filtered_bottom_flow),
    ORB_ID(onboard_mission),
    ORB_ID(control_state),
    ORB_ID(mount_orientation),
    ORB_ID(subsystem_info),
    ORB_ID(uavcan_parameter_value),
    ORB_ID(log_message),
    ORB_ID(actuator_outputs),
    ORB_ID(actuator_controls_virtual_fw),
    ORB_ID(esc_report),
    ORB_ID(test_motor),
    ORB_ID(esc_status),
    ORB_ID(differential_pressure),
    ORB_ID(fence),
    ORB_ID(adc_report),
    ORB_ID(cpuload),
    ORB_ID(rc_channels),
    ORB_ID(vehicle_status_flags),
    ORB_ID(ekf2_replay),
    ORB_ID(debug_key_value),
    ORB_ID(airspeed),
    ORB_ID(actuator_controls_0),
    ORB_ID(actuator_controls_1),
    ORB_ID(actuator_controls_2),
    ORB_ID(actuator_controls_3),
    ORB_ID(safety),
    ORB_ID(camera_trigger),
    ORB_ID(vehicle_attitude_setpoint),
    ORB_ID(telemetry_status),
    ORB_ID(multirotor_motor_limits),
    ORB_ID(offboard_mission),
    ORB_ID(sensor_accel),
    ORB_ID(vehicle_global_position),
    ORB_ID(servorail_status),
    ORB_ID(actuator_armed),
    ORB_ID(vehicle_command_ack),
    ORB_ID(vehicle_global_position_groundtruth),
    ORB_ID(position_setpoint_triplet),
    ORB_ID(estimator_status),
    ORB_ID(vehicle_control_mode),
    ORB_ID(task_stack_info),
    ORB_ID(fw_virtual_rates_setpoint),
    ORB_ID(system_power),
    ORB_ID(vehicle_force_setpoint),
    ORB_ID(sensor_combined),
    ORB_ID(vehicle_rates_setpoint),
    ORB_ID(vehicle_local_position_groundtruth),
    ORB_ID(wind_estimate),
    ORB_ID(tecs_status),
    ORB_ID(sensor_correction),
    ORB_ID(optical_flow),
    ORB_ID(pwm_input),
    ORB_ID(position_setpoint),
    ORB_ID(collision_report),
    ORB_ID(led_control),
    ORB_ID(mc_att_ctrl_status),
    ORB_ID(ekf2_innovations),
    ORB_ID(uavcan_parameter_request),
    ORB_ID(gps_dump),
    ORB_ID(rc_parameter_map),
    ORB_ID(parameter_update),
    ORB_ID(geofence_result),
    ORB_ID(vehicle_attitude),
    ORB_ID(att_pos_mocap),
    ORB_ID(battery_status),
    ORB_ID(vehicle_land_detected),
    ORB_ID(time_offset),
    ORB_ID(mission_result),
    ORB_ID(satellite_info),
    ORB_ID(vtol_vehicle_status),
    ORB_ID(qshell_req),
    ORB_ID(vehicle_local_position),
    ORB_ID(manual_control_setpoint),
    ORB_ID(hil_sensor),
    ORB_ID(vehicle_local_position_setpoint),
    ORB_ID(vehicle_global_velocity_setpoint),
    ORB_ID(vehicle_vision_position),
    ORB_ID(home_position),
    ORB_ID(fw_pos_ctrl_status),
    ORB_ID(sensor_preflight),
    ORB_ID(gps_inject_data),
    ORB_ID(follow_target),
    ORB_ID(ulog_stream_ack),
    ORB_ID(sensor_gyro),
    ORB_ID(commander_state),
    ORB_ID(ulog_stream),
    ORB_ID(fence_vertex),
    ORB_ID(vehicle_command),
    ORB_ID(actuator_controls_virtual_mc),
    ORB_ID(output_pwm),
    ORB_ID(ekf2_timestamps),
    ORB_ID(distance_sensor),
    ORB_ID(offboard_control_mode),
    ORB_ID(vehicle_gps_position),
    ORB_ID(mavlink_log),
    ORB_ID(actuator_direct),
    ORB_ID(vehicle_vision_attitude),
    ORB_ID(input_rc),
    ORB_ID(vehicle_roi),
    ORB_ID(vehicle_attitude_groundtruth),
    ORB_ID(sensor_mag),
    ORB_ID(mc_virtual_attitude_setpoint)

};

size_t orb_topics_count()
{
	return _uorb_topics_count;
}

const struct orb_metadata **orb_get_topics()
{
	return _uorb_topics_list;
}
