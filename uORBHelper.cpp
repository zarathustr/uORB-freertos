/****************************************************************************
 *
 *   Copyright (c) 2016 Jin Wu. All rights reserved.
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
 * 3. Neither the name AMOV nor the names of its contributors may be
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


/* @Author: Jin Wu
   
   E-mail: jin_wu_uestc@hotmail.com
   Website: www.jinwu.science
*/

#include <uORB/uORB.h>
#include <cstring>
#include <StartUP.h>

#include "uORBHelper.h"

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

using actuator_controls_0_s                             = actuator_controls_s;
using actuator_controls_1_s                             = actuator_controls_s;
using actuator_controls_2_s                             = actuator_controls_s;
using actuator_controls_3_s                             = actuator_controls_s;
using actuator_controls_virtual_fw_s                    = actuator_controls_s;
using actuator_controls_virtual_mc_s                    = actuator_controls_s;
using offboard_mission_s                                = mission_s;
using onboard_mission_s                                 = mission_s;
using vehicle_local_position_groundtruth_s              = vehicle_local_position_s;
using vehicle_vision_position_s                         = vehicle_local_position_s;
using vehicle_global_position_groundtruth_s             = vehicle_global_position_s;
using vehicle_attitude_groundtruth_s                    = vehicle_attitude_s;
using vehicle_vision_attitude_s                         = vehicle_attitude_s;


void orb_set_in_os(void)
{
    orb_in_os = true;
}

bool is_orb_multi(const int serial)
{
    for(int i = 0; i < MULTI_ORB_NUM; ++i)
    {
        if(serial == (int) orb_multi_list[i])
            return true;
    }
    
    return false;
}

int get_priority(const int instance)
{
    if(instance == 0)
    {
        return ORB_PRIO_MIN;
    }
    else if(instance == 1)
    {
        return ORB_PRIO_VERY_LOW;
    }
    else if(instance == 2)
    {
        return ORB_PRIO_LOW;
    }
    else if(instance == 3)
    {
        return ORB_PRIO_DEFAULT;
    }
    else if(instance == 4)
    {
        return ORB_PRIO_HIGH;
    }
    else if(instance == 5)
    {
        return ORB_PRIO_VERY_HIGH;
    }
    else if(instance == 6)
    {
        return ORB_PRIO_MAX;
    }
    else
        return -1;
}


int get_orb_instance_according_to_priority(const int priority)
{
    if(ORB_PRIO_MIN == priority)
    {
        return 0;
    }
    else if(ORB_PRIO_VERY_LOW == priority)
    {
        return 1;
    }
    else if(ORB_PRIO_LOW == priority)
    {
        return 2;
    }
    else if(ORB_PRIO_DEFAULT == priority)
    {
        return 3;
    }
    else if(ORB_PRIO_HIGH == priority)
    {
        return 4;
    }
    else if(ORB_PRIO_VERY_HIGH == priority)
    {
        return 5;
    }
    else if(ORB_PRIO_MAX == priority)
    {
        return 6;
    }
    else
        return -1;
}




void get_orb_name(const int serial, char * name)
{
  if (serial == ORB_actuator_armed)
  {
      std::strcpy(name, (ORB_ID(actuator_armed))->o_name);
  }
  else if (serial == ORB_actuator_controls)
  {
      std::strcpy(name, (ORB_ID(actuator_controls))->o_name);
  }
  else if (serial == ORB_actuator_direct)
  {
      std::strcpy(name, (ORB_ID(actuator_direct))->o_name);
  }
  else if (serial == ORB_actuator_outputs)
  {
      std::strcpy(name, (ORB_ID(actuator_outputs))->o_name);
  }
  else if (serial == ORB_adc_report)
  {
      std::strcpy(name, (ORB_ID(adc_report))->o_name);
  }
  else if (serial == ORB_airspeed)
  {
      std::strcpy(name, (ORB_ID(airspeed))->o_name);
  }
  else if (serial == ORB_att_pos_mocap)
  {
      std::strcpy(name, (ORB_ID(att_pos_mocap))->o_name);
  }
  else if (serial == ORB_battery_status)
  {
      std::strcpy(name, (ORB_ID(battery_status))->o_name);
  }
  else if (serial == ORB_camera_trigger)
  {
      std::strcpy(name, (ORB_ID(camera_trigger))->o_name);
  }
  else if (serial == ORB_commander_state)
  {
      std::strcpy(name, (ORB_ID(commander_state))->o_name);
  }
  else if (serial == ORB_collision_report)
  {
      std::strcpy(name, (ORB_ID(collision_report))->o_name);
  }
  else if (serial == ORB_control_state)
  {
      std::strcpy(name, (ORB_ID(control_state))->o_name);
  }
  else if (serial == ORB_cpuload)
  {
      std::strcpy(name, (ORB_ID(cpuload))->o_name);
  }
  else if (serial == ORB_debug_key_value)
  {
      std::strcpy(name, (ORB_ID(debug_key_value))->o_name);
  }
  else if (serial == ORB_differential_pressure)
  {
      std::strcpy(name, (ORB_ID(differential_pressure))->o_name);
  }
  else if (serial == ORB_distance_sensor)
  {
      std::strcpy(name, (ORB_ID(distance_sensor))->o_name);
  }
  else if (serial == ORB_ekf2_innovations)
  {
      std::strcpy(name, (ORB_ID(ekf2_innovations))->o_name);
  }
  else if (serial == ORB_ekf2_replay)
  {
      std::strcpy(name, (ORB_ID(ekf2_replay))->o_name);
  }
  else if (serial == ORB_ekf2_timestamps)
  {
      std::strcpy(name, (ORB_ID(ekf2_timestamps))->o_name);
  }
  else if (serial == ORB_esc_report)
  {
      std::strcpy(name, (ORB_ID(esc_report))->o_name);
  }
  else if (serial == ORB_esc_status)
  {
      std::strcpy(name, (ORB_ID(esc_status))->o_name);
  }
  else if (serial == ORB_estimator_status)
  {
      std::strcpy(name, (ORB_ID(estimator_status))->o_name);
  }
  else if (serial == ORB_fence)
  {
      std::strcpy(name, (ORB_ID(fence))->o_name);
  }
  else if (serial == ORB_fence_vertex)
  {
      std::strcpy(name, (ORB_ID(fence_vertex))->o_name);
  }
  else if (serial == ORB_filtered_bottom_flow)
  {
      std::strcpy(name, (ORB_ID(filtered_bottom_flow))->o_name);
  }
  else if (serial == ORB_follow_target)
  {
      std::strcpy(name, (ORB_ID(follow_target))->o_name);
  }
  else if (serial == ORB_fw_pos_ctrl_status)
  {
      std::strcpy(name, (ORB_ID(fw_pos_ctrl_status))->o_name);
  }
  else if (serial == ORB_fw_virtual_attitude_setpoint)
  {
      std::strcpy(name, (ORB_ID(fw_virtual_attitude_setpoint))->o_name);
  }
  else if (serial == ORB_fw_virtual_rates_setpoint)
  {
      std::strcpy(name, (ORB_ID(fw_virtual_rates_setpoint))->o_name);
  }
  else if (serial == ORB_geofence_result)
  {
      std::strcpy(name, (ORB_ID(geofence_result))->o_name);
  }
  else if (serial == ORB_gps_dump)
  {
      std::strcpy(name, (ORB_ID(gps_dump))->o_name);
  }
  else if (serial == ORB_gps_inject_data)
  {
      std::strcpy(name, (ORB_ID(gps_inject_data))->o_name);
  }
  else if (serial == ORB_hil_sensor)
  {
      std::strcpy(name, (ORB_ID(hil_sensor))->o_name);
  }
  else if (serial == ORB_home_position)
  {
      std::strcpy(name, (ORB_ID(home_position))->o_name);
  }
  else if (serial == ORB_input_rc)
  {
      std::strcpy(name, (ORB_ID(input_rc))->o_name);
  }
  else if (serial == ORB_led_control)
  {
      std::strcpy(name, (ORB_ID(led_control))->o_name);
  }
  else if (serial == ORB_log_message)
  {
      std::strcpy(name, (ORB_ID(log_message))->o_name);
  }
  else if (serial == ORB_manual_control_setpoint)
  {
      std::strcpy(name, (ORB_ID(manual_control_setpoint))->o_name);
  }
  else if (serial == ORB_mavlink_log)
  {
      std::strcpy(name, (ORB_ID(mavlink_log))->o_name);
  }
  else if (serial == ORB_mc_att_ctrl_status)
  {
      std::strcpy(name, (ORB_ID(mc_att_ctrl_status))->o_name);
  }
  else if (serial == ORB_mc_virtual_attitude_setpoint)
  {
      std::strcpy(name, (ORB_ID(mc_virtual_attitude_setpoint))->o_name);
  }
  else if (serial == ORB_mc_virtual_rates_setpoint)
  {
      std::strcpy(name, (ORB_ID(mc_virtual_rates_setpoint))->o_name);
  }
  else if (serial == ORB_mission)
  {
      std::strcpy(name, (ORB_ID(mission))->o_name);
  }
  else if (serial == ORB_mission_result)
  {
      std::strcpy(name, (ORB_ID(mission_result))->o_name);
  }
  else if (serial == ORB_mount_orientation)
  {
      std::strcpy(name, (ORB_ID(mount_orientation))->o_name);
  }
  else if (serial == ORB_multirotor_motor_limits)
  {
      std::strcpy(name, (ORB_ID(multirotor_motor_limits))->o_name);
  }
  else if (serial == ORB_offboard_control_mode)
  {
      std::strcpy(name, (ORB_ID(offboard_control_mode))->o_name);
  }
  else if (serial == ORB_optical_flow)
  {
      std::strcpy(name, (ORB_ID(optical_flow))->o_name);
  }
  else if (serial == ORB_output_pwm)
  {
      std::strcpy(name, (ORB_ID(output_pwm))->o_name);
  }
  else if (serial == ORB_parameter_update)
  {
      std::strcpy(name, (ORB_ID(parameter_update))->o_name);
  }
  else if (serial == ORB_position_setpoint)
  {
      std::strcpy(name, (ORB_ID(position_setpoint))->o_name);
  }
  else if (serial == ORB_position_setpoint_triplet)
  {
      std::strcpy(name, (ORB_ID(position_setpoint_triplet))->o_name);
  }
  else if (serial == ORB_pwm_input)
  {
      std::strcpy(name, (ORB_ID(pwm_input))->o_name);
  }
  else if (serial == ORB_qshell_req)
  {
      std::strcpy(name, (ORB_ID(qshell_req))->o_name);
  }
  else if (serial == ORB_rc_channels)
  {
      std::strcpy(name, (ORB_ID(rc_channels))->o_name);
  }
  else if (serial == ORB_rc_parameter_map)
  {
      std::strcpy(name, (ORB_ID(rc_parameter_map))->o_name);
  }
  else if (serial == ORB_safety)
  {
      std::strcpy(name, (ORB_ID(safety))->o_name);
  }
  else if (serial == ORB_satellite_info)
  {
      std::strcpy(name, (ORB_ID(satellite_info))->o_name);
  }
  else if (serial == ORB_sensor_accel)
  {
      std::strcpy(name, (ORB_ID(sensor_accel))->o_name);
  }
  else if (serial == ORB_sensor_baro)
  {
      std::strcpy(name, (ORB_ID(sensor_baro))->o_name);
  }
  else if (serial == ORB_sensor_combined)
  {
      std::strcpy(name, (ORB_ID(sensor_combined))->o_name);
  }
  else if (serial == ORB_sensor_correction)
  {
      std::strcpy(name, (ORB_ID(sensor_correction))->o_name);
  }
  else if (serial == ORB_sensor_gyro)
  {
      std::strcpy(name, (ORB_ID(sensor_gyro))->o_name);
  }
  else if (serial == ORB_sensor_mag)
  {
      std::strcpy(name, (ORB_ID(sensor_mag))->o_name);
  }
  else if (serial == ORB_sensor_preflight)
  {
      std::strcpy(name, (ORB_ID(sensor_preflight))->o_name);
  }
  else if (serial == ORB_servorail_status)
  {
      std::strcpy(name, (ORB_ID(servorail_status))->o_name);
  }
  else if (serial == ORB_subsystem_info)
  {
      std::strcpy(name, (ORB_ID(subsystem_info))->o_name);
  }
  else if (serial == ORB_system_power)
  {
      std::strcpy(name, (ORB_ID(system_power))->o_name);
  }
  else if (serial == ORB_task_stack_info)
  {
      std::strcpy(name, (ORB_ID(task_stack_info))->o_name);
  }
  else if (serial == ORB_tecs_status)
  {
      std::strcpy(name, (ORB_ID(tecs_status))->o_name);
  }
  else if (serial == ORB_telemetry_status)
  {
      std::strcpy(name, (ORB_ID(telemetry_status))->o_name);
  }
  else if (serial == ORB_test_motor)
  {
      std::strcpy(name, (ORB_ID(test_motor))->o_name);
  }
  else if (serial == ORB_time_offset)
  {
      std::strcpy(name, (ORB_ID(time_offset))->o_name);
  }
  else if (serial == ORB_transponder_report)
  {
      std::strcpy(name, (ORB_ID(transponder_report))->o_name);
  }
  else if (serial == ORB_uavcan_parameter_request)
  {
      std::strcpy(name, (ORB_ID(uavcan_parameter_request))->o_name);
  }
  else if (serial == ORB_uavcan_parameter_value)
  {
      std::strcpy(name, (ORB_ID(uavcan_parameter_value))->o_name);
  }
  else if (serial == ORB_ulog_stream)
  {
      std::strcpy(name, (ORB_ID(ulog_stream))->o_name);
  }
  else if (serial == ORB_ulog_stream_ack)
  {
      std::strcpy(name, (ORB_ID(ulog_stream_ack))->o_name);
  }
  else if (serial == ORB_vehicle_attitude)
  {
      std::strcpy(name, (ORB_ID(vehicle_attitude))->o_name);
  }
  else if (serial == ORB_vehicle_attitude_setpoint)
  {
      std::strcpy(name, (ORB_ID(vehicle_attitude_setpoint))->o_name);
  }
  else if (serial == ORB_vehicle_command_ack)
  {
      std::strcpy(name, (ORB_ID(vehicle_command_ack))->o_name);
  }
  else if (serial == ORB_vehicle_command)
  {
      std::strcpy(name, (ORB_ID(vehicle_command))->o_name);
  }
  else if (serial == ORB_vehicle_control_mode)
  {
      std::strcpy(name, (ORB_ID(vehicle_control_mode))->o_name);
  }
  else if (serial == ORB_vehicle_force_setpoint)
  {
      std::strcpy(name, (ORB_ID(vehicle_force_setpoint))->o_name);
  }
  else if (serial == ORB_vehicle_global_position)
  {
      std::strcpy(name, (ORB_ID(vehicle_global_position))->o_name);
  }
  else if (serial == ORB_vehicle_global_velocity_setpoint)
  {
      std::strcpy(name, (ORB_ID(vehicle_global_velocity_setpoint))->o_name);
  }
  else if (serial == ORB_vehicle_gps_position)
  {
      std::strcpy(name, (ORB_ID(vehicle_gps_position))->o_name);
  }
  else if (serial == ORB_vehicle_land_detected)
  {
      std::strcpy(name, (ORB_ID(vehicle_land_detected))->o_name);
  }
  else if (serial == ORB_vehicle_local_position)
  {
      std::strcpy(name, (ORB_ID(vehicle_local_position))->o_name);
  }
  else if (serial == ORB_vehicle_local_position_setpoint)
  {
      std::strcpy(name, (ORB_ID(vehicle_local_position_setpoint))->o_name);
  }
  else if (serial == ORB_vehicle_rates_setpoint)
  {
      std::strcpy(name, (ORB_ID(vehicle_rates_setpoint))->o_name);
  }
  else if (serial == ORB_vehicle_roi)
  {
      std::strcpy(name, (ORB_ID(vehicle_roi))->o_name);
  }
  else if (serial == ORB_vehicle_status)
  {
      std::strcpy(name, (ORB_ID(vehicle_status))->o_name);
  }
  else if (serial == ORB_vehicle_status_flags)
  {
      std::strcpy(name, (ORB_ID(vehicle_status_flags))->o_name);
  }
  else if (serial == ORB_vtol_vehicle_status)
  {
      std::strcpy(name, (ORB_ID(vtol_vehicle_status))->o_name);
  }
  else if (serial == ORB_wind_estimate)
  {
      std::strcpy(name, (ORB_ID(wind_estimate))->o_name);
  }
  else if (serial == ORB_actuator_controls_0)
  {
      std::strcpy(name, (ORB_ID(actuator_controls_0))->o_name);
  }
  else if (serial == ORB_actuator_controls_1)
  {
      std::strcpy(name, (ORB_ID(actuator_controls_1))->o_name);
  }
  else if (serial == ORB_actuator_controls_2)
  {
      std::strcpy(name, (ORB_ID(actuator_controls_2))->o_name);
  }
  else if (serial == ORB_actuator_controls_3)
  {
      std::strcpy(name, (ORB_ID(actuator_controls_3))->o_name);
  }
  else if (serial == ORB_actuator_controls_virtual_fw)
  {
      std::strcpy(name, (ORB_ID(actuator_controls_virtual_fw))->o_name);
  }
  else if (serial == ORB_actuator_controls_virtual_mc)
  {
      std::strcpy(name, (ORB_ID(actuator_controls_virtual_mc))->o_name);
  }
  else if (serial == ORB_offboard_mission)
  {
      std::strcpy(name, (ORB_ID(offboard_mission))->o_name);
  }
  else if (serial == ORB_onboard_mission)
  {
      std::strcpy(name, (ORB_ID(onboard_mission))->o_name);
  }
  else if (serial == ORB_vehicle_local_position_groundtruth)
  {
      std::strcpy(name, (ORB_ID(vehicle_local_position_groundtruth))->o_name);
  }
  else if (serial == ORB_vehicle_vision_position)
  {
      std::strcpy(name, (ORB_ID(vehicle_vision_position))->o_name);
  }
  else if (serial == ORB_vehicle_global_position_groundtruth)
  {
      std::strcpy(name, (ORB_ID(vehicle_global_position_groundtruth))->o_name);
  }
  else if (serial == ORB_vehicle_attitude_groundtruth)
  {
      std::strcpy(name, (ORB_ID(vehicle_attitude_groundtruth))->o_name);
  }
  else if (serial == ORB_vehicle_vision_attitude)
  {
      std::strcpy(name, (ORB_ID(vehicle_vision_attitude))->o_name);
  }
  else
      std::strcpy(name, "");
}


int  get_orb_serial(const char * name)
{
  if (0 == std::strcmp(name, (ORB_ID(actuator_armed))->o_name))
  {
      return ORB_actuator_armed;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_controls))->o_name))
  {
      return ORB_actuator_controls;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_direct))->o_name))
  {
      return ORB_actuator_direct;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_outputs))->o_name))
  {
      return ORB_actuator_outputs;
  }
  else if (0 == std::strcmp(name, (ORB_ID(adc_report))->o_name))
  {
      return ORB_adc_report;
  }
  else if (0 == std::strcmp(name, (ORB_ID(airspeed))->o_name))
  {
      return ORB_airspeed;
  }
  else if (0 == std::strcmp(name, (ORB_ID(att_pos_mocap))->o_name))
  {
      return ORB_att_pos_mocap;
  }
  else if (0 == std::strcmp(name, (ORB_ID(battery_status))->o_name))
  {
      return ORB_battery_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(camera_trigger))->o_name))
  {
      return ORB_camera_trigger;
  }
  else if (0 == std::strcmp(name, (ORB_ID(commander_state))->o_name))
  {
      return ORB_commander_state;
  }
  else if (0 == std::strcmp(name, (ORB_ID(collision_report))->o_name))
  {
      return ORB_collision_report;
  }
  else if (0 == std::strcmp(name, (ORB_ID(control_state))->o_name))
  {
      return ORB_control_state;
  }
  else if (0 == std::strcmp(name, (ORB_ID(cpuload))->o_name))
  {
      return ORB_cpuload;
  }
  else if (0 == std::strcmp(name, (ORB_ID(debug_key_value))->o_name))
  {
      return ORB_debug_key_value;
  }
  else if (0 == std::strcmp(name, (ORB_ID(differential_pressure))->o_name))
  {
      return ORB_differential_pressure;
  }
  else if (0 == std::strcmp(name, (ORB_ID(distance_sensor))->o_name))
  {
      return ORB_distance_sensor;
  }
  else if (0 == std::strcmp(name, (ORB_ID(ekf2_innovations))->o_name))
  {
      return ORB_ekf2_innovations;
  }
  else if (0 == std::strcmp(name, (ORB_ID(ekf2_replay))->o_name))
  {
      return ORB_ekf2_replay;
  }
  else if (0 == std::strcmp(name, (ORB_ID(ekf2_timestamps))->o_name))
  {
      return ORB_ekf2_timestamps;
  }
  else if (0 == std::strcmp(name, (ORB_ID(esc_report))->o_name))
  {
      return ORB_esc_report;
  }
  else if (0 == std::strcmp(name, (ORB_ID(esc_status))->o_name))
  {
      return ORB_esc_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(estimator_status))->o_name))
  {
      return ORB_estimator_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(fence))->o_name))
  {
      return ORB_fence;
  }
  else if (0 == std::strcmp(name, (ORB_ID(fence_vertex))->o_name))
  {
      return ORB_fence_vertex;
  }
  else if (0 == std::strcmp(name, (ORB_ID(filtered_bottom_flow))->o_name))
  {
      return ORB_filtered_bottom_flow;
  }
  else if (0 == std::strcmp(name, (ORB_ID(follow_target))->o_name))
  {
      return ORB_follow_target;
  }
  else if (0 == std::strcmp(name, (ORB_ID(fw_pos_ctrl_status))->o_name))
  {
      return ORB_fw_pos_ctrl_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(fw_virtual_attitude_setpoint))->o_name))
  {
      return ORB_fw_virtual_attitude_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(fw_virtual_rates_setpoint))->o_name))
  {
      return ORB_fw_virtual_rates_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(geofence_result))->o_name))
  {
      return ORB_geofence_result;
  }
  else if (0 == std::strcmp(name, (ORB_ID(gps_dump))->o_name))
  {
      return ORB_gps_dump;
  }
  else if (0 == std::strcmp(name, (ORB_ID(gps_inject_data))->o_name))
  {
      return ORB_gps_inject_data;
  }
  else if (0 == std::strcmp(name, (ORB_ID(hil_sensor))->o_name))
  {
      return ORB_hil_sensor;
  }
  else if (0 == std::strcmp(name, (ORB_ID(home_position))->o_name))
  {
      return ORB_home_position;
  }
  else if (0 == std::strcmp(name, (ORB_ID(input_rc))->o_name))
  {
      return ORB_input_rc;
  }
  else if (0 == std::strcmp(name, (ORB_ID(led_control))->o_name))
  {
      return ORB_led_control;
  }
  else if (0 == std::strcmp(name, (ORB_ID(log_message))->o_name))
  {
      return ORB_log_message;
  }
  else if (0 == std::strcmp(name, (ORB_ID(manual_control_setpoint))->o_name))
  {
      return ORB_manual_control_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mavlink_log))->o_name))
  {
      return ORB_mavlink_log;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mc_att_ctrl_status))->o_name))
  {
      return ORB_mc_att_ctrl_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mc_virtual_attitude_setpoint))->o_name))
  {
      return ORB_mc_virtual_attitude_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mc_virtual_rates_setpoint))->o_name))
  {
      return ORB_mc_virtual_rates_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mission))->o_name))
  {
      return ORB_mission;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mission_result))->o_name))
  {
      return ORB_mission_result;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mount_orientation))->o_name))
  {
      return ORB_mount_orientation;
  }
  else if (0 == std::strcmp(name, (ORB_ID(multirotor_motor_limits))->o_name))
  {
      return ORB_multirotor_motor_limits;
  }
  else if (0 == std::strcmp(name, (ORB_ID(offboard_control_mode))->o_name))
  {
      return ORB_offboard_control_mode;
  }
  else if (0 == std::strcmp(name, (ORB_ID(optical_flow))->o_name))
  {
      return ORB_optical_flow;
  }
  else if (0 == std::strcmp(name, (ORB_ID(output_pwm))->o_name))
  {
      return ORB_output_pwm;
  }
  else if (0 == std::strcmp(name, (ORB_ID(parameter_update))->o_name))
  {
      return ORB_parameter_update;
  }
  else if (0 == std::strcmp(name, (ORB_ID(position_setpoint))->o_name))
  {
      return ORB_position_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(position_setpoint_triplet))->o_name))
  {
      return ORB_position_setpoint_triplet;
  }
  else if (0 == std::strcmp(name, (ORB_ID(pwm_input))->o_name))
  {
      return ORB_pwm_input;
  }
  else if (0 == std::strcmp(name, (ORB_ID(qshell_req))->o_name))
  {
      return ORB_qshell_req;
  }
  else if (0 == std::strcmp(name, (ORB_ID(rc_channels))->o_name))
  {
      return ORB_rc_channels;
  }
  else if (0 == std::strcmp(name, (ORB_ID(rc_parameter_map))->o_name))
  {
      return ORB_rc_parameter_map;
  }
  else if (0 == std::strcmp(name, (ORB_ID(safety))->o_name))
  {
      return ORB_safety;
  }
  else if (0 == std::strcmp(name, (ORB_ID(satellite_info))->o_name))
  {
      return ORB_satellite_info;
  }
  else if (0 == std::strcmp(name, (ORB_ID(sensor_accel))->o_name))
  {
      return ORB_sensor_accel;
  }
  else if (0 == std::strcmp(name, (ORB_ID(sensor_baro))->o_name))
  {
      return ORB_sensor_baro;
  }
  else if (0 == std::strcmp(name, (ORB_ID(sensor_combined))->o_name))
  {
      return ORB_sensor_combined;
  }
  else if (0 == std::strcmp(name, (ORB_ID(sensor_correction))->o_name))
  {
      return ORB_sensor_correction;
  }
  else if (0 == std::strcmp(name, (ORB_ID(sensor_gyro))->o_name))
  {
      return ORB_sensor_gyro;
  }
  else if (0 == std::strcmp(name, (ORB_ID(sensor_mag))->o_name))
  {
      return ORB_sensor_mag;
  }
  else if (0 == std::strcmp(name, (ORB_ID(sensor_preflight))->o_name))
  {
      return ORB_sensor_preflight;
  }
  else if (0 == std::strcmp(name, (ORB_ID(servorail_status))->o_name))
  {
      return ORB_servorail_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(subsystem_info))->o_name))
  {
      return ORB_subsystem_info;
  }
  else if (0 == std::strcmp(name, (ORB_ID(system_power))->o_name))
  {
      return ORB_system_power;
  }
  else if (0 == std::strcmp(name, (ORB_ID(task_stack_info))->o_name))
  {
      return ORB_task_stack_info;
  }
  else if (0 == std::strcmp(name, (ORB_ID(tecs_status))->o_name))
  {
      return ORB_tecs_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(telemetry_status))->o_name))
  {
      return ORB_telemetry_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(test_motor))->o_name))
  {
      return ORB_test_motor;
  }
  else if (0 == std::strcmp(name, (ORB_ID(time_offset))->o_name))
  {
      return ORB_time_offset;
  }
  else if (0 == std::strcmp(name, (ORB_ID(transponder_report))->o_name))
  {
      return ORB_transponder_report;
  }
  else if (0 == std::strcmp(name, (ORB_ID(uavcan_parameter_request))->o_name))
  {
      return ORB_uavcan_parameter_request;
  }
  else if (0 == std::strcmp(name, (ORB_ID(uavcan_parameter_value))->o_name))
  {
      return ORB_uavcan_parameter_value;
  }
  else if (0 == std::strcmp(name, (ORB_ID(ulog_stream))->o_name))
  {
      return ORB_ulog_stream;
  }
  else if (0 == std::strcmp(name, (ORB_ID(ulog_stream_ack))->o_name))
  {
      return ORB_ulog_stream_ack;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_attitude))->o_name))
  {
      return ORB_vehicle_attitude;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_attitude_setpoint))->o_name))
  {
      return ORB_vehicle_attitude_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_command_ack))->o_name))
  {
      return ORB_vehicle_command_ack;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_command))->o_name))
  {
      return ORB_vehicle_command;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_control_mode))->o_name))
  {
      return ORB_vehicle_control_mode;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_force_setpoint))->o_name))
  {
      return ORB_vehicle_force_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_global_position))->o_name))
  {
      return ORB_vehicle_global_position;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_global_velocity_setpoint))->o_name))
  {
      return ORB_vehicle_global_velocity_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_gps_position))->o_name))
  {
      return ORB_vehicle_gps_position;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_land_detected))->o_name))
  {
      return ORB_vehicle_land_detected;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_local_position))->o_name))
  {
      return ORB_vehicle_local_position;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_local_position_setpoint))->o_name))
  {
      return ORB_vehicle_local_position_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_rates_setpoint))->o_name))
  {
      return ORB_vehicle_rates_setpoint;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_roi))->o_name))
  {
      return ORB_vehicle_roi;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_status))->o_name))
  {
      return ORB_vehicle_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_status_flags))->o_name))
  {
      return ORB_vehicle_status_flags;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vtol_vehicle_status))->o_name))
  {
      return ORB_vtol_vehicle_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(wind_estimate))->o_name))
  {
      return ORB_wind_estimate;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_controls_0))->o_name))
  {
      return ORB_actuator_controls_0;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_controls_1))->o_name))
  {
      return ORB_actuator_controls_1;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_controls_2))->o_name))
  {
      return ORB_actuator_controls_2;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_controls_3))->o_name))
  {
      return ORB_actuator_controls_3;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_controls_virtual_fw))->o_name))
  {
      return ORB_actuator_controls_virtual_fw;
  }
  else if (0 == std::strcmp(name, (ORB_ID(actuator_controls_virtual_mc))->o_name))
  {
      return ORB_actuator_controls_virtual_mc;
  }
  else if (0 == std::strcmp(name, (ORB_ID(offboard_mission))->o_name))
  {
      return ORB_offboard_mission;
  }
  else if (0 == std::strcmp(name, (ORB_ID(onboard_mission))->o_name))
  {
      return ORB_onboard_mission;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_local_position_groundtruth))->o_name))
  {
      return ORB_vehicle_local_position_groundtruth;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_vision_position))->o_name))
  {
      return ORB_vehicle_vision_position;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_global_position_groundtruth))->o_name))
  {
      return ORB_vehicle_global_position_groundtruth;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_attitude_groundtruth))->o_name))
  {
      return ORB_vehicle_attitude_groundtruth;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_vision_attitude))->o_name))
  {
      return ORB_vehicle_vision_attitude;
  }
  else
    return -1;
}


orb_id_t  get_orb_according_to_serial(const int serial)
{
  char orb_name[50];
  get_orb_name(serial,orb_name);

  if(0 == std::strcmp(orb_name, ""))
      return nullptr;

  if (0 == std::strcmp(orb_name, (ORB_ID(actuator_armed))->o_name))
  {
      return ORB_ID(actuator_armed);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_controls))->o_name))
  {
      return ORB_ID(actuator_controls);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_direct))->o_name))
  {
      return ORB_ID(actuator_direct);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_outputs))->o_name))
  {
      return ORB_ID(actuator_outputs);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(adc_report))->o_name))
  {
      return ORB_ID(adc_report);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(airspeed))->o_name))
  {
      return ORB_ID(airspeed);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(att_pos_mocap))->o_name))
  {
      return ORB_ID(att_pos_mocap);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(battery_status))->o_name))
  {
      return ORB_ID(battery_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(camera_trigger))->o_name))
  {
      return ORB_ID(camera_trigger);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(commander_state))->o_name))
  {
      return ORB_ID(commander_state);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(collision_report))->o_name))
  {
      return ORB_ID(collision_report);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(control_state))->o_name))
  {
      return ORB_ID(control_state);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(cpuload))->o_name))
  {
      return ORB_ID(cpuload);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(debug_key_value))->o_name))
  {
      return ORB_ID(debug_key_value);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(differential_pressure))->o_name))
  {
      return ORB_ID(differential_pressure);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(distance_sensor))->o_name))
  {
      return ORB_ID(distance_sensor);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(ekf2_innovations))->o_name))
  {
      return ORB_ID(ekf2_innovations);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(ekf2_replay))->o_name))
  {
      return ORB_ID(ekf2_replay);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(ekf2_timestamps))->o_name))
  {
      return ORB_ID(ekf2_timestamps);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(esc_report))->o_name))
  {
      return ORB_ID(esc_report);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(esc_status))->o_name))
  {
      return ORB_ID(esc_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(estimator_status))->o_name))
  {
      return ORB_ID(estimator_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(fence))->o_name))
  {
      return ORB_ID(fence);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(fence_vertex))->o_name))
  {
      return ORB_ID(fence_vertex);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(filtered_bottom_flow))->o_name))
  {
      return ORB_ID(filtered_bottom_flow);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(follow_target))->o_name))
  {
      return ORB_ID(follow_target);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(fw_pos_ctrl_status))->o_name))
  {
      return ORB_ID(fw_pos_ctrl_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(fw_virtual_attitude_setpoint))->o_name))
  {
      return ORB_ID(fw_virtual_attitude_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(fw_virtual_rates_setpoint))->o_name))
  {
      return ORB_ID(fw_virtual_rates_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(geofence_result))->o_name))
  {
      return ORB_ID(geofence_result);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(gps_dump))->o_name))
  {
      return ORB_ID(gps_dump);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(gps_inject_data))->o_name))
  {
      return ORB_ID(gps_inject_data);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(hil_sensor))->o_name))
  {
      return ORB_ID(hil_sensor);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(home_position))->o_name))
  {
      return ORB_ID(home_position);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(input_rc))->o_name))
  {
      return ORB_ID(input_rc);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(led_control))->o_name))
  {
      return ORB_ID(led_control);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(log_message))->o_name))
  {
      return ORB_ID(log_message);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(manual_control_setpoint))->o_name))
  {
      return ORB_ID(manual_control_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mavlink_log))->o_name))
  {
      return ORB_ID(mavlink_log);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mc_att_ctrl_status))->o_name))
  {
      return ORB_ID(mc_att_ctrl_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mc_virtual_attitude_setpoint))->o_name))
  {
      return ORB_ID(mc_virtual_attitude_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mc_virtual_rates_setpoint))->o_name))
  {
      return ORB_ID(mc_virtual_rates_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mission))->o_name))
  {
      return ORB_ID(mission);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mission_result))->o_name))
  {
      return ORB_ID(mission_result);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mount_orientation))->o_name))
  {
      return ORB_ID(mount_orientation);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(multirotor_motor_limits))->o_name))
  {
      return ORB_ID(multirotor_motor_limits);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(offboard_control_mode))->o_name))
  {
      return ORB_ID(offboard_control_mode);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(optical_flow))->o_name))
  {
      return ORB_ID(optical_flow);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(output_pwm))->o_name))
  {
      return ORB_ID(output_pwm);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(parameter_update))->o_name))
  {
      return ORB_ID(parameter_update);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(position_setpoint))->o_name))
  {
      return ORB_ID(position_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(position_setpoint_triplet))->o_name))
  {
      return ORB_ID(position_setpoint_triplet);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(pwm_input))->o_name))
  {
      return ORB_ID(pwm_input);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(qshell_req))->o_name))
  {
      return ORB_ID(qshell_req);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(rc_channels))->o_name))
  {
      return ORB_ID(rc_channels);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(rc_parameter_map))->o_name))
  {
      return ORB_ID(rc_parameter_map);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(safety))->o_name))
  {
      return ORB_ID(safety);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(satellite_info))->o_name))
  {
      return ORB_ID(satellite_info);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(sensor_accel))->o_name))
  {
      return ORB_ID(sensor_accel);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(sensor_baro))->o_name))
  {
      return ORB_ID(sensor_baro);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(sensor_combined))->o_name))
  {
      return ORB_ID(sensor_combined);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(sensor_correction))->o_name))
  {
      return ORB_ID(sensor_correction);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(sensor_gyro))->o_name))
  {
      return ORB_ID(sensor_gyro);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(sensor_mag))->o_name))
  {
      return ORB_ID(sensor_mag);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(sensor_preflight))->o_name))
  {
      return ORB_ID(sensor_preflight);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(servorail_status))->o_name))
  {
      return ORB_ID(servorail_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(subsystem_info))->o_name))
  {
      return ORB_ID(subsystem_info);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(system_power))->o_name))
  {
      return ORB_ID(system_power);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(task_stack_info))->o_name))
  {
      return ORB_ID(task_stack_info);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(tecs_status))->o_name))
  {
      return ORB_ID(tecs_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(telemetry_status))->o_name))
  {
      return ORB_ID(telemetry_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(test_motor))->o_name))
  {
      return ORB_ID(test_motor);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(time_offset))->o_name))
  {
      return ORB_ID(time_offset);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(transponder_report))->o_name))
  {
      return ORB_ID(transponder_report);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(uavcan_parameter_request))->o_name))
  {
      return ORB_ID(uavcan_parameter_request);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(uavcan_parameter_value))->o_name))
  {
      return ORB_ID(uavcan_parameter_value);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(ulog_stream))->o_name))
  {
      return ORB_ID(ulog_stream);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(ulog_stream_ack))->o_name))
  {
      return ORB_ID(ulog_stream_ack);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude))->o_name))
  {
      return ORB_ID(vehicle_attitude);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude_setpoint))->o_name))
  {
      return ORB_ID(vehicle_attitude_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_command_ack))->o_name))
  {
      return ORB_ID(vehicle_command_ack);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_command))->o_name))
  {
      return ORB_ID(vehicle_command);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_control_mode))->o_name))
  {
      return ORB_ID(vehicle_control_mode);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_force_setpoint))->o_name))
  {
      return ORB_ID(vehicle_force_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_global_position))->o_name))
  {
      return ORB_ID(vehicle_global_position);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_global_velocity_setpoint))->o_name))
  {
      return ORB_ID(vehicle_global_velocity_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_gps_position))->o_name))
  {
      return ORB_ID(vehicle_gps_position);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_land_detected))->o_name))
  {
      return ORB_ID(vehicle_land_detected);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_local_position))->o_name))
  {
      return ORB_ID(vehicle_local_position);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_local_position_setpoint))->o_name))
  {
      return ORB_ID(vehicle_local_position_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_rates_setpoint))->o_name))
  {
      return ORB_ID(vehicle_rates_setpoint);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_roi))->o_name))
  {
      return ORB_ID(vehicle_roi);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_status))->o_name))
  {
      return ORB_ID(vehicle_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_status_flags))->o_name))
  {
      return ORB_ID(vehicle_status_flags);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vtol_vehicle_status))->o_name))
  {
      return ORB_ID(vtol_vehicle_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(wind_estimate))->o_name))
  {
      return ORB_ID(wind_estimate);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_0))->o_name))
  {
      return ORB_ID(actuator_controls_0);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_1))->o_name))
  {
      return ORB_ID(actuator_controls_1);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_2))->o_name))
  {
      return ORB_ID(actuator_controls_2);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_3))->o_name))
  {
      return ORB_ID(actuator_controls_3);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_virtual_fw))->o_name))
  {
      return ORB_ID(actuator_controls_virtual_fw);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_virtual_mc))->o_name))
  {
      return ORB_ID(actuator_controls_virtual_mc);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(offboard_mission))->o_name))
  {
      return ORB_ID(offboard_mission);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(onboard_mission))->o_name))
  {
      return ORB_ID(onboard_mission);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_local_position_groundtruth))->o_name))
  {
      return ORB_ID(vehicle_local_position_groundtruth);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_vision_position))->o_name))
  {
      return ORB_ID(vehicle_vision_position);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_global_position_groundtruth))->o_name))
  {
      return ORB_ID(vehicle_global_position_groundtruth);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude_groundtruth))->o_name))
  {
      return ORB_ID(vehicle_attitude_groundtruth);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_vision_attitude))->o_name))
  {
      return ORB_ID(vehicle_vision_attitude);
  }
  else
      return nullptr;
}
