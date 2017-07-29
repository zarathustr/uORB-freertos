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
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_controls_virtual_fw.h>
#include <uORB/topics/actuator_controls_virtual_mc.h>
#include <uORB/topics/offboard_mission.h>
#include <uORB/topics/onboard_mission.h>
#include <uORB/topics/vehicle_local_position_groundtruth.h>
#include <uORB/topics/vehicle_vision_position.h>
#include <uORB/topics/vehicle_global_position_groundtruth.h>
#include <uORB/topics/vehicle_attitude_groundtruth.h>
#include <uORB/topics/vehicle_vision_attitude.h>



#pragma default_variable_attributes= @ ".ccmram"


  actuator_armed_s			ORB_actuator_armed_public;
  actuator_controls_s			ORB_actuator_controls_public;
  actuator_direct_s			ORB_actuator_direct_public;
  actuator_outputs_s			ORB_actuator_outputs_public;
  adc_report_s			ORB_adc_report_public;
  airspeed_s			ORB_airspeed_public;
  att_pos_mocap_s			ORB_att_pos_mocap_public;
  battery_status_s			ORB_battery_status_public;
  camera_trigger_s			ORB_camera_trigger_public;
  commander_state_s			ORB_commander_state_public;
  collision_report_s			ORB_collision_report_public;
  control_state_s			ORB_control_state_public[ORB_MULTI_MAX_INSTANCES];
  cpuload_s			ORB_cpuload_public;
  debug_key_value_s			ORB_debug_key_value_public;
  differential_pressure_s			ORB_differential_pressure_public;
  distance_sensor_s			ORB_distance_sensor_public[ORB_MULTI_MAX_INSTANCES];
  ekf2_innovations_s			ORB_ekf2_innovations_public;
  ekf2_replay_s			ORB_ekf2_replay_public;
  ekf2_timestamps_s			ORB_ekf2_timestamps_public;
  esc_report_s			ORB_esc_report_public;
  esc_status_s			ORB_esc_status_public;
  estimator_status_s			ORB_estimator_status_public[ORB_MULTI_MAX_INSTANCES];
  fence_s			ORB_fence_public;
  fence_vertex_s			ORB_fence_vertex_public;
  filtered_bottom_flow_s			ORB_filtered_bottom_flow_public;
  follow_target_s			ORB_follow_target_public;
  fw_pos_ctrl_status_s			ORB_fw_pos_ctrl_status_public;
  fw_virtual_attitude_setpoint_s			ORB_fw_virtual_attitude_setpoint_public;
  fw_virtual_rates_setpoint_s			ORB_fw_virtual_rates_setpoint_public;
  geofence_result_s			ORB_geofence_result_public;
  gps_dump_s			ORB_gps_dump_public;
  gps_inject_data_s			ORB_gps_inject_data_public;
  hil_sensor_s			ORB_hil_sensor_public;
  home_position_s			ORB_home_position_public;
  input_rc_s			ORB_input_rc_public;
  led_control_s			ORB_led_control_public;
  log_message_s			ORB_log_message_public;
  manual_control_setpoint_s			ORB_manual_control_setpoint_public;
  mavlink_log_s			ORB_mavlink_log_public;
  mc_att_ctrl_status_s			ORB_mc_att_ctrl_status_public;
  mc_virtual_attitude_setpoint_s			ORB_mc_virtual_attitude_setpoint_public;
  mc_virtual_rates_setpoint_s			ORB_mc_virtual_rates_setpoint_public;
  mission_s			ORB_mission_public;
  mission_result_s			ORB_mission_result_public;
  mount_orientation_s			ORB_mount_orientation_public;
  multirotor_motor_limits_s			ORB_multirotor_motor_limits_public;
  offboard_control_mode_s			ORB_offboard_control_mode_public;
  optical_flow_s			ORB_optical_flow_public;
  output_pwm_s			ORB_output_pwm_public;
  parameter_update_s			ORB_parameter_update_public;
  position_setpoint_s			ORB_position_setpoint_public;
  position_setpoint_triplet_s			ORB_position_setpoint_triplet_public;
  pwm_input_s			ORB_pwm_input_public;
  qshell_req_s			ORB_qshell_req_public;
  rc_channels_s			ORB_rc_channels_public;
  rc_parameter_map_s			ORB_rc_parameter_map_public;
  safety_s			ORB_safety_public;
  satellite_info_s			ORB_satellite_info_public;
  sensor_accel_s			ORB_sensor_accel_public[ORB_MULTI_MAX_INSTANCES];
  sensor_baro_s			ORB_sensor_baro_public[ORB_MULTI_MAX_INSTANCES];
  sensor_combined_s			ORB_sensor_combined_public[ORB_MULTI_MAX_INSTANCES];
  sensor_correction_s			ORB_sensor_correction_public;
  sensor_gyro_s			ORB_sensor_gyro_public[ORB_MULTI_MAX_INSTANCES];
  sensor_mag_s			ORB_sensor_mag_public[ORB_MULTI_MAX_INSTANCES];
  sensor_preflight_s			ORB_sensor_preflight_public;
  servorail_status_s			ORB_servorail_status_public;
  subsystem_info_s			ORB_subsystem_info_public;
  system_power_s			ORB_system_power_public;
  task_stack_info_s			ORB_task_stack_info_public;
  tecs_status_s			ORB_tecs_status_public;
  telemetry_status_s			ORB_telemetry_status_public[ORB_MULTI_MAX_INSTANCES];
  test_motor_s			ORB_test_motor_public;
  time_offset_s			ORB_time_offset_public;
  transponder_report_s			ORB_transponder_report_public;
  uavcan_parameter_request_s			ORB_uavcan_parameter_request_public;
  uavcan_parameter_value_s			ORB_uavcan_parameter_value_public;
  ulog_stream_s			ORB_ulog_stream_public;
  ulog_stream_ack_s			ORB_ulog_stream_ack_public;
  vehicle_attitude_s			ORB_vehicle_attitude_public[ORB_MULTI_MAX_INSTANCES];
  vehicle_attitude_setpoint_s			ORB_vehicle_attitude_setpoint_public;
  vehicle_command_ack_s			ORB_vehicle_command_ack_public;
  vehicle_command_s			ORB_vehicle_command_public;
  vehicle_control_mode_s			ORB_vehicle_control_mode_public;
  vehicle_force_setpoint_s			ORB_vehicle_force_setpoint_public;
  vehicle_global_position_s			ORB_vehicle_global_position_public[ORB_MULTI_MAX_INSTANCES];
  vehicle_global_velocity_setpoint_s			ORB_vehicle_global_velocity_setpoint_public;
  vehicle_gps_position_s			ORB_vehicle_gps_position_public;
  vehicle_land_detected_s			ORB_vehicle_land_detected_public;
  vehicle_local_position_s			ORB_vehicle_local_position_public[ORB_MULTI_MAX_INSTANCES];
  vehicle_local_position_setpoint_s			ORB_vehicle_local_position_setpoint_public;
  vehicle_rates_setpoint_s			ORB_vehicle_rates_setpoint_public;
  vehicle_roi_s			ORB_vehicle_roi_public;
  vehicle_status_s			ORB_vehicle_status_public;
  vehicle_status_flags_s			ORB_vehicle_status_flags_public;
  vtol_vehicle_status_s			ORB_vtol_vehicle_status_public;
  wind_estimate_s			ORB_wind_estimate_public;
  actuator_controls_0_s			ORB_actuator_controls_0_public;
  actuator_controls_1_s			ORB_actuator_controls_1_public;
  actuator_controls_2_s			ORB_actuator_controls_2_public;
  actuator_controls_3_s			ORB_actuator_controls_3_public;
  actuator_controls_virtual_fw_s			ORB_actuator_controls_virtual_fw_public;
  actuator_controls_virtual_mc_s			ORB_actuator_controls_virtual_mc_public;
  offboard_mission_s			ORB_offboard_mission_public;
  onboard_mission_s			ORB_onboard_mission_public;
  vehicle_local_position_groundtruth_s			ORB_vehicle_local_position_groundtruth_public;
  vehicle_vision_position_s			ORB_vehicle_vision_position_public;
  vehicle_global_position_groundtruth_s			ORB_vehicle_global_position_groundtruth_public;
  vehicle_attitude_groundtruth_s			ORB_vehicle_attitude_groundtruth_public;
  vehicle_vision_attitude_s			ORB_vehicle_vision_attitude_public;


#pragma default_variable_attributes = 





void orb_helper_init(void)
{
  std::memset(&ORB_actuator_armed_public, 0, sizeof(actuator_armed_s));
  std::memset(&ORB_actuator_controls_public, 0, sizeof(actuator_controls_s));
  std::memset(&ORB_actuator_direct_public, 0, sizeof(actuator_direct_s));
  std::memset(&ORB_actuator_outputs_public, 0, sizeof(actuator_outputs_s));
  std::memset(&ORB_adc_report_public, 0, sizeof(adc_report_s));
  std::memset(&ORB_airspeed_public, 0, sizeof(airspeed_s));
  std::memset(&ORB_att_pos_mocap_public, 0, sizeof(att_pos_mocap_s));
  std::memset(&ORB_battery_status_public, 0, sizeof(battery_status_s));
  std::memset(&ORB_camera_trigger_public, 0, sizeof(camera_trigger_s));
  std::memset(&ORB_commander_state_public, 0, sizeof(commander_state_s));
  std::memset(&ORB_collision_report_public, 0, sizeof(collision_report_s));
  std::memset(&ORB_control_state_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(control_state_s));
  std::memset(&ORB_cpuload_public, 0, sizeof(cpuload_s));
  std::memset(&ORB_debug_key_value_public, 0, sizeof(debug_key_value_s));
  std::memset(&ORB_differential_pressure_public, 0, sizeof(differential_pressure_s));
  std::memset(&ORB_distance_sensor_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(distance_sensor_s));
  std::memset(&ORB_ekf2_innovations_public, 0, sizeof(ekf2_innovations_s));
  std::memset(&ORB_ekf2_replay_public, 0, sizeof(ekf2_replay_s));
  std::memset(&ORB_ekf2_timestamps_public, 0, sizeof(ekf2_timestamps_s));
  std::memset(&ORB_esc_report_public, 0, sizeof(esc_report_s));
  std::memset(&ORB_esc_status_public, 0, sizeof(esc_status_s));
  std::memset(&ORB_estimator_status_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(estimator_status_s));
  std::memset(&ORB_fence_public, 0, sizeof(fence_s));
  std::memset(&ORB_fence_vertex_public, 0, sizeof(fence_vertex_s));
  std::memset(&ORB_filtered_bottom_flow_public, 0, sizeof(filtered_bottom_flow_s));
  std::memset(&ORB_follow_target_public, 0, sizeof(follow_target_s));
  std::memset(&ORB_fw_pos_ctrl_status_public, 0, sizeof(fw_pos_ctrl_status_s));
  std::memset(&ORB_fw_virtual_attitude_setpoint_public, 0, sizeof(fw_virtual_attitude_setpoint_s));
  std::memset(&ORB_fw_virtual_rates_setpoint_public, 0, sizeof(fw_virtual_rates_setpoint_s));
  std::memset(&ORB_geofence_result_public, 0, sizeof(geofence_result_s));
  std::memset(&ORB_gps_dump_public, 0, sizeof(gps_dump_s));
  std::memset(&ORB_gps_inject_data_public, 0, sizeof(gps_inject_data_s));
  std::memset(&ORB_hil_sensor_public, 0, sizeof(hil_sensor_s));
  std::memset(&ORB_home_position_public, 0, sizeof(home_position_s));
  std::memset(&ORB_input_rc_public, 0, sizeof(input_rc_s));
  std::memset(&ORB_led_control_public, 0, sizeof(led_control_s));
  std::memset(&ORB_log_message_public, 0, sizeof(log_message_s));
  std::memset(&ORB_manual_control_setpoint_public, 0, sizeof(manual_control_setpoint_s));
  std::memset(&ORB_mavlink_log_public, 0, sizeof(mavlink_log_s));
  std::memset(&ORB_mc_att_ctrl_status_public, 0, sizeof(mc_att_ctrl_status_s));
  std::memset(&ORB_mc_virtual_attitude_setpoint_public, 0, sizeof(mc_virtual_attitude_setpoint_s));
  std::memset(&ORB_mc_virtual_rates_setpoint_public, 0, sizeof(mc_virtual_rates_setpoint_s));
  std::memset(&ORB_mission_public, 0, sizeof(mission_s));
  std::memset(&ORB_mission_result_public, 0, sizeof(mission_result_s));
  std::memset(&ORB_mount_orientation_public, 0, sizeof(mount_orientation_s));
  std::memset(&ORB_multirotor_motor_limits_public, 0, sizeof(multirotor_motor_limits_s));
  std::memset(&ORB_offboard_control_mode_public, 0, sizeof(offboard_control_mode_s));
  std::memset(&ORB_optical_flow_public, 0, sizeof(optical_flow_s));
  std::memset(&ORB_output_pwm_public, 0, sizeof(output_pwm_s));
  std::memset(&ORB_parameter_update_public, 0, sizeof(parameter_update_s));
  std::memset(&ORB_position_setpoint_public, 0, sizeof(position_setpoint_s));
  std::memset(&ORB_position_setpoint_triplet_public, 0, sizeof(position_setpoint_triplet_s));
  std::memset(&ORB_pwm_input_public, 0, sizeof(pwm_input_s));
  std::memset(&ORB_qshell_req_public, 0, sizeof(qshell_req_s));
  std::memset(&ORB_rc_channels_public, 0, sizeof(rc_channels_s));
  std::memset(&ORB_rc_parameter_map_public, 0, sizeof(rc_parameter_map_s));
  std::memset(&ORB_safety_public, 0, sizeof(safety_s));
  std::memset(&ORB_satellite_info_public, 0, sizeof(satellite_info_s));
  std::memset(&ORB_sensor_accel_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_accel_s));
  std::memset(&ORB_sensor_baro_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_baro_s));
  std::memset(&ORB_sensor_combined_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_combined_s));
  std::memset(&ORB_sensor_correction_public, 0, sizeof(sensor_correction_s));
  std::memset(&ORB_sensor_gyro_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_gyro_s));
  std::memset(&ORB_sensor_mag_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_mag_s));
  std::memset(&ORB_sensor_preflight_public, 0, sizeof(sensor_preflight_s));
  std::memset(&ORB_servorail_status_public, 0, sizeof(servorail_status_s));
  std::memset(&ORB_subsystem_info_public, 0, sizeof(subsystem_info_s));
  std::memset(&ORB_system_power_public, 0, sizeof(system_power_s));
  std::memset(&ORB_task_stack_info_public, 0, sizeof(task_stack_info_s));
  std::memset(&ORB_tecs_status_public, 0, sizeof(tecs_status_s));
  std::memset(&ORB_telemetry_status_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(telemetry_status_s));
  std::memset(&ORB_test_motor_public, 0, sizeof(test_motor_s));
  std::memset(&ORB_time_offset_public, 0, sizeof(time_offset_s));
  std::memset(&ORB_transponder_report_public, 0, sizeof(transponder_report_s));
  std::memset(&ORB_uavcan_parameter_request_public, 0, sizeof(uavcan_parameter_request_s));
  std::memset(&ORB_uavcan_parameter_value_public, 0, sizeof(uavcan_parameter_value_s));
  std::memset(&ORB_ulog_stream_public, 0, sizeof(ulog_stream_s));
  std::memset(&ORB_ulog_stream_ack_public, 0, sizeof(ulog_stream_ack_s));
  std::memset(&ORB_vehicle_attitude_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(vehicle_attitude_s));
  std::memset(&ORB_vehicle_attitude_setpoint_public, 0, sizeof(vehicle_attitude_setpoint_s));
  std::memset(&ORB_vehicle_command_ack_public, 0, sizeof(vehicle_command_ack_s));
  std::memset(&ORB_vehicle_command_public, 0, sizeof(vehicle_command_s));
  std::memset(&ORB_vehicle_control_mode_public, 0, sizeof(vehicle_control_mode_s));
  std::memset(&ORB_vehicle_force_setpoint_public, 0, sizeof(vehicle_force_setpoint_s));
  std::memset(&ORB_vehicle_global_position_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(vehicle_global_position_s));
  std::memset(&ORB_vehicle_global_velocity_setpoint_public, 0, sizeof(vehicle_global_velocity_setpoint_s));
  std::memset(&ORB_vehicle_gps_position_public, 0, sizeof(vehicle_gps_position_s));
  std::memset(&ORB_vehicle_land_detected_public, 0, sizeof(vehicle_land_detected_s));
  std::memset(&ORB_vehicle_local_position_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(vehicle_local_position_s));
  std::memset(&ORB_vehicle_local_position_setpoint_public, 0, sizeof(vehicle_local_position_setpoint_s));
  std::memset(&ORB_vehicle_rates_setpoint_public, 0, sizeof(vehicle_rates_setpoint_s));
  std::memset(&ORB_vehicle_roi_public, 0, sizeof(vehicle_roi_s));
  std::memset(&ORB_vehicle_status_public, 0, sizeof(vehicle_status_s));
  std::memset(&ORB_vehicle_status_flags_public, 0, sizeof(vehicle_status_flags_s));
  std::memset(&ORB_vtol_vehicle_status_public, 0, sizeof(vtol_vehicle_status_s));
  std::memset(&ORB_wind_estimate_public, 0, sizeof(wind_estimate_s));
  std::memset(&ORB_actuator_controls_0_public, 0, sizeof(actuator_controls_0_s));
  std::memset(&ORB_actuator_controls_1_public, 0, sizeof(actuator_controls_1_s));
  std::memset(&ORB_actuator_controls_2_public, 0, sizeof(actuator_controls_2_s));
  std::memset(&ORB_actuator_controls_3_public, 0, sizeof(actuator_controls_3_s));
  std::memset(&ORB_actuator_controls_virtual_fw_public, 0, sizeof(actuator_controls_virtual_fw_s));
  std::memset(&ORB_actuator_controls_virtual_mc_public, 0, sizeof(actuator_controls_virtual_mc_s));
  std::memset(&ORB_offboard_mission_public, 0, sizeof(offboard_mission_s));
  std::memset(&ORB_onboard_mission_public, 0, sizeof(onboard_mission_s));
  std::memset(&ORB_vehicle_local_position_groundtruth_public, 0, sizeof(vehicle_local_position_groundtruth_s));
  std::memset(&ORB_vehicle_vision_position_public, 0, sizeof(vehicle_vision_position_s));
  std::memset(&ORB_vehicle_global_position_groundtruth_public, 0, sizeof(vehicle_global_position_groundtruth_s));
  std::memset(&ORB_vehicle_attitude_groundtruth_public, 0, sizeof(vehicle_attitude_groundtruth_s));
  std::memset(&ORB_vehicle_vision_attitude_public, 0, sizeof(vehicle_vision_attitude_s));
}


void orb_set_in_os(void)
{
    orb_in_os = true;
}


bool is_orb_multi(int serial)
{
    for (int i = 0; i<MULTI_ORB_NUM; ++i)
	   {
	       if (serial == (int)orb_multi_list[i])
		       return true;
	   }

	   return false;
}


int get_priority(int instance)
{
		if (instance == 0)
		{
			return ORB_PRIO_MIN;
		}
		else if (instance == 1)
		{
			return ORB_PRIO_VERY_LOW;
		}
		else if (instance == 2)
		{
			return ORB_PRIO_LOW;
		}
		else if (instance == 3)
		{
			return ORB_PRIO_DEFAULT;
		}
		else if (instance == 4)
		{
			return ORB_PRIO_HIGH;
		}
		else if (instance == 5)
		{
			return ORB_PRIO_VERY_HIGH;
		}
		else if (instance == 6)
		{
			return ORB_PRIO_MAX;
		}
		else
			return -1;
}


int get_orb_instance_according_to_priority(int priority)
{
		if (ORB_PRIO_MIN == priority)
		{
			return 0;
		}
		else if (ORB_PRIO_VERY_LOW == priority)
		{
			return 1;
		}
		else if (ORB_PRIO_LOW == priority)
		{
			return 2;
		}
		else if (ORB_PRIO_DEFAULT == priority)
		{
			return 3;
		}
		else if (ORB_PRIO_HIGH == priority)
		{
			return 4;
		}
		else if (ORB_PRIO_VERY_HIGH == priority)
		{
			return 5;
		}
		else if (ORB_PRIO_MAX == priority)
		{
			return 6;
		}
		else
			return -1;
}
void get_orb_name(ORB_serial serial, char * name)
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


orb_id_t  get_orb_according_to_serial(int serial)
{
  char orb_name[50];
  get_orb_name((ORB_serial)serial,orb_name);


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


void  *get_orb_public_according_to_serial_and_instance(int serial, int instance)
{
  char orb_name[50];
  get_orb_name((ORB_serial)serial,orb_name);

  bool is_multi = is_orb_multi(serial);

  if(!is_orb_multi(serial) && instance >0)
      return nullptr;

  if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_armed))->o_name))
  {
      return (void*)&(ORB_actuator_armed_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_controls))->o_name))
  {
      return (void*)&(ORB_actuator_controls_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_direct))->o_name))
  {
      return (void*)&(ORB_actuator_direct_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_outputs))->o_name))
  {
      return (void*)&(ORB_actuator_outputs_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(adc_report))->o_name))
  {
      return (void*)&(ORB_adc_report_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(airspeed))->o_name))
  {
      return (void*)&(ORB_airspeed_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(att_pos_mocap))->o_name))
  {
      return (void*)&(ORB_att_pos_mocap_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(battery_status))->o_name))
  {
      return (void*)&(ORB_battery_status_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(camera_trigger))->o_name))
  {
      return (void*)&(ORB_camera_trigger_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(commander_state))->o_name))
  {
      return (void*)&(ORB_commander_state_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(collision_report))->o_name))
  {
      return (void*)&(ORB_collision_report_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(control_state))->o_name))
  {
        return (void*)&(ORB_control_state_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(cpuload))->o_name))
  {
      return (void*)&(ORB_cpuload_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(debug_key_value))->o_name))
  {
      return (void*)&(ORB_debug_key_value_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(differential_pressure))->o_name))
  {
      return (void*)&(ORB_differential_pressure_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(distance_sensor))->o_name))
  {
        return (void*)&(ORB_distance_sensor_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(ekf2_innovations))->o_name))
  {
      return (void*)&(ORB_ekf2_innovations_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(ekf2_replay))->o_name))
  {
      return (void*)&(ORB_ekf2_replay_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(ekf2_timestamps))->o_name))
  {
      return (void*)&(ORB_ekf2_timestamps_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(esc_report))->o_name))
  {
      return (void*)&(ORB_esc_report_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(esc_status))->o_name))
  {
      return (void*)&(ORB_esc_status_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(estimator_status))->o_name))
  {
        return (void*)&(ORB_estimator_status_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(fence))->o_name))
  {
      return (void*)&(ORB_fence_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(fence_vertex))->o_name))
  {
      return (void*)&(ORB_fence_vertex_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(filtered_bottom_flow))->o_name))
  {
      return (void*)&(ORB_filtered_bottom_flow_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(follow_target))->o_name))
  {
      return (void*)&(ORB_follow_target_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(fw_pos_ctrl_status))->o_name))
  {
      return (void*)&(ORB_fw_pos_ctrl_status_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(fw_virtual_attitude_setpoint))->o_name))
  {
      return (void*)&(ORB_fw_virtual_attitude_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(fw_virtual_rates_setpoint))->o_name))
  {
      return (void*)&(ORB_fw_virtual_rates_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(geofence_result))->o_name))
  {
      return (void*)&(ORB_geofence_result_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(gps_dump))->o_name))
  {
      return (void*)&(ORB_gps_dump_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(gps_inject_data))->o_name))
  {
      return (void*)&(ORB_gps_inject_data_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(hil_sensor))->o_name))
  {
      return (void*)&(ORB_hil_sensor_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(home_position))->o_name))
  {
      return (void*)&(ORB_home_position_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(input_rc))->o_name))
  {
      return (void*)&(ORB_input_rc_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(led_control))->o_name))
  {
      return (void*)&(ORB_led_control_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(log_message))->o_name))
  {
      return (void*)&(ORB_log_message_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(manual_control_setpoint))->o_name))
  {
      return (void*)&(ORB_manual_control_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mavlink_log))->o_name))
  {
      return (void*)&(ORB_mavlink_log_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mc_att_ctrl_status))->o_name))
  {
      return (void*)&(ORB_mc_att_ctrl_status_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mc_virtual_attitude_setpoint))->o_name))
  {
      return (void*)&(ORB_mc_virtual_attitude_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mc_virtual_rates_setpoint))->o_name))
  {
      return (void*)&(ORB_mc_virtual_rates_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mission))->o_name))
  {
      return (void*)&(ORB_mission_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mission_result))->o_name))
  {
      return (void*)&(ORB_mission_result_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mount_orientation))->o_name))
  {
      return (void*)&(ORB_mount_orientation_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(multirotor_motor_limits))->o_name))
  {
      return (void*)&(ORB_multirotor_motor_limits_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(offboard_control_mode))->o_name))
  {
      return (void*)&(ORB_offboard_control_mode_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(optical_flow))->o_name))
  {
      return (void*)&(ORB_optical_flow_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(output_pwm))->o_name))
  {
      return (void*)&(ORB_output_pwm_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(parameter_update))->o_name))
  {
      return (void*)&(ORB_parameter_update_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(position_setpoint))->o_name))
  {
      return (void*)&(ORB_position_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(position_setpoint_triplet))->o_name))
  {
      return (void*)&(ORB_position_setpoint_triplet_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(pwm_input))->o_name))
  {
      return (void*)&(ORB_pwm_input_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(qshell_req))->o_name))
  {
      return (void*)&(ORB_qshell_req_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(rc_channels))->o_name))
  {
      return (void*)&(ORB_rc_channels_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(rc_parameter_map))->o_name))
  {
      return (void*)&(ORB_rc_parameter_map_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(safety))->o_name))
  {
      return (void*)&(ORB_safety_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(satellite_info))->o_name))
  {
      return (void*)&(ORB_satellite_info_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(sensor_accel))->o_name))
  {
        return (void*)&(ORB_sensor_accel_public[instance]);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(sensor_baro))->o_name))
  {
        return (void*)&(ORB_sensor_baro_public[instance]);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(sensor_combined))->o_name))
  {
        return (void*)&(ORB_sensor_combined_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(sensor_correction))->o_name))
  {
      return (void*)&(ORB_sensor_correction_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(sensor_gyro))->o_name))
  {
        return (void*)&(ORB_sensor_gyro_public[instance]);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(sensor_mag))->o_name))
  {
        return (void*)&(ORB_sensor_mag_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(sensor_preflight))->o_name))
  {
      return (void*)&(ORB_sensor_preflight_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(servorail_status))->o_name))
  {
      return (void*)&(ORB_servorail_status_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(subsystem_info))->o_name))
  {
      return (void*)&(ORB_subsystem_info_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(system_power))->o_name))
  {
      return (void*)&(ORB_system_power_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(task_stack_info))->o_name))
  {
      return (void*)&(ORB_task_stack_info_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(tecs_status))->o_name))
  {
      return (void*)&(ORB_tecs_status_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(telemetry_status))->o_name))
  {
        return (void*)&(ORB_telemetry_status_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(test_motor))->o_name))
  {
      return (void*)&(ORB_test_motor_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(time_offset))->o_name))
  {
      return (void*)&(ORB_time_offset_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(transponder_report))->o_name))
  {
      return (void*)&(ORB_transponder_report_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(uavcan_parameter_request))->o_name))
  {
      return (void*)&(ORB_uavcan_parameter_request_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(uavcan_parameter_value))->o_name))
  {
      return (void*)&(ORB_uavcan_parameter_value_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(ulog_stream))->o_name))
  {
      return (void*)&(ORB_ulog_stream_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(ulog_stream_ack))->o_name))
  {
      return (void*)&(ORB_ulog_stream_ack_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude))->o_name))
  {
        return (void*)&(ORB_vehicle_attitude_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude_setpoint))->o_name))
  {
      return (void*)&(ORB_vehicle_attitude_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_command_ack))->o_name))
  {
      return (void*)&(ORB_vehicle_command_ack_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_command))->o_name))
  {
      return (void*)&(ORB_vehicle_command_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_control_mode))->o_name))
  {
      return (void*)&(ORB_vehicle_control_mode_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_force_setpoint))->o_name))
  {
      return (void*)&(ORB_vehicle_force_setpoint_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_global_position))->o_name))
  {
        return (void*)&(ORB_vehicle_global_position_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_global_velocity_setpoint))->o_name))
  {
      return (void*)&(ORB_vehicle_global_velocity_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_gps_position))->o_name))
  {
      return (void*)&(ORB_vehicle_gps_position_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_land_detected))->o_name))
  {
      return (void*)&(ORB_vehicle_land_detected_public);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_local_position))->o_name))
  {
        return (void*)&(ORB_vehicle_local_position_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_local_position_setpoint))->o_name))
  {
      return (void*)&(ORB_vehicle_local_position_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_rates_setpoint))->o_name))
  {
      return (void*)&(ORB_vehicle_rates_setpoint_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_roi))->o_name))
  {
      return (void*)&(ORB_vehicle_roi_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_status))->o_name))
  {
      return (void*)&(ORB_vehicle_status_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_status_flags))->o_name))
  {
      return (void*)&(ORB_vehicle_status_flags_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vtol_vehicle_status))->o_name))
  {
      return (void*)&(ORB_vtol_vehicle_status_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(wind_estimate))->o_name))
  {
      return (void*)&(ORB_wind_estimate_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_0))->o_name))
  {
      return (void*)&(ORB_actuator_controls_0_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_1))->o_name))
  {
      return (void*)&(ORB_actuator_controls_1_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_2))->o_name))
  {
      return (void*)&(ORB_actuator_controls_2_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_3))->o_name))
  {
      return (void*)&(ORB_actuator_controls_3_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_virtual_fw))->o_name))
  {
      return (void*)&(ORB_actuator_controls_virtual_fw_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(actuator_controls_virtual_mc))->o_name))
  {
      return (void*)&(ORB_actuator_controls_virtual_mc_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(offboard_mission))->o_name))
  {
      return (void*)&(ORB_offboard_mission_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(onboard_mission))->o_name))
  {
      return (void*)&(ORB_onboard_mission_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_local_position_groundtruth))->o_name))
  {
      return (void*)&(ORB_vehicle_local_position_groundtruth_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_vision_position))->o_name))
  {
      return (void*)&(ORB_vehicle_vision_position_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_global_position_groundtruth))->o_name))
  {
      return (void*)&(ORB_vehicle_global_position_groundtruth_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude_groundtruth))->o_name))
  {
      return (void*)&(ORB_vehicle_attitude_groundtruth_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_vision_attitude))->o_name))
  {
      return (void*)&(ORB_vehicle_vision_attitude_public);
  }
else
    return nullptr;
}
