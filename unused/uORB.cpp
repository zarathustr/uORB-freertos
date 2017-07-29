#include <uORB/uORB.h>
#include <cstring>
#include "cmsis_os.h"


#pragma default_variable_attributes= @ ".ccmram"
namespace uORB{
  parameter_update_s			                ORB_parameter_update_public;
  vehicle_attitude_control_s			        ORB_vehicle_attitude_control_public;
  actuator_armed_s			                ORB_actuator_armed_public;
  sensor_combined_s			                ORB_sensor_combined_public[2];
  vehicle_attitude_s			                ORB_vehicle_attitude_public[8];
  optical_flow_s			                ORB_optical_flow_public;
  vehicle_gps_position_s			        ORB_vehicle_gps_position_public;
  vision_position_estimate_s			        ORB_vision_position_estimate_public;
  att_pos_mocap_s			                ORB_att_pos_mocap_public;
  distance_sensor_s			                ORB_distance_sensor_public[5];
  vehicle_rates_setpoint_s			        ORB_vehicle_rates_setpoint_public;
  sensor_gyro_s			                        ORB_sensor_gyro_public[5];
  sensor_accel_s			                ORB_sensor_accel_public[5];
  sensor_mag_s			                        ORB_sensor_mag_public[5];
  sensor_baro_s			                        ORB_sensor_baro_public[2];
  airspeed_s			                        ORB_airspeed_public;
  vehicle_land_detected_s			        ORB_vehicle_land_detected_public;
  vehicle_status_s			                ORB_vehicle_status_public;
  vehicle_control_mode_s			        ORB_vehicle_control_mode_public;
  vehicle_attitude_setpoint_s			        ORB_vehicle_attitude_setpoint_public;
  actuator_controls_s			                ORB_actuator_controls_public;
  vehicle_global_position_s			        ORB_vehicle_global_position_public;
  vehicle_local_position_s			        ORB_vehicle_local_position_public;
  control_state_s			                ORB_control_state_public;
  manual_control_setpoint_s			        ORB_manual_control_setpoint_public;
  multirotor_motor_limits_s			        ORB_multirotor_motor_limits_public;
  mc_att_ctrl_status_s			                ORB_mc_att_ctrl_status_public;
  estimator_status_s			                ORB_estimator_status_public;
  wind_estimate_s			                ORB_wind_estimate_public;
  ekf2_innovations_s			                ORB_ekf2_innovations_public;
  position_setpoint_triplet_s			        ORB_position_setpoint_triplet_public;
  vehicle_local_position_setpoint_s			ORB_vehicle_local_position_setpoint_public;
  vehicle_global_velocity_setpoint_s			ORB_vehicle_global_velocity_setpoint_public;
  follow_target_s			                ORB_follow_target_public;
  home_position_s			                ORB_home_position_public;
  mission_s			                        ORB_mission_public;
  fw_pos_ctrl_status_s			                ORB_fw_pos_ctrl_status_public;
  fence_vertex_s			                ORB_fence_vertex_public;
  fence_s			                        ORB_fence_public;
  mission_result_s			                ORB_mission_result_public;
  geofence_result_s			                ORB_geofence_result_public;
  vehicle_command_s			                ORB_vehicle_command_public;
  vtol_vehicle_status_s			                ORB_vtol_vehicle_status_public;
  battery_status_s			                ORB_battery_status_public;
  safety_s			                        ORB_safety_public;
  offboard_control_mode_s			        ORB_offboard_control_mode_public;
  commander_state_s			                ORB_commander_state_public;
  cpuload_s			                        ORB_cpuload_public;
  vehicle_command_ack_s			                ORB_vehicle_command_ack_public;
  differential_pressure_s			        ORB_differential_pressure_public;
  telemetry_status_s			                ORB_telemetry_status_public;
  subsystem_info_s			                ORB_subsystem_info_public;
  system_power_s			                ORB_system_power_public;
  mavlink_log_s			                        ORB_mavlink_log_public;
  debug_key_value_s			                ORB_debug_key_value_public;
  actuator_outputs_s			                ORB_actuator_outputs_public;
  actuator_direct_s			                ORB_actuator_direct_public;
  tecs_status_s			                        ORB_tecs_status_public;
  rc_channels_s			                        ORB_rc_channels_public;
  filtered_bottom_flow_s			        ORB_filtered_bottom_flow_public;
  satellite_info_s			                ORB_satellite_info_public;
  hil_sensor_s			                        ORB_hil_sensor_public;
  log_message_s			                        ORB_log_message_public;
  ekf2_timestamps_s			                ORB_ekf2_timestamps_public;
  led_control_s                                         ORB_led_control_public;
  vehicle_roi_s                                         ORB_vehicle_roi_public;
  sensor_correction_s                                   ORB_sensor_correction_public;
  mc_virtual_attitude_setpoint_s			ORB_mc_virtual_attitude_setpoint_public;
  mc_virtual_rates_setpoint_s			        ORB_mc_virtual_rates_setpoint_public;
  fw_virtual_attitude_setpoint_s			ORB_fw_virtual_attitude_setpoint_public;
  fw_virtual_rates_setpoint_s			        ORB_fw_virtual_rates_setpoint_public;
  vehicle_status_flags_s                                ORB_vehicle_status_flags_public;
  sensor_preflight_s                                    ORB_sensor_preflight_public;
  rc_parameter_map_s			                ORB_rc_parameter_map_public;
  transponder_report_s			                ORB_transponder_report_public;
  collision_report_s			                ORB_collision_report_public;
  camera_trigger_s			                ORB_camera_trigger_public;
  mount_orientation_s			                ORB_mount_orientation_public;
  input_rc_s                                            ORB_input_rc_public;
  vehicle_force_setpoint_s			        ORB_vehicle_force_setpoint_public;
  time_offset_s			                        ORB_time_offset_public;
  gps_inject_data_s			                ORB_gps_inject_data_public;

    orb_advert parameter_update_adv;
    orb_advert vehicle_attitude_control_adv;
    orb_advert actuator_armed_adv;
    orb_advert sensor_combined_adv;
    orb_advert vehicle_attitude_adv;
    orb_advert optical_flow_adv;
    orb_advert vehicle_gps_position_adv;
    orb_advert vision_position_estimate_adv;
    orb_advert att_pos_mocap_adv;
    orb_advert distance_sensor_adv;
    orb_advert vehicle_rates_setpoint_adv;
    orb_advert sensor_gyro_adv;
    orb_advert sensor_accel_adv;
    orb_advert sensor_mag_adv;
    orb_advert sensor_baro_adv;
    orb_advert airspeed_adv;
    orb_advert vehicle_land_detected_adv;
    orb_advert vehicle_status_adv;
    orb_advert vehicle_control_mode_adv;
    orb_advert vehicle_attitude_setpoint_adv;
    orb_advert actuator_controls_adv;
    orb_advert vehicle_global_position_adv;
    orb_advert vehicle_local_position_adv;
    orb_advert control_state_adv;
    orb_advert manual_control_setpoint_adv;
    orb_advert multirotor_motor_limits_adv;
    orb_advert mc_att_ctrl_status_adv;
    orb_advert estimator_status_adv;
    orb_advert wind_estimate_adv;
    orb_advert ekf2_innovations_adv;
    orb_advert position_setpoint_triplet_adv;
    orb_advert vehicle_local_position_setpoint_adv;
    orb_advert vehicle_global_velocity_setpoint_adv;
    orb_advert follow_target_adv;
    orb_advert home_position_adv;
    orb_advert mission_adv;
    orb_advert fw_pos_ctrl_status_adv;
    orb_advert fence_vertex_adv;
    orb_advert fence_adv;
    orb_advert mission_result_adv;
    orb_advert geofence_result_adv;
    orb_advert vehicle_command_adv;
    orb_advert vtol_vehicle_status_adv;
    orb_advert battery_status_adv;
    orb_advert safety_adv;
    orb_advert offboard_control_mode_adv;
    orb_advert commander_state_adv;
    orb_advert cpuload_adv;
    orb_advert vehicle_command_ack_adv;
    orb_advert differential_pressure_adv;
    orb_advert telemetry_status_adv;
    orb_advert subsystem_info_adv;
    orb_advert system_power_adv;
    orb_advert mavlink_log_adv;
    orb_advert debug_key_value_adv;
    orb_advert actuator_outputs_adv;
    orb_advert actuator_direct_adv;
    orb_advert tecs_status_adv;
    orb_advert rc_channels_adv;
    orb_advert filtered_bottom_flow_adv;
    orb_advert satellite_info_adv;
    orb_advert hil_sensor_adv;
    orb_advert log_message_adv;
    orb_advert ekf2_timestamps_adv;
    orb_advert led_control_adv;
    orb_advert vehicle_roi_adv;
    orb_advert sensor_correction_adv;
    orb_advert mc_virtual_attitude_setpoint_adv;
    orb_advert mc_virtual_rates_setpoint_adv;
    orb_advert fw_virtual_attitude_setpoint_adv;
    orb_advert fw_virtual_rates_setpoint_adv;
    orb_advert vehicle_status_flags_adv;
    orb_advert sensor_preflight_adv;
    orb_advert rc_parameter_map_adv;
    orb_advert transponder_report_adv;
    orb_advert collision_report_adv;
    orb_advert camera_trigger_adv;
    orb_advert mount_orientation_adv;
    orb_advert input_rc_adv;
    orb_advert vehicle_force_setpoint_adv;
    orb_advert time_offset_adv;
    orb_advert gps_inject_data_adv;
}
#pragma default_variable_attributes = 



void MEMCPY(void *dest, const void *source, size_t len)
{
  taskENTER_CRITICAL();
  
  std::memcpy(dest, source, len);
  
  taskEXIT_CRITICAL();
}


void orb_init(void)
{

  std::memset(&uORB::ORB_parameter_update_public, 0, sizeof(parameter_update_s));
  std::memset(&uORB::ORB_vehicle_attitude_control_public, 0, sizeof(vehicle_attitude_control_s));
  std::memset(&uORB::ORB_actuator_armed_public, 0, sizeof(actuator_armed_s));
  std::memset(&uORB::ORB_sensor_combined_public[0], 0, sizeof(sensor_combined_s));
  std::memset(&uORB::ORB_sensor_combined_public[1], 0, sizeof(sensor_combined_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[0], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[1], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[2], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[3], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[4], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[5], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[6], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_vehicle_attitude_public[7], 0, sizeof(vehicle_attitude_s));
  std::memset(&uORB::ORB_optical_flow_public, 0, sizeof(optical_flow_s));
  std::memset(&uORB::ORB_vehicle_gps_position_public, 0, sizeof(vehicle_gps_position_s));
  std::memset(&uORB::ORB_vision_position_estimate_public, 0, sizeof(vision_position_estimate_s));
  std::memset(&uORB::ORB_att_pos_mocap_public, 0, sizeof(att_pos_mocap_s));
  std::memset(&uORB::ORB_distance_sensor_public[0], 0, sizeof(distance_sensor_s));
  std::memset(&uORB::ORB_distance_sensor_public[1], 0, sizeof(distance_sensor_s));
  std::memset(&uORB::ORB_distance_sensor_public[2], 0, sizeof(distance_sensor_s));
  std::memset(&uORB::ORB_distance_sensor_public[3], 0, sizeof(distance_sensor_s));
  std::memset(&uORB::ORB_distance_sensor_public[4], 0, sizeof(distance_sensor_s));
  std::memset(&uORB::ORB_vehicle_rates_setpoint_public, 0, sizeof(vehicle_rates_setpoint_s));
  std::memset(&uORB::ORB_sensor_gyro_public[0], 0, sizeof(sensor_gyro_s));
  std::memset(&uORB::ORB_sensor_gyro_public[1], 0, sizeof(sensor_gyro_s));
  std::memset(&uORB::ORB_sensor_gyro_public[2], 0, sizeof(sensor_gyro_s));
  std::memset(&uORB::ORB_sensor_gyro_public[3], 0, sizeof(sensor_gyro_s));
  std::memset(&uORB::ORB_sensor_gyro_public[4], 0, sizeof(sensor_gyro_s));
  std::memset(&uORB::ORB_sensor_accel_public[0], 0, sizeof(sensor_accel_s));
  std::memset(&uORB::ORB_sensor_accel_public[1], 0, sizeof(sensor_accel_s));
  std::memset(&uORB::ORB_sensor_accel_public[2], 0, sizeof(sensor_accel_s));
  std::memset(&uORB::ORB_sensor_accel_public[3], 0, sizeof(sensor_accel_s));
  std::memset(&uORB::ORB_sensor_accel_public[4], 0, sizeof(sensor_accel_s));
  std::memset(&uORB::ORB_sensor_mag_public[0], 0, sizeof(sensor_mag_s));
  std::memset(&uORB::ORB_sensor_mag_public[1], 0, sizeof(sensor_mag_s));
  std::memset(&uORB::ORB_sensor_mag_public[2], 0, sizeof(sensor_mag_s));
  std::memset(&uORB::ORB_sensor_mag_public[3], 0, sizeof(sensor_mag_s));
  std::memset(&uORB::ORB_sensor_mag_public[4], 0, sizeof(sensor_mag_s));
  std::memset(&uORB::ORB_sensor_baro_public[0], 0, sizeof(sensor_baro_s));
  std::memset(&uORB::ORB_sensor_baro_public[1], 0, sizeof(sensor_baro_s));
  std::memset(&uORB::ORB_airspeed_public, 0, sizeof(airspeed_s));
  std::memset(&uORB::ORB_vehicle_land_detected_public, 0, sizeof(vehicle_land_detected_s));
  std::memset(&uORB::ORB_vehicle_status_public, 0, sizeof(vehicle_status_s));
  std::memset(&uORB::ORB_vehicle_control_mode_public, 0, sizeof(vehicle_control_mode_s));
  std::memset(&uORB::ORB_vehicle_attitude_setpoint_public, 0, sizeof(vehicle_attitude_setpoint_s));
  std::memset(&uORB::ORB_actuator_controls_public, 0, sizeof(actuator_controls_s));
  std::memset(&uORB::ORB_vehicle_global_position_public, 0, sizeof(vehicle_global_position_s));
  std::memset(&uORB::ORB_vehicle_local_position_public, 0, sizeof(vehicle_local_position_s));
  std::memset(&uORB::ORB_control_state_public, 0, sizeof(control_state_s));
  std::memset(&uORB::ORB_manual_control_setpoint_public, 0, sizeof(manual_control_setpoint_s));
  std::memset(&uORB::ORB_multirotor_motor_limits_public, 0, sizeof(multirotor_motor_limits_s));
  std::memset(&uORB::ORB_mc_att_ctrl_status_public, 0, sizeof(mc_att_ctrl_status_s));
  std::memset(&uORB::ORB_estimator_status_public, 0, sizeof(estimator_status_s));
  std::memset(&uORB::ORB_wind_estimate_public, 0, sizeof(wind_estimate_s));
  std::memset(&uORB::ORB_ekf2_innovations_public, 0, sizeof(ekf2_innovations_s));
  std::memset(&uORB::ORB_position_setpoint_triplet_public, 0, sizeof(position_setpoint_triplet_s));
  std::memset(&uORB::ORB_vehicle_local_position_setpoint_public, 0, sizeof(vehicle_local_position_setpoint_s));
  std::memset(&uORB::ORB_vehicle_global_velocity_setpoint_public, 0, sizeof(vehicle_global_velocity_setpoint_s));
  std::memset(&uORB::ORB_follow_target_public, 0, sizeof(follow_target_s));
  std::memset(&uORB::ORB_home_position_public, 0, sizeof(home_position_s));
  std::memset(&uORB::ORB_mission_public, 0, sizeof(mission_s));
  std::memset(&uORB::ORB_fw_pos_ctrl_status_public, 0, sizeof(fw_pos_ctrl_status_s));
  std::memset(&uORB::ORB_fence_vertex_public, 0, sizeof(fence_vertex_s));
  std::memset(&uORB::ORB_fence_public, 0, sizeof(fence_s));
  std::memset(&uORB::ORB_mission_result_public, 0, sizeof(mission_result_s));
  std::memset(&uORB::ORB_geofence_result_public, 0, sizeof(geofence_result_s));
  std::memset(&uORB::ORB_vehicle_command_public, 0, sizeof(vehicle_command_s));
  std::memset(&uORB::ORB_vtol_vehicle_status_public, 0, sizeof(vtol_vehicle_status_s));
  std::memset(&uORB::ORB_battery_status_public, 0, sizeof(battery_status_s));
  std::memset(&uORB::ORB_safety_public, 0, sizeof(safety_s));
  std::memset(&uORB::ORB_offboard_control_mode_public, 0, sizeof(offboard_control_mode_s));
  std::memset(&uORB::ORB_commander_state_public, 0, sizeof(commander_state_s));
  std::memset(&uORB::ORB_cpuload_public, 0, sizeof(cpuload_s));
  std::memset(&uORB::ORB_vehicle_command_ack_public, 0, sizeof(vehicle_command_ack_s));
  std::memset(&uORB::ORB_differential_pressure_public, 0, sizeof(differential_pressure_s));
  std::memset(&uORB::ORB_telemetry_status_public, 0, sizeof(telemetry_status_s));
  std::memset(&uORB::ORB_subsystem_info_public, 0, sizeof(subsystem_info_s));
  std::memset(&uORB::ORB_system_power_public, 0, sizeof(system_power_s));
  std::memset(&uORB::ORB_mavlink_log_public, 0, sizeof(mavlink_log_s));
  std::memset(&uORB::ORB_debug_key_value_public, 0, sizeof(debug_key_value_s));
  std::memset(&uORB::ORB_actuator_outputs_public, 0, sizeof(actuator_outputs_s));
  std::memset(&uORB::ORB_actuator_direct_public, 0, sizeof(actuator_direct_s));
  std::memset(&uORB::ORB_tecs_status_public, 0, sizeof(tecs_status_s));
  std::memset(&uORB::ORB_rc_channels_public, 0, sizeof(rc_channels_s));
  std::memset(&uORB::ORB_filtered_bottom_flow_public, 0, sizeof(filtered_bottom_flow_s));
  std::memset(&uORB::ORB_satellite_info_public, 0, sizeof(satellite_info_s));
  std::memset(&uORB::ORB_hil_sensor_public, 0, sizeof(hil_sensor_s));
  std::memset(&uORB::ORB_log_message_public, 0, sizeof(log_message_s));
  std::memset(&uORB::ORB_ekf2_timestamps_public, 0, sizeof(ekf2_timestamps_s));
  std::memset(&uORB::ORB_led_control_public,0,sizeof(led_control_s));
  std::memset(&uORB::ORB_vehicle_roi_public,0,sizeof(vehicle_roi_s));
  std::memset(&uORB::ORB_sensor_correction_public,0,sizeof(sensor_correction_s));
  std::memset(&uORB::ORB_mc_virtual_attitude_setpoint_public, 0, sizeof(mc_virtual_attitude_setpoint_s));
  std::memset(&uORB::ORB_mc_virtual_rates_setpoint_public, 0, sizeof(mc_virtual_rates_setpoint_s));
  std::memset(&uORB::ORB_fw_virtual_attitude_setpoint_public, 0, sizeof(fw_virtual_attitude_setpoint_s));
  std::memset(&uORB::ORB_fw_virtual_rates_setpoint_public, 0, sizeof(fw_virtual_rates_setpoint_s));
  std::memset(&uORB::ORB_vehicle_status_flags_public, 0, sizeof(vehicle_status_flags_s));
  std::memset(&uORB::ORB_sensor_preflight_public, 0, sizeof(sensor_preflight_s));
  std::memset(&uORB::ORB_rc_parameter_map_public, 0, sizeof(rc_parameter_map_s));
  std::memset(&uORB::ORB_transponder_report_public, 0, sizeof(transponder_report_s));
  std::memset(&uORB::ORB_collision_report_public, 0, sizeof(collision_report_s));
  std::memset(&uORB::ORB_camera_trigger_public, 0, sizeof(camera_trigger_s));
  std::memset(&uORB::ORB_mount_orientation_public, 0, sizeof(mount_orientation_s));
  std::memset(&uORB::ORB_input_rc_public, 0, sizeof(input_rc_s));
  std::memset(&uORB::ORB_vehicle_force_setpoint_public, 0, sizeof(vehicle_force_setpoint_s));
  std::memset(&uORB::ORB_time_offset_public, 0, sizeof(time_offset_s));
  std::memset(&uORB::ORB_gps_inject_data_public, 0, sizeof(gps_inject_data_s));
  
  uORB::ORB_sensor_combined_public[0].current_prio = ORB_PRIO_DEFAULT;
  uORB::ORB_sensor_combined_public[1].current_prio = ORB_PRIO_MAX;
  
  uORB::ORB_vehicle_attitude_public[0].current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_attitude_public[1].current_prio = ORB_PRIO_VERY_LOW;
  uORB::ORB_vehicle_attitude_public[2].current_prio = ORB_PRIO_LOW;
  uORB::ORB_vehicle_attitude_public[3].current_prio = ORB_PRIO_DEFAULT;
  uORB::ORB_vehicle_attitude_public[4].current_prio = ORB_PRIO_ABOVE_DEFAULT;
  uORB::ORB_vehicle_attitude_public[5].current_prio = ORB_PRIO_HIGH;
  uORB::ORB_vehicle_attitude_public[6].current_prio = ORB_PRIO_VERY_HIGH;
  uORB::ORB_vehicle_attitude_public[7].current_prio = ORB_PRIO_MAX;  
  
  uORB::ORB_sensor_gyro_public[0].current_prio = ORB_PRIO_MIN;
  uORB::ORB_sensor_gyro_public[1].current_prio = ORB_PRIO_LOW;
  uORB::ORB_sensor_gyro_public[2].current_prio = ORB_PRIO_DEFAULT;
  uORB::ORB_sensor_gyro_public[3].current_prio = ORB_PRIO_HIGH;
  uORB::ORB_sensor_gyro_public[4].current_prio = ORB_PRIO_MAX;
  
  uORB::ORB_sensor_accel_public[0].current_prio = ORB_PRIO_MIN;
  uORB::ORB_sensor_accel_public[1].current_prio = ORB_PRIO_LOW;
  uORB::ORB_sensor_accel_public[2].current_prio = ORB_PRIO_DEFAULT;
  uORB::ORB_sensor_accel_public[3].current_prio = ORB_PRIO_HIGH;
  uORB::ORB_sensor_accel_public[4].current_prio = ORB_PRIO_MAX;
  
  uORB::ORB_distance_sensor_public[0].current_prio = ORB_PRIO_MIN;
  uORB::ORB_distance_sensor_public[1].current_prio = ORB_PRIO_LOW;
  uORB::ORB_distance_sensor_public[2].current_prio = ORB_PRIO_DEFAULT;
  uORB::ORB_distance_sensor_public[3].current_prio = ORB_PRIO_HIGH;
  uORB::ORB_distance_sensor_public[4].current_prio = ORB_PRIO_MAX;
  
  uORB::ORB_sensor_mag_public[0].current_prio = ORB_PRIO_MIN;
  uORB::ORB_sensor_mag_public[1].current_prio = ORB_PRIO_LOW;
  uORB::ORB_sensor_mag_public[2].current_prio = ORB_PRIO_DEFAULT;
  uORB::ORB_sensor_mag_public[3].current_prio = ORB_PRIO_HIGH;
  uORB::ORB_sensor_mag_public[4].current_prio = ORB_PRIO_MAX;
  
  uORB::ORB_sensor_baro_public[0].current_prio = ORB_PRIO_DEFAULT;
  uORB::ORB_sensor_baro_public[1].current_prio = ORB_PRIO_MAX;
  


  uORB::ORB_parameter_update_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_attitude_control_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_actuator_armed_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_optical_flow_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_gps_position_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vision_position_estimate_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_att_pos_mocap_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_rates_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_airspeed_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_land_detected_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_control_mode_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_attitude_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_actuator_controls_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_global_position_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_local_position_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_control_state_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_manual_control_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_multirotor_motor_limits_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_mc_att_ctrl_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_estimator_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_wind_estimate_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_ekf2_innovations_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_position_setpoint_triplet_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_local_position_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_global_velocity_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_follow_target_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_home_position_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_mission_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_fw_pos_ctrl_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_fence_vertex_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_fence_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_mission_result_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_geofence_result_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_command_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vtol_vehicle_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_battery_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_safety_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_offboard_control_mode_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_commander_state_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_cpuload_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_command_ack_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_differential_pressure_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_telemetry_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_subsystem_info_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_system_power_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_mavlink_log_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_debug_key_value_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_actuator_outputs_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_actuator_direct_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_tecs_status_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_rc_channels_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_filtered_bottom_flow_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_satellite_info_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_hil_sensor_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_log_message_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_ekf2_timestamps_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_led_control_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_roi_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_sensor_correction_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_mc_virtual_attitude_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_mc_virtual_rates_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_fw_virtual_attitude_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_fw_virtual_rates_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_status_flags_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_sensor_preflight_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_rc_parameter_map_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_transponder_report_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_collision_report_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_camera_trigger_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_mount_orientation_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_input_rc_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_vehicle_force_setpoint_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_time_offset_public.current_prio = ORB_PRIO_MIN;
  uORB::ORB_gps_inject_data_public.current_prio = ORB_PRIO_MIN;

  std::strcpy(uORB::parameter_update_adv.orb_name, "parameter_update");
  std::strcpy(uORB::vehicle_attitude_control_adv.orb_name, "vehicle_attitude_control");
  std::strcpy(uORB::actuator_armed_adv.orb_name, "actuator_armed");
  std::strcpy(uORB::sensor_combined_adv.orb_name, "sensor_combined");
  std::strcpy(uORB::vehicle_attitude_adv.orb_name, "vehicle_attitude");
  std::strcpy(uORB::optical_flow_adv.orb_name, "optical_flow");
  std::strcpy(uORB::vehicle_gps_position_adv.orb_name, "vehicle_gps_position");
  std::strcpy(uORB::vision_position_estimate_adv.orb_name, "vision_position_estimate");
  std::strcpy(uORB::att_pos_mocap_adv.orb_name, "att_pos_mocap");
  std::strcpy(uORB::distance_sensor_adv.orb_name, "distance_sensor");
  std::strcpy(uORB::vehicle_rates_setpoint_adv.orb_name, "vehicle_rates_setpoint");
  std::strcpy(uORB::sensor_gyro_adv.orb_name, "sensor_gyro");
  std::strcpy(uORB::sensor_accel_adv.orb_name, "sensor_accel");
  std::strcpy(uORB::sensor_mag_adv.orb_name, "sensor_mag");
  std::strcpy(uORB::sensor_baro_adv.orb_name, "sensor_baro");
  std::strcpy(uORB::airspeed_adv.orb_name, "airspeed");
  std::strcpy(uORB::vehicle_land_detected_adv.orb_name, "vehicle_land_detected");
  std::strcpy(uORB::vehicle_status_adv.orb_name, "vehicle_status");
  std::strcpy(uORB::vehicle_control_mode_adv.orb_name, "vehicle_control_mode");
  std::strcpy(uORB::vehicle_attitude_setpoint_adv.orb_name, "vehicle_attitude_setpoint");
  std::strcpy(uORB::actuator_controls_adv.orb_name, "actuator_controls");
  std::strcpy(uORB::vehicle_global_position_adv.orb_name, "vehicle_global_position");
  std::strcpy(uORB::vehicle_local_position_adv.orb_name, "vehicle_local_position");
  std::strcpy(uORB::control_state_adv.orb_name, "control_state");
  std::strcpy(uORB::manual_control_setpoint_adv.orb_name, "manual_control_setpoint");
  std::strcpy(uORB::multirotor_motor_limits_adv.orb_name, "multirotor_motor_limits");
  std::strcpy(uORB::mc_att_ctrl_status_adv.orb_name, "mc_att_ctrl_status");
  std::strcpy(uORB::estimator_status_adv.orb_name, "estimator_status");
  std::strcpy(uORB::wind_estimate_adv.orb_name, "wind_estimate");
  std::strcpy(uORB::ekf2_innovations_adv.orb_name, "ekf2_innovations");
  std::strcpy(uORB::position_setpoint_triplet_adv.orb_name, "position_setpoint_triplet");
  std::strcpy(uORB::vehicle_local_position_setpoint_adv.orb_name, "vehicle_local_position_setpoint");
  std::strcpy(uORB::vehicle_global_velocity_setpoint_adv.orb_name, "vehicle_global_velocity_setpoint");
  std::strcpy(uORB::follow_target_adv.orb_name, "follow_target");
  std::strcpy(uORB::home_position_adv.orb_name, "home_position");
  std::strcpy(uORB::mission_adv.orb_name, "mission");
  std::strcpy(uORB::fw_pos_ctrl_status_adv.orb_name, "fw_pos_ctrl_status");
  std::strcpy(uORB::fence_vertex_adv.orb_name, "fence_vertex");
  std::strcpy(uORB::fence_adv.orb_name, "fence");
  std::strcpy(uORB::mission_result_adv.orb_name, "mission_result");
  std::strcpy(uORB::geofence_result_adv.orb_name, "geofence_result");
  std::strcpy(uORB::vehicle_command_adv.orb_name, "vehicle_command");
  std::strcpy(uORB::vtol_vehicle_status_adv.orb_name, "vtol_vehicle_status");
  std::strcpy(uORB::battery_status_adv.orb_name, "battery_status");
  std::strcpy(uORB::safety_adv.orb_name, "safety");
  std::strcpy(uORB::offboard_control_mode_adv.orb_name, "offboard_control_mode");
  std::strcpy(uORB::commander_state_adv.orb_name, "commander_state");
  std::strcpy(uORB::cpuload_adv.orb_name, "cpuload");
  std::strcpy(uORB::vehicle_command_ack_adv.orb_name, "vehicle_command_ack");
  std::strcpy(uORB::differential_pressure_adv.orb_name, "differential_pressure");
  std::strcpy(uORB::telemetry_status_adv.orb_name, "telemetry_status");
  std::strcpy(uORB::subsystem_info_adv.orb_name, "subsystem_info");
  std::strcpy(uORB::system_power_adv.orb_name, "system_power");
  std::strcpy(uORB::mavlink_log_adv.orb_name, "mavlink_log");
  std::strcpy(uORB::debug_key_value_adv.orb_name, "debug_key_value");
  std::strcpy(uORB::actuator_outputs_adv.orb_name, "actuator_outputs");
  std::strcpy(uORB::actuator_direct_adv.orb_name, "actuator_direct");
  std::strcpy(uORB::tecs_status_adv.orb_name, "tecs_status");
  std::strcpy(uORB::rc_channels_adv.orb_name, "rc_channels");
  std::strcpy(uORB::filtered_bottom_flow_adv.orb_name, "filtered_bottom_flow");
  std::strcpy(uORB::satellite_info_adv.orb_name, "satellite_info");
  std::strcpy(uORB::hil_sensor_adv.orb_name, "hil_sensor");
  std::strcpy(uORB::log_message_adv.orb_name, "log_message");
  std::strcpy(uORB::ekf2_timestamps_adv.orb_name, "ekf2_timestamps");
  std::strcpy(uORB::led_control_adv.orb_name, "led_control");
  std::strcpy(uORB::vehicle_roi_adv.orb_name, "vehicle_roi");
  std::strcpy(uORB::sensor_correction_adv.orb_name, "sensor_correction");
  std::strcpy(uORB::mc_virtual_attitude_setpoint_adv.orb_name, "mc_virtual_attitude_setpoint");
  std::strcpy(uORB::mc_virtual_rates_setpoint_adv.orb_name, "mc_virtual_rates_setpoint");
  std::strcpy(uORB::fw_virtual_attitude_setpoint_adv.orb_name, "fw_virtual_attitude_setpoint");
  std::strcpy(uORB::fw_virtual_rates_setpoint_adv.orb_name, "fw_virtual_rates_setpoint");
  std::strcpy(uORB::vehicle_status_flags_adv.orb_name, "vehicle_status_flags");
  std::strcpy(uORB::sensor_preflight_adv.orb_name, "sensor_preflight");
  std::strcpy(uORB::rc_parameter_map_adv.orb_name, "rc_parameter_map");
  std::strcpy(uORB::transponder_report_adv.orb_name, "transponder_report");
  std::strcpy(uORB::collision_report_adv.orb_name, "collision_report");
  std::strcpy(uORB::camera_trigger_adv.orb_name, "camera_trigger");
  std::strcpy(uORB::mount_orientation_adv.orb_name, "mount_orientation");
  std::strcpy(uORB::input_rc_adv.orb_name, "input_rc");
  std::strcpy(uORB::vehicle_force_setpoint_adv.orb_name, "vehicle_force_setpoint");
  std::strcpy(uORB::time_offset_adv.orb_name, "time_offset");
  std::strcpy(uORB::gps_inject_data_adv.orb_name, "gps_inject_data");
}


int orb_subscribe(char* orb_name)
{
  if(0 == std::strcmp(orb_name, "parameter_update"))
  {
    return(uORB::ORB_parameter_update);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_attitude_control"))
  {
    return(uORB::ORB_vehicle_attitude_control);
  }
  else if(0 == std::strcmp(orb_name, "actuator_armed"))
  {
    return(uORB::ORB_actuator_armed);
  }
  else if(0 == std::strcmp(orb_name, "sensor_combined"))
  {
    return(uORB::ORB_sensor_combined);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_attitude"))
  {
    return(uORB::ORB_vehicle_attitude);
  }
  else if(0 == std::strcmp(orb_name, "optical_flow"))
  {
    return(uORB::ORB_optical_flow);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_gps_position"))
  {
    return(uORB::ORB_vehicle_gps_position);
  }
  else if(0 == std::strcmp(orb_name, "vision_position_estimate"))
  {
    return(uORB::ORB_vision_position_estimate);
  }
  else if(0 == std::strcmp(orb_name, "att_pos_mocap"))
  {
    return(uORB::ORB_att_pos_mocap);
  }
  else if(0 == std::strcmp(orb_name, "distance_sensor"))
  {
    return(uORB::ORB_distance_sensor);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_rates_setpoint"))
  {
    return(uORB::ORB_vehicle_rates_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "sensor_gyro"))
  {
    return(uORB::ORB_sensor_gyro);
  }
  else if(0 == std::strcmp(orb_name, "sensor_accel"))
  {
    return(uORB::ORB_sensor_accel);
  }
  else if(0 == std::strcmp(orb_name, "sensor_mag"))
  {
    return(uORB::ORB_sensor_mag);
  }
  else if(0 == std::strcmp(orb_name, "sensor_baro"))
  {
    return(uORB::ORB_sensor_baro);
  }
  else if(0 == std::strcmp(orb_name, "airspeed"))
  {
    return(uORB::ORB_airspeed);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_land_detected"))
  {
    return(uORB::ORB_vehicle_land_detected);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_status"))
  {
    return(uORB::ORB_vehicle_status);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_control_mode"))
  {
    return(uORB::ORB_vehicle_control_mode);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_attitude_setpoint"))
  {
    return(uORB::ORB_vehicle_attitude_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "actuator_controls"))
  {
    return(uORB::ORB_actuator_controls);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_global_position"))
  {
    return(uORB::ORB_vehicle_global_position);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_local_position"))
  {
    return(uORB::ORB_vehicle_local_position);
  }
  else if(0 == std::strcmp(orb_name, "control_state"))
  {
    return(uORB::ORB_control_state);
  }
  else if(0 == std::strcmp(orb_name, "manual_control_setpoint"))
  {
    return(uORB::ORB_manual_control_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "multirotor_motor_limits"))
  {
    return(uORB::ORB_multirotor_motor_limits);
  }
  else if(0 == std::strcmp(orb_name, "mc_att_ctrl_status"))
  {
    return(uORB::ORB_mc_att_ctrl_status);
  }
  else if(0 == std::strcmp(orb_name, "estimator_status"))
  {
    return(uORB::ORB_estimator_status);
  }
  else if(0 == std::strcmp(orb_name, "wind_estimate"))
  {
    return(uORB::ORB_wind_estimate);
  }
  else if(0 == std::strcmp(orb_name, "ekf2_innovations"))
  {
    return(uORB::ORB_ekf2_innovations);
  }
  else if(0 == std::strcmp(orb_name, "position_setpoint_triplet"))
  {
    return(uORB::ORB_position_setpoint_triplet);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_local_position_setpoint"))
  {
    return(uORB::ORB_vehicle_local_position_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_global_velocity_setpoint"))
  {
    return(uORB::ORB_vehicle_global_velocity_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "follow_target"))
  {
    return(uORB::ORB_follow_target);
  }
  else if(0 == std::strcmp(orb_name, "home_position"))
  {
    return(uORB::ORB_home_position);
  }
  else if(0 == std::strcmp(orb_name, "mission"))
  {
    return(uORB::ORB_mission);
  }
  else if(0 == std::strcmp(orb_name, "fw_pos_ctrl_status"))
  {
    return(uORB::ORB_fw_pos_ctrl_status);
  }
  else if(0 == std::strcmp(orb_name, "fence_vertex"))
  {
    return(uORB::ORB_fence_vertex);
  }
  else if(0 == std::strcmp(orb_name, "fence"))
  {
    return(uORB::ORB_fence);
  }
  else if(0 == std::strcmp(orb_name, "mission_result"))
  {
    return(uORB::ORB_mission_result);
  }
  else if(0 == std::strcmp(orb_name, "geofence_result"))
  {
    return(uORB::ORB_geofence_result);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_command"))
  {
    return(uORB::ORB_vehicle_command);
  }
  else if(0 == std::strcmp(orb_name, "vtol_vehicle_status"))
  {
    return(uORB::ORB_vtol_vehicle_status);
  }
  else if(0 == std::strcmp(orb_name, "battery_status"))
  {
    return(uORB::ORB_battery_status);
  }
  else if(0 == std::strcmp(orb_name, "safety"))
  {
    return(uORB::ORB_safety);
  }
  else if(0 == std::strcmp(orb_name, "offboard_control_mode"))
  {
    return(uORB::ORB_offboard_control_mode);
  }
  else if(0 == std::strcmp(orb_name, "commander_state"))
  {
    return(uORB::ORB_commander_state);
  }
  else if(0 == std::strcmp(orb_name, "cpuload"))
  {
    return(uORB::ORB_cpuload);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_command_ack"))
  {
    return(uORB::ORB_vehicle_command_ack);
  }
  else if(0 == std::strcmp(orb_name, "differential_pressure"))
  {
    return(uORB::ORB_differential_pressure);
  }
  else if(0 == std::strcmp(orb_name, "telemetry_status"))
  {
    return(uORB::ORB_telemetry_status);
  }
  else if(0 == std::strcmp(orb_name, "subsystem_info"))
  {
    return(uORB::ORB_subsystem_info);
  }
  else if(0 == std::strcmp(orb_name, "system_power"))
  {
    return(uORB::ORB_system_power);
  }
  else if(0 == std::strcmp(orb_name, "mavlink_log"))
  {
    return(uORB::ORB_mavlink_log);
  }
  else if(0 == std::strcmp(orb_name, "debug_key_value"))
  {
    return(uORB::ORB_debug_key_value);
  }
  else if(0 == std::strcmp(orb_name, "actuator_outputs"))
  {
    return(uORB::ORB_actuator_outputs);
  }
  else if(0 == std::strcmp(orb_name, "actuator_direct"))
  {
    return(uORB::ORB_actuator_direct);
  }
  else if(0 == std::strcmp(orb_name, "tecs_status"))
  {
    return(uORB::ORB_tecs_status);
  }
  else if(0 == std::strcmp(orb_name, "rc_channels"))
  {
    return(uORB::ORB_rc_channels);
  }
  else if(0 == std::strcmp(orb_name, "filtered_bottom_flow"))
  {
    return(uORB::ORB_filtered_bottom_flow);
  }
  else if(0 == std::strcmp(orb_name, "satellite_info"))
  {
    return(uORB::ORB_satellite_info);
  }
  else if(0 == std::strcmp(orb_name, "hil_sensor"))
  {
    return(uORB::ORB_hil_sensor);
  }
  else if(0 == std::strcmp(orb_name, "log_message"))
  {
    return(uORB::ORB_log_message);
  }
  else if(0 == std::strcmp(orb_name, "ekf2_timestamps"))
  {
    return(uORB::ORB_ekf2_timestamps);
  }
  else if(0 == std::strcmp(orb_name, "led_control"))
  {
    return(uORB::ORB_led_control);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_roi"))
  {
    return(uORB::ORB_vehicle_roi);
  }
  else if(0 == std::strcmp(orb_name, "sensor_correction"))
  {
    return(uORB::ORB_sensor_correction);
  }
  else if(0 == std::strcmp(orb_name, "mc_virtual_attitude_setpoint"))
  {
    return(uORB::ORB_mc_virtual_attitude_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "mc_virtual_rates_setpoint"))
  {
    return(uORB::ORB_mc_virtual_rates_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "fw_virtual_attitude_setpoint"))
  {
    return(uORB::ORB_fw_virtual_attitude_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "fw_virtual_rates_setpoint"))
  {
    return(uORB::ORB_fw_virtual_rates_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_status_flags"))
  {
    return(uORB::ORB_vehicle_status_flags);
  }
  else if(0 == std::strcmp(orb_name, "sensor_preflight"))
  {
    return(uORB::ORB_sensor_preflight);
  }
  else if(0 == std::strcmp(orb_name, "rc_parameter_map"))
  {
    return(uORB::ORB_rc_parameter_map);
  }
  else if(0 == std::strcmp(orb_name, "transponder_report"))
  {
    return(uORB::ORB_transponder_report);
  }
  else if(0 == std::strcmp(orb_name, "collision_report"))
  {
    return(uORB::ORB_collision_report);
  }
  else if(0 == std::strcmp(orb_name, "camera_trigger"))
  {
    return(uORB::ORB_camera_trigger);
  }
  else if(0 == std::strcmp(orb_name, "mount_orientation"))
  {
    return(uORB::ORB_mount_orientation);
  }
  else if(0 == std::strcmp(orb_name, "input_rc"))
  {
    return(uORB::ORB_input_rc);
  }
  else if(0 == std::strcmp(orb_name, "vehicle_force_setpoint"))
  {
    return(uORB::ORB_vehicle_force_setpoint);
  }
  else if(0 == std::strcmp(orb_name, "time_offset"))
  {
    return(uORB::ORB_time_offset);
  }
  else if(0 == std::strcmp(orb_name, "gps_inject_data"))
  {
    return(uORB::ORB_gps_inject_data);
  }
  return(-1);
}

int orb_unsubscribe(int &handle)
{
    if(handle!=-1){
        handle=-1;
        return 0;
    }
    else
        return -1;
}

int orb_unadvertise(orb_advert_t adv)
{
    if(adv!=nullptr){
        adv=nullptr;
        return 0;
    }
    else
        return -1;
}


orb_advert_t orb_advertise(char* orb_name, void* data)
{
        if (0 == std::strcmp(orb_name, "parameter_update"))
        {
            std::strcpy(uORB::parameter_update_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_parameter_update_public, data, sizeof(parameter_update_s));
            uORB::ORB_parameter_update_public.updated = true;
            return (&uORB::parameter_update_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_attitude_control"))
        {
            std::strcpy(uORB::vehicle_attitude_control_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_attitude_control_public, data, sizeof(vehicle_attitude_control_s));
            uORB::ORB_vehicle_attitude_control_public.updated = true;
            return (&uORB::vehicle_attitude_control_adv);
        }
        else if (0 == std::strcmp(orb_name, "actuator_armed"))
        {
            std::strcpy(uORB::actuator_armed_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_actuator_armed_public, data, sizeof(actuator_armed_s));
            uORB::ORB_actuator_armed_public.updated = true;
            return (&uORB::actuator_armed_adv);
        }
        else if (0 == std::strcmp(orb_name, "sensor_combined"))
        {
            std::strcpy(uORB::sensor_combined_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_sensor_combined_public[1], data, sizeof(sensor_combined_s));
            uORB::ORB_sensor_combined_public[1].updated = true;
            uORB::ORB_sensor_combined_public[1].current_prio = ORB_PRIO_MAX;
            return (&uORB::sensor_combined_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_attitude"))
        {
            std::strcpy(uORB::vehicle_attitude_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_attitude_public[7], data, sizeof(vehicle_attitude_s));
            uORB::ORB_vehicle_attitude_public[7].updated = true;
            uORB::ORB_vehicle_attitude_public[7].current_prio = ORB_PRIO_MAX;
            return (&uORB::vehicle_attitude_adv);
        }
        else if (0 == std::strcmp(orb_name, "optical_flow"))
        {
            std::strcpy(uORB::optical_flow_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_optical_flow_public, data, sizeof(optical_flow_s));
            uORB::ORB_optical_flow_public.updated = true;
            return (&uORB::optical_flow_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_gps_position"))
        {
            std::strcpy(uORB::vehicle_gps_position_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_gps_position_public, data, sizeof(vehicle_gps_position_s));
            uORB::ORB_vehicle_gps_position_public.updated = true;
            return (&uORB::vehicle_gps_position_adv);
        }
        else if (0 == std::strcmp(orb_name, "vision_position_estimate"))
        {
            std::strcpy(uORB::vision_position_estimate_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vision_position_estimate_public, data, sizeof(vision_position_estimate_s));
            uORB::ORB_vision_position_estimate_public.updated = true;
            return (&uORB::vision_position_estimate_adv);
        }
        else if (0 == std::strcmp(orb_name, "att_pos_mocap"))
        {
            std::strcpy(uORB::att_pos_mocap_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_att_pos_mocap_public, data, sizeof(att_pos_mocap_s));
            uORB::ORB_att_pos_mocap_public.updated = true;
            return (&uORB::att_pos_mocap_adv);
        }
        else if (0 == std::strcmp(orb_name, "distance_sensor"))
        {
            std::strcpy(uORB::distance_sensor_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_distance_sensor_public[4], data, sizeof(distance_sensor_s));
            uORB::ORB_distance_sensor_public[4].updated = true;
            uORB::ORB_distance_sensor_public[4].current_prio = ORB_PRIO_MAX;
            return (&uORB::distance_sensor_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_rates_setpoint"))
        {
            std::strcpy(uORB::vehicle_rates_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_rates_setpoint_public, data, sizeof(vehicle_rates_setpoint_s));
            uORB::ORB_vehicle_rates_setpoint_public.updated = true;
            return (&uORB::vehicle_rates_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "sensor_gyro"))
        {
            std::strcpy(uORB::sensor_gyro_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_sensor_gyro_public[4], data, sizeof(sensor_gyro_s));
            uORB::ORB_sensor_gyro_public[4].updated = true;
            uORB::ORB_sensor_gyro_public[4].current_prio = ORB_PRIO_MAX;
            return (&uORB::sensor_gyro_adv);
        }
        else if (0 == std::strcmp(orb_name, "sensor_accel"))
        {
            std::strcpy(uORB::sensor_accel_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_sensor_accel_public[4], data, sizeof(sensor_accel_s));
            uORB::ORB_sensor_accel_public[4].updated = true;
            uORB::ORB_sensor_accel_public[4].current_prio = ORB_PRIO_MAX;
            return (&uORB::sensor_accel_adv);
        }
        else if (0 == std::strcmp(orb_name, "sensor_mag"))
        {
            std::strcpy(uORB::sensor_mag_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_sensor_mag_public[4], data, sizeof(sensor_mag_s));
            uORB::ORB_sensor_mag_public[4].updated = true;
            uORB::ORB_sensor_mag_public[4].current_prio = ORB_PRIO_MAX;
            return (&uORB::sensor_mag_adv);
        }
        else if (0 == std::strcmp(orb_name, "sensor_baro"))
        {
            std::strcpy(uORB::sensor_baro_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_sensor_baro_public[1], data, sizeof(sensor_baro_s));
            uORB::ORB_sensor_baro_public[1].updated = true;
            uORB::ORB_sensor_baro_public[1].current_prio = ORB_PRIO_MAX;
            return (&uORB::sensor_baro_adv);
        }
        else if (0 == std::strcmp(orb_name, "airspeed"))
        {
            std::strcpy(uORB::airspeed_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_airspeed_public, data, sizeof(airspeed_s));
            uORB::ORB_airspeed_public.updated = true;
            return (&uORB::airspeed_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_land_detected"))
        {
            std::strcpy(uORB::vehicle_land_detected_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_land_detected_public, data, sizeof(vehicle_land_detected_s));
            uORB::ORB_vehicle_land_detected_public.updated = true;
            return (&uORB::vehicle_land_detected_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_status"))
        {
            std::strcpy(uORB::vehicle_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_status_public, data, sizeof(vehicle_status_s));
            uORB::ORB_vehicle_status_public.updated = true;
            return (&uORB::vehicle_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_control_mode"))
        {
            std::strcpy(uORB::vehicle_control_mode_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_control_mode_public, data, sizeof(vehicle_control_mode_s));
            uORB::ORB_vehicle_control_mode_public.updated = true;
            return (&uORB::vehicle_control_mode_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_attitude_setpoint"))
        {
            std::strcpy(uORB::vehicle_attitude_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_attitude_setpoint_public, data, sizeof(vehicle_attitude_setpoint_s));
            uORB::ORB_vehicle_attitude_setpoint_public.updated = true;
            return (&uORB::vehicle_attitude_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "actuator_controls"))
        {
            std::strcpy(uORB::actuator_controls_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_actuator_controls_public, data, sizeof(actuator_controls_s));
            uORB::ORB_actuator_controls_public.updated = true;
            return (&uORB::actuator_controls_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_global_position"))
        {
            std::strcpy(uORB::vehicle_global_position_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_global_position_public, data, sizeof(vehicle_global_position_s));
            uORB::ORB_vehicle_global_position_public.updated = true;
            return (&uORB::vehicle_global_position_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_local_position"))
        {
            std::strcpy(uORB::vehicle_local_position_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_local_position_public, data, sizeof(vehicle_local_position_s));
            uORB::ORB_vehicle_local_position_public.updated = true;
            return (&uORB::vehicle_local_position_adv);
        }
        else if (0 == std::strcmp(orb_name, "control_state"))
        {
            std::strcpy(uORB::control_state_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_control_state_public, data, sizeof(control_state_s));
            uORB::ORB_control_state_public.updated = true;
            return (&uORB::control_state_adv);
        }
        else if (0 == std::strcmp(orb_name, "manual_control_setpoint"))
        {
            std::strcpy(uORB::manual_control_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_manual_control_setpoint_public, data, sizeof(manual_control_setpoint_s));
            uORB::ORB_manual_control_setpoint_public.updated = true;
            return (&uORB::manual_control_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "multirotor_motor_limits"))
        {
            std::strcpy(uORB::multirotor_motor_limits_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_multirotor_motor_limits_public, data, sizeof(multirotor_motor_limits_s));
            uORB::ORB_multirotor_motor_limits_public.updated = true;
            return (&uORB::multirotor_motor_limits_adv);
        }
        else if (0 == std::strcmp(orb_name, "mc_att_ctrl_status"))
        {
            std::strcpy(uORB::mc_att_ctrl_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_mc_att_ctrl_status_public, data, sizeof(mc_att_ctrl_status_s));
            uORB::ORB_mc_att_ctrl_status_public.updated = true;
            return (&uORB::mc_att_ctrl_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "estimator_status"))
        {
            std::strcpy(uORB::estimator_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_estimator_status_public, data, sizeof(estimator_status_s));
            uORB::ORB_estimator_status_public.updated = true;
            return (&uORB::estimator_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "wind_estimate"))
        {
            std::strcpy(uORB::wind_estimate_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_wind_estimate_public, data, sizeof(wind_estimate_s));
            uORB::ORB_wind_estimate_public.updated = true;
            return (&uORB::wind_estimate_adv);
        }
        else if (0 == std::strcmp(orb_name, "ekf2_innovations"))
        {
            std::strcpy(uORB::ekf2_innovations_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_ekf2_innovations_public, data, sizeof(ekf2_innovations_s));
            uORB::ORB_ekf2_innovations_public.updated = true;
            return (&uORB::ekf2_innovations_adv);
        }
        else if (0 == std::strcmp(orb_name, "position_setpoint_triplet"))
        {
            std::strcpy(uORB::position_setpoint_triplet_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_position_setpoint_triplet_public, data, sizeof(position_setpoint_triplet_s));
            uORB::ORB_position_setpoint_triplet_public.updated = true;
            return (&uORB::position_setpoint_triplet_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_local_position_setpoint"))
        {
            std::strcpy(uORB::vehicle_local_position_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_local_position_setpoint_public, data, sizeof(vehicle_local_position_setpoint_s));
            uORB::ORB_vehicle_local_position_setpoint_public.updated = true;
            return (&uORB::vehicle_local_position_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_global_velocity_setpoint"))
        {
            std::strcpy(uORB::vehicle_global_velocity_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_global_velocity_setpoint_public, data, sizeof(vehicle_global_velocity_setpoint_s));
            uORB::ORB_vehicle_global_velocity_setpoint_public.updated = true;
            return (&uORB::vehicle_global_velocity_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "follow_target"))
        {
            std::strcpy(uORB::follow_target_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_follow_target_public, data, sizeof(follow_target_s));
            uORB::ORB_follow_target_public.updated = true;
            return (&uORB::follow_target_adv);
        }
        else if (0 == std::strcmp(orb_name, "home_position"))
        {
            std::strcpy(uORB::home_position_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_home_position_public, data, sizeof(home_position_s));
            uORB::ORB_home_position_public.updated = true;
            return (&uORB::home_position_adv);
        }
        else if (0 == std::strcmp(orb_name, "mission"))
        {
            std::strcpy(uORB::mission_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_mission_public, data, sizeof(mission_s));
            uORB::ORB_mission_public.updated = true;
            return (&uORB::mission_adv);
        }
        else if (0 == std::strcmp(orb_name, "fw_pos_ctrl_status"))
        {
            std::strcpy(uORB::fw_pos_ctrl_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_fw_pos_ctrl_status_public, data, sizeof(fw_pos_ctrl_status_s));
            uORB::ORB_fw_pos_ctrl_status_public.updated = true;
            return (&uORB::fw_pos_ctrl_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "fence_vertex"))
        {
            std::strcpy(uORB::fence_vertex_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_fence_vertex_public, data, sizeof(fence_vertex_s));
            uORB::ORB_fence_vertex_public.updated = true;
            return (&uORB::fence_vertex_adv);
        }
        else if (0 == std::strcmp(orb_name, "fence"))
        {
            std::strcpy(uORB::fence_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_fence_public, data, sizeof(fence_s));
            uORB::ORB_fence_public.updated = true;
            return (&uORB::fence_adv);
        }
        else if (0 == std::strcmp(orb_name, "mission_result"))
        {
            std::strcpy(uORB::mission_result_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_mission_result_public, data, sizeof(mission_result_s));
            uORB::ORB_mission_result_public.updated = true;
            return (&uORB::mission_result_adv);
        }
        else if (0 == std::strcmp(orb_name, "geofence_result"))
        {
            std::strcpy(uORB::geofence_result_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_geofence_result_public, data, sizeof(geofence_result_s));
            uORB::ORB_geofence_result_public.updated = true;
            return (&uORB::geofence_result_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_command"))
        {
            std::strcpy(uORB::vehicle_command_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_command_public, data, sizeof(vehicle_command_s));
            uORB::ORB_vehicle_command_public.updated = true;
            return (&uORB::vehicle_command_adv);
        }
        else if (0 == std::strcmp(orb_name, "vtol_vehicle_status"))
        {
            std::strcpy(uORB::vtol_vehicle_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vtol_vehicle_status_public, data, sizeof(vtol_vehicle_status_s));
            uORB::ORB_vtol_vehicle_status_public.updated = true;
            return (&uORB::vtol_vehicle_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "battery_status"))
        {
            std::strcpy(uORB::battery_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_battery_status_public, data, sizeof(battery_status_s));
            uORB::ORB_battery_status_public.updated = true;
            return (&uORB::battery_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "safety"))
        {
            std::strcpy(uORB::safety_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_safety_public, data, sizeof(safety_s));
            uORB::ORB_safety_public.updated = true;
            return (&uORB::safety_adv);
        }
        else if (0 == std::strcmp(orb_name, "offboard_control_mode"))
        {
            std::strcpy(uORB::offboard_control_mode_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_offboard_control_mode_public, data, sizeof(offboard_control_mode_s));
            uORB::ORB_offboard_control_mode_public.updated = true;
            return (&uORB::offboard_control_mode_adv);
        }
        else if (0 == std::strcmp(orb_name, "commander_state"))
        {
            std::strcpy(uORB::commander_state_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_commander_state_public, data, sizeof(commander_state_s));
            uORB::ORB_commander_state_public.updated = true;
            return (&uORB::commander_state_adv);
        }
        else if (0 == std::strcmp(orb_name, "cpuload"))
        {
            std::strcpy(uORB::cpuload_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_cpuload_public, data, sizeof(cpuload_s));
            uORB::ORB_cpuload_public.updated = true;
            return (&uORB::cpuload_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_command_ack"))
        {
            std::strcpy(uORB::vehicle_command_ack_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_command_ack_public, data, sizeof(vehicle_command_ack_s));
            uORB::ORB_vehicle_command_ack_public.updated = true;
            return (&uORB::vehicle_command_ack_adv);
        }
        else if (0 == std::strcmp(orb_name, "differential_pressure"))
        {
            std::strcpy(uORB::differential_pressure_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_differential_pressure_public, data, sizeof(differential_pressure_s));
            uORB::ORB_differential_pressure_public.updated = true;
            return (&uORB::differential_pressure_adv);
        }
        else if (0 == std::strcmp(orb_name, "telemetry_status"))
        {
            std::strcpy(uORB::telemetry_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_telemetry_status_public, data, sizeof(telemetry_status_s));
            uORB::ORB_telemetry_status_public.updated = true;
            return (&uORB::telemetry_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "subsystem_info"))
        {
            std::strcpy(uORB::subsystem_info_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_subsystem_info_public, data, sizeof(subsystem_info_s));
            uORB::ORB_subsystem_info_public.updated = true;
            return (&uORB::subsystem_info_adv);
        }
        else if (0 == std::strcmp(orb_name, "system_power"))
        {
            std::strcpy(uORB::system_power_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_system_power_public, data, sizeof(system_power_s));
            uORB::ORB_system_power_public.updated = true;
            return (&uORB::system_power_adv);
        }
        else if (0 == std::strcmp(orb_name, "mavlink_log"))
        {
            std::strcpy(uORB::mavlink_log_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_mavlink_log_public, data, sizeof(mavlink_log_s));
            uORB::ORB_mavlink_log_public.updated = true;
            return (&uORB::mavlink_log_adv);
        }
        else if (0 == std::strcmp(orb_name, "debug_key_value"))
        {
            std::strcpy(uORB::debug_key_value_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_debug_key_value_public, data, sizeof(debug_key_value_s));
            uORB::ORB_debug_key_value_public.updated = true;
            return (&uORB::debug_key_value_adv);
        }
        else if (0 == std::strcmp(orb_name, "actuator_outputs"))
        {
            std::strcpy(uORB::actuator_outputs_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_actuator_outputs_public, data, sizeof(actuator_outputs_s));
            uORB::ORB_actuator_outputs_public.updated = true;
            return (&uORB::actuator_outputs_adv);
        }
        else if (0 == std::strcmp(orb_name, "actuator_direct"))
        {
            std::strcpy(uORB::actuator_direct_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_actuator_direct_public, data, sizeof(actuator_direct_s));
            uORB::ORB_actuator_direct_public.updated = true;
            return (&uORB::actuator_direct_adv);
        }
        else if (0 == std::strcmp(orb_name, "tecs_status"))
        {
            std::strcpy(uORB::tecs_status_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_tecs_status_public, data, sizeof(tecs_status_s));
            uORB::ORB_tecs_status_public.updated = true;
            return (&uORB::tecs_status_adv);
        }
        else if (0 == std::strcmp(orb_name, "rc_channels"))
        {
            std::strcpy(uORB::rc_channels_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_rc_channels_public, data, sizeof(rc_channels_s));
            uORB::ORB_rc_channels_public.updated = true;
            return (&uORB::rc_channels_adv);
        }
        else if (0 == std::strcmp(orb_name, "filtered_bottom_flow"))
        {
            std::strcpy(uORB::filtered_bottom_flow_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_filtered_bottom_flow_public, data, sizeof(filtered_bottom_flow_s));
            uORB::ORB_filtered_bottom_flow_public.updated = true;
            return (&uORB::filtered_bottom_flow_adv);
        }
        else if (0 == std::strcmp(orb_name, "satellite_info"))
        {
            std::strcpy(uORB::satellite_info_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_satellite_info_public, data, sizeof(satellite_info_s));
            uORB::ORB_satellite_info_public.updated = true;
            return (&uORB::satellite_info_adv);
        }
        else if (0 == std::strcmp(orb_name, "hil_sensor"))
        {
            std::strcpy(uORB::hil_sensor_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_hil_sensor_public, data, sizeof(hil_sensor_s));
            uORB::ORB_hil_sensor_public.updated = true;
            return (&uORB::hil_sensor_adv);
        }
        else if (0 == std::strcmp(orb_name, "log_message"))
        {
            std::strcpy(uORB::log_message_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_log_message_public, data, sizeof(log_message_s));
            uORB::ORB_log_message_public.updated = true;
            return (&uORB::log_message_adv);
        }
        else if (0 == std::strcmp(orb_name, "ekf2_timestamps"))
        {
            std::strcpy(uORB::ekf2_timestamps_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_ekf2_timestamps_public, data, sizeof(ekf2_timestamps_s));
            uORB::ORB_ekf2_timestamps_public.updated = true;
            return (&uORB::ekf2_timestamps_adv);
        }
        else if (0 == std::strcmp(orb_name, "led_control"))
        {
            std::strcpy(uORB::led_control_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_led_control_public, data, sizeof(led_control_s));
            uORB::ORB_led_control_public.updated = true;
            return (&uORB::led_control_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_roi"))
        {
            std::strcpy(uORB::vehicle_roi_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_roi_public, data, sizeof(vehicle_roi_s));
            uORB::ORB_vehicle_roi_public.updated = true;
            return (&uORB::vehicle_roi_adv);
        }
        else if (0 == std::strcmp(orb_name, "sensor_correction"))
        {
            std::strcpy(uORB::sensor_correction_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_sensor_correction_public, data, sizeof(sensor_correction_s));
            uORB::ORB_sensor_correction_public.updated = true;
            return (&uORB::sensor_correction_adv);
        }
        else if (0 == std::strcmp(orb_name, "mc_virtual_attitude_setpoint"))
        {
            std::strcpy(uORB::mc_virtual_attitude_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_mc_virtual_attitude_setpoint_public, data, sizeof(mc_virtual_attitude_setpoint_s));
            uORB::ORB_mc_virtual_attitude_setpoint_public.updated = true;
            return (&uORB::mc_virtual_attitude_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "mc_virtual_rates_setpoint"))
        {
            std::strcpy(uORB::mc_virtual_rates_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_mc_virtual_rates_setpoint_public, data, sizeof(mc_virtual_rates_setpoint_s));
            uORB::ORB_mc_virtual_rates_setpoint_public.updated = true;
            return (&uORB::mc_virtual_rates_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "fw_virtual_attitude_setpoint"))
        {
            std::strcpy(uORB::fw_virtual_attitude_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_fw_virtual_attitude_setpoint_public, data, sizeof(fw_virtual_attitude_setpoint_s));
            uORB::ORB_fw_virtual_attitude_setpoint_public.updated = true;
            return (&uORB::fw_virtual_attitude_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "fw_virtual_rates_setpoint"))
        {
            std::strcpy(uORB::fw_virtual_rates_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_fw_virtual_rates_setpoint_public, data, sizeof(fw_virtual_rates_setpoint_s));
            uORB::ORB_fw_virtual_rates_setpoint_public.updated = true;
            return (&uORB::fw_virtual_rates_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_status_flags"))
        {
            std::strcpy(uORB::vehicle_status_flags_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_status_flags_public, data, sizeof(vehicle_status_flags_s));
            uORB::ORB_vehicle_status_flags_public.updated = true;
            return (&uORB::vehicle_status_flags_adv);
        }
        else if (0 == std::strcmp(orb_name, "rc_parameter_map"))
        {
            std::strcpy(uORB::rc_parameter_map_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_rc_parameter_map_public, data, sizeof(rc_parameter_map_s));
            uORB::ORB_rc_parameter_map_public.updated = true;
            return (&uORB::rc_parameter_map_adv);
        }
        else if (0 == std::strcmp(orb_name, "transponder_report"))
        {
            std::strcpy(uORB::transponder_report_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_transponder_report_public, data, sizeof(transponder_report_s));
            uORB::ORB_transponder_report_public.updated = true;
            return (&uORB::transponder_report_adv);
        }
        else if (0 == std::strcmp(orb_name, "collision_report"))
        {
            std::strcpy(uORB::collision_report_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_collision_report_public, data, sizeof(collision_report_s));
            uORB::ORB_collision_report_public.updated = true;
            return (&uORB::collision_report_adv);
        }
        else if (0 == std::strcmp(orb_name, "camera_trigger"))
        {
            std::strcpy(uORB::camera_trigger_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_camera_trigger_public, data, sizeof(camera_trigger_s));
            uORB::ORB_camera_trigger_public.updated = true;
            return (&uORB::camera_trigger_adv);
        }
        else if (0 == std::strcmp(orb_name, "mount_orientation"))
        {
            std::strcpy(uORB::mount_orientation_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_mount_orientation_public, data, sizeof(mount_orientation_s));
            uORB::ORB_mount_orientation_public.updated = true;
            return (&uORB::mount_orientation_adv);
        }
        else if (0 == std::strcmp(orb_name, "input_rc"))
        {
            std::strcpy(uORB::input_rc_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_input_rc_public, data, sizeof(input_rc_s));
            uORB::ORB_input_rc_public.updated = true;
            return (&uORB::input_rc_adv);
        }
        else if (0 == std::strcmp(orb_name, "vehicle_force_setpoint"))
        {
            std::strcpy(uORB::vehicle_force_setpoint_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_vehicle_force_setpoint_public, data, sizeof(vehicle_force_setpoint_s));
            uORB::ORB_vehicle_force_setpoint_public.updated = true;
            return (&uORB::vehicle_force_setpoint_adv);
        }
        else if (0 == std::strcmp(orb_name, "time_offset"))
        {
            std::strcpy(uORB::time_offset_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_time_offset_public, data, sizeof(time_offset_s));
            uORB::ORB_time_offset_public.updated = true;
            return (&uORB::time_offset_adv);
        }
        else if (0 == std::strcmp(orb_name, "gps_inject_data"))
        {
            std::strcpy(uORB::gps_inject_data_adv.orb_name, orb_name);
            std::memcpy(&uORB::ORB_gps_inject_data_public, data, sizeof(gps_inject_data_s));
            uORB::ORB_gps_inject_data_public.updated = true;
            return (&uORB::gps_inject_data_adv);
        }
        else
        {
            return nullptr;
        }
}


//If subscribed, copy the source data
int orb_copy(char* orb_source_name, int orb_subscribe_handle, void* destination)
{
  if((0 == std::strcmp(orb_source_name, "parameter_update"))&&(orb_subscribe_handle == uORB::ORB_parameter_update))
  {
    MEMCPY(destination, &uORB::ORB_parameter_update_public,sizeof(parameter_update_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_attitude_control"))&&(orb_subscribe_handle == uORB::ORB_vehicle_attitude_control))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_attitude_control_public,sizeof(vehicle_attitude_control_s));
  }
  else if((0 == std::strcmp(orb_source_name, "actuator_armed"))&&(orb_subscribe_handle == uORB::ORB_actuator_armed))
  {
    MEMCPY(destination, &uORB::ORB_actuator_armed_public,sizeof(actuator_armed_s));
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_combined"))&&(orb_subscribe_handle == uORB::ORB_sensor_combined))
  {
    MEMCPY(destination, &uORB::ORB_sensor_combined_public[1],sizeof(sensor_combined_s));
    uORB::ORB_sensor_combined_public[1].current_prio = ORB_PRIO_MAX;
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_attitude"))&&(orb_subscribe_handle == uORB::ORB_vehicle_attitude))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[7],sizeof(vehicle_attitude_s));
    uORB::ORB_vehicle_attitude_public[7].current_prio = ORB_PRIO_MAX;
  }
  else if((0 == std::strcmp(orb_source_name, "optical_flow"))&&(orb_subscribe_handle == uORB::ORB_optical_flow))
  {
    MEMCPY(destination, &uORB::ORB_optical_flow_public,sizeof(optical_flow_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_gps_position"))&&(orb_subscribe_handle == uORB::ORB_vehicle_gps_position))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_gps_position_public,sizeof(vehicle_gps_position_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vision_position_estimate"))&&(orb_subscribe_handle == uORB::ORB_vision_position_estimate))
  {
    MEMCPY(destination, &uORB::ORB_vision_position_estimate_public,sizeof(vision_position_estimate_s));
  }
  else if((0 == std::strcmp(orb_source_name, "att_pos_mocap"))&&(orb_subscribe_handle == uORB::ORB_att_pos_mocap))
  {
    MEMCPY(destination, &uORB::ORB_att_pos_mocap_public,sizeof(att_pos_mocap_s));
  }
  else if((0 == std::strcmp(orb_source_name, "distance_sensor"))&&(orb_subscribe_handle == uORB::ORB_distance_sensor))
  {
    MEMCPY(destination, &uORB::ORB_distance_sensor_public[4],sizeof(distance_sensor_s));
    uORB::ORB_distance_sensor_public[4].current_prio = ORB_PRIO_MAX;
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_rates_setpoint"))&&(orb_subscribe_handle == uORB::ORB_vehicle_rates_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_rates_setpoint_public,sizeof(vehicle_rates_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_gyro"))&&(orb_subscribe_handle == uORB::ORB_sensor_gyro))
  {
    MEMCPY(destination, &uORB::ORB_sensor_gyro_public[4],sizeof(sensor_gyro_s));
    uORB::ORB_sensor_gyro_public[4].current_prio = ORB_PRIO_MAX;
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_accel"))&&(orb_subscribe_handle == uORB::ORB_sensor_accel))
  {
    MEMCPY(destination, &uORB::ORB_sensor_accel_public[4],sizeof(sensor_accel_s));
    uORB::ORB_sensor_accel_public[4].current_prio = ORB_PRIO_MAX;
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_mag"))&&(orb_subscribe_handle == uORB::ORB_sensor_mag))
  {
    MEMCPY(destination, &uORB::ORB_sensor_mag_public[4],sizeof(sensor_mag_s));
    uORB::ORB_sensor_mag_public[4].current_prio = ORB_PRIO_MAX;
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_baro"))&&(orb_subscribe_handle == uORB::ORB_sensor_baro))
  {
    MEMCPY(destination, &uORB::ORB_sensor_baro_public[1],sizeof(sensor_baro_s));
    uORB::ORB_sensor_baro_public[1].current_prio = ORB_PRIO_MAX;
  }
  else if((0 == std::strcmp(orb_source_name, "airspeed"))&&(orb_subscribe_handle == uORB::ORB_airspeed))
  {
    MEMCPY(destination, &uORB::ORB_airspeed_public,sizeof(airspeed_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_land_detected"))&&(orb_subscribe_handle == uORB::ORB_vehicle_land_detected))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_land_detected_public,sizeof(vehicle_land_detected_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_status"))&&(orb_subscribe_handle == uORB::ORB_vehicle_status))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_status_public,sizeof(vehicle_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_control_mode"))&&(orb_subscribe_handle == uORB::ORB_vehicle_control_mode))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_control_mode_public,sizeof(vehicle_control_mode_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_attitude_setpoint"))&&(orb_subscribe_handle == uORB::ORB_vehicle_attitude_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_attitude_setpoint_public,sizeof(vehicle_attitude_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "actuator_controls"))&&(orb_subscribe_handle == uORB::ORB_actuator_controls))
  {
    MEMCPY(destination, &uORB::ORB_actuator_controls_public,sizeof(actuator_controls_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_global_position"))&&(orb_subscribe_handle == uORB::ORB_vehicle_global_position))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_global_position_public,sizeof(vehicle_global_position_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_local_position"))&&(orb_subscribe_handle == uORB::ORB_vehicle_local_position))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_local_position_public,sizeof(vehicle_local_position_s));
  }
  else if((0 == std::strcmp(orb_source_name, "control_state"))&&(orb_subscribe_handle == uORB::ORB_control_state))
  {
    MEMCPY(destination, &uORB::ORB_control_state_public,sizeof(control_state_s));
  }
  else if((0 == std::strcmp(orb_source_name, "manual_control_setpoint"))&&(orb_subscribe_handle == uORB::ORB_manual_control_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_manual_control_setpoint_public,sizeof(manual_control_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "multirotor_motor_limits"))&&(orb_subscribe_handle == uORB::ORB_multirotor_motor_limits))
  {
    MEMCPY(destination, &uORB::ORB_multirotor_motor_limits_public,sizeof(multirotor_motor_limits_s));
  }
  else if((0 == std::strcmp(orb_source_name, "mc_att_ctrl_status"))&&(orb_subscribe_handle == uORB::ORB_mc_att_ctrl_status))
  {
    MEMCPY(destination, &uORB::ORB_mc_att_ctrl_status_public,sizeof(mc_att_ctrl_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "estimator_status"))&&(orb_subscribe_handle == uORB::ORB_estimator_status))
  {
    MEMCPY(destination, &uORB::ORB_estimator_status_public,sizeof(estimator_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "wind_estimate"))&&(orb_subscribe_handle == uORB::ORB_wind_estimate))
  {
    MEMCPY(destination, &uORB::ORB_wind_estimate_public,sizeof(wind_estimate_s));
  }
  else if((0 == std::strcmp(orb_source_name, "ekf2_innovations"))&&(orb_subscribe_handle == uORB::ORB_ekf2_innovations))
  {
    MEMCPY(destination, &uORB::ORB_ekf2_innovations_public,sizeof(ekf2_innovations_s));
  }
  else if((0 == std::strcmp(orb_source_name, "position_setpoint_triplet"))&&(orb_subscribe_handle == uORB::ORB_position_setpoint_triplet))
  {
    MEMCPY(destination, &uORB::ORB_position_setpoint_triplet_public,sizeof(position_setpoint_triplet_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_local_position_setpoint"))&&(orb_subscribe_handle == uORB::ORB_vehicle_local_position_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_local_position_setpoint_public,sizeof(vehicle_local_position_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_global_velocity_setpoint"))&&(orb_subscribe_handle == uORB::ORB_vehicle_global_velocity_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_global_velocity_setpoint_public,sizeof(vehicle_global_velocity_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "follow_target"))&&(orb_subscribe_handle == uORB::ORB_follow_target))
  {
    MEMCPY(destination, &uORB::ORB_follow_target_public,sizeof(follow_target_s));
  }
  else if((0 == std::strcmp(orb_source_name, "home_position"))&&(orb_subscribe_handle == uORB::ORB_home_position))
  {
    MEMCPY(destination, &uORB::ORB_home_position_public,sizeof(home_position_s));
  }
  else if((0 == std::strcmp(orb_source_name, "mission"))&&(orb_subscribe_handle == uORB::ORB_mission))
  {
    MEMCPY(destination, &uORB::ORB_mission_public,sizeof(mission_s));
  }
  else if((0 == std::strcmp(orb_source_name, "fw_pos_ctrl_status"))&&(orb_subscribe_handle == uORB::ORB_fw_pos_ctrl_status))
  {
    MEMCPY(destination, &uORB::ORB_fw_pos_ctrl_status_public,sizeof(fw_pos_ctrl_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "fence_vertex"))&&(orb_subscribe_handle == uORB::ORB_fence_vertex))
  {
    MEMCPY(destination, &uORB::ORB_fence_vertex_public,sizeof(fence_vertex_s));
  }
  else if((0 == std::strcmp(orb_source_name, "fence"))&&(orb_subscribe_handle == uORB::ORB_fence))
  {
    MEMCPY(destination, &uORB::ORB_fence_public,sizeof(fence_s));
  }
  else if((0 == std::strcmp(orb_source_name, "mission_result"))&&(orb_subscribe_handle == uORB::ORB_mission_result))
  {
    MEMCPY(destination, &uORB::ORB_mission_result_public,sizeof(mission_result_s));
  }
  else if((0 == std::strcmp(orb_source_name, "geofence_result"))&&(orb_subscribe_handle == uORB::ORB_geofence_result))
  {
    MEMCPY(destination, &uORB::ORB_geofence_result_public,sizeof(geofence_result_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_command"))&&(orb_subscribe_handle == uORB::ORB_vehicle_command))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_command_public,sizeof(vehicle_command_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vtol_vehicle_status"))&&(orb_subscribe_handle == uORB::ORB_vtol_vehicle_status))
  {
    MEMCPY(destination, &uORB::ORB_vtol_vehicle_status_public,sizeof(vtol_vehicle_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "battery_status"))&&(orb_subscribe_handle == uORB::ORB_battery_status))
  {
    MEMCPY(destination, &uORB::ORB_battery_status_public,sizeof(battery_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "safety"))&&(orb_subscribe_handle == uORB::ORB_safety))
  {
    MEMCPY(destination, &uORB::ORB_safety_public,sizeof(safety_s));
  }
  else if((0 == std::strcmp(orb_source_name, "offboard_control_mode"))&&(orb_subscribe_handle == uORB::ORB_offboard_control_mode))
  {
    MEMCPY(destination, &uORB::ORB_offboard_control_mode_public,sizeof(offboard_control_mode_s));
  }
  else if((0 == std::strcmp(orb_source_name, "commander_state"))&&(orb_subscribe_handle == uORB::ORB_commander_state))
  {
    MEMCPY(destination, &uORB::ORB_commander_state_public,sizeof(commander_state_s));
  }
  else if((0 == std::strcmp(orb_source_name, "cpuload"))&&(orb_subscribe_handle == uORB::ORB_cpuload))
  {
    MEMCPY(destination, &uORB::ORB_cpuload_public,sizeof(cpuload_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_command_ack"))&&(orb_subscribe_handle == uORB::ORB_vehicle_command_ack))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_command_ack_public,sizeof(vehicle_command_ack_s));
  }
  else if((0 == std::strcmp(orb_source_name, "differential_pressure"))&&(orb_subscribe_handle == uORB::ORB_differential_pressure))
  {
    MEMCPY(destination, &uORB::ORB_differential_pressure_public,sizeof(differential_pressure_s));
  }
  else if((0 == std::strcmp(orb_source_name, "telemetry_status"))&&(orb_subscribe_handle == uORB::ORB_telemetry_status))
  {
    MEMCPY(destination, &uORB::ORB_telemetry_status_public,sizeof(telemetry_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "subsystem_info"))&&(orb_subscribe_handle == uORB::ORB_subsystem_info))
  {
    MEMCPY(destination, &uORB::ORB_subsystem_info_public,sizeof(subsystem_info_s));
  }
  else if((0 == std::strcmp(orb_source_name, "system_power"))&&(orb_subscribe_handle == uORB::ORB_system_power))
  {
    MEMCPY(destination, &uORB::ORB_system_power_public,sizeof(system_power_s));
  }
  else if((0 == std::strcmp(orb_source_name, "mavlink_log"))&&(orb_subscribe_handle == uORB::ORB_mavlink_log))
  {
    MEMCPY(destination, &uORB::ORB_mavlink_log_public,sizeof(mavlink_log_s));
  }
  else if((0 == std::strcmp(orb_source_name, "debug_key_value"))&&(orb_subscribe_handle == uORB::ORB_debug_key_value))
  {
    MEMCPY(destination, &uORB::ORB_debug_key_value_public,sizeof(debug_key_value_s));
  }
  else if((0 == std::strcmp(orb_source_name, "actuator_outputs"))&&(orb_subscribe_handle == uORB::ORB_actuator_outputs))
  {
    MEMCPY(destination, &uORB::ORB_actuator_outputs_public,sizeof(actuator_outputs_s));
  }
  else if((0 == std::strcmp(orb_source_name, "actuator_direct"))&&(orb_subscribe_handle == uORB::ORB_actuator_direct))
  {
    MEMCPY(destination, &uORB::ORB_actuator_direct_public,sizeof(actuator_direct_s));
  }
  else if((0 == std::strcmp(orb_source_name, "tecs_status"))&&(orb_subscribe_handle == uORB::ORB_tecs_status))
  {
    MEMCPY(destination, &uORB::ORB_tecs_status_public,sizeof(tecs_status_s));
  }
  else if((0 == std::strcmp(orb_source_name, "rc_channels"))&&(orb_subscribe_handle == uORB::ORB_rc_channels))
  {
    MEMCPY(destination, &uORB::ORB_rc_channels_public,sizeof(rc_channels_s));
  }
  else if((0 == std::strcmp(orb_source_name, "filtered_bottom_flow"))&&(orb_subscribe_handle == uORB::ORB_filtered_bottom_flow))
  {
    MEMCPY(destination, &uORB::ORB_filtered_bottom_flow_public,sizeof(filtered_bottom_flow_s));
  }
  else if((0 == std::strcmp(orb_source_name, "satellite_info"))&&(orb_subscribe_handle == uORB::ORB_satellite_info))
  {
    MEMCPY(destination, &uORB::ORB_satellite_info_public,sizeof(satellite_info_s));
  }
  else if((0 == std::strcmp(orb_source_name, "hil_sensor"))&&(orb_subscribe_handle == uORB::ORB_hil_sensor))
  {
    MEMCPY(destination, &uORB::ORB_hil_sensor_public,sizeof(hil_sensor_s));
  }
  else if((0 == std::strcmp(orb_source_name, "log_message"))&&(orb_subscribe_handle == uORB::ORB_log_message))
  {
    MEMCPY(destination, &uORB::ORB_log_message_public,sizeof(log_message_s));
  }
  else if((0 == std::strcmp(orb_source_name, "ekf2_timestamps"))&&(orb_subscribe_handle == uORB::ORB_ekf2_timestamps))
  {
    MEMCPY(destination, &uORB::ORB_ekf2_timestamps_public,sizeof(ekf2_timestamps_s));
  }
  else if((0 == std::strcmp(orb_source_name, "led_control"))&&(orb_subscribe_handle == uORB::ORB_led_control))
  {
    MEMCPY(destination, &uORB::ORB_led_control_public,sizeof(led_control_s));
    uORB::ORB_led_control_public.updated=false;
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_roi"))&&(orb_subscribe_handle == uORB::ORB_vehicle_roi))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_roi_public,sizeof(vehicle_roi_s));
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_correction"))&&(orb_subscribe_handle == uORB::ORB_sensor_correction))
  {
    MEMCPY(destination, &uORB::ORB_sensor_correction_public,sizeof(sensor_correction_s));
  }
  else if((0 == std::strcmp(orb_source_name, "mc_virtual_attitude_setpoint"))&&(orb_subscribe_handle == uORB::ORB_mc_virtual_attitude_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_mc_virtual_attitude_setpoint_public,sizeof(mc_virtual_attitude_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "mc_virtual_rates_setpoint"))&&(orb_subscribe_handle == uORB::ORB_mc_virtual_rates_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_mc_virtual_rates_setpoint_public,sizeof(mc_virtual_rates_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "fw_virtual_attitude_setpoint"))&&(orb_subscribe_handle == uORB::ORB_fw_virtual_attitude_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_fw_virtual_attitude_setpoint_public,sizeof(fw_virtual_attitude_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "fw_virtual_rates_setpoint"))&&(orb_subscribe_handle == uORB::ORB_fw_virtual_rates_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_fw_virtual_rates_setpoint_public,sizeof(fw_virtual_rates_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_status_flags"))&&(orb_subscribe_handle == uORB::ORB_vehicle_status_flags))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_status_flags_public,sizeof(vehicle_status_flags_s));
  }
  else if((0 == std::strcmp(orb_source_name, "rc_parameter_map"))&&(orb_subscribe_handle == uORB::ORB_rc_parameter_map))
  {
    MEMCPY(destination, &uORB::ORB_rc_parameter_map_public,sizeof(rc_parameter_map_s));
  }
  else if((0 == std::strcmp(orb_source_name, "transponder_report"))&&(orb_subscribe_handle == uORB::ORB_transponder_report))
  {
    MEMCPY(destination, &uORB::ORB_transponder_report_public,sizeof(transponder_report_s));
  }
  else if((0 == std::strcmp(orb_source_name, "collision_report"))&&(orb_subscribe_handle == uORB::ORB_collision_report))
  {
    MEMCPY(destination, &uORB::ORB_collision_report_public,sizeof(collision_report_s));
  }
  else if((0 == std::strcmp(orb_source_name, "camera_trigger"))&&(orb_subscribe_handle == uORB::ORB_camera_trigger))
  {
    MEMCPY(destination, &uORB::ORB_camera_trigger_public,sizeof(camera_trigger_s));
  }
  else if((0 == std::strcmp(orb_source_name, "mount_orientation"))&&(orb_subscribe_handle == uORB::ORB_mount_orientation))
  {
    MEMCPY(destination, &uORB::ORB_mount_orientation_public,sizeof(mount_orientation_s));
  }
  else if((0 == std::strcmp(orb_source_name, "input_rc"))&&(orb_subscribe_handle == uORB::ORB_input_rc))
  {
    MEMCPY(destination, &uORB::ORB_input_rc_public,sizeof(input_rc_s));
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_force_setpoint"))&&(orb_subscribe_handle == uORB::ORB_vehicle_force_setpoint))
  {
    MEMCPY(destination, &uORB::ORB_vehicle_force_setpoint_public,sizeof(vehicle_force_setpoint_s));
  }
  else if((0 == std::strcmp(orb_source_name, "time_offset"))&&(orb_subscribe_handle == uORB::ORB_time_offset))
  {
    MEMCPY(destination, &uORB::ORB_time_offset_public,sizeof(time_offset_s));
  }
  else if((0 == std::strcmp(orb_source_name, "gps_inject_data"))&&(orb_subscribe_handle == uORB::ORB_gps_inject_data))
  {
    MEMCPY(destination, &uORB::ORB_gps_inject_data_public,sizeof(gps_inject_data_s));
  }
  else
      return -1;
  return 0;
}


void orb_copy_multi(char* orb_source_name, int orb_subscribe_handle, void* destination, ORB_PRIORITY priority)
{
  if((0 == std::strcmp(orb_source_name, "sensor_combined"))&&(orb_subscribe_handle == uORB::ORB_sensor_combined))
  {
    switch(priority)
    {
      case      ORB_PRIO_DEFAULT:
                MEMCPY(destination, &uORB::ORB_sensor_combined_public[0],sizeof(sensor_combined_s));
                break;
      case      ORB_PRIO_MAX:
                MEMCPY(destination, &uORB::ORB_sensor_combined_public[1],sizeof(sensor_combined_s));
                break;
      default:
                return;
    }
  }
  else if((0 == std::strcmp(orb_source_name, "vehicle_attitude"))&&(orb_subscribe_handle == uORB::ORB_vehicle_attitude))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[0],sizeof(vehicle_attitude_s));
                break;
      case      ORB_PRIO_VERY_LOW:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[1],sizeof(vehicle_attitude_s));
                break;
      case      ORB_PRIO_LOW:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[2],sizeof(vehicle_attitude_s));
                break;
      case      ORB_PRIO_DEFAULT:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[3],sizeof(vehicle_attitude_s));
                break;
      case      ORB_PRIO_ABOVE_DEFAULT:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[4],sizeof(vehicle_attitude_s));
                break;
      case      ORB_PRIO_HIGH:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[5],sizeof(vehicle_attitude_s));
                break;
      case      ORB_PRIO_VERY_HIGH:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[6],sizeof(vehicle_attitude_s));
                break;
      case      ORB_PRIO_MAX:
                MEMCPY(destination, &uORB::ORB_vehicle_attitude_public[7],sizeof(vehicle_attitude_s));
                break;          
      default:
                return;
    }
  }
  else if((0 == std::strcmp(orb_source_name, "distance_sensor"))&&(orb_subscribe_handle == uORB::ORB_distance_sensor))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                MEMCPY(destination, &uORB::ORB_distance_sensor_public[0],sizeof(distance_sensor_s));
                break;
      case      ORB_PRIO_LOW:
                MEMCPY(destination, &uORB::ORB_distance_sensor_public[1],sizeof(distance_sensor_s));
                break;
      case      ORB_PRIO_DEFAULT:
                MEMCPY(destination, &uORB::ORB_distance_sensor_public[2],sizeof(distance_sensor_s));
                break;
      case      ORB_PRIO_HIGH:
                MEMCPY(destination, &uORB::ORB_distance_sensor_public[3],sizeof(distance_sensor_s));
                break;
      case      ORB_PRIO_MAX:
                MEMCPY(destination, &uORB::ORB_distance_sensor_public[4],sizeof(distance_sensor_s));
                break;          
      default:
                return;
    }
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_gyro"))&&(orb_subscribe_handle == uORB::ORB_sensor_gyro))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                MEMCPY(destination, &uORB::ORB_sensor_gyro_public[0],sizeof(sensor_gyro_s));
                break;
      case      ORB_PRIO_LOW:
                MEMCPY(destination, &uORB::ORB_sensor_gyro_public[1],sizeof(sensor_gyro_s));
                break;
      case      ORB_PRIO_DEFAULT:
                MEMCPY(destination, &uORB::ORB_sensor_gyro_public[2],sizeof(sensor_gyro_s));
                break;
      case      ORB_PRIO_HIGH:
                MEMCPY(destination, &uORB::ORB_sensor_gyro_public[3],sizeof(sensor_gyro_s));
                break;
      case      ORB_PRIO_MAX:
                MEMCPY(destination, &uORB::ORB_sensor_gyro_public[4],sizeof(sensor_gyro_s));
                break;          
      default:
                return;
    }
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_accel"))&&(orb_subscribe_handle == uORB::ORB_sensor_accel))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                MEMCPY(destination, &uORB::ORB_sensor_accel_public[0],sizeof(sensor_accel_s));
                break;
      case      ORB_PRIO_LOW:
                MEMCPY(destination, &uORB::ORB_sensor_accel_public[1],sizeof(sensor_accel_s));
                break;
      case      ORB_PRIO_DEFAULT:
                MEMCPY(destination, &uORB::ORB_sensor_accel_public[2],sizeof(sensor_accel_s));
                break;
      case      ORB_PRIO_HIGH:
                MEMCPY(destination, &uORB::ORB_sensor_accel_public[3],sizeof(sensor_accel_s));
                break;
      case      ORB_PRIO_MAX:
                MEMCPY(destination, &uORB::ORB_sensor_accel_public[4],sizeof(sensor_accel_s));
                break;          
      default:
                return;
    }
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_mag"))&&(orb_subscribe_handle == uORB::ORB_sensor_mag))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                MEMCPY(destination, &uORB::ORB_sensor_mag_public[0],sizeof(sensor_mag_s));
                break;
      case      ORB_PRIO_LOW:
                MEMCPY(destination, &uORB::ORB_sensor_mag_public[1],sizeof(sensor_mag_s));
                break;
      case      ORB_PRIO_DEFAULT:
                MEMCPY(destination, &uORB::ORB_sensor_mag_public[2],sizeof(sensor_mag_s));
                break;
      case      ORB_PRIO_HIGH:
                MEMCPY(destination, &uORB::ORB_sensor_mag_public[3],sizeof(sensor_mag_s));
                break;
      case      ORB_PRIO_MAX:
                MEMCPY(destination, &uORB::ORB_sensor_mag_public[4],sizeof(sensor_mag_s));
                break;          
      default:
                return;
    }
  }
  else if((0 == std::strcmp(orb_source_name, "sensor_baro"))&&(orb_subscribe_handle == uORB::ORB_sensor_baro))
  {
    switch(priority)
    {
      case      ORB_PRIO_DEFAULT:
                MEMCPY(destination, &uORB::ORB_sensor_baro_public[0],sizeof(sensor_baro_s));
                break;
      case      ORB_PRIO_MAX:
                MEMCPY(destination, &uORB::ORB_sensor_baro_public[1],sizeof(sensor_baro_s));
                break;          
      default:
                return;
    }
  }
  else
    return;
}



int orb_check_multi(int orb_subscribe_handle, bool *updated, ORB_PRIORITY priority)
{
  if(orb_subscribe_handle == uORB::ORB_sensor_combined)
  {
    switch(priority)
    {
      case      ORB_PRIO_DEFAULT:
                *updated = uORB::ORB_sensor_combined_public[0].updated;
                break;
      case      ORB_PRIO_MAX:
                *updated = uORB::ORB_sensor_combined_public[1].updated;
                break;
      default:
                *updated=false;
                break;
    }
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_attitude)
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                *updated = uORB::ORB_vehicle_attitude_public[0].updated;
                break;
      case      ORB_PRIO_VERY_LOW:
                *updated = uORB::ORB_vehicle_attitude_public[1].updated;
                break;
      case      ORB_PRIO_LOW:
                *updated = uORB::ORB_vehicle_attitude_public[2].updated;
                break;
      case      ORB_PRIO_DEFAULT:
                *updated = uORB::ORB_vehicle_attitude_public[3].updated;
                break;
      case      ORB_PRIO_ABOVE_DEFAULT:
                *updated = uORB::ORB_vehicle_attitude_public[4].updated;
                break;
      case      ORB_PRIO_HIGH:
                *updated = uORB::ORB_vehicle_attitude_public[5].updated;
                break;
      case      ORB_PRIO_VERY_HIGH:
                *updated = uORB::ORB_vehicle_attitude_public[6].updated;
                break;          
      case      ORB_PRIO_MAX:
                *updated = uORB::ORB_vehicle_attitude_public[7].updated;
                break;
      default:
                *updated=false;
                break;
    }
  }
  else if(orb_subscribe_handle == uORB::ORB_distance_sensor)
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                *updated = uORB::ORB_distance_sensor_public[0].updated;
                break;
      case      ORB_PRIO_LOW:
                *updated = uORB::ORB_distance_sensor_public[1].updated;
                break;
      case      ORB_PRIO_DEFAULT:
                *updated = uORB::ORB_distance_sensor_public[2].updated;
                break;
      case      ORB_PRIO_HIGH:
                *updated = uORB::ORB_distance_sensor_public[3].updated;
                break;      
      case      ORB_PRIO_MAX:
                *updated = uORB::ORB_distance_sensor_public[4].updated;
                break;
      default:
                *updated=false;
                break;
    }
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_gyro)
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                *updated = uORB::ORB_sensor_gyro_public[0].updated;
                break;
      case      ORB_PRIO_LOW:
                *updated = uORB::ORB_sensor_gyro_public[1].updated;
                break;
      case      ORB_PRIO_DEFAULT:
                *updated = uORB::ORB_sensor_gyro_public[2].updated;
                break;
      case      ORB_PRIO_HIGH:
                *updated = uORB::ORB_sensor_gyro_public[3].updated;
                break;      
      case      ORB_PRIO_MAX:
                *updated = uORB::ORB_sensor_gyro_public[4].updated;
                break;
      default:
                *updated=false;
                break;
    }
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_accel)
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                *updated = uORB::ORB_sensor_accel_public[0].updated;
                break;
      case      ORB_PRIO_LOW:
                *updated = uORB::ORB_sensor_accel_public[1].updated;
                break;
      case      ORB_PRIO_DEFAULT:
                *updated = uORB::ORB_sensor_accel_public[2].updated;
                break;
      case      ORB_PRIO_HIGH:
                *updated = uORB::ORB_sensor_accel_public[3].updated;
                break;      
      case      ORB_PRIO_MAX:
                *updated = uORB::ORB_sensor_accel_public[4].updated;
                break;
      default:
                *updated=false;
                break;
    }
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_mag)
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
                *updated = uORB::ORB_sensor_mag_public[0].updated;
                break;
      case      ORB_PRIO_LOW:
                *updated = uORB::ORB_sensor_mag_public[1].updated;
                break;
      case      ORB_PRIO_DEFAULT:
                *updated = uORB::ORB_sensor_mag_public[2].updated;
                break;
      case      ORB_PRIO_HIGH:
                *updated = uORB::ORB_sensor_mag_public[3].updated;
                break;      
      case      ORB_PRIO_MAX:
                *updated = uORB::ORB_sensor_mag_public[4].updated;
                break;
      default:
                *updated=false;
                break;
    }
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_baro)
  {
    switch(priority)
    {
      case      ORB_PRIO_DEFAULT:
                *updated = uORB::ORB_sensor_baro_public[0].updated;
                break;
      case      ORB_PRIO_MAX:
                *updated = uORB::ORB_sensor_baro_public[1].updated;
                break;
      default:
                *updated=false;
                break;
    }
  }
  else
    return -1;
  
  
  return 0;
}


int orb_check(int orb_subscribe_handle, bool *updated)
{
  if(orb_subscribe_handle == uORB::ORB_parameter_update)
  {
    *updated = uORB::ORB_parameter_update_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_attitude_control)
  {
    *updated = uORB::ORB_vehicle_attitude_control_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_actuator_armed)
  {
    *updated = uORB::ORB_actuator_armed_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_combined)
  {
    *updated = uORB::ORB_sensor_combined_public[1].updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_attitude)
  {
    *updated = uORB::ORB_vehicle_attitude_public[7].updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_optical_flow)
  {
    *updated = uORB::ORB_optical_flow_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_gps_position)
  {
    *updated = uORB::ORB_vehicle_gps_position_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vision_position_estimate)
  {
    *updated = uORB::ORB_vision_position_estimate_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_att_pos_mocap)
  {
    *updated = uORB::ORB_att_pos_mocap_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_distance_sensor)
  {
    *updated = uORB::ORB_distance_sensor_public[4].updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_rates_setpoint)
  {
    *updated = uORB::ORB_vehicle_rates_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_gyro)
  {
    *updated = uORB::ORB_sensor_gyro_public[4].updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_accel)
  {
    *updated = uORB::ORB_sensor_accel_public[4].updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_mag)
  {
    *updated = uORB::ORB_sensor_mag_public[4].updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_baro)
  {
    *updated = uORB::ORB_sensor_baro_public[1].updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_airspeed)
  {
    *updated = uORB::ORB_airspeed_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_land_detected)
  {
    *updated = uORB::ORB_vehicle_land_detected_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_status)
  {
    *updated = uORB::ORB_vehicle_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_control_mode)
  {
    *updated = uORB::ORB_vehicle_control_mode_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_attitude_setpoint)
  {
    *updated = uORB::ORB_vehicle_attitude_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_actuator_controls)
  {
    *updated = uORB::ORB_actuator_controls_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_global_position)
  {
    *updated = uORB::ORB_vehicle_global_position_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_local_position)
  {
    *updated = uORB::ORB_vehicle_local_position_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_control_state)
  {
    *updated = uORB::ORB_control_state_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_manual_control_setpoint)
  {
    *updated = uORB::ORB_manual_control_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_multirotor_motor_limits)
  {
    *updated = uORB::ORB_multirotor_motor_limits_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_mc_att_ctrl_status)
  {
    *updated = uORB::ORB_mc_att_ctrl_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_estimator_status)
  {
    *updated = uORB::ORB_estimator_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_wind_estimate)
  {
    *updated = uORB::ORB_wind_estimate_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_ekf2_innovations)
  {
    *updated = uORB::ORB_ekf2_innovations_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_position_setpoint_triplet)
  {
    *updated = uORB::ORB_position_setpoint_triplet_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_local_position_setpoint)
  {
    *updated = uORB::ORB_vehicle_local_position_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_global_velocity_setpoint)
  {
    *updated = uORB::ORB_vehicle_global_velocity_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_follow_target)
  {
    *updated = uORB::ORB_follow_target_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_home_position)
  {
    *updated = uORB::ORB_home_position_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_mission)
  {
    *updated = uORB::ORB_mission_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_fw_pos_ctrl_status)
  {
    *updated = uORB::ORB_fw_pos_ctrl_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_fence_vertex)
  {
    *updated = uORB::ORB_fence_vertex_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_fence)
  {
    *updated = uORB::ORB_fence_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_mission_result)
  {
    *updated = uORB::ORB_mission_result_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_geofence_result)
  {
    *updated = uORB::ORB_geofence_result_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_command)
  {
    *updated = uORB::ORB_vehicle_command_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vtol_vehicle_status)
  {
    *updated = uORB::ORB_vtol_vehicle_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_battery_status)
  {
    *updated = uORB::ORB_battery_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_safety)
  {
    *updated = uORB::ORB_safety_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_offboard_control_mode)
  {
    *updated = uORB::ORB_offboard_control_mode_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_commander_state)
  {
    *updated = uORB::ORB_commander_state_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_cpuload)
  {
    *updated = uORB::ORB_cpuload_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_command_ack)
  {
    *updated = uORB::ORB_vehicle_command_ack_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_differential_pressure)
  {
    *updated = uORB::ORB_differential_pressure_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_telemetry_status)
  {
    *updated = uORB::ORB_telemetry_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_subsystem_info)
  {
    *updated = uORB::ORB_subsystem_info_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_system_power)
  {
    *updated = uORB::ORB_system_power_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_mavlink_log)
  {
    *updated = uORB::ORB_mavlink_log_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_debug_key_value)
  {
    *updated = uORB::ORB_debug_key_value_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_actuator_outputs)
  {
    *updated = uORB::ORB_actuator_outputs_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_actuator_direct)
  {
    *updated = uORB::ORB_actuator_direct_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_tecs_status)
  {
    *updated = uORB::ORB_tecs_status_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_rc_channels)
  {
    *updated = uORB::ORB_rc_channels_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_filtered_bottom_flow)
  {
    *updated = uORB::ORB_filtered_bottom_flow_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_satellite_info)
  {
    *updated = uORB::ORB_satellite_info_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_hil_sensor)
  {
    *updated = uORB::ORB_hil_sensor_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_log_message)
  {
    *updated = uORB::ORB_log_message_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_ekf2_timestamps)
  {
    *updated = uORB::ORB_ekf2_timestamps_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_led_control)
  {
    *updated = uORB::ORB_led_control_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_roi)
  {
    *updated = uORB::ORB_vehicle_roi_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_sensor_correction)
  {
    *updated = uORB::ORB_sensor_correction_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_mc_virtual_attitude_setpoint)
  {
    *updated = uORB::ORB_mc_virtual_attitude_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_mc_virtual_rates_setpoint)
  {
    *updated = uORB::ORB_mc_virtual_rates_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_fw_virtual_attitude_setpoint)
  {
    *updated = uORB::ORB_fw_virtual_attitude_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_fw_virtual_rates_setpoint)
  {
    *updated = uORB::ORB_fw_virtual_rates_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_status_flags)
  {
    *updated = uORB::ORB_vehicle_status_flags_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_rc_parameter_map)
  {
    *updated = uORB::ORB_rc_parameter_map_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_transponder_report)
  {
    *updated = uORB::ORB_transponder_report_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_collision_report)
  {
    *updated = uORB::ORB_collision_report_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_camera_trigger)
  {
    *updated = uORB::ORB_camera_trigger_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_mount_orientation)
  {
    *updated = uORB::ORB_mount_orientation_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_input_rc)
  {
    *updated = uORB::ORB_input_rc_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_vehicle_force_setpoint)
  {
    *updated = uORB::ORB_vehicle_force_setpoint_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_time_offset)
  {
    *updated = uORB::ORB_time_offset_public.updated;
  }
  else if(orb_subscribe_handle == uORB::ORB_gps_inject_data)
  {
    *updated = uORB::ORB_gps_inject_data_public.updated;
  }
  else
    *updated = false;
  
  if(*updated)
      return 0;
  else
      return -1;
}


int orb_publish(char* orb_source_name, orb_advert_t pub_handle, void* data){
  if(0 == std::strcmp(orb_source_name,"parameter_update") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_parameter_update_public.updated = false;
    MEMCPY(&uORB::ORB_parameter_update_public, data, sizeof(parameter_update_s));
    uORB::ORB_parameter_update_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_attitude_control") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_attitude_control_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_attitude_control_public, data, sizeof(vehicle_attitude_control_s));
    uORB::ORB_vehicle_attitude_control_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"actuator_armed") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_actuator_armed_public.updated = false;
    MEMCPY(&uORB::ORB_actuator_armed_public, data, sizeof(actuator_armed_s));
    uORB::ORB_actuator_armed_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"sensor_combined") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_sensor_combined_public[1].updated = false;
    MEMCPY(&uORB::ORB_sensor_combined_public[1], data, sizeof(sensor_combined_s));
    uORB::ORB_sensor_combined_public[1].updated = true;
    uORB::ORB_sensor_combined_public[1].current_prio = ORB_PRIO_MAX;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_attitude") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_attitude_public[7].updated = false;
    MEMCPY(&uORB::ORB_vehicle_attitude_public[7], data, sizeof(vehicle_attitude_s));
    uORB::ORB_vehicle_attitude_public[7].updated = true;
    uORB::ORB_vehicle_attitude_public[7].current_prio = ORB_PRIO_MAX;
  }
  else if(0 == std::strcmp(orb_source_name,"optical_flow") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_optical_flow_public.updated = false;
    MEMCPY(&uORB::ORB_optical_flow_public, data, sizeof(optical_flow_s));
    uORB::ORB_optical_flow_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_gps_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_gps_position_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_gps_position_public, data, sizeof(vehicle_gps_position_s));
    uORB::ORB_vehicle_gps_position_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vision_position_estimate") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vision_position_estimate_public.updated = false;
    MEMCPY(&uORB::ORB_vision_position_estimate_public, data, sizeof(vision_position_estimate_s));
    uORB::ORB_vision_position_estimate_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"att_pos_mocap") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_att_pos_mocap_public.updated = false;
    MEMCPY(&uORB::ORB_att_pos_mocap_public, data, sizeof(att_pos_mocap_s));
    uORB::ORB_att_pos_mocap_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"distance_sensor") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_distance_sensor_public[4].updated = false;
    MEMCPY(&uORB::ORB_distance_sensor_public[4], data, sizeof(distance_sensor_s));
    uORB::ORB_distance_sensor_public[4].updated = true;
    uORB::ORB_distance_sensor_public[4].current_prio = ORB_PRIO_MAX;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_rates_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_rates_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_rates_setpoint_public, data, sizeof(vehicle_rates_setpoint_s));
    uORB::ORB_vehicle_rates_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"sensor_gyro") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_sensor_gyro_public[4].updated = false;
    MEMCPY(&uORB::ORB_sensor_gyro_public[4], data, sizeof(sensor_gyro_s));
    uORB::ORB_sensor_gyro_public[4].updated = true;
    uORB::ORB_sensor_gyro_public[4].current_prio = ORB_PRIO_MAX;
  }
  else if(0 == std::strcmp(orb_source_name,"sensor_accel") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_sensor_accel_public[4].updated = false;
    MEMCPY(&uORB::ORB_sensor_accel_public[4], data, sizeof(sensor_accel_s));
    uORB::ORB_sensor_accel_public[4].updated = true;
    uORB::ORB_sensor_accel_public[1].current_prio = ORB_PRIO_MAX;
  }
  else if(0 == std::strcmp(orb_source_name,"sensor_mag") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_sensor_mag_public[4].updated = false;
    MEMCPY(&uORB::ORB_sensor_mag_public[4], data, sizeof(sensor_mag_s));
    uORB::ORB_sensor_mag_public[4].updated = true;
    uORB::ORB_sensor_mag_public[4].current_prio = ORB_PRIO_MAX;
  }
  else if(0 == std::strcmp(orb_source_name,"sensor_baro") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_sensor_baro_public[1].updated = false;
    MEMCPY(&uORB::ORB_sensor_baro_public[1], data, sizeof(sensor_baro_s));
    uORB::ORB_sensor_baro_public[1].updated = true;
    uORB::ORB_sensor_baro_public[1].current_prio = ORB_PRIO_MAX;
  }
  else if(0 == std::strcmp(orb_source_name,"airspeed") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_airspeed_public.updated = false;
    MEMCPY(&uORB::ORB_airspeed_public, data, sizeof(airspeed_s));
    uORB::ORB_airspeed_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_land_detected") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_land_detected_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_land_detected_public, data, sizeof(vehicle_land_detected_s));
    uORB::ORB_vehicle_land_detected_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_status_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_status_public, data, sizeof(vehicle_status_s));
    uORB::ORB_vehicle_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_control_mode") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_control_mode_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_control_mode_public, data, sizeof(vehicle_control_mode_s));
    uORB::ORB_vehicle_control_mode_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_attitude_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_attitude_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_attitude_setpoint_public, data, sizeof(vehicle_attitude_setpoint_s));
    uORB::ORB_vehicle_attitude_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"actuator_controls") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_actuator_controls_public.updated = false;
    MEMCPY(&uORB::ORB_actuator_controls_public, data, sizeof(actuator_controls_s));
    uORB::ORB_actuator_controls_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_global_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_global_position_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_global_position_public, data, sizeof(vehicle_global_position_s));
    uORB::ORB_vehicle_global_position_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_local_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_local_position_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_local_position_public, data, sizeof(vehicle_local_position_s));
    uORB::ORB_vehicle_local_position_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"control_state") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_control_state_public.updated = false;
    MEMCPY(&uORB::ORB_control_state_public, data, sizeof(control_state_s));
    uORB::ORB_control_state_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"manual_control_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_manual_control_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_manual_control_setpoint_public, data, sizeof(manual_control_setpoint_s));
    uORB::ORB_manual_control_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"multirotor_motor_limits") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_multirotor_motor_limits_public.updated = false;
    MEMCPY(&uORB::ORB_multirotor_motor_limits_public, data, sizeof(multirotor_motor_limits_s));
    uORB::ORB_multirotor_motor_limits_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"mc_att_ctrl_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_mc_att_ctrl_status_public.updated = false;
    MEMCPY(&uORB::ORB_mc_att_ctrl_status_public, data, sizeof(mc_att_ctrl_status_s));
    uORB::ORB_mc_att_ctrl_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"estimator_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_estimator_status_public.updated = false;
    MEMCPY(&uORB::ORB_estimator_status_public, data, sizeof(estimator_status_s));
    uORB::ORB_estimator_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"wind_estimate") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_wind_estimate_public.updated = false;
    MEMCPY(&uORB::ORB_wind_estimate_public, data, sizeof(wind_estimate_s));
    uORB::ORB_wind_estimate_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"ekf2_innovations") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_ekf2_innovations_public.updated = false;
    MEMCPY(&uORB::ORB_ekf2_innovations_public, data, sizeof(ekf2_innovations_s));
    uORB::ORB_ekf2_innovations_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"position_setpoint_triplet") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_position_setpoint_triplet_public.updated = false;
    MEMCPY(&uORB::ORB_position_setpoint_triplet_public, data, sizeof(position_setpoint_triplet_s));
    uORB::ORB_position_setpoint_triplet_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_local_position_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_local_position_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_local_position_setpoint_public, data, sizeof(vehicle_local_position_setpoint_s));
    uORB::ORB_vehicle_local_position_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_global_velocity_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_global_velocity_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_global_velocity_setpoint_public, data, sizeof(vehicle_global_velocity_setpoint_s));
    uORB::ORB_vehicle_global_velocity_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"follow_target") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_follow_target_public.updated = false;
    MEMCPY(&uORB::ORB_follow_target_public, data, sizeof(follow_target_s));
    uORB::ORB_follow_target_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"home_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_home_position_public.updated = false;
    MEMCPY(&uORB::ORB_home_position_public, data, sizeof(home_position_s));
    uORB::ORB_home_position_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"mission") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_mission_public.updated = false;
    MEMCPY(&uORB::ORB_mission_public, data, sizeof(mission_s));
    uORB::ORB_mission_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"fw_pos_ctrl_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_fw_pos_ctrl_status_public.updated = false;
    MEMCPY(&uORB::ORB_fw_pos_ctrl_status_public, data, sizeof(fw_pos_ctrl_status_s));
    uORB::ORB_fw_pos_ctrl_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"fence_vertex") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_fence_vertex_public.updated = false;
    MEMCPY(&uORB::ORB_fence_vertex_public, data, sizeof(fence_vertex_s));
    uORB::ORB_fence_vertex_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"fence") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_fence_public.updated = false;
    MEMCPY(&uORB::ORB_fence_public, data, sizeof(fence_s));
    uORB::ORB_fence_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"mission_result") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_mission_result_public.updated = false;
    MEMCPY(&uORB::ORB_mission_result_public, data, sizeof(mission_result_s));
    uORB::ORB_mission_result_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"geofence_result") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_geofence_result_public.updated = false;
    MEMCPY(&uORB::ORB_geofence_result_public, data, sizeof(geofence_result_s));
    uORB::ORB_geofence_result_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_command") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_command_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_command_public, data, sizeof(vehicle_command_s));
    uORB::ORB_vehicle_command_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vtol_vehicle_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vtol_vehicle_status_public.updated = false;
    MEMCPY(&uORB::ORB_vtol_vehicle_status_public, data, sizeof(vtol_vehicle_status_s));
    uORB::ORB_vtol_vehicle_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"battery_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_battery_status_public.updated = false;
    MEMCPY(&uORB::ORB_battery_status_public, data, sizeof(battery_status_s));
    uORB::ORB_battery_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"safety") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_safety_public.updated = false;
    MEMCPY(&uORB::ORB_safety_public, data, sizeof(safety_s));
    uORB::ORB_safety_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"offboard_control_mode") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_offboard_control_mode_public.updated = false;
    MEMCPY(&uORB::ORB_offboard_control_mode_public, data, sizeof(offboard_control_mode_s));
    uORB::ORB_offboard_control_mode_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"commander_state") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_commander_state_public.updated = false;
    MEMCPY(&uORB::ORB_commander_state_public, data, sizeof(commander_state_s));
    uORB::ORB_commander_state_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"cpuload") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_cpuload_public.updated = false;
    MEMCPY(&uORB::ORB_cpuload_public, data, sizeof(cpuload_s));
    uORB::ORB_cpuload_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_command_ack") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_command_ack_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_command_ack_public, data, sizeof(vehicle_command_ack_s));
    uORB::ORB_vehicle_command_ack_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"differential_pressure") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_differential_pressure_public.updated = false;
    MEMCPY(&uORB::ORB_differential_pressure_public, data, sizeof(differential_pressure_s));
    uORB::ORB_differential_pressure_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"telemetry_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_telemetry_status_public.updated = false;
    MEMCPY(&uORB::ORB_telemetry_status_public, data, sizeof(telemetry_status_s));
    uORB::ORB_telemetry_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"subsystem_info") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_subsystem_info_public.updated = false;
    MEMCPY(&uORB::ORB_subsystem_info_public, data, sizeof(subsystem_info_s));
    uORB::ORB_subsystem_info_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"system_power") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_system_power_public.updated = false;
    MEMCPY(&uORB::ORB_system_power_public, data, sizeof(system_power_s));
    uORB::ORB_system_power_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"mavlink_log") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_mavlink_log_public.updated = false;
    MEMCPY(&uORB::ORB_mavlink_log_public, data, sizeof(mavlink_log_s));
    uORB::ORB_mavlink_log_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"debug_key_value") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_debug_key_value_public.updated = false;
    MEMCPY(&uORB::ORB_debug_key_value_public, data, sizeof(debug_key_value_s));
    uORB::ORB_debug_key_value_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"actuator_outputs") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_actuator_outputs_public.updated = false;
    MEMCPY(&uORB::ORB_actuator_outputs_public, data, sizeof(actuator_outputs_s));
    uORB::ORB_actuator_outputs_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"actuator_direct") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_actuator_direct_public.updated = false;
    MEMCPY(&uORB::ORB_actuator_direct_public, data, sizeof(actuator_direct_s));
    uORB::ORB_actuator_direct_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"tecs_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_tecs_status_public.updated = false;
    MEMCPY(&uORB::ORB_tecs_status_public, data, sizeof(tecs_status_s));
    uORB::ORB_tecs_status_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"rc_channels") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_rc_channels_public.updated = false;
    MEMCPY(&uORB::ORB_rc_channels_public, data, sizeof(rc_channels_s));
    uORB::ORB_rc_channels_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"filtered_bottom_flow") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_filtered_bottom_flow_public.updated = false;
    MEMCPY(&uORB::ORB_filtered_bottom_flow_public, data, sizeof(filtered_bottom_flow_s));
    uORB::ORB_filtered_bottom_flow_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"satellite_info") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_satellite_info_public.updated = false;
    MEMCPY(&uORB::ORB_satellite_info_public, data, sizeof(satellite_info_s));
    uORB::ORB_satellite_info_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"hil_sensor") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_hil_sensor_public.updated = false;
    MEMCPY(&uORB::ORB_hil_sensor_public, data, sizeof(hil_sensor_s));
    uORB::ORB_hil_sensor_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"log_message") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_log_message_public.updated = false;
    MEMCPY(&uORB::ORB_log_message_public, data, sizeof(log_message_s));
    uORB::ORB_log_message_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"ekf2_timestamps") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_ekf2_timestamps_public.updated = false;
    MEMCPY(&uORB::ORB_ekf2_timestamps_public, data, sizeof(ekf2_timestamps_s));
    uORB::ORB_ekf2_timestamps_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"led_control") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_led_control_public.updated = false;
    MEMCPY(&uORB::ORB_led_control_public, data, sizeof(led_control_s));
    uORB::ORB_led_control_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_roi") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_roi_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_roi_public, data, sizeof(vehicle_roi_s));
    uORB::ORB_vehicle_roi_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"mc_virtual_attitude_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_mc_virtual_attitude_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_mc_virtual_attitude_setpoint_public, data, sizeof(mc_virtual_attitude_setpoint_s));
    uORB::ORB_mc_virtual_attitude_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"mc_virtual_rates_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_mc_virtual_rates_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_mc_virtual_rates_setpoint_public, data, sizeof(mc_virtual_rates_setpoint_s));
    uORB::ORB_mc_virtual_rates_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"fw_virtual_attitude_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_fw_virtual_attitude_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_fw_virtual_attitude_setpoint_public, data, sizeof(fw_virtual_attitude_setpoint_s));
    uORB::ORB_fw_virtual_attitude_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"fw_virtual_rates_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_fw_virtual_rates_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_fw_virtual_rates_setpoint_public, data, sizeof(fw_virtual_rates_setpoint_s));
    uORB::ORB_fw_virtual_rates_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_status_flags") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_status_flags_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_status_flags_public, data, sizeof(vehicle_status_flags_s));
    uORB::ORB_vehicle_status_flags_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"rc_parameter_map") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_rc_parameter_map_public.updated = false;
    MEMCPY(&uORB::ORB_rc_parameter_map_public, data, sizeof(rc_parameter_map_s));
    uORB::ORB_rc_parameter_map_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"transponder_report") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_transponder_report_public.updated = false;
    MEMCPY(&uORB::ORB_transponder_report_public, data, sizeof(transponder_report_s));
    uORB::ORB_transponder_report_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"collision_report") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_collision_report_public.updated = false;
    MEMCPY(&uORB::ORB_collision_report_public, data, sizeof(collision_report_s));
    uORB::ORB_collision_report_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"camera_trigger") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_camera_trigger_public.updated = false;
    MEMCPY(&uORB::ORB_camera_trigger_public, data, sizeof(camera_trigger_s));
    uORB::ORB_camera_trigger_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"mount_orientation") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_mount_orientation_public.updated = false;
    MEMCPY(&uORB::ORB_mount_orientation_public, data, sizeof(mount_orientation_s));
    uORB::ORB_mount_orientation_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"input_rc") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_input_rc_public.updated = false;
    MEMCPY(&uORB::ORB_input_rc_public, data, sizeof(input_rc_s));
    uORB::ORB_input_rc_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"vehicle_force_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_vehicle_force_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_force_setpoint_public, data, sizeof(vehicle_force_setpoint_s));
    uORB::ORB_vehicle_force_setpoint_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"time_offset") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_time_offset_public.updated = false;
    MEMCPY(&uORB::ORB_time_offset_public, data, sizeof(time_offset_s));
    uORB::ORB_time_offset_public.updated = true;
  }
  else if(0 == std::strcmp(orb_source_name,"gps_inject_data") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    uORB::ORB_gps_inject_data_public.updated = false;
    MEMCPY(&uORB::ORB_gps_inject_data_public, data, sizeof(gps_inject_data_s));
    uORB::ORB_gps_inject_data_public.updated = true;
  }
  else
      return -1;
  return 0;
}








int orb_publish_multi(char* orb_source_name, orb_advert_t pub_handle, void* data, ORB_PRIORITY priority)
{
  if(0 == std::strcmp(orb_source_name, "parameter_update") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_parameter_update_public.current_prio <= priority))
  {
    uORB::ORB_parameter_update_public.updated = false;
    MEMCPY(&uORB::ORB_parameter_update_public, data, sizeof(parameter_update_s));
    uORB::ORB_parameter_update_public.updated = true;
    uORB::ORB_parameter_update_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_attitude_control") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_attitude_control_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_attitude_control_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_attitude_control_public, data, sizeof(vehicle_attitude_control_s));
    uORB::ORB_vehicle_attitude_control_public.updated = true;
    uORB::ORB_vehicle_attitude_control_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "actuator_armed") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_actuator_armed_public.current_prio <= priority))
  {
    uORB::ORB_actuator_armed_public.updated = false;
    MEMCPY(&uORB::ORB_actuator_armed_public, data, sizeof(actuator_armed_s));
    uORB::ORB_actuator_armed_public.updated = true;
    uORB::ORB_actuator_armed_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "sensor_combined") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    switch(priority)
    {
      case      ORB_PRIO_DEFAULT:
        
                uORB::ORB_sensor_combined_public[0].updated = false;
                MEMCPY(&uORB::ORB_sensor_combined_public[0], data, sizeof(sensor_combined_s));
                uORB::ORB_sensor_combined_public[0].updated = true;
                uORB::ORB_sensor_combined_public[0].current_prio = priority;
                
                break;
      case      ORB_PRIO_MAX:
                
                uORB::ORB_sensor_combined_public[1].updated = false;
                MEMCPY(&uORB::ORB_sensor_combined_public[1], data, sizeof(sensor_combined_s));
                uORB::ORB_sensor_combined_public[1].updated = true;
                uORB::ORB_sensor_combined_public[1].current_prio = priority;
                
                break;
      default:
                break;
    }
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_attitude") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
        
                uORB::ORB_vehicle_attitude_public[0].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[0], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[0].updated = true;
                uORB::ORB_vehicle_attitude_public[0].current_prio = priority;
                
                break;
      case      ORB_PRIO_VERY_LOW:
        
                uORB::ORB_vehicle_attitude_public[1].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[1], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[1].updated = true;
                uORB::ORB_vehicle_attitude_public[1].current_prio = priority;
                
                break;
      case      ORB_PRIO_LOW:
        
                uORB::ORB_vehicle_attitude_public[2].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[2], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[2].updated = true;
                uORB::ORB_vehicle_attitude_public[2].current_prio = priority;
                
                break;
      case      ORB_PRIO_DEFAULT:
        
                uORB::ORB_vehicle_attitude_public[3].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[3], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[3].updated = true;
                uORB::ORB_vehicle_attitude_public[3].current_prio = priority;
                
                break;
      case      ORB_PRIO_ABOVE_DEFAULT:
        
                uORB::ORB_vehicle_attitude_public[4].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[4], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[4].updated = true;
                uORB::ORB_vehicle_attitude_public[4].current_prio = priority;
                
                break;
      case      ORB_PRIO_HIGH:
        
                uORB::ORB_vehicle_attitude_public[5].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[5], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[5].updated = true;
                uORB::ORB_vehicle_attitude_public[5].current_prio = priority;
                
                break;
      case      ORB_PRIO_VERY_HIGH:
        
                uORB::ORB_vehicle_attitude_public[6].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[6], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[6].updated = true;
                uORB::ORB_vehicle_attitude_public[6].current_prio = priority;
                
                break;          
      case      ORB_PRIO_MAX:
                
                uORB::ORB_vehicle_attitude_public[7].updated = false;
                MEMCPY(&uORB::ORB_vehicle_attitude_public[7], data, sizeof(vehicle_attitude_s));
                uORB::ORB_vehicle_attitude_public[7].updated = true;
                uORB::ORB_vehicle_attitude_public[7].current_prio = priority;
                
                break;
      default:
                break;
    }
  }
  else if(0 == std::strcmp(orb_source_name, "optical_flow") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_optical_flow_public.current_prio <= priority))
  {
    uORB::ORB_optical_flow_public.updated = false;
    MEMCPY(&uORB::ORB_optical_flow_public, data, sizeof(optical_flow_s));
    uORB::ORB_optical_flow_public.updated = true;
    uORB::ORB_optical_flow_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_gps_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_gps_position_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_gps_position_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_gps_position_public, data, sizeof(vehicle_gps_position_s));
    uORB::ORB_vehicle_gps_position_public.updated = true;
    uORB::ORB_vehicle_gps_position_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vision_position_estimate") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vision_position_estimate_public.current_prio <= priority))
  {
    uORB::ORB_vision_position_estimate_public.updated = false;
    MEMCPY(&uORB::ORB_vision_position_estimate_public, data, sizeof(vision_position_estimate_s));
    uORB::ORB_vision_position_estimate_public.updated = true;
    uORB::ORB_vision_position_estimate_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "att_pos_mocap") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_att_pos_mocap_public.current_prio <= priority))
  {
    uORB::ORB_att_pos_mocap_public.updated = false;
    MEMCPY(&uORB::ORB_att_pos_mocap_public, data, sizeof(att_pos_mocap_s));
    uORB::ORB_att_pos_mocap_public.updated = true;
    uORB::ORB_att_pos_mocap_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "distance_sensor") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
        
                uORB::ORB_distance_sensor_public[0].updated = false;
                MEMCPY(&uORB::ORB_distance_sensor_public[0], data, sizeof(distance_sensor_s));
                uORB::ORB_distance_sensor_public[0].updated = true;
                uORB::ORB_distance_sensor_public[0].current_prio = priority;
                
                break;
      case      ORB_PRIO_LOW:
        
                uORB::ORB_distance_sensor_public[1].updated = false;
                MEMCPY(&uORB::ORB_distance_sensor_public[1], data, sizeof(distance_sensor_s));
                uORB::ORB_distance_sensor_public[1].updated = true;
                uORB::ORB_distance_sensor_public[1].current_prio = priority;
                
                break;
      case      ORB_PRIO_DEFAULT:
        
                uORB::ORB_distance_sensor_public[2].updated = false;
                MEMCPY(&uORB::ORB_distance_sensor_public[2], data, sizeof(distance_sensor_s));
                uORB::ORB_distance_sensor_public[2].updated = true;
                uORB::ORB_distance_sensor_public[2].current_prio = priority;
                
                break;
      case      ORB_PRIO_HIGH:
        
                uORB::ORB_distance_sensor_public[3].updated = false;
                MEMCPY(&uORB::ORB_distance_sensor_public[3], data, sizeof(distance_sensor_s));
                uORB::ORB_distance_sensor_public[3].updated = true;
                uORB::ORB_distance_sensor_public[3].current_prio = priority;
                
                break;
      case      ORB_PRIO_MAX:
                
                uORB::ORB_distance_sensor_public[4].updated = false;
                MEMCPY(&uORB::ORB_distance_sensor_public[4], data, sizeof(distance_sensor_s));
                uORB::ORB_distance_sensor_public[4].updated = true;
                uORB::ORB_distance_sensor_public[4].current_prio = priority;
                
                break;
      default:
                break;
    }
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_rates_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_rates_setpoint_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_rates_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_rates_setpoint_public, data, sizeof(vehicle_rates_setpoint_s));
    uORB::ORB_vehicle_rates_setpoint_public.updated = true;
    uORB::ORB_vehicle_rates_setpoint_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "sensor_gyro") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
        
                uORB::ORB_sensor_gyro_public[0].updated = false;
                MEMCPY(&uORB::ORB_sensor_gyro_public[0], data, sizeof(sensor_gyro_s));
                uORB::ORB_sensor_gyro_public[0].updated = true;
                uORB::ORB_sensor_gyro_public[0].current_prio = priority;
                
                break;
      case      ORB_PRIO_LOW:
        
                uORB::ORB_sensor_gyro_public[1].updated = false;
                MEMCPY(&uORB::ORB_sensor_gyro_public[1], data, sizeof(sensor_gyro_s));
                uORB::ORB_sensor_gyro_public[1].updated = true;
                uORB::ORB_sensor_gyro_public[1].current_prio = priority;
                
                break;
      case      ORB_PRIO_DEFAULT:
        
                uORB::ORB_sensor_gyro_public[2].updated = false;
                MEMCPY(&uORB::ORB_sensor_gyro_public[2], data, sizeof(sensor_gyro_s));
                uORB::ORB_sensor_gyro_public[2].updated = true;
                uORB::ORB_sensor_gyro_public[2].current_prio = priority;
                
                break;
      case      ORB_PRIO_HIGH:
        
                uORB::ORB_sensor_gyro_public[3].updated = false;
                MEMCPY(&uORB::ORB_sensor_gyro_public[3], data, sizeof(sensor_gyro_s));
                uORB::ORB_sensor_gyro_public[3].updated = true;
                uORB::ORB_sensor_gyro_public[3].current_prio = priority;
                
                break;
      case      ORB_PRIO_MAX:
                
                uORB::ORB_sensor_gyro_public[4].updated = false;
                MEMCPY(&uORB::ORB_sensor_gyro_public[4], data, sizeof(sensor_gyro_s));
                uORB::ORB_sensor_gyro_public[4].updated = true;
                uORB::ORB_sensor_gyro_public[4].current_prio = priority;
                
                break;
      default:
                break;
    }
  }
  else if(0 == std::strcmp(orb_source_name, "sensor_accel") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
        
                uORB::ORB_sensor_accel_public[0].updated = false;
                MEMCPY(&uORB::ORB_sensor_accel_public[0], data, sizeof(sensor_accel_s));
                uORB::ORB_sensor_accel_public[0].updated = true;
                uORB::ORB_sensor_accel_public[0].current_prio = priority;
                
                break;
      case      ORB_PRIO_LOW:
        
                uORB::ORB_sensor_accel_public[1].updated = false;
                MEMCPY(&uORB::ORB_sensor_accel_public[1], data, sizeof(sensor_accel_s));
                uORB::ORB_sensor_accel_public[1].updated = true;
                uORB::ORB_sensor_accel_public[1].current_prio = priority;
                
                break;
      case      ORB_PRIO_DEFAULT:
        
                uORB::ORB_sensor_accel_public[2].updated = false;
                MEMCPY(&uORB::ORB_sensor_accel_public[2], data, sizeof(sensor_accel_s));
                uORB::ORB_sensor_accel_public[2].updated = true;
                uORB::ORB_sensor_accel_public[2].current_prio = priority;
                
                break;
      case      ORB_PRIO_HIGH:
        
                uORB::ORB_sensor_accel_public[3].updated = false;
                MEMCPY(&uORB::ORB_sensor_accel_public[3], data, sizeof(sensor_accel_s));
                uORB::ORB_sensor_accel_public[3].updated = true;
                uORB::ORB_sensor_accel_public[3].current_prio = priority;
                
                break;
      case      ORB_PRIO_MAX:
                
                uORB::ORB_sensor_accel_public[4].updated = false;
                MEMCPY(&uORB::ORB_sensor_accel_public[4], data, sizeof(sensor_accel_s));
                uORB::ORB_sensor_accel_public[4].updated = true;
                uORB::ORB_sensor_accel_public[4].current_prio = priority;
                
                break;
      default:
                break;
    }
  }
  else if(0 == std::strcmp(orb_source_name, "sensor_mag") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    switch(priority)
    {
      case      ORB_PRIO_MIN:
        
                uORB::ORB_sensor_mag_public[0].updated = false;
                MEMCPY(&uORB::ORB_sensor_mag_public[0], data, sizeof(sensor_mag_s));
                uORB::ORB_sensor_mag_public[0].updated = true;
                uORB::ORB_sensor_mag_public[0].current_prio = priority;
                
                break;
      case      ORB_PRIO_LOW:
        
                uORB::ORB_sensor_mag_public[1].updated = false;
                MEMCPY(&uORB::ORB_sensor_mag_public[1], data, sizeof(sensor_mag_s));
                uORB::ORB_sensor_mag_public[1].updated = true;
                uORB::ORB_sensor_mag_public[1].current_prio = priority;
                
                break;
      case      ORB_PRIO_DEFAULT:
        
                uORB::ORB_sensor_mag_public[2].updated = false;
                MEMCPY(&uORB::ORB_sensor_mag_public[2], data, sizeof(sensor_mag_s));
                uORB::ORB_sensor_mag_public[2].updated = true;
                uORB::ORB_sensor_mag_public[2].current_prio = priority;
                
                break;
      case      ORB_PRIO_HIGH:
        
                uORB::ORB_sensor_mag_public[3].updated = false;
                MEMCPY(&uORB::ORB_sensor_mag_public[3], data, sizeof(sensor_mag_s));
                uORB::ORB_sensor_mag_public[3].updated = true;
                uORB::ORB_sensor_mag_public[3].current_prio = priority;
                
                break;
      case      ORB_PRIO_MAX:
                
                uORB::ORB_sensor_mag_public[4].updated = false;
                MEMCPY(&uORB::ORB_sensor_mag_public[4], data, sizeof(sensor_mag_s));
                uORB::ORB_sensor_mag_public[4].updated = true;
                uORB::ORB_sensor_mag_public[4].current_prio = priority;
                
                break;
      default:
                break;
    }
  }
  else if(0 == std::strcmp(orb_source_name, "sensor_baro") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name))
  {
    switch(priority)
    {
      case      ORB_PRIO_DEFAULT:
        
                uORB::ORB_sensor_baro_public[0].updated = false;
                MEMCPY(&uORB::ORB_sensor_baro_public[0], data, sizeof(sensor_baro_s));
                uORB::ORB_sensor_baro_public[0].updated = true;
                uORB::ORB_sensor_baro_public[0].current_prio = priority;
                
                break;
      case      ORB_PRIO_MAX:
                
                uORB::ORB_sensor_baro_public[1].updated = false;
                MEMCPY(&uORB::ORB_sensor_baro_public[1], data, sizeof(sensor_baro_s));
                uORB::ORB_sensor_baro_public[1].updated = true;
                uORB::ORB_sensor_baro_public[1].current_prio = priority;
                
                break;
      default:
                break;
    }
  }
  else if(0 == std::strcmp(orb_source_name, "airspeed") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_airspeed_public.current_prio <= priority))
  {
    uORB::ORB_airspeed_public.updated = false;
    MEMCPY(&uORB::ORB_airspeed_public, data, sizeof(airspeed_s));
    uORB::ORB_airspeed_public.updated = true;
    uORB::ORB_airspeed_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_land_detected") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_land_detected_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_land_detected_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_land_detected_public, data, sizeof(vehicle_land_detected_s));
    uORB::ORB_vehicle_land_detected_public.updated = true;
    uORB::ORB_vehicle_land_detected_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_status_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_status_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_status_public, data, sizeof(vehicle_status_s));
    uORB::ORB_vehicle_status_public.updated = true;
    uORB::ORB_vehicle_status_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_control_mode") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_control_mode_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_control_mode_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_control_mode_public, data, sizeof(vehicle_control_mode_s));
    uORB::ORB_vehicle_control_mode_public.updated = true;
    uORB::ORB_vehicle_control_mode_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_attitude_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_attitude_setpoint_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_attitude_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_attitude_setpoint_public, data, sizeof(vehicle_attitude_setpoint_s));
    uORB::ORB_vehicle_attitude_setpoint_public.updated = true;
    uORB::ORB_vehicle_attitude_setpoint_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "actuator_controls") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_actuator_controls_public.current_prio <= priority))
  {
    uORB::ORB_actuator_controls_public.updated = false;
    MEMCPY(&uORB::ORB_actuator_controls_public, data, sizeof(actuator_controls_s));
    uORB::ORB_actuator_controls_public.updated = true;
    uORB::ORB_actuator_controls_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_global_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_global_position_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_global_position_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_global_position_public, data, sizeof(vehicle_global_position_s));
    uORB::ORB_vehicle_global_position_public.updated = true;
    uORB::ORB_vehicle_global_position_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_local_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_local_position_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_local_position_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_local_position_public, data, sizeof(vehicle_local_position_s));
    uORB::ORB_vehicle_local_position_public.updated = true;
    uORB::ORB_vehicle_local_position_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "control_state") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_control_state_public.current_prio <= priority))
  {
    uORB::ORB_control_state_public.updated = false;
    MEMCPY(&uORB::ORB_control_state_public, data, sizeof(control_state_s));
    uORB::ORB_control_state_public.updated = true;
    uORB::ORB_control_state_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "manual_control_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_manual_control_setpoint_public.current_prio <= priority))
  {
    uORB::ORB_manual_control_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_manual_control_setpoint_public, data, sizeof(manual_control_setpoint_s));
    uORB::ORB_manual_control_setpoint_public.updated = true;
    uORB::ORB_manual_control_setpoint_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "multirotor_motor_limits") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_multirotor_motor_limits_public.current_prio <= priority))
  {
    uORB::ORB_multirotor_motor_limits_public.updated = false;
    MEMCPY(&uORB::ORB_multirotor_motor_limits_public, data, sizeof(multirotor_motor_limits_s));
    uORB::ORB_multirotor_motor_limits_public.updated = true;
    uORB::ORB_multirotor_motor_limits_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "mc_att_ctrl_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_mc_att_ctrl_status_public.current_prio <= priority))
  {
    uORB::ORB_mc_att_ctrl_status_public.updated = false;
    MEMCPY(&uORB::ORB_mc_att_ctrl_status_public, data, sizeof(mc_att_ctrl_status_s));
    uORB::ORB_mc_att_ctrl_status_public.updated = true;
    uORB::ORB_mc_att_ctrl_status_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "estimator_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_estimator_status_public.current_prio <= priority))
  {
    uORB::ORB_estimator_status_public.updated = false;
    MEMCPY(&uORB::ORB_estimator_status_public, data, sizeof(estimator_status_s));
    uORB::ORB_estimator_status_public.updated = true;
    uORB::ORB_estimator_status_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "wind_estimate") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_wind_estimate_public.current_prio <= priority))
  {
    uORB::ORB_wind_estimate_public.updated = false;
    MEMCPY(&uORB::ORB_wind_estimate_public, data, sizeof(wind_estimate_s));
    uORB::ORB_wind_estimate_public.updated = true;
    uORB::ORB_wind_estimate_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "ekf2_innovations") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_ekf2_innovations_public.current_prio <= priority))
  {
    uORB::ORB_ekf2_innovations_public.updated = false;
    MEMCPY(&uORB::ORB_ekf2_innovations_public, data, sizeof(ekf2_innovations_s));
    uORB::ORB_ekf2_innovations_public.updated = true;
    uORB::ORB_ekf2_innovations_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "position_setpoint_triplet") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_position_setpoint_triplet_public.current_prio <= priority))
  {
    uORB::ORB_position_setpoint_triplet_public.updated = false;
    MEMCPY(&uORB::ORB_position_setpoint_triplet_public, data, sizeof(position_setpoint_triplet_s));
    uORB::ORB_position_setpoint_triplet_public.updated = true;
    uORB::ORB_position_setpoint_triplet_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_local_position_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_local_position_setpoint_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_local_position_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_local_position_setpoint_public, data, sizeof(vehicle_local_position_setpoint_s));
    uORB::ORB_vehicle_local_position_setpoint_public.updated = true;
    uORB::ORB_vehicle_local_position_setpoint_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_global_velocity_setpoint") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_global_velocity_setpoint_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_global_velocity_setpoint_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_global_velocity_setpoint_public, data, sizeof(vehicle_global_velocity_setpoint_s));
    uORB::ORB_vehicle_global_velocity_setpoint_public.updated = true;
    uORB::ORB_vehicle_global_velocity_setpoint_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "follow_target") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_follow_target_public.current_prio <= priority))
  {
    uORB::ORB_follow_target_public.updated = false;
    MEMCPY(&uORB::ORB_follow_target_public, data, sizeof(follow_target_s));
    uORB::ORB_follow_target_public.updated = true;
    uORB::ORB_follow_target_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "home_position") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_home_position_public.current_prio <= priority))
  {
    uORB::ORB_home_position_public.updated = false;
    MEMCPY(&uORB::ORB_home_position_public, data, sizeof(home_position_s));
    uORB::ORB_home_position_public.updated = true;
    uORB::ORB_home_position_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "mission") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_mission_public.current_prio <= priority))
  {
    uORB::ORB_mission_public.updated = false;
    MEMCPY(&uORB::ORB_mission_public, data, sizeof(mission_s));
    uORB::ORB_mission_public.updated = true;
    uORB::ORB_mission_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "fw_pos_ctrl_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_fw_pos_ctrl_status_public.current_prio <= priority))
  {
    uORB::ORB_fw_pos_ctrl_status_public.updated = false;
    MEMCPY(&uORB::ORB_fw_pos_ctrl_status_public, data, sizeof(fw_pos_ctrl_status_s));
    uORB::ORB_fw_pos_ctrl_status_public.updated = true;
    uORB::ORB_fw_pos_ctrl_status_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "fence_vertex") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_fence_vertex_public.current_prio <= priority))
  {
    uORB::ORB_fence_vertex_public.updated = false;
    MEMCPY(&uORB::ORB_fence_vertex_public, data, sizeof(fence_vertex_s));
    uORB::ORB_fence_vertex_public.updated = true;
    uORB::ORB_fence_vertex_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "fence") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_fence_public.current_prio <= priority))
  {
    uORB::ORB_fence_public.updated = false;
    MEMCPY(&uORB::ORB_fence_public, data, sizeof(fence_s));
    uORB::ORB_fence_public.updated = true;
    uORB::ORB_fence_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "mission_result") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_mission_result_public.current_prio <= priority))
  {
    uORB::ORB_mission_result_public.updated = false;
    MEMCPY(&uORB::ORB_mission_result_public, data, sizeof(mission_result_s));
    uORB::ORB_mission_result_public.updated = true;
    uORB::ORB_mission_result_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "geofence_result") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_geofence_result_public.current_prio <= priority))
  {
    uORB::ORB_geofence_result_public.updated = false;
    MEMCPY(&uORB::ORB_geofence_result_public, data, sizeof(geofence_result_s));
    uORB::ORB_geofence_result_public.updated = true;
    uORB::ORB_geofence_result_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_command") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_command_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_command_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_command_public, data, sizeof(vehicle_command_s));
    uORB::ORB_vehicle_command_public.updated = true;
    uORB::ORB_vehicle_command_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vtol_vehicle_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vtol_vehicle_status_public.current_prio <= priority))
  {
    uORB::ORB_vtol_vehicle_status_public.updated = false;
    MEMCPY(&uORB::ORB_vtol_vehicle_status_public, data, sizeof(vtol_vehicle_status_s));
    uORB::ORB_vtol_vehicle_status_public.updated = true;
    uORB::ORB_vtol_vehicle_status_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "battery_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_battery_status_public.current_prio <= priority))
  {
    uORB::ORB_battery_status_public.updated = false;
    MEMCPY(&uORB::ORB_battery_status_public, data, sizeof(battery_status_s));
    uORB::ORB_battery_status_public.updated = true;
    uORB::ORB_battery_status_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "safety") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_safety_public.current_prio <= priority))
  {
    uORB::ORB_safety_public.updated = false;
    MEMCPY(&uORB::ORB_safety_public, data, sizeof(safety_s));
    uORB::ORB_safety_public.updated = true;
    uORB::ORB_safety_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "offboard_control_mode") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_offboard_control_mode_public.current_prio <= priority))
  {
    uORB::ORB_offboard_control_mode_public.updated = false;
    MEMCPY(&uORB::ORB_offboard_control_mode_public, data, sizeof(offboard_control_mode_s));
    uORB::ORB_offboard_control_mode_public.updated = true;
    uORB::ORB_offboard_control_mode_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "commander_state") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_commander_state_public.current_prio <= priority))
  {
    uORB::ORB_commander_state_public.updated = false;
    MEMCPY(&uORB::ORB_commander_state_public, data, sizeof(commander_state_s));
    uORB::ORB_commander_state_public.updated = true;
    uORB::ORB_commander_state_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "cpuload") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_cpuload_public.current_prio <= priority))
  {
    uORB::ORB_cpuload_public.updated = false;
    MEMCPY(&uORB::ORB_cpuload_public, data, sizeof(cpuload_s));
    uORB::ORB_cpuload_public.updated = true;
    uORB::ORB_cpuload_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "vehicle_command_ack") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_vehicle_command_ack_public.current_prio <= priority))
  {
    uORB::ORB_vehicle_command_ack_public.updated = false;
    MEMCPY(&uORB::ORB_vehicle_command_ack_public, data, sizeof(vehicle_command_ack_s));
    uORB::ORB_vehicle_command_ack_public.updated = true;
    uORB::ORB_vehicle_command_ack_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "differential_pressure") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_differential_pressure_public.current_prio <= priority))
  {
    uORB::ORB_differential_pressure_public.updated = false;
    MEMCPY(&uORB::ORB_differential_pressure_public, data, sizeof(differential_pressure_s));
    uORB::ORB_differential_pressure_public.updated = true;
    uORB::ORB_differential_pressure_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "telemetry_status") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_telemetry_status_public.current_prio <= priority))
  {
    uORB::ORB_telemetry_status_public.updated = false;
    MEMCPY(&uORB::ORB_telemetry_status_public, data, sizeof(telemetry_status_s));
    uORB::ORB_telemetry_status_public.updated = true;
    uORB::ORB_telemetry_status_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "subsystem_info") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_subsystem_info_public.current_prio <= priority))
  {
    uORB::ORB_subsystem_info_public.updated = false;
    MEMCPY(&uORB::ORB_subsystem_info_public, data, sizeof(subsystem_info_s));
    uORB::ORB_subsystem_info_public.updated = true;
    uORB::ORB_subsystem_info_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "system_power") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_system_power_public.current_prio <= priority))
  {
    uORB::ORB_system_power_public.updated = false;
    MEMCPY(&uORB::ORB_system_power_public, data, sizeof(system_power_s));
    uORB::ORB_system_power_public.updated = true;
    uORB::ORB_system_power_public.current_prio = priority;
  }
  else if(0 == std::strcmp(orb_source_name, "mavlink_log") && 0 == std::strcmp(pub_handle->orb_name, orb_source_name) && (uORB::ORB_mavlink_log_public.current_prio <= priority))
  {
    uORB::ORB_mavlink_log_public.updated = false;
    MEMCPY(&uORB::ORB_mavlink_log_public, data, sizeof(mavlink_log_s));
    uORB::ORB_mavlink_log_public.updated = true;
    uORB::ORB_mavlink_log_public.current_prio = priority;
  }
  else
    return -1;

  return 0;
}
