#include "uORB.h"
#include <cstring>
#pragma default_variable_attributes= @ ".ccram"
namespace uORB{
  parameter_update_s				ORB_parameter_update_public;
  vehicle_attitude_control_s				ORB_vehicle_attitude_control_public;
  actuator_armed_s				ORB_actuator_armed_public;
  sensor_combined_s				ORB_sensor_combined_public;
  sensor_combined_s				ORB_sensor_combined_slave_public;
  sensor_combined_s				ORB_sensor_combined_backup_public;
  vehicle_attitude_s				ORB_vehicle_attitude_public;
  vehicle_attitude_s				ORB_vehicle_attitude_slave_public;
  vehicle_attitude_s				ORB_vehicle_attitude_backup_public;
  vehicle_attitude_s				ORB_vehicle_attitude_emergency_public;
  optical_flow_s				ORB_optical_flow_public;
  vehicle_gps_position_s				ORB_vehicle_gps_position_public;
  vision_position_estimate_s				ORB_vision_position_estimate_public;
  att_pos_mocap_s				ORB_att_pos_mocap_public;
  distance_sensor_s				ORB_distance_sensor_public;
  range_finder_s				ORB_range_finder_public;
  sonar_sensor_s				ORB_sonar_sensor_public;
  vehicle_rates_setpoint_s				ORB_vehicle_rates_setpoint_public;
  sensor_gyro_s				ORB_sensor_gyro_public;
  sensor_gyro_s				ORB_sensor_gyro_slave_public;
  sensor_gyro_s				ORB_sensor_gyro_backup_public;
  sensor_accel_s				ORB_sensor_accel_public;
  sensor_accel_s				ORB_sensor_accel_slave_public;
  sensor_accel_s				ORB_sensor_accel_backup_public;
  sensor_mag_s				ORB_sensor_mag_public;
  sensor_mag_s				ORB_sensor_mag_slave_public;
  sensor_mag_s				ORB_sensor_mag_backup_public;
  sensor_baro_s				ORB_sensor_baro_public;
  sensor_baro_s				ORB_sensor_baro_slave_public;
  sensor_baro_s				ORB_sensor_baro_backup_public;
  airspeed_s				ORB_airspeed_public;
  vehicle_land_detected_s				ORB_vehicle_land_detected_public;
  vehicle_status_s				ORB_vehicle_status_public;
  vehicle_control_mode_s				ORB_vehicle_control_mode_public;
  vehicle_attitude_setpoint_s				ORB_vehicle_attitude_setpoint_public;
  actuator_controls_s				ORB_actuator_controls_public;
  vehicle_global_position_s				ORB_vehicle_global_position_public;
  vehicle_local_position_s				ORB_vehicle_local_position_public;
  vehicle_global_position_s				ORB_vehicle_global_position_slave_public;
  vehicle_local_position_s				ORB_vehicle_local_position_slave_public;
  vehicle_global_position_s				ORB_vehicle_global_position_backup_public;
  vehicle_local_position_s				ORB_vehicle_local_position_backup_public;
  control_state_s				ORB_control_state_public;
  control_state_s				ORB_control_state_slave_public;
  control_state_s				ORB_control_state_backup_public;
  manual_control_setpoint_s				ORB_manual_control_setpoint_public;
  multirotor_motor_limits_s				ORB_multirotor_motor_limits_public;
  mc_att_ctrl_status_s				ORB_mc_att_ctrl_status_public;
  estimator_status_s				ORB_estimator_status_public;
  wind_estimate_s				ORB_wind_estimate_public;
  ekf2_innovations_s				ORB_ekf2_innovations_public;
  position_setpoint_triplet_s				ORB_position_setpoint_triplet_public;
  vehicle_local_position_setpoint_s				ORB_vehicle_local_position_setpoint_public;
  vehicle_global_velocity_setpoint_s				ORB_vehicle_global_velocity_setpoint_public;
  follow_target_s				ORB_follow_target_public;
  home_position_s				ORB_home_position_public;
  mission_s				ORB_mission_public;
  fw_pos_ctrl_status_s				ORB_fw_pos_ctrl_status_public;
  fence_vertex_s				ORB_fence_vertex_public;
  fence_s				ORB_fence_public;
  mission_result_s				ORB_mission_result_public;
  geofence_result_s				ORB_geofence_result_public;
  vehicle_command_s				ORB_vehicle_command_public;
  vtol_vehicle_status_s				ORB_vtol_vehicle_status_public;
  battery_status_s				ORB_battery_status_public;
  safety_s				ORB_safety_public;
  offboard_control_mode_s				ORB_offboard_control_mode_public;
  commander_state_s				ORB_commander_state_public;
  cpuload_s				ORB_cpuload_public;
  vehicle_command_ack_s				ORB_vehicle_command_ack_public;
  differential_pressure_s				ORB_differential_pressure_public;
  telemetry_status_s				ORB_telemetry_status_public;
  subsystem_info_s				ORB_subsystem_info_public;
  system_power_s				ORB_system_power_public;
  mavlink_log0_s				ORB_mavlink_log0_public;
  mavlink_log1_s				ORB_mavlink_log1_public;
  mavlink_log2_s				ORB_mavlink_log2_public;
  mavlink_log3_s				ORB_mavlink_log3_public;
  mavlink_log4_s				ORB_mavlink_log4_public;
orb_advert adv;
}
#pragma default_variable_attributes=

void orb_init(void)
{
  memset(&uORB::ORB_parameter_update_public,0,sizeof(parameter_update_s));
  memset(&uORB::ORB_vehicle_attitude_control_public,0,sizeof(vehicle_attitude_control_s));
  memset(&uORB::ORB_actuator_armed_public,0,sizeof(actuator_armed_s));
  memset(&uORB::ORB_sensor_combined_public,0,sizeof(sensor_combined_s));
  memset(&uORB::ORB_sensor_combined_slave_public,0,sizeof(sensor_combined_s));
  memset(&uORB::ORB_sensor_combined_backup_public,0,sizeof(sensor_combined_s));
  memset(&uORB::ORB_vehicle_attitude_public,0,sizeof(vehicle_attitude_s));
  memset(&uORB::ORB_vehicle_attitude_slave_public,0,sizeof(vehicle_attitude_s));
  memset(&uORB::ORB_vehicle_attitude_backup_public,0,sizeof(vehicle_attitude_s));
  memset(&uORB::ORB_vehicle_attitude_emergency_public,0,sizeof(vehicle_attitude_s));
  memset(&uORB::ORB_optical_flow_public,0,sizeof(optical_flow_s));
  memset(&uORB::ORB_vehicle_gps_position_public,0,sizeof(vehicle_gps_position_s));
  memset(&uORB::ORB_vision_position_estimate_public,0,sizeof(vision_position_estimate_s));
  memset(&uORB::ORB_att_pos_mocap_public,0,sizeof(att_pos_mocap_s));
  memset(&uORB::ORB_distance_sensor_public,0,sizeof(distance_sensor_s));
  memset(&uORB::ORB_range_finder_public,0,sizeof(range_finder_s));
  memset(&uORB::ORB_sonar_sensor_public,0,sizeof(sonar_sensor_s));
  memset(&uORB::ORB_vehicle_rates_setpoint_public,0,sizeof(vehicle_rates_setpoint_s));
  memset(&uORB::ORB_sensor_gyro_public,0,sizeof(sensor_gyro_s));
  memset(&uORB::ORB_sensor_gyro_slave_public,0,sizeof(sensor_gyro_s));
  memset(&uORB::ORB_sensor_gyro_backup_public,0,sizeof(sensor_gyro_s));
  memset(&uORB::ORB_sensor_accel_public,0,sizeof(sensor_accel_s));
  memset(&uORB::ORB_sensor_accel_slave_public,0,sizeof(sensor_accel_s));
  memset(&uORB::ORB_sensor_accel_backup_public,0,sizeof(sensor_accel_s));
  memset(&uORB::ORB_sensor_mag_public,0,sizeof(sensor_mag_s));
  memset(&uORB::ORB_sensor_mag_slave_public,0,sizeof(sensor_mag_s));
  memset(&uORB::ORB_sensor_mag_backup_public,0,sizeof(sensor_mag_s));
  memset(&uORB::ORB_sensor_baro_public,0,sizeof(sensor_baro_s));
  memset(&uORB::ORB_sensor_baro_slave_public,0,sizeof(sensor_baro_s));
  memset(&uORB::ORB_sensor_baro_backup_public,0,sizeof(sensor_baro_s));
  memset(&uORB::ORB_airspeed_public,0,sizeof(airspeed_s));
  memset(&uORB::ORB_vehicle_land_detected_public,0,sizeof(vehicle_land_detected_s));
  memset(&uORB::ORB_vehicle_status_public,0,sizeof(vehicle_status_s));
  memset(&uORB::ORB_vehicle_control_mode_public,0,sizeof(vehicle_control_mode_s));
  memset(&uORB::ORB_vehicle_attitude_setpoint_public,0,sizeof(vehicle_attitude_setpoint_s));
  memset(&uORB::ORB_actuator_controls_public,0,sizeof(actuator_controls_s));
  memset(&uORB::ORB_vehicle_global_position_public,0,sizeof(vehicle_global_position_s));
  memset(&uORB::ORB_vehicle_local_position_public,0,sizeof(vehicle_local_position_s));
  memset(&uORB::ORB_vehicle_global_position_slave_public,0,sizeof(vehicle_global_position_s));
  memset(&uORB::ORB_vehicle_local_position_slave_public,0,sizeof(vehicle_local_position_s));
  memset(&uORB::ORB_vehicle_global_position_backup_public,0,sizeof(vehicle_global_position_s));
  memset(&uORB::ORB_vehicle_local_position_backup_public,0,sizeof(vehicle_local_position_s));
  memset(&uORB::ORB_control_state_public,0,sizeof(control_state_s));
  memset(&uORB::ORB_control_state_slave_public,0,sizeof(control_state_s));
  memset(&uORB::ORB_control_state_backup_public,0,sizeof(control_state_s));
  memset(&uORB::ORB_manual_control_setpoint_public,0,sizeof(manual_control_setpoint_s));
  memset(&uORB::ORB_multirotor_motor_limits_public,0,sizeof(multirotor_motor_limits_s));
  memset(&uORB::ORB_mc_att_ctrl_status_public,0,sizeof(mc_att_ctrl_status_s));
  memset(&uORB::ORB_estimator_status_public,0,sizeof(estimator_status_s));
  memset(&uORB::ORB_wind_estimate_public,0,sizeof(wind_estimate_s));
  memset(&uORB::ORB_ekf2_innovations_public,0,sizeof(ekf2_innovations_s));
  memset(&uORB::ORB_position_setpoint_triplet_public,0,sizeof(position_setpoint_triplet_s));
  memset(&uORB::ORB_vehicle_local_position_setpoint_public,0,sizeof(vehicle_local_position_setpoint_s));
  memset(&uORB::ORB_vehicle_global_velocity_setpoint_public,0,sizeof(vehicle_global_velocity_setpoint_s));
  memset(&uORB::ORB_follow_target_public,0,sizeof(follow_target_s));
  memset(&uORB::ORB_home_position_public,0,sizeof(home_position_s));
  memset(&uORB::ORB_mission_public,0,sizeof(mission_s));
  memset(&uORB::ORB_fw_pos_ctrl_status_public,0,sizeof(fw_pos_ctrl_status_s));
  memset(&uORB::ORB_fence_vertex_public,0,sizeof(fence_vertex_s));
  memset(&uORB::ORB_fence_public,0,sizeof(fence_s));
  memset(&uORB::ORB_mission_result_public,0,sizeof(mission_result_s));
  memset(&uORB::ORB_geofence_result_public,0,sizeof(geofence_result_s));
  memset(&uORB::ORB_vehicle_command_public,0,sizeof(vehicle_command_s));
  memset(&uORB::ORB_vtol_vehicle_status_public,0,sizeof(vtol_vehicle_status_s));
  memset(&uORB::ORB_battery_status_public,0,sizeof(battery_status_s));
  memset(&uORB::ORB_safety_public,0,sizeof(safety_s));
  memset(&uORB::ORB_offboard_control_mode_public,0,sizeof(offboard_control_mode_s));
  memset(&uORB::ORB_commander_state_public,0,sizeof(commander_state_s));
  memset(&uORB::ORB_cpuload_public,0,sizeof(cpuload_s));
  memset(&uORB::ORB_vehicle_command_ack_public,0,sizeof(vehicle_command_ack_s));
  memset(&uORB::ORB_differential_pressure_public,0,sizeof(differential_pressure_s));
  memset(&uORB::ORB_telemetry_status_public,0,sizeof(telemetry_status_s));
  memset(&uORB::ORB_subsystem_info_public,0,sizeof(subsystem_info_s));
  memset(&uORB::ORB_system_power_public,0,sizeof(system_power_s));
  memset(&uORB::ORB_mavlink_log0_public,0,sizeof(mavlink_log0_s));
  memset(&uORB::ORB_mavlink_log1_public,0,sizeof(mavlink_log1_s));
  memset(&uORB::ORB_mavlink_log2_public,0,sizeof(mavlink_log2_s));
  memset(&uORB::ORB_mavlink_log3_public,0,sizeof(mavlink_log3_s));
  memset(&uORB::ORB_mavlink_log4_public,0,sizeof(mavlink_log4_s));
}


int orb_subscribe(char* orb_name)
{
  if(!strcmp(orb_name, "parameter_update"))
  {
    return(uORB::ORB_parameter_update);
  }
  else if(!strcmp(orb_name, "vehicle_attitude_control"))
  {
    return(uORB::ORB_vehicle_attitude_control);
  }
  else if(!strcmp(orb_name, "actuator_armed"))
  {
    return(uORB::ORB_actuator_armed);
  }
  else if(!strcmp(orb_name, "sensor_combined"))
  {
    return(uORB::ORB_sensor_combined);
  }
  else if(!strcmp(orb_name, "sensor_combined_slave"))
  {
    return(uORB::ORB_sensor_combined_slave);
  }
  else if(!strcmp(orb_name, "sensor_combined_backup"))
  {
    return(uORB::ORB_sensor_combined_backup);
  }
  else if(!strcmp(orb_name, "vehicle_attitude"))
  {
    return(uORB::ORB_vehicle_attitude);
  }
  else if(!strcmp(orb_name, "vehicle_attitude_slave"))
  {
    return(uORB::ORB_vehicle_attitude_slave);
  }
  else if(!strcmp(orb_name, "vehicle_attitude_backup"))
  {
    return(uORB::ORB_vehicle_attitude_backup);
  }
  else if(!strcmp(orb_name, "vehicle_attitude_emergency"))
  {
    return(uORB::ORB_vehicle_attitude_emergency);
  }
  else if(!strcmp(orb_name, "optical_flow"))
  {
    return(uORB::ORB_optical_flow);
  }
  else if(!strcmp(orb_name, "vehicle_gps_position"))
  {
    return(uORB::ORB_vehicle_gps_position);
  }
  else if(!strcmp(orb_name, "vision_position_estimate"))
  {
    return(uORB::ORB_vision_position_estimate);
  }
  else if(!strcmp(orb_name, "att_pos_mocap"))
  {
    return(uORB::ORB_att_pos_mocap);
  }
  else if(!strcmp(orb_name, "distance_sensor"))
  {
    return(uORB::ORB_distance_sensor);
  }
  else if(!strcmp(orb_name, "range_finder"))
  {
    return(uORB::ORB_range_finder);
  }
  else if(!strcmp(orb_name, "sonar_sensor"))
  {
    return(uORB::ORB_sonar_sensor);
  }
  else if(!strcmp(orb_name, "vehicle_rates_setpoint"))
  {
    return(uORB::ORB_vehicle_rates_setpoint);
  }
  else if(!strcmp(orb_name, "sensor_gyro"))
  {
    return(uORB::ORB_sensor_gyro);
  }
  else if(!strcmp(orb_name, "sensor_gyro_slave"))
  {
    return(uORB::ORB_sensor_gyro_slave);
  }
  else if(!strcmp(orb_name, "sensor_gyro_backup"))
  {
    return(uORB::ORB_sensor_gyro_backup);
  }
  else if(!strcmp(orb_name, "sensor_accel"))
  {
    return(uORB::ORB_sensor_accel);
  }
  else if(!strcmp(orb_name, "sensor_accel_slave"))
  {
    return(uORB::ORB_sensor_accel_slave);
  }
  else if(!strcmp(orb_name, "sensor_accel_backup"))
  {
    return(uORB::ORB_sensor_accel_backup);
  }
  else if(!strcmp(orb_name, "sensor_mag"))
  {
    return(uORB::ORB_sensor_mag);
  }
  else if(!strcmp(orb_name, "sensor_mag_slave"))
  {
    return(uORB::ORB_sensor_mag_slave);
  }
  else if(!strcmp(orb_name, "sensor_mag_backup"))
  {
    return(uORB::ORB_sensor_mag_backup);
  }
  else if(!strcmp(orb_name, "sensor_baro"))
  {
    return(uORB::ORB_sensor_baro);
  }
  else if(!strcmp(orb_name, "sensor_baro_slave"))
  {
    return(uORB::ORB_sensor_baro_slave);
  }
  else if(!strcmp(orb_name, "sensor_baro_backup"))
  {
    return(uORB::ORB_sensor_baro_backup);
  }
  else if(!strcmp(orb_name, "airspeed"))
  {
    return(uORB::ORB_airspeed);
  }
  else if(!strcmp(orb_name, "vehicle_land_detected"))
  {
    return(uORB::ORB_vehicle_land_detected);
  }
  else if(!strcmp(orb_name, "vehicle_status"))
  {
    return(uORB::ORB_vehicle_status);
  }
  else if(!strcmp(orb_name, "vehicle_control_mode"))
  {
    return(uORB::ORB_vehicle_control_mode);
  }
  else if(!strcmp(orb_name, "vehicle_attitude_setpoint"))
  {
    return(uORB::ORB_vehicle_attitude_setpoint);
  }
  else if(!strcmp(orb_name, "actuator_controls"))
  {
    return(uORB::ORB_actuator_controls);
  }
  else if(!strcmp(orb_name, "vehicle_global_position"))
  {
    return(uORB::ORB_vehicle_global_position);
  }
  else if(!strcmp(orb_name, "vehicle_local_position"))
  {
    return(uORB::ORB_vehicle_local_position);
  }
  else if(!strcmp(orb_name, "vehicle_global_position_slave"))
  {
    return(uORB::ORB_vehicle_global_position_slave);
  }
  else if(!strcmp(orb_name, "vehicle_local_position_slave"))
  {
    return(uORB::ORB_vehicle_local_position_slave);
  }
  else if(!strcmp(orb_name, "vehicle_global_position_backup"))
  {
    return(uORB::ORB_vehicle_global_position_backup);
  }
  else if(!strcmp(orb_name, "vehicle_local_position_backup"))
  {
    return(uORB::ORB_vehicle_local_position_backup);
  }
  else if(!strcmp(orb_name, "control_state"))
  {
    return(uORB::ORB_control_state);
  }
  else if(!strcmp(orb_name, "control_state_slave"))
  {
    return(uORB::ORB_control_state_slave);
  }
  else if(!strcmp(orb_name, "control_state_backup"))
  {
    return(uORB::ORB_control_state_backup);
  }
  else if(!strcmp(orb_name, "manual_control_setpoint"))
  {
    return(uORB::ORB_manual_control_setpoint);
  }
  else if(!strcmp(orb_name, "multirotor_motor_limits"))
  {
    return(uORB::ORB_multirotor_motor_limits);
  }
  else if(!strcmp(orb_name, "mc_att_ctrl_status"))
  {
    return(uORB::ORB_mc_att_ctrl_status);
  }
  else if(!strcmp(orb_name, "estimator_status"))
  {
    return(uORB::ORB_estimator_status);
  }
  else if(!strcmp(orb_name, "wind_estimate"))
  {
    return(uORB::ORB_wind_estimate);
  }
  else if(!strcmp(orb_name, "ekf2_innovations"))
  {
    return(uORB::ORB_ekf2_innovations);
  }
  else if(!strcmp(orb_name, "position_setpoint_triplet"))
  {
    return(uORB::ORB_position_setpoint_triplet);
  }
  else if(!strcmp(orb_name, "vehicle_local_position_setpoint"))
  {
    return(uORB::ORB_vehicle_local_position_setpoint);
  }
  else if(!strcmp(orb_name, "vehicle_global_velocity_setpoint"))
  {
    return(uORB::ORB_vehicle_global_velocity_setpoint);
  }
  else if(!strcmp(orb_name, "follow_target"))
  {
    return(uORB::ORB_follow_target);
  }
  else if(!strcmp(orb_name, "home_position"))
  {
    return(uORB::ORB_home_position);
  }
  else if(!strcmp(orb_name, "mission"))
  {
    return(uORB::ORB_mission);
  }
  else if(!strcmp(orb_name, "fw_pos_ctrl_status"))
  {
    return(uORB::ORB_fw_pos_ctrl_status);
  }
  else if(!strcmp(orb_name, "fence_vertex"))
  {
    return(uORB::ORB_fence_vertex);
  }
  else if(!strcmp(orb_name, "fence"))
  {
    return(uORB::ORB_fence);
  }
  else if(!strcmp(orb_name, "mission_result"))
  {
    return(uORB::ORB_mission_result);
  }
  else if(!strcmp(orb_name, "geofence_result"))
  {
    return(uORB::ORB_geofence_result);
  }
  else if(!strcmp(orb_name, "vehicle_command"))
  {
    return(uORB::ORB_vehicle_command);
  }
  else if(!strcmp(orb_name, "vtol_vehicle_status"))
  {
    return(uORB::ORB_vtol_vehicle_status);
  }
  else if(!strcmp(orb_name, "battery_status"))
  {
    return(uORB::ORB_battery_status);
  }
  else if(!strcmp(orb_name, "safety"))
  {
    return(uORB::ORB_safety);
  }
  else if(!strcmp(orb_name, "offboard_control_mode"))
  {
    return(uORB::ORB_offboard_control_mode);
  }
  else if(!strcmp(orb_name, "commander_state"))
  {
    return(uORB::ORB_commander_state);
  }
  else if(!strcmp(orb_name, "cpuload"))
  {
    return(uORB::ORB_cpuload);
  }
  else if(!strcmp(orb_name, "vehicle_command_ack"))
  {
    return(uORB::ORB_vehicle_command_ack);
  }
  else if(!strcmp(orb_name, "differential_pressure"))
  {
    return(uORB::ORB_differential_pressure);
  }
  else if(!strcmp(orb_name, "telemetry_status"))
  {
    return(uORB::ORB_telemetry_status);
  }
  else if(!strcmp(orb_name, "subsystem_info"))
  {
    return(uORB::ORB_subsystem_info);
  }
  else if(!strcmp(orb_name, "system_power"))
  {
    return(uORB::ORB_system_power);
  }
  else if(!strcmp(orb_name, "mavlink_log0"))
  {
    return(uORB::ORB_mavlink_log0);
  }
  else if(!strcmp(orb_name, "mavlink_log1"))
  {
    return(uORB::ORB_mavlink_log1);
  }
  else if(!strcmp(orb_name, "mavlink_log2"))
  {
    return(uORB::ORB_mavlink_log2);
  }
  else if(!strcmp(orb_name, "mavlink_log3"))
  {
    return(uORB::ORB_mavlink_log3);
  }
  else if(!strcmp(orb_name, "mavlink_log4"))
  {
    return(uORB::ORB_mavlink_log4);
  }
  return(-1);
}


orb_advert_t orb_advertise(char* orb_name,void* data)
{
  if (!strcmp(orb_name, "parameter_update"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_parameter_update_public, data, sizeof(parameter_update_s));
    uORB::ORB_parameter_update_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_attitude_control"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_attitude_control_public, data, sizeof(vehicle_attitude_control_s));
    uORB::ORB_vehicle_attitude_control_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "actuator_armed"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_actuator_armed_public, data, sizeof(actuator_armed_s));
    uORB::ORB_actuator_armed_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_combined"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_combined_public, data, sizeof(sensor_combined_s));
    uORB::ORB_sensor_combined_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_combined_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_combined_slave_public, data, sizeof(sensor_combined_s));
    uORB::ORB_sensor_combined_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_combined_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_combined_backup_public, data, sizeof(sensor_combined_s));
    uORB::ORB_sensor_combined_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_attitude"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_attitude_public, data, sizeof(vehicle_attitude_s));
    uORB::ORB_vehicle_attitude_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_attitude_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_attitude_slave_public, data, sizeof(vehicle_attitude_s));
    uORB::ORB_vehicle_attitude_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_attitude_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_attitude_backup_public, data, sizeof(vehicle_attitude_s));
    uORB::ORB_vehicle_attitude_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_attitude_emergency"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_attitude_emergency_public, data, sizeof(vehicle_attitude_s));
    uORB::ORB_vehicle_attitude_emergency_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "optical_flow"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_optical_flow_public, data, sizeof(optical_flow_s));
    uORB::ORB_optical_flow_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_gps_position"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_gps_position_public, data, sizeof(vehicle_gps_position_s));
    uORB::ORB_vehicle_gps_position_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vision_position_estimate"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vision_position_estimate_public, data, sizeof(vision_position_estimate_s));
    uORB::ORB_vision_position_estimate_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "att_pos_mocap"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_att_pos_mocap_public, data, sizeof(att_pos_mocap_s));
    uORB::ORB_att_pos_mocap_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "distance_sensor"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_distance_sensor_public, data, sizeof(distance_sensor_s));
    uORB::ORB_distance_sensor_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "range_finder"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_range_finder_public, data, sizeof(range_finder_s));
    uORB::ORB_range_finder_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sonar_sensor"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sonar_sensor_public, data, sizeof(sonar_sensor_s));
    uORB::ORB_sonar_sensor_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_rates_setpoint"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_rates_setpoint_public, data, sizeof(vehicle_rates_setpoint_s));
    uORB::ORB_vehicle_rates_setpoint_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_gyro"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_gyro_public, data, sizeof(sensor_gyro_s));
    uORB::ORB_sensor_gyro_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_gyro_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_gyro_slave_public, data, sizeof(sensor_gyro_s));
    uORB::ORB_sensor_gyro_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_gyro_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_gyro_backup_public, data, sizeof(sensor_gyro_s));
    uORB::ORB_sensor_gyro_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_accel"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_accel_public, data, sizeof(sensor_accel_s));
    uORB::ORB_sensor_accel_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_accel_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_accel_slave_public, data, sizeof(sensor_accel_s));
    uORB::ORB_sensor_accel_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_accel_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_accel_backup_public, data, sizeof(sensor_accel_s));
    uORB::ORB_sensor_accel_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_mag"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_mag_public, data, sizeof(sensor_mag_s));
    uORB::ORB_sensor_mag_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_mag_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_mag_slave_public, data, sizeof(sensor_mag_s));
    uORB::ORB_sensor_mag_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_mag_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_mag_backup_public, data, sizeof(sensor_mag_s));
    uORB::ORB_sensor_mag_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_baro"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_baro_public, data, sizeof(sensor_baro_s));
    uORB::ORB_sensor_baro_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_baro_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_baro_slave_public, data, sizeof(sensor_baro_s));
    uORB::ORB_sensor_baro_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "sensor_baro_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_sensor_baro_backup_public, data, sizeof(sensor_baro_s));
    uORB::ORB_sensor_baro_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "airspeed"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_airspeed_public, data, sizeof(airspeed_s));
    uORB::ORB_airspeed_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_land_detected"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_land_detected_public, data, sizeof(vehicle_land_detected_s));
    uORB::ORB_vehicle_land_detected_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_status"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_status_public, data, sizeof(vehicle_status_s));
    uORB::ORB_vehicle_status_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_control_mode"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_control_mode_public, data, sizeof(vehicle_control_mode_s));
    uORB::ORB_vehicle_control_mode_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_attitude_setpoint"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_attitude_setpoint_public, data, sizeof(vehicle_attitude_setpoint_s));
    uORB::ORB_vehicle_attitude_setpoint_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "actuator_controls"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_actuator_controls_public, data, sizeof(actuator_controls_s));
    uORB::ORB_actuator_controls_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_global_position"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_global_position_public, data, sizeof(vehicle_global_position_s));
    uORB::ORB_vehicle_global_position_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_local_position"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_local_position_public, data, sizeof(vehicle_local_position_s));
    uORB::ORB_vehicle_local_position_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_global_position_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_global_position_slave_public, data, sizeof(vehicle_global_position_s));
    uORB::ORB_vehicle_global_position_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_local_position_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_local_position_slave_public, data, sizeof(vehicle_local_position_s));
    uORB::ORB_vehicle_local_position_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_global_position_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_global_position_backup_public, data, sizeof(vehicle_global_position_s));
    uORB::ORB_vehicle_global_position_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_local_position_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_local_position_backup_public, data, sizeof(vehicle_local_position_s));
    uORB::ORB_vehicle_local_position_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "control_state"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_control_state_public, data, sizeof(control_state_s));
    uORB::ORB_control_state_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "control_state_slave"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_control_state_slave_public, data, sizeof(control_state_s));
    uORB::ORB_control_state_slave_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "control_state_backup"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_control_state_backup_public, data, sizeof(control_state_s));
    uORB::ORB_control_state_backup_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "manual_control_setpoint"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_manual_control_setpoint_public, data, sizeof(manual_control_setpoint_s));
    uORB::ORB_manual_control_setpoint_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "multirotor_motor_limits"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_multirotor_motor_limits_public, data, sizeof(multirotor_motor_limits_s));
    uORB::ORB_multirotor_motor_limits_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mc_att_ctrl_status"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mc_att_ctrl_status_public, data, sizeof(mc_att_ctrl_status_s));
    uORB::ORB_mc_att_ctrl_status_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "estimator_status"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_estimator_status_public, data, sizeof(estimator_status_s));
    uORB::ORB_estimator_status_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "wind_estimate"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_wind_estimate_public, data, sizeof(wind_estimate_s));
    uORB::ORB_wind_estimate_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "ekf2_innovations"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_ekf2_innovations_public, data, sizeof(ekf2_innovations_s));
    uORB::ORB_ekf2_innovations_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "position_setpoint_triplet"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_position_setpoint_triplet_public, data, sizeof(position_setpoint_triplet_s));
    uORB::ORB_position_setpoint_triplet_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_local_position_setpoint"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_local_position_setpoint_public, data, sizeof(vehicle_local_position_setpoint_s));
    uORB::ORB_vehicle_local_position_setpoint_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_global_velocity_setpoint"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_global_velocity_setpoint_public, data, sizeof(vehicle_global_velocity_setpoint_s));
    uORB::ORB_vehicle_global_velocity_setpoint_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "follow_target"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_follow_target_public, data, sizeof(follow_target_s));
    uORB::ORB_follow_target_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "home_position"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_home_position_public, data, sizeof(home_position_s));
    uORB::ORB_home_position_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mission"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mission_public, data, sizeof(mission_s));
    uORB::ORB_mission_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "fw_pos_ctrl_status"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_fw_pos_ctrl_status_public, data, sizeof(fw_pos_ctrl_status_s));
    uORB::ORB_fw_pos_ctrl_status_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "fence_vertex"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_fence_vertex_public, data, sizeof(fence_vertex_s));
    uORB::ORB_fence_vertex_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "fence"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_fence_public, data, sizeof(fence_s));
    uORB::ORB_fence_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mission_result"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mission_result_public, data, sizeof(mission_result_s));
    uORB::ORB_mission_result_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "geofence_result"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_geofence_result_public, data, sizeof(geofence_result_s));
    uORB::ORB_geofence_result_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_command"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_command_public, data, sizeof(vehicle_command_s));
    uORB::ORB_vehicle_command_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vtol_vehicle_status"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vtol_vehicle_status_public, data, sizeof(vtol_vehicle_status_s));
    uORB::ORB_vtol_vehicle_status_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "battery_status"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_battery_status_public, data, sizeof(battery_status_s));
    uORB::ORB_battery_status_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "safety"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_safety_public, data, sizeof(safety_s));
    uORB::ORB_safety_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "offboard_control_mode"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_offboard_control_mode_public, data, sizeof(offboard_control_mode_s));
    uORB::ORB_offboard_control_mode_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "commander_state"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_commander_state_public, data, sizeof(commander_state_s));
    uORB::ORB_commander_state_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "cpuload"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_cpuload_public, data, sizeof(cpuload_s));
    uORB::ORB_cpuload_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "vehicle_command_ack"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_vehicle_command_ack_public, data, sizeof(vehicle_command_ack_s));
    uORB::ORB_vehicle_command_ack_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "differential_pressure"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_differential_pressure_public, data, sizeof(differential_pressure_s));
    uORB::ORB_differential_pressure_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "telemetry_status"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_telemetry_status_public, data, sizeof(telemetry_status_s));
    uORB::ORB_telemetry_status_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "subsystem_info"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_subsystem_info_public, data, sizeof(subsystem_info_s));
    uORB::ORB_subsystem_info_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "system_power"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_system_power_public, data, sizeof(system_power_s));
    uORB::ORB_system_power_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mavlink_log0"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mavlink_log0_public, data, sizeof(mavlink_log0_s));
    uORB::ORB_mavlink_log0_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mavlink_log1"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mavlink_log1_public, data, sizeof(mavlink_log1_s));
    uORB::ORB_mavlink_log1_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mavlink_log2"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mavlink_log2_public, data, sizeof(mavlink_log2_s));
    uORB::ORB_mavlink_log2_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mavlink_log3"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mavlink_log3_public, data, sizeof(mavlink_log3_s));
    uORB::ORB_mavlink_log3_public.updated=true;
    return(uORB::adv);
  }
  else if (!strcmp(orb_name, "mavlink_log4"))
  {
    strcpy(uORB::adv.orb_name,orb_name);
    memcpy(&uORB::ORB_mavlink_log4_public, data, sizeof(mavlink_log4_s));
    uORB::ORB_mavlink_log4_public.updated=true;
    return(uORB::adv);
  }
  return(uORB::adv);
}


//If subscribed, copy the source data
void orb_copy(char* orb_source_name,int orb_subscribe_handle,void* destination)
{
  if((!strcmp(orb_source_name,"parameter_update"))&&(orb_subscribe_handle==uORB::ORB_parameter_update))
  {
    memcpy(destination,&uORB::ORB_parameter_update_public,sizeof(parameter_update_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_attitude_control"))&&(orb_subscribe_handle==uORB::ORB_vehicle_attitude_control))
  {
    memcpy(destination,&uORB::ORB_vehicle_attitude_control_public,sizeof(vehicle_attitude_control_s));
  }
  else if((!strcmp(orb_source_name,"actuator_armed"))&&(orb_subscribe_handle==uORB::ORB_actuator_armed))
  {
    memcpy(destination,&uORB::ORB_actuator_armed_public,sizeof(actuator_armed_s));
  }
  else if((!strcmp(orb_source_name,"sensor_combined"))&&(orb_subscribe_handle==uORB::ORB_sensor_combined))
  {
    memcpy(destination,&uORB::ORB_sensor_combined_public,sizeof(sensor_combined_s));
  }
  else if((!strcmp(orb_source_name,"sensor_combined_slave"))&&(orb_subscribe_handle==uORB::ORB_sensor_combined_slave))
  {
    memcpy(destination,&uORB::ORB_sensor_combined_slave_public,sizeof(sensor_combined_s));
  }
  else if((!strcmp(orb_source_name,"sensor_combined_backup"))&&(orb_subscribe_handle==uORB::ORB_sensor_combined_backup))
  {
    memcpy(destination,&uORB::ORB_sensor_combined_backup_public,sizeof(sensor_combined_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_attitude"))&&(orb_subscribe_handle==uORB::ORB_vehicle_attitude))
  {
    memcpy(destination,&uORB::ORB_vehicle_attitude_public,sizeof(vehicle_attitude_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_attitude_slave"))&&(orb_subscribe_handle==uORB::ORB_vehicle_attitude_slave))
  {
    memcpy(destination,&uORB::ORB_vehicle_attitude_slave_public,sizeof(vehicle_attitude_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_attitude_backup"))&&(orb_subscribe_handle==uORB::ORB_vehicle_attitude_backup))
  {
    memcpy(destination,&uORB::ORB_vehicle_attitude_backup_public,sizeof(vehicle_attitude_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_attitude_emergency"))&&(orb_subscribe_handle==uORB::ORB_vehicle_attitude_emergency))
  {
    memcpy(destination,&uORB::ORB_vehicle_attitude_emergency_public,sizeof(vehicle_attitude_s));
  }
  else if((!strcmp(orb_source_name,"optical_flow"))&&(orb_subscribe_handle==uORB::ORB_optical_flow))
  {
    memcpy(destination,&uORB::ORB_optical_flow_public,sizeof(optical_flow_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_gps_position"))&&(orb_subscribe_handle==uORB::ORB_vehicle_gps_position))
  {
    memcpy(destination,&uORB::ORB_vehicle_gps_position_public,sizeof(vehicle_gps_position_s));
  }
  else if((!strcmp(orb_source_name,"vision_position_estimate"))&&(orb_subscribe_handle==uORB::ORB_vision_position_estimate))
  {
    memcpy(destination,&uORB::ORB_vision_position_estimate_public,sizeof(vision_position_estimate_s));
  }
  else if((!strcmp(orb_source_name,"att_pos_mocap"))&&(orb_subscribe_handle==uORB::ORB_att_pos_mocap))
  {
    memcpy(destination,&uORB::ORB_att_pos_mocap_public,sizeof(att_pos_mocap_s));
  }
  else if((!strcmp(orb_source_name,"distance_sensor"))&&(orb_subscribe_handle==uORB::ORB_distance_sensor))
  {
    memcpy(destination,&uORB::ORB_distance_sensor_public,sizeof(distance_sensor_s));
  }
  else if((!strcmp(orb_source_name,"range_finder"))&&(orb_subscribe_handle==uORB::ORB_range_finder))
  {
    memcpy(destination,&uORB::ORB_range_finder_public,sizeof(range_finder_s));
  }
  else if((!strcmp(orb_source_name,"sonar_sensor"))&&(orb_subscribe_handle==uORB::ORB_sonar_sensor))
  {
    memcpy(destination,&uORB::ORB_sonar_sensor_public,sizeof(sonar_sensor_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_rates_setpoint"))&&(orb_subscribe_handle==uORB::ORB_vehicle_rates_setpoint))
  {
    memcpy(destination,&uORB::ORB_vehicle_rates_setpoint_public,sizeof(vehicle_rates_setpoint_s));
  }
  else if((!strcmp(orb_source_name,"sensor_gyro"))&&(orb_subscribe_handle==uORB::ORB_sensor_gyro))
  {
    memcpy(destination,&uORB::ORB_sensor_gyro_public,sizeof(sensor_gyro_s));
  }
  else if((!strcmp(orb_source_name,"sensor_gyro_slave"))&&(orb_subscribe_handle==uORB::ORB_sensor_gyro_slave))
  {
    memcpy(destination,&uORB::ORB_sensor_gyro_slave_public,sizeof(sensor_gyro_s));
  }
  else if((!strcmp(orb_source_name,"sensor_gyro_backup"))&&(orb_subscribe_handle==uORB::ORB_sensor_gyro_backup))
  {
    memcpy(destination,&uORB::ORB_sensor_gyro_backup_public,sizeof(sensor_gyro_s));
  }
  else if((!strcmp(orb_source_name,"sensor_accel"))&&(orb_subscribe_handle==uORB::ORB_sensor_accel))
  {
    memcpy(destination,&uORB::ORB_sensor_accel_public,sizeof(sensor_accel_s));
  }
  else if((!strcmp(orb_source_name,"sensor_accel_slave"))&&(orb_subscribe_handle==uORB::ORB_sensor_accel_slave))
  {
    memcpy(destination,&uORB::ORB_sensor_accel_slave_public,sizeof(sensor_accel_s));
  }
  else if((!strcmp(orb_source_name,"sensor_accel_backup"))&&(orb_subscribe_handle==uORB::ORB_sensor_accel_backup))
  {
    memcpy(destination,&uORB::ORB_sensor_accel_backup_public,sizeof(sensor_accel_s));
  }
  else if((!strcmp(orb_source_name,"sensor_mag"))&&(orb_subscribe_handle==uORB::ORB_sensor_mag))
  {
    memcpy(destination,&uORB::ORB_sensor_mag_public,sizeof(sensor_mag_s));
  }
  else if((!strcmp(orb_source_name,"sensor_mag_slave"))&&(orb_subscribe_handle==uORB::ORB_sensor_mag_slave))
  {
    memcpy(destination,&uORB::ORB_sensor_mag_slave_public,sizeof(sensor_mag_s));
  }
  else if((!strcmp(orb_source_name,"sensor_mag_backup"))&&(orb_subscribe_handle==uORB::ORB_sensor_mag_backup))
  {
    memcpy(destination,&uORB::ORB_sensor_mag_backup_public,sizeof(sensor_mag_s));
  }
  else if((!strcmp(orb_source_name,"sensor_baro"))&&(orb_subscribe_handle==uORB::ORB_sensor_baro))
  {
    memcpy(destination,&uORB::ORB_sensor_baro_public,sizeof(sensor_baro_s));
  }
  else if((!strcmp(orb_source_name,"sensor_baro_slave"))&&(orb_subscribe_handle==uORB::ORB_sensor_baro_slave))
  {
    memcpy(destination,&uORB::ORB_sensor_baro_slave_public,sizeof(sensor_baro_s));
  }
  else if((!strcmp(orb_source_name,"sensor_baro_backup"))&&(orb_subscribe_handle==uORB::ORB_sensor_baro_backup))
  {
    memcpy(destination,&uORB::ORB_sensor_baro_backup_public,sizeof(sensor_baro_s));
  }
  else if((!strcmp(orb_source_name,"airspeed"))&&(orb_subscribe_handle==uORB::ORB_airspeed))
  {
    memcpy(destination,&uORB::ORB_airspeed_public,sizeof(airspeed_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_land_detected"))&&(orb_subscribe_handle==uORB::ORB_vehicle_land_detected))
  {
    memcpy(destination,&uORB::ORB_vehicle_land_detected_public,sizeof(vehicle_land_detected_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_status"))&&(orb_subscribe_handle==uORB::ORB_vehicle_status))
  {
    memcpy(destination,&uORB::ORB_vehicle_status_public,sizeof(vehicle_status_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_control_mode"))&&(orb_subscribe_handle==uORB::ORB_vehicle_control_mode))
  {
    memcpy(destination,&uORB::ORB_vehicle_control_mode_public,sizeof(vehicle_control_mode_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_attitude_setpoint"))&&(orb_subscribe_handle==uORB::ORB_vehicle_attitude_setpoint))
  {
    memcpy(destination,&uORB::ORB_vehicle_attitude_setpoint_public,sizeof(vehicle_attitude_setpoint_s));
  }
  else if((!strcmp(orb_source_name,"actuator_controls"))&&(orb_subscribe_handle==uORB::ORB_actuator_controls))
  {
    memcpy(destination,&uORB::ORB_actuator_controls_public,sizeof(actuator_controls_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_global_position"))&&(orb_subscribe_handle==uORB::ORB_vehicle_global_position))
  {
    memcpy(destination,&uORB::ORB_vehicle_global_position_public,sizeof(vehicle_global_position_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_local_position"))&&(orb_subscribe_handle==uORB::ORB_vehicle_local_position))
  {
    memcpy(destination,&uORB::ORB_vehicle_local_position_public,sizeof(vehicle_local_position_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_global_position_slave"))&&(orb_subscribe_handle==uORB::ORB_vehicle_global_position_slave))
  {
    memcpy(destination,&uORB::ORB_vehicle_global_position_slave_public,sizeof(vehicle_global_position_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_local_position_slave"))&&(orb_subscribe_handle==uORB::ORB_vehicle_local_position_slave))
  {
    memcpy(destination,&uORB::ORB_vehicle_local_position_slave_public,sizeof(vehicle_local_position_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_global_position_backup"))&&(orb_subscribe_handle==uORB::ORB_vehicle_global_position_backup))
  {
    memcpy(destination,&uORB::ORB_vehicle_global_position_backup_public,sizeof(vehicle_global_position_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_local_position_backup"))&&(orb_subscribe_handle==uORB::ORB_vehicle_local_position_backup))
  {
    memcpy(destination,&uORB::ORB_vehicle_local_position_backup_public,sizeof(vehicle_local_position_s));
  }
  else if((!strcmp(orb_source_name,"control_state"))&&(orb_subscribe_handle==uORB::ORB_control_state))
  {
    memcpy(destination,&uORB::ORB_control_state_public,sizeof(control_state_s));
  }
  else if((!strcmp(orb_source_name,"control_state_slave"))&&(orb_subscribe_handle==uORB::ORB_control_state_slave))
  {
    memcpy(destination,&uORB::ORB_control_state_slave_public,sizeof(control_state_s));
  }
  else if((!strcmp(orb_source_name,"control_state_backup"))&&(orb_subscribe_handle==uORB::ORB_control_state_backup))
  {
    memcpy(destination,&uORB::ORB_control_state_backup_public,sizeof(control_state_s));
  }
  else if((!strcmp(orb_source_name,"manual_control_setpoint"))&&(orb_subscribe_handle==uORB::ORB_manual_control_setpoint))
  {
    memcpy(destination,&uORB::ORB_manual_control_setpoint_public,sizeof(manual_control_setpoint_s));
  }
  else if((!strcmp(orb_source_name,"multirotor_motor_limits"))&&(orb_subscribe_handle==uORB::ORB_multirotor_motor_limits))
  {
    memcpy(destination,&uORB::ORB_multirotor_motor_limits_public,sizeof(multirotor_motor_limits_s));
  }
  else if((!strcmp(orb_source_name,"mc_att_ctrl_status"))&&(orb_subscribe_handle==uORB::ORB_mc_att_ctrl_status))
  {
    memcpy(destination,&uORB::ORB_mc_att_ctrl_status_public,sizeof(mc_att_ctrl_status_s));
  }
  else if((!strcmp(orb_source_name,"estimator_status"))&&(orb_subscribe_handle==uORB::ORB_estimator_status))
  {
    memcpy(destination,&uORB::ORB_estimator_status_public,sizeof(estimator_status_s));
  }
  else if((!strcmp(orb_source_name,"wind_estimate"))&&(orb_subscribe_handle==uORB::ORB_wind_estimate))
  {
    memcpy(destination,&uORB::ORB_wind_estimate_public,sizeof(wind_estimate_s));
  }
  else if((!strcmp(orb_source_name,"ekf2_innovations"))&&(orb_subscribe_handle==uORB::ORB_ekf2_innovations))
  {
    memcpy(destination,&uORB::ORB_ekf2_innovations_public,sizeof(ekf2_innovations_s));
  }
  else if((!strcmp(orb_source_name,"position_setpoint_triplet"))&&(orb_subscribe_handle==uORB::ORB_position_setpoint_triplet))
  {
    memcpy(destination,&uORB::ORB_position_setpoint_triplet_public,sizeof(position_setpoint_triplet_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_local_position_setpoint"))&&(orb_subscribe_handle==uORB::ORB_vehicle_local_position_setpoint))
  {
    memcpy(destination,&uORB::ORB_vehicle_local_position_setpoint_public,sizeof(vehicle_local_position_setpoint_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_global_velocity_setpoint"))&&(orb_subscribe_handle==uORB::ORB_vehicle_global_velocity_setpoint))
  {
    memcpy(destination,&uORB::ORB_vehicle_global_velocity_setpoint_public,sizeof(vehicle_global_velocity_setpoint_s));
  }
  else if((!strcmp(orb_source_name,"follow_target"))&&(orb_subscribe_handle==uORB::ORB_follow_target))
  {
    memcpy(destination,&uORB::ORB_follow_target_public,sizeof(follow_target_s));
  }
  else if((!strcmp(orb_source_name,"home_position"))&&(orb_subscribe_handle==uORB::ORB_home_position))
  {
    memcpy(destination,&uORB::ORB_home_position_public,sizeof(home_position_s));
  }
  else if((!strcmp(orb_source_name,"mission"))&&(orb_subscribe_handle==uORB::ORB_mission))
  {
    memcpy(destination,&uORB::ORB_mission_public,sizeof(mission_s));
  }
  else if((!strcmp(orb_source_name,"fw_pos_ctrl_status"))&&(orb_subscribe_handle==uORB::ORB_fw_pos_ctrl_status))
  {
    memcpy(destination,&uORB::ORB_fw_pos_ctrl_status_public,sizeof(fw_pos_ctrl_status_s));
  }
  else if((!strcmp(orb_source_name,"fence_vertex"))&&(orb_subscribe_handle==uORB::ORB_fence_vertex))
  {
    memcpy(destination,&uORB::ORB_fence_vertex_public,sizeof(fence_vertex_s));
  }
  else if((!strcmp(orb_source_name,"fence"))&&(orb_subscribe_handle==uORB::ORB_fence))
  {
    memcpy(destination,&uORB::ORB_fence_public,sizeof(fence_s));
  }
  else if((!strcmp(orb_source_name,"mission_result"))&&(orb_subscribe_handle==uORB::ORB_mission_result))
  {
    memcpy(destination,&uORB::ORB_mission_result_public,sizeof(mission_result_s));
  }
  else if((!strcmp(orb_source_name,"geofence_result"))&&(orb_subscribe_handle==uORB::ORB_geofence_result))
  {
    memcpy(destination,&uORB::ORB_geofence_result_public,sizeof(geofence_result_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_command"))&&(orb_subscribe_handle==uORB::ORB_vehicle_command))
  {
    memcpy(destination,&uORB::ORB_vehicle_command_public,sizeof(vehicle_command_s));
  }
  else if((!strcmp(orb_source_name,"vtol_vehicle_status"))&&(orb_subscribe_handle==uORB::ORB_vtol_vehicle_status))
  {
    memcpy(destination,&uORB::ORB_vtol_vehicle_status_public,sizeof(vtol_vehicle_status_s));
  }
  else if((!strcmp(orb_source_name,"battery_status"))&&(orb_subscribe_handle==uORB::ORB_battery_status))
  {
    memcpy(destination,&uORB::ORB_battery_status_public,sizeof(battery_status_s));
  }
  else if((!strcmp(orb_source_name,"safety"))&&(orb_subscribe_handle==uORB::ORB_safety))
  {
    memcpy(destination,&uORB::ORB_safety_public,sizeof(safety_s));
  }
  else if((!strcmp(orb_source_name,"offboard_control_mode"))&&(orb_subscribe_handle==uORB::ORB_offboard_control_mode))
  {
    memcpy(destination,&uORB::ORB_offboard_control_mode_public,sizeof(offboard_control_mode_s));
  }
  else if((!strcmp(orb_source_name,"commander_state"))&&(orb_subscribe_handle==uORB::ORB_commander_state))
  {
    memcpy(destination,&uORB::ORB_commander_state_public,sizeof(commander_state_s));
  }
  else if((!strcmp(orb_source_name,"cpuload"))&&(orb_subscribe_handle==uORB::ORB_cpuload))
  {
    memcpy(destination,&uORB::ORB_cpuload_public,sizeof(cpuload_s));
  }
  else if((!strcmp(orb_source_name,"vehicle_command_ack"))&&(orb_subscribe_handle==uORB::ORB_vehicle_command_ack))
  {
    memcpy(destination,&uORB::ORB_vehicle_command_ack_public,sizeof(vehicle_command_ack_s));
  }
  else if((!strcmp(orb_source_name,"differential_pressure"))&&(orb_subscribe_handle==uORB::ORB_differential_pressure))
  {
    memcpy(destination,&uORB::ORB_differential_pressure_public,sizeof(differential_pressure_s));
  }
  else if((!strcmp(orb_source_name,"telemetry_status"))&&(orb_subscribe_handle==uORB::ORB_telemetry_status))
  {
    memcpy(destination,&uORB::ORB_telemetry_status_public,sizeof(telemetry_status_s));
  }
  else if((!strcmp(orb_source_name,"subsystem_info"))&&(orb_subscribe_handle==uORB::ORB_subsystem_info))
  {
    memcpy(destination,&uORB::ORB_subsystem_info_public,sizeof(subsystem_info_s));
  }
  else if((!strcmp(orb_source_name,"system_power"))&&(orb_subscribe_handle==uORB::ORB_system_power))
  {
    memcpy(destination,&uORB::ORB_system_power_public,sizeof(system_power_s));
  }
  else if((!strcmp(orb_source_name,"mavlink_log0"))&&(orb_subscribe_handle==uORB::ORB_mavlink_log0))
  {
    memcpy(destination,&uORB::ORB_mavlink_log0_public,sizeof(mavlink_log0_s));
  }
  else if((!strcmp(orb_source_name,"mavlink_log1"))&&(orb_subscribe_handle==uORB::ORB_mavlink_log1))
  {
    memcpy(destination,&uORB::ORB_mavlink_log1_public,sizeof(mavlink_log1_s));
  }
  else if((!strcmp(orb_source_name,"mavlink_log2"))&&(orb_subscribe_handle==uORB::ORB_mavlink_log2))
  {
    memcpy(destination,&uORB::ORB_mavlink_log2_public,sizeof(mavlink_log2_s));
  }
  else if((!strcmp(orb_source_name,"mavlink_log3"))&&(orb_subscribe_handle==uORB::ORB_mavlink_log3))
  {
    memcpy(destination,&uORB::ORB_mavlink_log3_public,sizeof(mavlink_log3_s));
  }
  else if((!strcmp(orb_source_name,"mavlink_log4"))&&(orb_subscribe_handle==uORB::ORB_mavlink_log4))
  {
    memcpy(destination,&uORB::ORB_mavlink_log4_public,sizeof(mavlink_log4_s));
  }
}


void orb_check(int orb_subscribe_handle,bool *updated)
{
  if(orb_subscribe_handle==uORB::ORB_parameter_update)
  {
    *updated=uORB::ORB_parameter_update_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_attitude_control)
  {
    *updated=uORB::ORB_vehicle_attitude_control_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_actuator_armed)
  {
    *updated=uORB::ORB_actuator_armed_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_combined)
  {
    *updated=uORB::ORB_sensor_combined_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_combined_slave)
  {
    *updated=uORB::ORB_sensor_combined_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_combined_backup)
  {
    *updated=uORB::ORB_sensor_combined_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_attitude)
  {
    *updated=uORB::ORB_vehicle_attitude_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_attitude_slave)
  {
    *updated=uORB::ORB_vehicle_attitude_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_attitude_backup)
  {
    *updated=uORB::ORB_vehicle_attitude_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_attitude_emergency)
  {
    *updated=uORB::ORB_vehicle_attitude_emergency_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_optical_flow)
  {
    *updated=uORB::ORB_optical_flow_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_gps_position)
  {
    *updated=uORB::ORB_vehicle_gps_position_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vision_position_estimate)
  {
    *updated=uORB::ORB_vision_position_estimate_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_att_pos_mocap)
  {
    *updated=uORB::ORB_att_pos_mocap_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_distance_sensor)
  {
    *updated=uORB::ORB_distance_sensor_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_range_finder)
  {
    *updated=uORB::ORB_range_finder_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sonar_sensor)
  {
    *updated=uORB::ORB_sonar_sensor_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_rates_setpoint)
  {
    *updated=uORB::ORB_vehicle_rates_setpoint_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_gyro)
  {
    *updated=uORB::ORB_sensor_gyro_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_gyro_slave)
  {
    *updated=uORB::ORB_sensor_gyro_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_gyro_backup)
  {
    *updated=uORB::ORB_sensor_gyro_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_accel)
  {
    *updated=uORB::ORB_sensor_accel_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_accel_slave)
  {
    *updated=uORB::ORB_sensor_accel_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_accel_backup)
  {
    *updated=uORB::ORB_sensor_accel_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_mag)
  {
    *updated=uORB::ORB_sensor_mag_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_mag_slave)
  {
    *updated=uORB::ORB_sensor_mag_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_mag_backup)
  {
    *updated=uORB::ORB_sensor_mag_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_baro)
  {
    *updated=uORB::ORB_sensor_baro_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_baro_slave)
  {
    *updated=uORB::ORB_sensor_baro_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_sensor_baro_backup)
  {
    *updated=uORB::ORB_sensor_baro_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_airspeed)
  {
    *updated=uORB::ORB_airspeed_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_land_detected)
  {
    *updated=uORB::ORB_vehicle_land_detected_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_status)
  {
    *updated=uORB::ORB_vehicle_status_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_control_mode)
  {
    *updated=uORB::ORB_vehicle_control_mode_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_attitude_setpoint)
  {
    *updated=uORB::ORB_vehicle_attitude_setpoint_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_actuator_controls)
  {
    *updated=uORB::ORB_actuator_controls_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_global_position)
  {
    *updated=uORB::ORB_vehicle_global_position_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_local_position)
  {
    *updated=uORB::ORB_vehicle_local_position_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_global_position_slave)
  {
    *updated=uORB::ORB_vehicle_global_position_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_local_position_slave)
  {
    *updated=uORB::ORB_vehicle_local_position_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_global_position_backup)
  {
    *updated=uORB::ORB_vehicle_global_position_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_local_position_backup)
  {
    *updated=uORB::ORB_vehicle_local_position_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_control_state)
  {
    *updated=uORB::ORB_control_state_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_control_state_slave)
  {
    *updated=uORB::ORB_control_state_slave_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_control_state_backup)
  {
    *updated=uORB::ORB_control_state_backup_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_manual_control_setpoint)
  {
    *updated=uORB::ORB_manual_control_setpoint_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_multirotor_motor_limits)
  {
    *updated=uORB::ORB_multirotor_motor_limits_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mc_att_ctrl_status)
  {
    *updated=uORB::ORB_mc_att_ctrl_status_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_estimator_status)
  {
    *updated=uORB::ORB_estimator_status_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_wind_estimate)
  {
    *updated=uORB::ORB_wind_estimate_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_ekf2_innovations)
  {
    *updated=uORB::ORB_ekf2_innovations_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_position_setpoint_triplet)
  {
    *updated=uORB::ORB_position_setpoint_triplet_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_local_position_setpoint)
  {
    *updated=uORB::ORB_vehicle_local_position_setpoint_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_global_velocity_setpoint)
  {
    *updated=uORB::ORB_vehicle_global_velocity_setpoint_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_follow_target)
  {
    *updated=uORB::ORB_follow_target_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_home_position)
  {
    *updated=uORB::ORB_home_position_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mission)
  {
    *updated=uORB::ORB_mission_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_fw_pos_ctrl_status)
  {
    *updated=uORB::ORB_fw_pos_ctrl_status_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_fence_vertex)
  {
    *updated=uORB::ORB_fence_vertex_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_fence)
  {
    *updated=uORB::ORB_fence_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mission_result)
  {
    *updated=uORB::ORB_mission_result_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_geofence_result)
  {
    *updated=uORB::ORB_geofence_result_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_command)
  {
    *updated=uORB::ORB_vehicle_command_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vtol_vehicle_status)
  {
    *updated=uORB::ORB_vtol_vehicle_status_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_battery_status)
  {
    *updated=uORB::ORB_battery_status_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_safety)
  {
    *updated=uORB::ORB_safety_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_offboard_control_mode)
  {
    *updated=uORB::ORB_offboard_control_mode_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_commander_state)
  {
    *updated=uORB::ORB_commander_state_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_cpuload)
  {
    *updated=uORB::ORB_cpuload_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_vehicle_command_ack)
  {
    *updated=uORB::ORB_vehicle_command_ack_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_differential_pressure)
  {
    *updated=uORB::ORB_differential_pressure_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_telemetry_status)
  {
    *updated=uORB::ORB_telemetry_status_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_subsystem_info)
  {
    *updated=uORB::ORB_subsystem_info_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_system_power)
  {
    *updated=uORB::ORB_system_power_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mavlink_log0)
  {
    *updated=uORB::ORB_mavlink_log0_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mavlink_log1)
  {
    *updated=uORB::ORB_mavlink_log1_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mavlink_log2)
  {
    *updated=uORB::ORB_mavlink_log2_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mavlink_log3)
  {
    *updated=uORB::ORB_mavlink_log3_public.updated;
  }
  if(orb_subscribe_handle==uORB::ORB_mavlink_log4)
  {
    *updated=uORB::ORB_mavlink_log4_public.updated;
  }
  else
    *updated = false; 
}


void orb_publish(char* orb_source_name, orb_advert_t pub_handle, void* data){
  if (!strcmp(pub_handle.orb_name, orb_source_name))
  {
    orb_advertise(pub_handle.orb_name, data);
  }
}
