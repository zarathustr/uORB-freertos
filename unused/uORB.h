#ifndef UORB_H
#define UORB_H

#include <message_types.h>

#define ORB_ID(x)    ((char*)(#x))

typedef uint8_t arming_state_t;
typedef uint8_t main_state_t;
typedef uint8_t hil_state_t;
typedef uint8_t navigation_state_t;
typedef uint8_t switch_pos_t;
#define ORB_MULTI_MAX_INSTANCES 4

namespace uORB{

enum {
     ORB_parameter_update=1,
     ORB_vehicle_attitude_control,
     ORB_actuator_armed,
     ORB_sensor_combined,
     ORB_vehicle_attitude,
     ORB_optical_flow,
     ORB_vehicle_gps_position,
     ORB_vision_position_estimate,
     ORB_att_pos_mocap,
     ORB_distance_sensor,
     ORB_vehicle_rates_setpoint,
     ORB_sensor_gyro,
     ORB_sensor_accel,
     ORB_sensor_mag,
     ORB_sensor_baro,
     ORB_airspeed,
     ORB_vehicle_land_detected,
     ORB_vehicle_status,
     ORB_vehicle_control_mode,
     ORB_vehicle_attitude_setpoint,
     ORB_actuator_controls,
     ORB_vehicle_global_position,
     ORB_vehicle_local_position,
     ORB_control_state,
     ORB_manual_control_setpoint,
     ORB_multirotor_motor_limits,
     ORB_mc_att_ctrl_status,
     ORB_estimator_status,
     ORB_wind_estimate,
     ORB_ekf2_innovations,
     ORB_position_setpoint_triplet,
     ORB_vehicle_local_position_setpoint,
     ORB_vehicle_global_velocity_setpoint,
     ORB_follow_target,
     ORB_home_position,
     ORB_mission,
     ORB_fw_pos_ctrl_status,
     ORB_fence_vertex,
     ORB_fence,
     ORB_mission_result,
     ORB_geofence_result,
     ORB_vehicle_command,
     ORB_vtol_vehicle_status,
     ORB_battery_status,
     ORB_safety,
     ORB_offboard_control_mode,
     ORB_commander_state,
     ORB_cpuload,
     ORB_vehicle_command_ack,
     ORB_differential_pressure,
     ORB_telemetry_status,
     ORB_subsystem_info,
     ORB_system_power,
     ORB_mavlink_log,
     ORB_debug_key_value,
     ORB_actuator_outputs,
     ORB_actuator_direct,
     ORB_tecs_status,
     ORB_rc_channels,
     ORB_filtered_bottom_flow,
     ORB_satellite_info,
     ORB_hil_sensor,
     ORB_log_message,
     ORB_ekf2_timestamps,
     ORB_led_control,
     ORB_vehicle_roi,
     ORB_sensor_correction,
     ORB_mc_virtual_attitude_setpoint,
     ORB_mc_virtual_rates_setpoint,
     ORB_fw_virtual_attitude_setpoint,
     ORB_fw_virtual_rates_setpoint,
     ORB_vehicle_status_flags,
     ORB_sensor_preflight,
     ORB_rc_parameter_map,
     ORB_transponder_report,
     ORB_collision_report,
     ORB_camera_trigger,
     ORB_mount_orientation,
     ORB_input_rc,
     ORB_vehicle_force_setpoint,
     ORB_time_offset,
     ORB_gps_inject_data
};


extern          parameter_update_s                              ORB_parameter_update_public;
extern          vehicle_attitude_control_s                      ORB_vehicle_attitude_control_public;
extern          actuator_armed_s                                ORB_actuator_armed_public;
extern          sensor_combined_s                               ORB_sensor_combined_public[2];
extern          vehicle_attitude_s                              ORB_vehicle_attitude_public[8];
extern          optical_flow_s                                  ORB_optical_flow_public;
extern          vehicle_gps_position_s                          ORB_vehicle_gps_position_public;
extern          vision_position_estimate_s                      ORB_vision_position_estimate_public;
extern          att_pos_mocap_s                                 ORB_att_pos_mocap_public;
extern          distance_sensor_s                               ORB_distance_sensor_public[5];
extern          vehicle_rates_setpoint_s                        ORB_vehicle_rates_setpoint_public;
extern          sensor_gyro_s                                   ORB_sensor_gyro_public[5];
extern          sensor_accel_s                                  ORB_sensor_accel_public[5];
extern          sensor_mag_s                                    ORB_sensor_mag_public[5];
extern          sensor_baro_s                                   ORB_sensor_baro_public[2];
extern          airspeed_s                                      ORB_airspeed_public;
extern          vehicle_land_detected_s                         ORB_vehicle_land_detected_public;
extern          vehicle_status_s                                ORB_vehicle_status_public;
extern          vehicle_control_mode_s                          ORB_vehicle_control_mode_public;
extern          vehicle_attitude_setpoint_s                     ORB_vehicle_attitude_setpoint_public;
extern          actuator_controls_s                             ORB_actuator_controls_public;
extern          vehicle_global_position_s                       ORB_vehicle_global_position_public;
extern          vehicle_local_position_s                        ORB_vehicle_local_position_public;
extern          control_state_s                                 ORB_control_state_public;
extern          manual_control_setpoint_s                       ORB_manual_control_setpoint_public;
extern          multirotor_motor_limits_s                       ORB_multirotor_motor_limits_public;
extern          mc_att_ctrl_status_s                            ORB_mc_att_ctrl_status_public;
extern          estimator_status_s                              ORB_estimator_status_public;
extern          wind_estimate_s                                 ORB_wind_estimate_public;
extern          ekf2_innovations_s                              ORB_ekf2_innovations_public;
extern          position_setpoint_triplet_s                     ORB_position_setpoint_triplet_public;
extern          vehicle_local_position_setpoint_s               ORB_vehicle_local_position_setpoint_public;
extern          vehicle_global_velocity_setpoint_s              ORB_vehicle_global_velocity_setpoint_public;
extern          follow_target_s				        ORB_follow_target_public;
extern          home_position_s				        ORB_home_position_public;
extern          mission_s				        ORB_mission_public;
extern          fw_pos_ctrl_status_s				ORB_fw_pos_ctrl_status_public;
extern          fence_vertex_s				        ORB_fence_vertex_public;
extern          fence_s				                ORB_fence_public;
extern          mission_result_s				ORB_mission_result_public;
extern          geofence_result_s				ORB_geofence_result_public;
extern          vehicle_command_s				ORB_vehicle_command_public;
extern          vtol_vehicle_status_s				ORB_vtol_vehicle_status_public;
extern          battery_status_s				ORB_battery_status_public;
extern          safety_s				        ORB_safety_public;
extern          offboard_control_mode_s				ORB_offboard_control_mode_public;
extern          commander_state_s				ORB_commander_state_public;
extern          cpuload_s				        ORB_cpuload_public;
extern          vehicle_command_ack_s				ORB_vehicle_command_ack_public;
extern          differential_pressure_s				ORB_differential_pressure_public;
extern          telemetry_status_s				ORB_telemetry_status_public;
extern          subsystem_info_s				ORB_subsystem_info_public;
extern          system_power_s				        ORB_system_power_public;
extern          mavlink_log_s				        ORB_mavlink_log_public;
extern          debug_key_value_s			        ORB_debug_key_value_public;
extern          actuator_outputs_s			        ORB_actuator_outputs_public;
extern          actuator_direct_s			        ORB_actuator_direct_public;
extern          tecs_status_s			                ORB_tecs_status_public;
extern          rc_channels_s			                ORB_rc_channels_public;
extern          filtered_bottom_flow_s			        ORB_filtered_bottom_flow_public;
extern          satellite_info_s			        ORB_satellite_info_public;
extern          hil_sensor_s			                ORB_hil_sensor_public;
extern          log_message_s			                ORB_log_message_public;
extern          ekf2_timestamps_s			        ORB_ekf2_timestamps_public;
extern          led_control_s                                   ORB_led_control_public;
extern          vehicle_roi_s                                   ORB_vehicle_roi_public;
extern          sensor_correction_s                             ORB_sensor_correction_public;
extern          mc_virtual_attitude_setpoint_s			ORB_mc_virtual_attitude_setpoint_public;
extern          mc_virtual_rates_setpoint_s			ORB_mc_virtual_rates_setpoint_public;
extern          fw_virtual_attitude_setpoint_s			ORB_fw_virtual_attitude_setpoint_public;
extern          fw_virtual_rates_setpoint_s			ORB_fw_virtual_rates_setpoint_public;
extern          vehicle_status_flags_s                          ORB_vehicle_status_flags_public;
extern          sensor_preflight_s                              ORB_sensor_preflight_public;
extern          rc_parameter_map_s			        ORB_rc_parameter_map_public;
extern          transponder_report_s			        ORB_transponder_report_public;
extern          collision_report_s			        ORB_collision_report_public;
extern          camera_trigger_s			        ORB_camera_trigger_public;
extern          mount_orientation_s			        ORB_mount_orientation_public;
extern          input_rc_s                                      ORB_input_rc_public;
extern          vehicle_force_setpoint_s			ORB_vehicle_force_setpoint_public;
extern          time_offset_s			                ORB_time_offset_public;
extern          gps_inject_data_s			        ORB_gps_inject_data_public;
}


typedef struct orb_advert{
  char orb_name[50];
}*orb_advert_t;


#define orb_metadata orb_advert


#ifdef __cplusplus
extern "C"{
#endif

void orb_init(void);
int  orb_subscribe(char* orb_name);
int orb_unsubscribe(int &handle);
orb_advert_t orb_advertise(char* orb_name,void* data);
int orb_unadvertise(orb_advert_t adv);
int orb_copy(char* orb_source_name,int orb_subscribe_handle,void* destination);
int orb_check(int orb_subscribe_handle,bool *updated);
int orb_publish(char* orb_source_name, orb_advert_t pub_handle, void* data);
int orb_publish_multi(char* orb_source_name, orb_advert_t pub_handle, void* data, ORB_PRIORITY priority);
int orb_check_multi(int orb_subscribe_handle, bool *updated, ORB_PRIORITY priority);
void orb_copy_multi(char* orb_source_name, int orb_subscribe_handle, void* destination, ORB_PRIORITY priority);

#ifdef __cplusplus
}
#endif
                   
#endif
