#include <uORB/uORB.h>
#include <cstring>
#include <StartUP.h>

#include "uORBHelper.h"


#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/hil_sensor.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
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
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/vehicle_local_position_groundtruth.h>
#include <uORB/topics/vehicle_vision_position.h>
#include <uORB/topics/vehicle_global_position_groundtruth.h>
#include <uORB/topics/vehicle_attitude_groundtruth.h>
#include <uORB/topics/vehicle_vision_attitude.h>



#pragma default_variable_attributes= @ ".ccmram"


  actuator_armed_s			ORB_actuator_armed_public;
  airspeed_s			ORB_airspeed_public;
  att_pos_mocap_s			ORB_att_pos_mocap_public;
  battery_status_s			ORB_battery_status_public;
  cpuload_s			ORB_cpuload_public;
  differential_pressure_s			ORB_differential_pressure_public;
  distance_sensor_s			ORB_distance_sensor_public[ORB_MULTI_MAX_INSTANCES];
  ekf2_innovations_s			ORB_ekf2_innovations_public;
  ekf2_replay_s			ORB_ekf2_replay_public;
  ekf2_timestamps_s			ORB_ekf2_timestamps_public;
  estimator_status_s			ORB_estimator_status_public[ORB_MULTI_MAX_INSTANCES];
  filtered_bottom_flow_s			ORB_filtered_bottom_flow_public;
  gps_dump_s			ORB_gps_dump_public;
  gps_inject_data_s			ORB_gps_inject_data_public;
  hil_sensor_s			ORB_hil_sensor_public;
  home_position_s			ORB_home_position_public;
  input_rc_s			ORB_input_rc_public;
  mavlink_log_s			ORB_mavlink_log_public;
  mount_orientation_s			ORB_mount_orientation_public;
  optical_flow_s			ORB_optical_flow_public;
  parameter_update_s			ORB_parameter_update_public;
  sensor_accel_s			ORB_sensor_accel_public[ORB_MULTI_MAX_INSTANCES];
  sensor_baro_s			ORB_sensor_baro_public[ORB_MULTI_MAX_INSTANCES];
  sensor_combined_s			ORB_sensor_combined_public[ORB_MULTI_MAX_INSTANCES];
  sensor_correction_s			ORB_sensor_correction_public;
  sensor_gyro_s			ORB_sensor_gyro_public[ORB_MULTI_MAX_INSTANCES];
  sensor_mag_s			ORB_sensor_mag_public[ORB_MULTI_MAX_INSTANCES];
  sensor_preflight_s			ORB_sensor_preflight_public;
  telemetry_status_s			ORB_telemetry_status_public[ORB_MULTI_MAX_INSTANCES];
  vehicle_attitude_s			ORB_vehicle_attitude_public[ORB_MULTI_MAX_INSTANCES];
  vehicle_control_mode_s			ORB_vehicle_control_mode_public;
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
  wind_estimate_s			ORB_wind_estimate_public;
  vehicle_local_position_groundtruth_s			ORB_vehicle_local_position_groundtruth_public;
  vehicle_vision_position_s			ORB_vehicle_vision_position_public;
  vehicle_global_position_groundtruth_s			ORB_vehicle_global_position_groundtruth_public;
  vehicle_attitude_groundtruth_s			ORB_vehicle_attitude_groundtruth_public;
  vehicle_vision_attitude_s			ORB_vehicle_vision_attitude_public;


#pragma default_variable_attributes = 





void orb_helper_init(void)
{
  std::memset(&ORB_actuator_armed_public, 0, sizeof(actuator_armed_s));
  std::memset(&ORB_airspeed_public, 0, sizeof(airspeed_s));
  std::memset(&ORB_att_pos_mocap_public, 0, sizeof(att_pos_mocap_s));
  std::memset(&ORB_battery_status_public, 0, sizeof(battery_status_s));
  std::memset(&ORB_cpuload_public, 0, sizeof(cpuload_s));
  std::memset(&ORB_differential_pressure_public, 0, sizeof(differential_pressure_s));
  std::memset(&ORB_distance_sensor_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(distance_sensor_s));
  std::memset(&ORB_ekf2_innovations_public, 0, sizeof(ekf2_innovations_s));
  std::memset(&ORB_ekf2_replay_public, 0, sizeof(ekf2_replay_s));
  std::memset(&ORB_ekf2_timestamps_public, 0, sizeof(ekf2_timestamps_s));
  std::memset(&ORB_estimator_status_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(estimator_status_s));
  std::memset(&ORB_filtered_bottom_flow_public, 0, sizeof(filtered_bottom_flow_s));
  std::memset(&ORB_gps_dump_public, 0, sizeof(gps_dump_s));
  std::memset(&ORB_gps_inject_data_public, 0, sizeof(gps_inject_data_s));
  std::memset(&ORB_hil_sensor_public, 0, sizeof(hil_sensor_s));
  std::memset(&ORB_home_position_public, 0, sizeof(home_position_s));
  std::memset(&ORB_input_rc_public, 0, sizeof(input_rc_s));
  std::memset(&ORB_mavlink_log_public, 0, sizeof(mavlink_log_s));
  std::memset(&ORB_mount_orientation_public, 0, sizeof(mount_orientation_s));
  std::memset(&ORB_optical_flow_public, 0, sizeof(optical_flow_s));
  std::memset(&ORB_parameter_update_public, 0, sizeof(parameter_update_s));
  std::memset(&ORB_sensor_accel_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_accel_s));
  std::memset(&ORB_sensor_baro_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_baro_s));
  std::memset(&ORB_sensor_combined_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_combined_s));
  std::memset(&ORB_sensor_correction_public, 0, sizeof(sensor_correction_s));
  std::memset(&ORB_sensor_gyro_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_gyro_s));
  std::memset(&ORB_sensor_mag_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(sensor_mag_s));
  std::memset(&ORB_sensor_preflight_public, 0, sizeof(sensor_preflight_s));
  std::memset(&ORB_telemetry_status_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(telemetry_status_s));
  std::memset(&ORB_vehicle_attitude_public, 0, ORB_MULTI_MAX_INSTANCES*sizeof(vehicle_attitude_s));
  std::memset(&ORB_vehicle_control_mode_public, 0, sizeof(vehicle_control_mode_s));
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
  std::memset(&ORB_wind_estimate_public, 0, sizeof(wind_estimate_s));
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
  else if (serial == ORB_cpuload)
  {
      std::strcpy(name, (ORB_ID(cpuload))->o_name);
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
  else if (serial == ORB_estimator_status)
  {
      std::strcpy(name, (ORB_ID(estimator_status))->o_name);
  }
  else if (serial == ORB_filtered_bottom_flow)
  {
      std::strcpy(name, (ORB_ID(filtered_bottom_flow))->o_name);
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
  else if (serial == ORB_mavlink_log)
  {
      std::strcpy(name, (ORB_ID(mavlink_log))->o_name);
  }
  else if (serial == ORB_mount_orientation)
  {
      std::strcpy(name, (ORB_ID(mount_orientation))->o_name);
  }
  else if (serial == ORB_optical_flow)
  {
      std::strcpy(name, (ORB_ID(optical_flow))->o_name);
  }
  else if (serial == ORB_parameter_update)
  {
      std::strcpy(name, (ORB_ID(parameter_update))->o_name);
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
  else if (serial == ORB_telemetry_status)
  {
      std::strcpy(name, (ORB_ID(telemetry_status))->o_name);
  }
  else if (serial == ORB_vehicle_attitude)
  {
      std::strcpy(name, (ORB_ID(vehicle_attitude))->o_name);
  }
  else if (serial == ORB_vehicle_control_mode)
  {
      std::strcpy(name, (ORB_ID(vehicle_control_mode))->o_name);
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
  else if (serial == ORB_wind_estimate)
  {
      std::strcpy(name, (ORB_ID(wind_estimate))->o_name);
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
  else if (0 == std::strcmp(name, (ORB_ID(cpuload))->o_name))
  {
      return ORB_cpuload;
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
  else if (0 == std::strcmp(name, (ORB_ID(estimator_status))->o_name))
  {
      return ORB_estimator_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(filtered_bottom_flow))->o_name))
  {
      return ORB_filtered_bottom_flow;
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
  else if (0 == std::strcmp(name, (ORB_ID(mavlink_log))->o_name))
  {
      return ORB_mavlink_log;
  }
  else if (0 == std::strcmp(name, (ORB_ID(mount_orientation))->o_name))
  {
      return ORB_mount_orientation;
  }
  else if (0 == std::strcmp(name, (ORB_ID(optical_flow))->o_name))
  {
      return ORB_optical_flow;
  }
  else if (0 == std::strcmp(name, (ORB_ID(parameter_update))->o_name))
  {
      return ORB_parameter_update;
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
  else if (0 == std::strcmp(name, (ORB_ID(telemetry_status))->o_name))
  {
      return ORB_telemetry_status;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_attitude))->o_name))
  {
      return ORB_vehicle_attitude;
  }
  else if (0 == std::strcmp(name, (ORB_ID(vehicle_control_mode))->o_name))
  {
      return ORB_vehicle_control_mode;
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
  else if (0 == std::strcmp(name, (ORB_ID(wind_estimate))->o_name))
  {
      return ORB_wind_estimate;
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
  else if (0 == std::strcmp(orb_name, (ORB_ID(cpuload))->o_name))
  {
      return ORB_ID(cpuload);
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
  else if (0 == std::strcmp(orb_name, (ORB_ID(estimator_status))->o_name))
  {
      return ORB_ID(estimator_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(filtered_bottom_flow))->o_name))
  {
      return ORB_ID(filtered_bottom_flow);
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
  else if (0 == std::strcmp(orb_name, (ORB_ID(mavlink_log))->o_name))
  {
      return ORB_ID(mavlink_log);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(mount_orientation))->o_name))
  {
      return ORB_ID(mount_orientation);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(optical_flow))->o_name))
  {
      return ORB_ID(optical_flow);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(parameter_update))->o_name))
  {
      return ORB_ID(parameter_update);
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
  else if (0 == std::strcmp(orb_name, (ORB_ID(telemetry_status))->o_name))
  {
      return ORB_ID(telemetry_status);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude))->o_name))
  {
      return ORB_ID(vehicle_attitude);
  }
  else if (0 == std::strcmp(orb_name, (ORB_ID(vehicle_control_mode))->o_name))
  {
      return ORB_ID(vehicle_control_mode);
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
  else if (0 == std::strcmp(orb_name, (ORB_ID(wind_estimate))->o_name))
  {
      return ORB_ID(wind_estimate);
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
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(cpuload))->o_name))
  {
      return (void*)&(ORB_cpuload_public);
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
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(estimator_status))->o_name))
  {
        return (void*)&(ORB_estimator_status_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(filtered_bottom_flow))->o_name))
  {
      return (void*)&(ORB_filtered_bottom_flow_public);
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
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mavlink_log))->o_name))
  {
      return (void*)&(ORB_mavlink_log_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(mount_orientation))->o_name))
  {
      return (void*)&(ORB_mount_orientation_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(optical_flow))->o_name))
  {
      return (void*)&(ORB_optical_flow_public);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(parameter_update))->o_name))
  {
      return (void*)&(ORB_parameter_update_public);
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
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(telemetry_status))->o_name))
  {
        return (void*)&(ORB_telemetry_status_public[instance]);
  }
  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_attitude))->o_name))
  {
        return (void*)&(ORB_vehicle_attitude_public[instance]);
  }
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(vehicle_control_mode))->o_name))
  {
      return (void*)&(ORB_vehicle_control_mode_public);
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
  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(wind_estimate))->o_name))
  {
      return (void*)&(ORB_wind_estimate_public);
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
