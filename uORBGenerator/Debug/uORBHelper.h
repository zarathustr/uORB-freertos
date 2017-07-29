#ifndef __UORB_HELPER_H
#define __UORB_HELPER_H


#include <uORB/uORB.h>
#include <cmsis_os.h>


enum ORB_serial
{
    ORB_actuator_armed = 0,
    ORB_airspeed = 1,
    ORB_att_pos_mocap = 2,
    ORB_battery_status = 3,
    ORB_cpuload = 4,
    ORB_differential_pressure = 5,
    ORB_distance_sensor = 6,
    ORB_ekf2_innovations = 7,
    ORB_ekf2_replay = 8,
    ORB_ekf2_timestamps = 9,
    ORB_estimator_status = 10,
    ORB_filtered_bottom_flow = 11,
    ORB_gps_dump = 12,
    ORB_gps_inject_data = 13,
    ORB_hil_sensor = 14,
    ORB_home_position = 15,
    ORB_input_rc = 16,
    ORB_mavlink_log = 17,
    ORB_mount_orientation = 18,
    ORB_optical_flow = 19,
    ORB_parameter_update = 20,
    ORB_sensor_accel = 21,
    ORB_sensor_baro = 22,
    ORB_sensor_combined = 23,
    ORB_sensor_correction = 24,
    ORB_sensor_gyro = 25,
    ORB_sensor_mag = 26,
    ORB_sensor_preflight = 27,
    ORB_telemetry_status = 28,
    ORB_vehicle_attitude = 29,
    ORB_vehicle_control_mode = 30,
    ORB_vehicle_global_position = 31,
    ORB_vehicle_global_velocity_setpoint = 32,
    ORB_vehicle_gps_position = 33,
    ORB_vehicle_land_detected = 34,
    ORB_vehicle_local_position = 35,
    ORB_vehicle_local_position_setpoint = 36,
    ORB_vehicle_rates_setpoint = 37,
    ORB_vehicle_roi = 38,
    ORB_vehicle_status = 39,
    ORB_vehicle_status_flags = 40,
    ORB_wind_estimate = 41,
    ORB_vehicle_local_position_groundtruth = 42,
    ORB_vehicle_vision_position = 43,
    ORB_vehicle_global_position_groundtruth = 44,
    ORB_vehicle_attitude_groundtruth = 45,
    ORB_vehicle_vision_attitude = 46,
    total_uorb_num
};
struct ORBData{
    void                *data = nullptr;
    ORB_PRIO            priority;
    int                 serial;
    uint64_t            interval;
    xQueueHandle        queue;
    bool                published;
};


const int MULTI_ORB_NUM = 11;


const ORB_serial orb_multi_list[MULTI_ORB_NUM]=
{
    ORB_distance_sensor,
    ORB_telemetry_status,
    ORB_vehicle_attitude,
    ORB_estimator_status,
    ORB_vehicle_local_position,
    ORB_vehicle_global_position,
    ORB_sensor_gyro,
    ORB_sensor_accel,
    ORB_sensor_mag,
    ORB_sensor_baro,
    ORB_sensor_combined,
};


#define general_type_ptr        parameter_update_s*


extern ORBData          *orb_data[total_uorb_num];
extern void             get_orb_name(ORB_serial serial, char * name);
extern int              get_orb_serial(const char * name);
extern int              get_orb_instance_according_to_priority(int priority);
extern int              get_priority(int instance);
extern bool             orb_in_os;
extern void             orb_helper_init(void);
extern void             orb_set_in_os(void);
extern bool             is_orb_multi(int serial);
extern orb_id_t         get_orb_according_to_serial(int serial);
extern void             *get_orb_public_according_to_serial_and_instance(int serial, int instance);


#endif
