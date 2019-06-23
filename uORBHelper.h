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


#ifndef __UORB_HELPER_H
#define __UORB_HELPER_H


#include <uORB/uORB.h>
#include <cmsis_os.h>
#include <RingBuffer/RingBuffer.h>

enum ORB_serial
{
    ORB_actuator_armed = 0,
    ORB_actuator_controls = 1,
    ORB_actuator_direct = 2,
    ORB_actuator_outputs = 3,
    ORB_adc_report = 4,
    ORB_airspeed = 5,
    ORB_att_pos_mocap = 6,
    ORB_battery_status = 7,
    ORB_camera_trigger = 8,
    ORB_commander_state = 9,
    ORB_collision_report = 10,
    ORB_control_state = 11,
    ORB_cpuload = 12,
    ORB_debug_key_value = 13,
    ORB_differential_pressure = 14,
    ORB_distance_sensor = 15,
    ORB_ekf2_innovations = 16,
    ORB_ekf2_replay = 17,
    ORB_ekf2_timestamps = 18,
    ORB_esc_report = 19,
    ORB_esc_status = 20,
    ORB_estimator_status = 21,
    ORB_fence = 22,
    ORB_fence_vertex = 23,
    ORB_filtered_bottom_flow = 24,
    ORB_follow_target = 25,
    ORB_fw_pos_ctrl_status = 26,
    ORB_fw_virtual_attitude_setpoint = 27,
    ORB_fw_virtual_rates_setpoint = 28,
    ORB_geofence_result = 29,
    ORB_gps_dump = 30,
    ORB_gps_inject_data = 31,
    ORB_hil_sensor = 32,
    ORB_home_position = 33,
    ORB_input_rc = 34,
    ORB_led_control = 35,
    ORB_log_message = 36,
    ORB_manual_control_setpoint = 37,
    ORB_mavlink_log = 38,
    ORB_mc_att_ctrl_status = 39,
    ORB_mc_virtual_attitude_setpoint = 40,
    ORB_mc_virtual_rates_setpoint = 41,
    ORB_mission = 42,
    ORB_mission_result = 43,
    ORB_mount_orientation = 44,
    ORB_multirotor_motor_limits = 45,
    ORB_offboard_control_mode = 46,
    ORB_optical_flow = 47,
    ORB_output_pwm = 48,
    ORB_parameter_update = 49,
    ORB_position_setpoint = 50,
    ORB_position_setpoint_triplet = 51,
    ORB_pwm_input = 52,
    ORB_qshell_req = 53,
    ORB_rc_channels = 54,
    ORB_rc_parameter_map = 55,
    ORB_safety = 56,
    ORB_satellite_info = 57,
    ORB_sensor_accel = 58,
    ORB_sensor_baro = 59,
    ORB_sensor_combined = 60,
    ORB_sensor_correction = 61,
    ORB_sensor_gyro = 62,
    ORB_sensor_mag = 63,
    ORB_sensor_preflight = 64,
    ORB_servorail_status = 65,
    ORB_subsystem_info = 66,
    ORB_system_power = 67,
    ORB_task_stack_info = 68,
    ORB_tecs_status = 69,
    ORB_telemetry_status = 70,
    ORB_test_motor = 71,
    ORB_time_offset = 72,
    ORB_transponder_report = 73,
    ORB_uavcan_parameter_request = 74,
    ORB_uavcan_parameter_value = 75,
    ORB_ulog_stream = 76,
    ORB_ulog_stream_ack = 77,
    ORB_vehicle_attitude = 78,
    ORB_vehicle_attitude_setpoint = 79,
    ORB_vehicle_command_ack = 80,
    ORB_vehicle_command = 81,
    ORB_vehicle_control_mode = 82,
    ORB_vehicle_force_setpoint = 83,
    ORB_vehicle_global_position = 84,
    ORB_vehicle_global_velocity_setpoint = 85,
    ORB_vehicle_gps_position = 86,
    ORB_vehicle_land_detected = 87,
    ORB_vehicle_local_position = 88,
    ORB_vehicle_local_position_setpoint = 89,
    ORB_vehicle_rates_setpoint = 90,
    ORB_vehicle_roi = 91,
    ORB_vehicle_status = 92,
    ORB_vehicle_status_flags = 93,
    ORB_vtol_vehicle_status = 94,
    ORB_wind_estimate = 95,
    ORB_actuator_controls_0 = 96,
    ORB_actuator_controls_1 = 97,
    ORB_actuator_controls_2 = 98,
    ORB_actuator_controls_3 = 99,
    ORB_actuator_controls_virtual_fw = 100,
    ORB_actuator_controls_virtual_mc = 101,
    ORB_offboard_mission = 102,
    ORB_onboard_mission = 103,
    ORB_vehicle_local_position_groundtruth = 104,
    ORB_vehicle_vision_position = 105,
    ORB_vehicle_global_position_groundtruth = 106,
    ORB_vehicle_attitude_groundtruth = 107,
    ORB_vehicle_vision_attitude = 108,
    total_uorb_num
};

#define UORB_MAX_SUB    10
#define UORB_MAX_POLL   10

const int MULTI_ORB_NUM = 13;


const ORB_serial orb_multi_list[MULTI_ORB_NUM] =
{
    ORB_distance_sensor,
    ORB_telemetry_status,
    ORB_vehicle_attitude,
    ORB_estimator_status,
    ORB_control_state,
    ORB_vehicle_local_position,
    ORB_vehicle_global_position,
    ORB_sensor_gyro,
    ORB_sensor_accel,
    ORB_sensor_mag,
    ORB_sensor_baro,
    ORB_sensor_combined,
    ORB_vehicle_gps_position
};

struct ORBData{
    void                                *data = nullptr;
    ORB_PRIO                            priority;
    int                                 serial;
    uint64_t                            interval;
    ringbuffer::RingBuffer              *queue = nullptr;
    bool                                published;
    osSemaphoreId                       sem;
    bool                                sem_taken;
    uint64_t                            last_updated_time;
    int32_t                             *registered_list = nullptr;
    int32_t                             *authority_list = nullptr;
};

extern ORBData          *orb_data[total_uorb_num];
extern bool             orb_in_os;


extern void             get_orb_name(const int serial, char * name);
extern int              get_orb_serial(const char * name);
extern int              get_orb_instance_according_to_priority(const int priority);
extern int              get_priority(const int instance);
extern void             orb_set_in_os(void);
extern bool             is_orb_multi(const int serial);
extern orb_id_t         get_orb_according_to_serial(const int serial);

#endif
