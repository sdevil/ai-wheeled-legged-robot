#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "algorithm/lqi.h"
#include "common/pid.h"
#include "common/lowpass_filter.h"
#include "controller_base.h"
#include "fsm.h"
#include "host_data_processing.h"
#include "motion_buttons.h"
#include "motor.h"
#include "mpu6050_dev.h"
#include "ptk7350.h"
#include "sts3032.h"

class controller {
    public:
        controller();

    public:
        void init();
        void lqi_loop(uint32_t tick);
        void boot_loop(uint32_t tick);
        void sit_loop(uint32_t tick);
        void leg_loop();
        void jump_loop(uint32_t tick);
        void begin_balance_recover();
        bool balance_recover_prepare_loop(uint32_t tick);
        bool balance_recover_loop(uint32_t tick);

    public:
        lqi lqi_param;
        fsm fsm_state_machine;
        controller_base base_components;
        host_data_processing host_data;

    public:
        uint16_t buttons;
        float axes[6];
        float dead_zone = lqi_param.limit.max_linear_vel * 0.05f;

    public:
        float input[2];
        float output[2];

    public:
        uint8_t enable_balance = 0;
        uint8_t enable_steering = 0;
        uint8_t enable_motor = 0;

    public:
        uint32_t sit_timer = 0;
        int16_t sit_start_left_position = SERVO_LEFT_MIN;
        int16_t sit_start_right_position = SERVO_RIGHT_MIN;
        float sit_start_leg_height = (float)LEG_HEIGHT_BASE;
        uint8_t sit_retract_started = 0;
        uint8_t balance_recover_active = 0;
        uint32_t balance_recover_prepare_timer = 0;
        uint32_t balance_recover_timer = 0;
        uint32_t balance_recover_last_elapsed = 0;
        uint32_t balance_recover_ready_timer = 0;
        uint32_t balance_recover_upright_timer = 0;
        uint32_t balance_recover_leg_rise_timer = 0;
        uint8_t balance_recover_leg_rise_started = 0;
        float balance_recover_start_leg_height = (float)LEG_HEIGHT_BASE;
        float balance_recover_target_leg_height = (float)LEG_HEIGHT_BASE;
        float balance_recover_origin_position = 0.0f;
        float balance_recover_origin_yaw = 0.0f;
        float balance_recover_max_displacement = 0.0f;
        float balance_recover_min_signed_displacement = 0.0f;
        float balance_recover_max_signed_displacement = 0.0f;
        uint32_t balance_recover_peak_time = 0;
        float balance_recover_displacement = 0.0f;
        float balance_recover_position_correction = 0.0f;
        float balance_recover_yaw_correction = 0.0f;
        uint8_t balance_idle_hold_active = 0;
        uint32_t balance_idle_hold_settle_timer = 0;
        float balance_idle_hold_position = 0.0f;
        float balance_idle_hold_yaw = 0.0f;
        uint8_t symmetric_leg_motion = 0;
        uint8_t force_sync_leg_motion = 0;
        float roll_adjust = 0.0f;
        float roll_adjust_target = 0.0f;
        float leg_lean = 0.0f;
        float leg_lean_target = 0.0f;
        float leg_height_base = (float)LEG_HEIGHT_BASE;
        PIDController pid_roll_angle{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
        LowPassFilter lpf_roll{0.3f};

        uint32_t jump_timer = 0;
        uint32_t jump_recover_elapsed = 0;
        int8_t jump_linear_direction = 0;
        int8_t jump_turn_direction = 0;
        float jump_linear_vel_cmd = 0.0f;
        float jump_turn_target_yaw = 0.0f;
        float jump_turn_yaw_rate_cmd = 0.0f;

    public:
        static void loop_proc(uint32_t tick);
        static void sensor_update_proc(uint32_t tick);
        static void motor_update_proc(uint32_t tick);

    public:
        float x_debug[6];
        float input_debug[2];
        float output_debug[2];
        float sit_mode_flag = 0;
        float jump_mode_flag = 0;
        float yaw_angle_debug[5];
};

extern controller ctrl;

#endif
