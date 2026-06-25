#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <Arduino.h>

class controller;

class controller_base {
    public:
        controller_base();

    public:
        void update_feedback_gain(float height);
        float leg_servo_count_to_height(void);
        void update_linear_reference(float dt, float target_speed);
        void update_yaw_reference(float dt, float target_speed);
        void reset_motion_reference();
        void set_cam_angle(uint32_t tick);
        void reset();

    private:
        controller *ctrl = nullptr;

    public:
        void bind(controller &ctrl){this->ctrl = &ctrl;}

    private:
        float last_height = 0.0f;

        int16_t last_position[2];
        float last_value = 0.0f;

        float lpf_target_linear_vel = 0.0f;
        float lpf_target_steering_vel = 0.0f;
        float last_linear_target_speed = 0.0f;
        float linear_release_timer = 0.0f;
        uint8_t linear_release_active = 0;

        float cam_max_speed = 60.0f;
        float cam_angle = 60.0f;
        int16_t cam_last_angle = 0;
        float cam_lpf_target_speed = 0.0f;
};

#endif
