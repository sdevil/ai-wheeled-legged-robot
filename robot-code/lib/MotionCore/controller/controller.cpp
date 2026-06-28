#include "controller.h"

controller ctrl;

static float wrap_pi(float angle)
{
    while(angle > PI){angle -= 2.0f * PI;}
    while(angle < -PI){angle += 2.0f * PI;}
    return angle;
}

static float shortest_angle_error(float target, float current)
{
    return wrap_pi(target - current);
}

static float apply_axis_deadband(float value, float deadband)
{
    if(fabsf(value) <= deadband)
    {
        return 0.0f;
    }

    float magnitude = (fabsf(value) - deadband) / (1.0f - deadband);
    return (value > 0.0f ? 1.0f : -1.0f) * magnitude;
}

controller::controller()
{
    fsm_state_machine.bind(*this);
    base_components.bind(*this);
    host_data.bind(*this);
}

void controller::begin_balance_recover()
{
    balance_recover_active = 1;
    balance_recover_prepare_timer = 0;
    balance_recover_timer = 0;
    balance_recover_ready_timer = 0;
}

bool controller::balance_recover_prepare_loop(uint32_t tick)
{
    enable_steering = 0;
    enable_balance = 0;
    enable_motor = 0;

    if((balance_recover_prepare_timer += tick) < 350)
    {
        return false;
    }

    balance_recover_prepare_timer = 0;
    balance_recover_timer = 0;
    balance_recover_ready_timer = 0;
    base_components.reset_motion_reference();
    return true;
}

bool controller::balance_recover_loop(uint32_t tick)
{
    const float pitch_ready = 0.16f;
    const float pitch_rate_ready = 1.2f;
    const uint32_t min_recover_ms = 250;
    const uint32_t ready_hold_ms = 140;
    const uint32_t max_recover_ms = 2500;

    enable_steering = 0;
    enable_balance = 1;
    enable_motor = 1;
    leg_loop();

    balance_recover_timer += tick;
    if(fabsf(mpu6050_dev.angle[1]) < pitch_ready && fabsf(mpu6050_dev.gyro[1]) < pitch_rate_ready)
    {
        balance_recover_ready_timer += tick;
    }
    else
    {
        balance_recover_ready_timer = 0;
    }

    if((balance_recover_timer >= min_recover_ms && balance_recover_ready_timer >= ready_hold_ms) ||
       balance_recover_timer >= max_recover_ms)
    {
        balance_recover_active = 0;
        balance_recover_timer = 0;
        balance_recover_ready_timer = 0;
        base_components.reset_motion_reference();
        return true;
    }

    return false;
}

void controller::init()
{
    host_data.init();
    sts3032_init();
    mpu6050_dev.init(1);
    motor_init();
}

void controller::lqi_loop(uint32_t tick)
{
    float dt = (float)tick * 1.0e-3f;
    float output_blend = 1.0f;
    const float linear_axis_deadband = 0.05f;
    const float steer_axis_deadband = 0.05f;
    const bool jump_active = (fsm_state_machine.mode == fsm::mode_state::JUMP);

    base_components.update_feedback_gain(base_components.leg_servo_count_to_height());

    float linear_axis = apply_axis_deadband(axes[3], linear_axis_deadband);
    float steer_axis = apply_axis_deadband(axes[0], steer_axis_deadband);

    float target_linear_vel = linear_axis * lqi_param.limit.max_linear_vel;
    if(linear_axis < 0.0f){target_linear_vel *= 0.8f;}
    input[0] += target_linear_vel;
    input[1] += steer_axis * lqi_param.limit.max_steer_vel;

    if(enable_balance)
    {
        base_components.update_linear_reference(dt, input[0]);
        base_components.update_yaw_reference(dt, input[1]);
    }
    else
    {
        input[0] = 0.0f;
        input[1] = 0.0f;
        base_components.reset_motion_reference();
    }

    input_debug[0] = lqi_param.ref.linear_vel;
    input_debug[1] = lqi_param.ref.yaw_rate;
    memset(input, 0, sizeof(input));

    if((buttons & BTN_LS) && (fabsf(target_linear_vel) < dead_zone))
    {
        base_components.reset();
    }

    float x[6] = {
        lqi_param.state.pitch_angle,
        lqi_param.state.pitch_rate,
        lqi_param.state.avg_linear_vel - lqi_param.ref.linear_vel,
        lqi_param.state.yaw_rate - lqi_param.ref.yaw_rate,
        lqi_param.integral.linear_vel_error,
        lqi_param.integral.yaw_rate_error
    };

    if(balance_recover_active)
    {
        x[2] = 0.0f;
        x[3] = 0.0f;
        x[4] = 0.0f;
        x[5] = 0.0f;
        output_blend = constrain((float)balance_recover_timer * 1.0e-3f / 0.22f, 0.0f, 1.0f);
    }
    else if(jump_active)
    {
        // During jump keep wheel balance alive, but ignore the velocity/yaw
        // channels that are easily polluted by the leg actuation.
        const bool jump_linear_ground_phase =
            fsm_state_machine.jump == fsm::jump_state::PUSH ||
            (jump_linear_direction > 0 &&
             fsm_state_machine.jump == fsm::jump_state::FLY &&
             jump_timer < 60);
        const bool jump_yaw_active = (jump_turn_direction != 0) || (jump_linear_direction != 0);
        const bool jump_turn_recover_phase =
            jump_turn_direction != 0 &&
            (fsm_state_machine.jump == fsm::jump_state::LAND ||
             fsm_state_machine.jump == fsm::jump_state::RECOVER);

        if(jump_linear_direction == 0 || !jump_linear_ground_phase)
        {
            x[2] = 0.0f;
        }
        if(!jump_yaw_active)
        {
            x[3] = 0.0f;
            x[5] = 0.0f;
        }
        else if(jump_turn_recover_phase)
        {
            x[5] = 0.0f;
        }
    }

    if(!enable_steering)
    {
        x[3] = 0.0f;
        x[5] = 0.0f;
    }

    memcpy(x_debug, x, sizeof(x_debug));

    for(uint8_t i = 0; i < 2; i++)
    {
        output[i] = 0.0f;
        for(uint8_t j = 0; j < 6; j++)
        {
            output[i] += lqi_param.feedback_gain[i][j] * x[j];
        }
        output[i] *= output_blend;
    }

    memcpy(output_debug, output, sizeof(output_debug));

    if(enable_balance)
    {
        left_motor.move(output[0]);
        right_motor.move(output[1]);
    }
}

void controller::boot_loop(uint32_t tick)
{
    switch(fsm_state_machine.boot)
    {
        case fsm::boot_state::PREPARE:
            enable_balance = 0;
            enable_steering = 0;
            enable_motor = 0;

            sts3032.set_torque_switch(SERVO_LEFT, 0);
            sts3032.set_torque_switch(SERVO_RIGHT, 0);

            fsm_state_machine.boot = fsm::boot_state::WAIT_FOR_SIGNAL;
            break;

        case fsm::boot_state::WAIT_FOR_SIGNAL:
            if(buttons & BTN_RB)
            {
                fsm_state_machine.boot = fsm::boot_state::INIT;
            }
            break;

        case fsm::boot_state::INIT:
            sts3032.set(SERVO_LEFT, SERVO_LEFT_MIN, 450, 250);
            sts3032.set(SERVO_RIGHT, SERVO_RIGHT_MIN, 450, 250);
            sts3032.move();

            base_components.reset();
            begin_balance_recover();
            fsm_state_machine.boot = fsm::boot_state::INIT_PREPARE;
            break;

        case fsm::boot_state::INIT_PREPARE:
            if(balance_recover_prepare_loop(tick))
            {
                fsm_state_machine.boot = fsm::boot_state::INIT_RECOVER;
            }
            break;

        case fsm::boot_state::INIT_RECOVER:
            if(balance_recover_loop(tick))
            {
                fsm_state_machine.boot = fsm::boot_state::DONE;
            }
            break;

        case fsm::boot_state::DONE:
            break;
    }
}

void controller::sit_loop(uint32_t tick)  // 坐下状态循环
{
    switch(fsm_state_machine.sit)       // 坐下状态
    {
        case fsm::sit_state::PREPARE:  // 坐下状态准备
            sit_mode_flag = 1.0f;
            enable_steering = 0;
            enable_balance = 0;

            //sts3032.set(SERVO_LEFT, SERVO_LEFT_MIN, 450, 250);  
            //sts3032.set(SERVO_RIGHT, SERVO_RIGHT_MIN, 450, 250);

            sts3032.set_torque_switch(SERVO_LEFT, 2);
            sts3032.set_torque_switch(SERVO_RIGHT, 2);

            fsm_state_machine.sit = fsm::sit_state::MOVING;
            break;

        case fsm::sit_state::MOVING:
            left_motor.move(-0.2f);   // 坐下状态移动
            right_motor.move(-0.2f);  // 坐下状态移动

            if(fabsf(mpu6050_dev.angle[1]) >= 0.25f || (sit_timer += tick) >= 5000)
            {
                enable_motor = 0;    
                fsm_state_machine.sit = fsm::sit_state::DONE;
            }
            break;

        case fsm::sit_state::DONE:  
            if(buttons & BTN_LS)     // 坐下状态保持
            {
                sts3032.set_torque_switch(SERVO_LEFT, 0);
                sts3032.set_torque_switch(SERVO_RIGHT, 0);
            }

            if(buttons & BTN_RB)     // 如果按下右肩键，则准备退出坐下状态
            {
                //sts3032.set(SERVO_LEFT, SERVO_LEFT_MIN, 450, 250);
                //sts3032.set(SERVO_RIGHT, SERVO_RIGHT_MIN, 450, 250);
                //sts3032.move();
                sit_timer = 0;                       // 重置坐下计时器
                base_components.reset();             // 重置基础组件    
                begin_balance_recover();             // 开始平衡恢复
                fsm_state_machine.sit = fsm::sit_state::EXIT_PREPARE;        // 坐下状态退出准备
            }
            break;

        case fsm::sit_state::EXIT_PREPARE:   // 坐下状态退出准备
            if(balance_recover_prepare_loop(tick))
            {
                fsm_state_machine.sit = fsm::sit_state::EXIT_RECOVER;
            }
            break;

        case fsm::sit_state::EXIT_RECOVER:   // 坐下状态退出恢复
            if(balance_recover_loop(tick))
            {
                sit_mode_flag = 0.0f;
                fsm_state_machine.sit = fsm::sit_state::EXIT;
            }
            break;

        case fsm::sit_state::EXIT:  // 坐下状态退出
            break;
    }
}

void controller::leg_loop()
{
    if((buttons & BTN_RIGHT) && !(buttons & ~BTN_RIGHT)){roll_adjust += 0.025f;}
    if((buttons & BTN_LEFT) && !(buttons & ~BTN_LEFT)){roll_adjust -= 0.025f;}
    if((buttons & BTN_UP) && !(buttons & ~BTN_UP)){leg_height_base -= 0.025f;}
    if((buttons & BTN_DOWN) && !(buttons & ~BTN_DOWN)){leg_height_base += 0.025f;}

    float roll_angle = lpf_roll(mpu6050_dev.angle[0] / (float)PI * 180.0f);
    float leg_position_add = pid_roll_angle(roll_angle - roll_adjust);

    int16_t left_position = (int16_t)(2048.0f + 8.4f * (30.0f - leg_height_base) - leg_position_add);
    int16_t right_position = (int16_t)(2048.0f - 8.4f * (30.0f - leg_height_base) - leg_position_add);

    left_position = constrain(left_position, SERVO_LEFT_MIN, SERVO_LEFT_MAX);
    right_position = constrain(right_position, SERVO_RIGHT_MAX, SERVO_RIGHT_MIN);

    sts3032.set(SERVO_LEFT, left_position, 1000, 0);
    sts3032.set(SERVO_RIGHT, right_position, 1000, 0);
    sts3032.move();
}

void controller::jump_loop(uint32_t tick)
{
    const float recover_pitch = 0.18f;
    const float recover_pitch_rate = 1.6f;
    const uint32_t recover_hold_ms = 80;
    const uint32_t recover_timeout_ms = 350;
    const uint32_t recover_force_exit_ms = 220;
    const bool forward_jump = jump_linear_direction > 0;
    const bool backward_jump = jump_linear_direction < 0;
    const bool linear_jump = jump_linear_direction != 0;
    const uint32_t jump_push_wait_ms =
        forward_jump ? 650 :
        backward_jump ? 700 :
        200;
    const uint32_t jump_push_ramp_ms =
        forward_jump ? 160 :
        backward_jump ? 240 :
        80;
    const float jump_linear_push_vel =
        forward_jump ? min(lqi_param.limit.max_linear_vel, 0.40f) :
        backward_jump ? min(lqi_param.limit.max_linear_vel, 0.34f) :
        0.0f;
    const float jump_linear_fly_vel = 0.0f;
    const float jump_linear_land_vel = 0.0f;
    const float jump_turn_angle = PI * 0.5f;
    const float jump_yaw_hold_kp = 3.0f;
    const float jump_yaw_hold_ground_max_rate = 1.8f;
    const float jump_yaw_hold_air_max_rate = 0.8f;
    const float jump_turn_prepare_ff = 0.2f;
    const float jump_turn_push_ff = 1.2f;
    const float jump_turn_fly_ff = 6.4f;
    const float jump_turn_land_ff = 0.0f;
    const float jump_turn_prepare_kp = 1.0f;
    const float jump_turn_push_kp = 1.4f;
    const float jump_turn_fly_kp = 2.0f;
    const float jump_turn_land_kp = 0.35f;
    const float jump_turn_prepare_max_rate = 0.6f;
    const float jump_turn_push_max_rate = 1.8f;
    const float jump_turn_fly_max_rate = 6.4f;
    const float jump_turn_land_max_rate = 0.4f;
    const float jump_turn_recover_kp = 0.8f;
    const float jump_turn_recover_max_rate = 0.5f;
    const float jump_turn_ready = 5.0f / 180.0f * PI;

    if(fsm_state_machine.jump != fsm::jump_state::DONE)
    {
        enable_steering = (jump_turn_direction != 0) || (jump_linear_direction != 0);
        enable_balance = 1;
        enable_motor = 1;
        pid_roll_angle = PIDController(pid_roll_angle.P, pid_roll_angle.I,
                                       pid_roll_angle.D,
                                       pid_roll_angle.output_ramp,
                                       pid_roll_angle.limit);

        switch(fsm_state_machine.jump)
        {
            case fsm::jump_state::PREPARE:
                jump_linear_vel_cmd = 0.0f;
                break;

            case fsm::jump_state::PUSH:
            {
                float push_scale = constrain(
                    (float)jump_timer / ((float)jump_push_ramp_ms),
                    0.0f,
                    1.0f
                );
                jump_linear_vel_cmd = (float)jump_linear_direction * jump_linear_push_vel * push_scale;
                break;
            }

            case fsm::jump_state::FLY:
                jump_linear_vel_cmd = (float)jump_linear_direction * jump_linear_fly_vel;
                break;

            case fsm::jump_state::LAND:
                jump_linear_vel_cmd = (float)jump_linear_direction * jump_linear_land_vel;
                break;

            case fsm::jump_state::RECOVER:
            case fsm::jump_state::DONE:
                jump_linear_vel_cmd = 0.0f;
                break;
        }

        if(jump_turn_direction != 0 || jump_linear_direction != 0)
        {
            float yaw_error = shortest_angle_error(jump_turn_target_yaw, lqi_param.state.yaw_angle);
            float ff_rate = 0.0f;
            float fb_kp = 0.0f;
            float max_rate = jump_turn_recover_max_rate;

            if(jump_turn_direction == 0)
            {
                fb_kp = jump_yaw_hold_kp;
                max_rate =
                    (fsm_state_machine.jump == fsm::jump_state::PUSH) ?
                    jump_yaw_hold_ground_max_rate :
                    jump_yaw_hold_air_max_rate;
            }
            else
            {
                switch(fsm_state_machine.jump)
                {
                    case fsm::jump_state::PREPARE:
                        yaw_angle_debug[0] = mpu6050_dev.angle[2];
                        ff_rate = jump_turn_prepare_ff;
                        fb_kp = jump_turn_prepare_kp;
                        max_rate = jump_turn_prepare_max_rate;
                        break;

                    case fsm::jump_state::PUSH:
                        yaw_angle_debug[1] = mpu6050_dev.angle[2];
                        ff_rate = jump_turn_push_ff;
                        fb_kp = jump_turn_push_kp;
                        max_rate = jump_turn_push_max_rate;
                        break;

                    case fsm::jump_state::FLY:
                        yaw_angle_debug[2] = mpu6050_dev.angle[2];
                        ff_rate = jump_turn_fly_ff;
                        fb_kp = jump_turn_fly_kp;
                        max_rate = jump_turn_fly_max_rate;
                        break;

                    case fsm::jump_state::LAND:
                        yaw_angle_debug[3] = mpu6050_dev.angle[2];
                        ff_rate = jump_turn_land_ff;
                        fb_kp = jump_turn_land_kp;
                        max_rate = jump_turn_land_max_rate;
                        break;

                    case fsm::jump_state::RECOVER:
                    case fsm::jump_state::DONE:
                        yaw_angle_debug[4] = mpu6050_dev.angle[2];
                        ff_rate = 0.0f;
                        fb_kp = jump_turn_recover_kp;
                        max_rate = jump_turn_recover_max_rate;
                        break;
                }
            }

            jump_turn_yaw_rate_cmd = constrain(
                (float)jump_turn_direction * ff_rate + fb_kp * yaw_error,
                -max_rate,
                max_rate
            );
        }
        else
        {
            jump_turn_yaw_rate_cmd = 0.0f;
            lqi_param.ref.yaw_rate = 0.0f;
            lqi_param.integral.yaw_rate_error = 0.0f;
        }
    }

    switch(fsm_state_machine.jump)
    {
        case fsm::jump_state::PREPARE:
            jump_mode_flag = 1.0f;
            jump_timer = 0;
            jump_recover_elapsed = 0;
            jump_turn_target_yaw = wrap_pi(
                lqi_param.state.yaw_angle + (float)jump_turn_direction * jump_turn_angle
            );
            lqi_param.integral.yaw_rate_error = 0.0f;
            sts3032.set(SERVO_LEFT, SERVO_LEFT_MIN + 60, 450, 250);
            sts3032.set(SERVO_RIGHT, SERVO_RIGHT_MIN - 60, 450, 250);
            sts3032.move();

            fsm_state_machine.jump = fsm::jump_state::PUSH;
            break;

        case fsm::jump_state::PUSH:
            if((jump_timer += tick) >= jump_push_wait_ms && (jump_timer = 0, 1))
            {
                sts3032.set(SERVO_LEFT, SERVO_LEFT_MAX + 20, 0, 0);
                sts3032.set(SERVO_RIGHT, SERVO_RIGHT_MAX - 20, 0, 0);
                sts3032.move();
                fsm_state_machine.jump = fsm::jump_state::FLY;
            }
            break;

        case fsm::jump_state::FLY:
            if((jump_timer += tick) >= 130 && (jump_timer = 0, 1))
            {
                sts3032.set(SERVO_LEFT, SERVO_LEFT_MIN + 60, 0, 0);
                sts3032.set(SERVO_RIGHT, SERVO_RIGHT_MIN - 60, 0, 0);
                sts3032.move();

                fsm_state_machine.jump = fsm::jump_state::LAND;
            }
            break;

        case fsm::jump_state::LAND:
            if((jump_timer += tick) >= 260 && (jump_timer = 0, 1))
            {
                jump_recover_elapsed = 0;
                fsm_state_machine.jump = fsm::jump_state::RECOVER;
            }
            break;

        case fsm::jump_state::RECOVER:
        {
            bool yaw_ready = true;
            bool yaw_can_release = true;
            if(jump_turn_direction != 0)
            {
                float yaw_angle_error = fabsf(shortest_angle_error(jump_turn_target_yaw, lqi_param.state.yaw_angle));
                yaw_ready =
                    yaw_angle_error < jump_turn_ready ||
                    (yaw_angle_error < 18.0f / 180.0f * PI &&
                     fabsf(jump_turn_yaw_rate_cmd) < 0.18f &&
                     fabsf(lqi_param.state.yaw_rate) < 1.2f);
                yaw_can_release =
                    yaw_angle_error < 25.0f / 180.0f * PI ||
                    (fabsf(jump_turn_yaw_rate_cmd) < 0.35f &&
                     fabsf(lqi_param.state.yaw_rate) < 1.4f);
            }

            jump_recover_elapsed += tick;

            const bool posture_ready =
                fabsf(mpu6050_dev.angle[1]) < recover_pitch &&
                fabsf(mpu6050_dev.gyro[1]) < recover_pitch_rate;

            if(posture_ready && yaw_ready)
            {
                jump_timer += tick;
            }
            else
            {
                jump_timer = 0;
            }

            const bool recover_force_exit =
                posture_ready &&
                yaw_can_release &&
                jump_recover_elapsed >= recover_force_exit_ms;

            if(jump_timer >= recover_hold_ms ||
               recover_force_exit ||
               jump_recover_elapsed >= recover_timeout_ms)
            {
                jump_mode_flag = 0.0f;
                jump_timer = 0;
                jump_recover_elapsed = 0;
                jump_linear_direction = 0;
                jump_turn_direction = 0;
                jump_linear_vel_cmd = 0.0f;
                jump_turn_yaw_rate_cmd = 0.0f;
                lqi_param.ref.yaw_rate = 0.0f;
                lqi_param.integral.yaw_rate_error = 0.0f;
                fsm_state_machine.jump = fsm::jump_state::DONE;
            }
            break;
        }

        case fsm::jump_state::DONE:
            break;
    }
}

void controller::loop_proc(uint32_t tick)
{
    ctrl.buttons = ctrl.host_data.buttons;
    memcpy(ctrl.axes, ctrl.host_data.axes, sizeof(ctrl.axes));

    ctrl.fsm_state_machine.update();
    ctrl.fsm_state_machine.loop(tick);
}

void controller::sensor_update_proc(uint32_t tick)
{
    static uint32_t mpu6050_update_cnt = 0;
    if((mpu6050_update_cnt += tick) >= 5 && (mpu6050_update_cnt = 0, 1))
    {
        mpu6050_dev.update();
    }

    left_motor.sensor->update();
    left_motor.electrical_angle = left_motor.electricalAngle();
    right_motor.sensor->update();
    right_motor.electrical_angle = right_motor.electricalAngle();

    ctrl.lqi_param.state.pitch_angle = mpu6050_dev.angle[1];
    ctrl.lqi_param.state.pitch_rate = mpu6050_dev.gyro[1];
    ctrl.lqi_param.state.avg_linear_pos = -(left_motor.shaft_angle + right_motor.shaft_angle) * ctrl.lqi_param.car.r * 0.5f;
    ctrl.lqi_param.state.avg_linear_vel = -(left_motor.shaft_velocity + right_motor.shaft_velocity) * ctrl.lqi_param.car.r * 0.5f;
    ctrl.lqi_param.state.yaw_angle = mpu6050_dev.angle[2];
    ctrl.lqi_param.state.yaw_rate = mpu6050_dev.gyro[2];

    static uint8_t first_run_flag = 1;
    if(first_run_flag && (first_run_flag = 0, 1))
    {
        ctrl.base_components.reset_motion_reference();
    }

    static uint32_t servo_update_cnt = 0;
    if((servo_update_cnt += tick) >= 100 && (servo_update_cnt = 0, 1))
    {
        sts3032.get_position_and_load();
    }
}

void controller::motor_update_proc(uint32_t tick)
{
    if(!ctrl.enable_motor){return;}

    left_motor.setPhaseVoltage(left_motor.voltage.q, left_motor.voltage.d, left_motor.electrical_angle);
    right_motor.setPhaseVoltage(right_motor.voltage.q, right_motor.voltage.d, right_motor.electrical_angle);
}
