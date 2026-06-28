#include "fsm.h"
#include "controller.h"

/**
 * @brief fsm 构造函数
 */
fsm::fsm()
    : mode(mode_state::FIRST_BOOT),
      boot(boot_state::PREPARE),
      sit(sit_state::PREPARE),
      jump(jump_state::PREPARE){}

/**
 * @brief fsm 更新
 */
void fsm::update()
{
    if(!ctrl){return;}

    switch(mode)
    {
        case mode_state::FIRST_BOOT:
            if(boot == boot_state::DONE){mode = mode_state::BALANCE;}
            break;
        
        case mode_state::BALANCE:
            if(ctrl->buttons & BTN_LB)
            {
                mode = mode_state::SIT;
                sit = sit_state::PREPARE;
            }
            else if(ctrl->buttons & BTN_RS)
            {
                ctrl->jump_linear_direction = 0;
                ctrl->jump_turn_direction = 0;
                ctrl->jump_linear_vel_cmd = 0.0f;
                ctrl->jump_turn_yaw_rate_cmd = 0.0f;
                mode = mode_state::JUMP;
                jump = jump_state::PREPARE;
            }
            else if(ctrl->buttons & BTN_Y)
            {
                ctrl->jump_linear_direction = 1;
                ctrl->jump_turn_direction = 0;
                ctrl->jump_linear_vel_cmd = 0.0f;
                ctrl->jump_turn_yaw_rate_cmd = 0.0f;
                mode = mode_state::JUMP;
                jump = jump_state::PREPARE;
            }
            else if(ctrl->buttons & BTN_A) //
            {
                ctrl->jump_linear_direction = -1;
                ctrl->jump_turn_direction = 0;
                ctrl->jump_linear_vel_cmd = 0.0f;
                ctrl->jump_turn_yaw_rate_cmd = 0.0f;
                mode = mode_state::JUMP;
                jump = jump_state::PREPARE;
            }
            else if(ctrl->buttons & BTN_X)   //
            {
                ctrl->jump_linear_direction = 0;
                ctrl->jump_turn_direction = 1;
                ctrl->jump_linear_vel_cmd = 0.0f;
                ctrl->jump_turn_yaw_rate_cmd = 0.0f;
                mode = mode_state::JUMP;
                jump = jump_state::PREPARE;
            }
            else if(ctrl->buttons & BTN_B)
            {
                ctrl->jump_linear_direction = 0;
                ctrl->jump_turn_direction = -1;
                ctrl->jump_linear_vel_cmd = 0.0f;
                ctrl->jump_turn_yaw_rate_cmd = 0.0f;
                mode = mode_state::JUMP;
                jump = jump_state::PREPARE;
            }
            break;

        case mode_state::SIT:   
            if(sit == sit_state::EXIT){mode = mode_state::BALANCE;}
            break;

        case mode_state::JUMP:
            if(jump == jump_state::DONE){mode = mode_state::BALANCE;}
            break;

        case mode_state::ERROR:
            break;
    }
}

/**
 * @brief fsm 循环
 */
void fsm::loop(uint32_t tick)
{
    if(!ctrl){return;}

    switch(mode)
    {
        case mode_state::FIRST_BOOT:
            ctrl->boot_loop(tick);
            break;
        
        case mode_state::BALANCE:  // 平衡状态
            ctrl->enable_balance = 1;
            ctrl->enable_steering = 1;
            ctrl->enable_motor = 1;
            ctrl->leg_loop();
            break;

        case mode_state::SIT:   // 坐下状态
            ctrl->sit_loop(tick);
            break;

        case mode_state::JUMP:  // 跳跃状态 
            ctrl->jump_loop(tick);
            break;

        case mode_state::ERROR:
            ctrl->enable_balance = 0;
            ctrl->enable_steering = 0;
            ctrl->enable_motor = 0;
            ctrl->base_components.reset_motion_reference();
            break;
    }

    // Camera pitch is owned by CameraGimbalController, outside the motion core.
    ctrl->lqi_loop(tick);                       // LQR 循环
}
