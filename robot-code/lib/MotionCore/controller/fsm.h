#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

// 前向声明
class controller;

class fsm {
    public:
        enum class mode_state {
            FIRST_BOOT = 0,
            BALANCE,
            SIT,
            JUMP,
            ERROR
        };

        enum class boot_state {
            PREPARE = 0,
            WAIT_FOR_SIGNAL,
            INIT,
            INIT_PREPARE,
            INIT_RECOVER,
            DONE
        };

        enum class sit_state {
            PREPARE = 0,
            MOVING,
            DONE,
            EXIT_PREPARE,
            EXIT_RECOVER,
            EXIT
        };

        enum class jump_state {
            PREPARE = 0,
            PUSH,
            FLY,
            LAND,
            RECOVER,
            DONE
        };

    public:
        mode_state mode;
        boot_state boot;
        sit_state sit;
        jump_state jump;
    
    public:
        fsm();
    
    private:
        controller *ctrl = nullptr;
    
    public:
        void bind(controller &ctrl){this->ctrl = &ctrl;}

    public:
        void update();
        void loop(uint32_t tick);
};

#endif
