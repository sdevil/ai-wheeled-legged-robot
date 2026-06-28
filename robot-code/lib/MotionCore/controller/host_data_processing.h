#ifndef HOST_DATA_PROCESSING_H
#define HOST_DATA_PROCESSING_H

#include <Arduino.h>

// 前向声明
class controller;

class host_data_processing {
    public:
        host_data_processing();

    public:
        void init();
        void send(uint32_t tick);
        void update();

    public:
        uint16_t buttons;
        float axes[6];

    public:
        void bind(controller &ctrl){this->ctrl = &ctrl;}

    public:
        static void host_data_update_proc(uint32_t tick);

    private:
        controller *ctrl = nullptr;

    private:
        void parse_rx_buffer();
        void handle_frame(uint8_t *frame, uint32_t len);
        void parse_xbox(uint8_t *frame);

    private:
        uint32_t send_timer = 0;

        uint32_t rx_len = 0;
        uint8_t send_buffer[256];
        uint8_t recv_buffer[256];
};

#endif
