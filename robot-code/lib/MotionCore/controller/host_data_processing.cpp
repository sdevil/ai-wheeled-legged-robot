#include "host_data_processing.h"
#include "controller.h"

/**
 * @brief host_data_processing 构造函数
 */
host_data_processing::host_data_processing()
{
    buttons = 0;
    memset(axes, 0, sizeof(axes));
}

/**
 * @brief 初始化
 */
void host_data_processing::init()
{
    uart_bus_init(&uart0);
}

/**
 * @brief 发送数据
 */
void host_data_processing::send(uint32_t tick)
{
    if(!ctrl){return;}

    // 发送频率 50Hz
    if((send_timer += tick) < 20){return;}
    send_timer = 0;

    uint32_t idx = 0;

    // 包头
    send_buffer[idx++] = 0xFF;
    send_buffer[idx++] = 0xAA;

    // 长度先占位
    idx++;

    // pitch 角度
    send_buffer[idx++] = 0x01;
    memcpy(&send_buffer[idx], &ctrl->x_debug[0], sizeof(float));
    idx += sizeof(float);

    // pitch 角速度
    send_buffer[idx++] = 0x02;
    memcpy(&send_buffer[idx], &ctrl->x_debug[1], sizeof(float));
    idx += sizeof(float);

    // 线速度误差
    send_buffer[idx++] = 0x03;
    memcpy(&send_buffer[idx], &ctrl->x_debug[2], sizeof(float));
    idx += sizeof(float);

    // yaw 角速度误差
    send_buffer[idx++] = 0x04;
    memcpy(&send_buffer[idx], &ctrl->x_debug[3], sizeof(float));
    idx += sizeof(float);

    // 线速度误差积分
    send_buffer[idx++] = 0x05;
    memcpy(&send_buffer[idx], &ctrl->x_debug[4], sizeof(float));
    idx += sizeof(float);

    // yaw 角速度误差积分
    send_buffer[idx++] = 0x06;
    memcpy(&send_buffer[idx], &ctrl->x_debug[5], sizeof(float));
    idx += sizeof(float);

    // 目标线速度
    send_buffer[idx++] = 0x07;
    memcpy(&send_buffer[idx], &ctrl->input_debug[0], sizeof(float));
    idx += sizeof(float);

    // 目标转向速度
    send_buffer[idx++] = 0x08;
    memcpy(&send_buffer[idx], &ctrl->input_debug[1], sizeof(float));
    idx += sizeof(float);

    // 总输出（左轮）
    send_buffer[idx++] = 0x09;
    memcpy(&send_buffer[idx], &ctrl->output_debug[0], sizeof(float));
    idx += sizeof(float);

    // 总输出（右轮）
    send_buffer[idx++] = 0x0A;
    memcpy(&send_buffer[idx], &ctrl->output_debug[1], sizeof(float));
    idx += sizeof(float);

    // 线位移
    send_buffer[idx++] = 0x0B;
    memcpy(&send_buffer[idx], &ctrl->lqi_param.state.avg_linear_pos, sizeof(float));
    idx += sizeof(float);

    // 坐立标志位
    send_buffer[idx++] = 0x0C;
    memcpy(&send_buffer[idx], &ctrl->sit_mode_flag, sizeof(float));
    idx += sizeof(float);

    // 跳跃标志位
    send_buffer[idx++] = 0x0D;
    memcpy(&send_buffer[idx], &ctrl->jump_mode_flag, sizeof(float));
    idx += sizeof(float);

    // 偏航角 PREPARE
    send_buffer[idx++] = 0x0E;
    memcpy(&send_buffer[idx], &ctrl->yaw_angle_debug[0], sizeof(float));
    idx += sizeof(float);

    // 偏航角 PUSH
    send_buffer[idx++] = 0x0F;
    memcpy(&send_buffer[idx], &ctrl->yaw_angle_debug[1], sizeof(float));
    idx += sizeof(float);

    // 偏航角 FLY
    send_buffer[idx++] = 0x10;
    memcpy(&send_buffer[idx], &ctrl->yaw_angle_debug[2], sizeof(float));
    idx += sizeof(float);

    // 偏航角 LAND
    send_buffer[idx++] = 0x11;
    memcpy(&send_buffer[idx], &ctrl->yaw_angle_debug[3], sizeof(float));
    idx += sizeof(float);

    // 偏航角 DONE
    send_buffer[idx++] = 0x12;
    memcpy(&send_buffer[idx], &ctrl->yaw_angle_debug[4], sizeof(float));
    idx += sizeof(float);

    // 填写长度
    send_buffer[2] = idx + 1;

    // 校验和
    uint8_t checksum = 0;
    for(uint32_t i = 2; i < idx; i++)
    {
        checksum += send_buffer[i];
    }
    send_buffer[idx] = checksum;
    idx++;

    uart0.write_bytes(&uart0, send_buffer, idx);
}

/**
 * @brief 更新数据
 */
void host_data_processing::update()
{
    if(!ctrl){return;}

    uint8_t tmp[32];
    uint32_t len = uart0.read_bytes(&uart0, tmp, sizeof(tmp));
    if(!len){return;}

    // 防止溢出
    if(rx_len + len > sizeof(recv_buffer))
    {
        rx_len = 0;
        return;
    }

    memcpy(&recv_buffer[rx_len], tmp, len);
    rx_len += len;

    // 解析数据
    parse_rx_buffer();
}

/**
 * @brief 解析接收缓冲区
 */
void host_data_processing::parse_rx_buffer()
{
    uint32_t idx = 0;
    while(rx_len - idx >= 5)
    {
        if(recv_buffer[idx] != 0xFF || recv_buffer[idx + 1] != 0xAA)
        {
            idx++;
            continue;
        }

        uint8_t type = recv_buffer[idx + 2];
        uint8_t payload_len = recv_buffer[idx + 3];
        uint32_t frame_len = 2 + 1 + 1 + payload_len + 1;

        // 不完整
        if(rx_len - idx < frame_len){break;}

        // 校验和
        uint8_t checksum = 0;
        for(uint32_t i = 4; i < 4 + payload_len; i++)
        {
            checksum += recv_buffer[i + idx];
        }

        if(checksum != recv_buffer[idx + frame_len - 1])
        {
            idx++;      // 校验失败丢一个字节
            continue;
        }

        // 成功解析一帧
        handle_frame(&recv_buffer[idx], frame_len);
        idx += frame_len;
    }

    // 移动剩余数据
    if(idx > 0)
    {
        memmove(recv_buffer, &recv_buffer[idx], rx_len - idx);
        rx_len -= idx;
    }
}

/**
 * @brief 处理一帧数据
 */
void host_data_processing::handle_frame(uint8_t *frame, uint32_t len)
{
    uint8_t type = frame[2];

    switch(type)
    {
        case 0x01:
            parse_xbox(frame);
            break;
    }
}

/**
 * @brief 处理摇杆数据
 */
void host_data_processing::parse_xbox(uint8_t *frame)
{
    uint8_t payload_len = frame[3];
    if(payload_len < 14){return;}

    uint8_t *p = &frame[4];

    buttons = (uint16_t)(p[0] | (p[1] << 8));

    int16_t axes_tmp[6];
    for(int i = 0; i < 6; i++)
    {
        axes_tmp[i] = (int16_t)(p[2 + i * 2] | (p[3 + i * 2] << 8));
        axes[i] = (float)axes_tmp[i] * 1.0e-3f;
    }
}

/**
 * @brief host_data_processing 数据更新进程函数
 */
void host_data_processing::host_data_update_proc(uint32_t tick)
{
    ::ctrl.host_data.send(tick);
    ::ctrl.host_data.update();
}
