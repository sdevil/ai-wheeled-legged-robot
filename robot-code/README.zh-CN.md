# ESP32 主板固件

[English](README.md) | 中文

此 PlatformIO 工程是 ESP32-WROOM-32 主板的实时固件，负责平衡控制、FOC、智能舵机、动作、安全、Web/WiFi 控制、手柄输入、设置和 OTA。

手柄映射：RB 站起，LB 坐下；按住十字上/下同时伸缩两条腿，松开后保持当前腿高；按住十字左/右向对应方向侧下身。左摇杆以高速档移动，右摇杆控制摄像头俯仰与底盘原地转向。Web 与 Gamepad 移动共用同一条归一化 `MotionCommand` 控制路径。Web UI 中，腿高使用与固件实时同步的绝对 slider，左右侧身使用松手回中的 slider。

RGB 状态灯在启动期间保持熄灭；电池、MPU6050、两只智能舵机、运动核心和控制接口通过就绪检查后快速绿闪三次；只有电量百分比低于或等于 10% 时才进入红色慢呼吸。独立的 GPIO13 绿色单色灯在 Gamepad 蓝牙模式下常亮，在 WiFi/Web 模式下熄灭。

## 刷写说明

在 VS Code 中打开此 PlatformIO 工程后，可以直接使用 Build 和 Upload，也可以运行 `pio run` 或 `pio run --target upload`。

通过 USB 刷主板固件时，建议临时断开 MaixCam 的 UART TX/RX 线，避免串口噪声导致 `Serial data stream stopped` 或 `Failed to write to target RAM`。

当前主板 Flash 为 4 MB，使用 `min_spiffs.csv` 分区表提供两个约 1.875 MB 的 OTA app 分区和最小 SPIFFS。可运行 `scripts/size-report.ps1` 检查固件体积和 Web UI 占用。
