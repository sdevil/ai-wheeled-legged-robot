# 固件版本记录

## 2026-06-29 Stable 版本

这是本仓库第一个公开稳定基线：

- 主板固件版本：`3.2.111`
- 主板固件文件：`robot-code/.pio/build/esp32dev/wrobot_firmware_3.2.111.bin`
- MaixCam 应用版本：`3.2.29`
- MaixCam 应用源码：`maixcam-code/`
- Git tag：`stable-2026-06-29`

这个 stable 点把当前 Web UI、WiFi/captive portal、OTA/设置系统、
手柄切换、MaixCam 视频预览、目标识别标识/锁定 UI、摄像头俯仰追踪和
MotionCore 集成作为一组已测试基线保存。

说明：

- 运行时 WiFi 凭据不保存在源码中。
- 默认 gamepad MAC 为空；用户需要在 Web UI 设置页填写自己的手柄 MAC。
- 生成的主板 OTA `.bin` 固件文件被 Git 忽略，需要时从源码重新编译。

## 主板固件 3.2.82 源码回滚版

- 文件：`robot-code/.pio/build/esp32dev/wrobot_firmware_3.2.82.bin`
- 行为基线：根据 Codex patch 日志，把主板运动源码恢复到 3.2.73 的行为
- 新上报版本号：`3.2.82`

这是源码回滚，不是二进制 patch。3.2.73 之后引入的追踪和运动适配层改动已经从源码中移除，然后重新编译为新版本号，因此不会覆盖原始 3.2.73。
