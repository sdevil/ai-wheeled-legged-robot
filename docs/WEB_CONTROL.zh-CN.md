# WiFi 网页控制

中文 | [English](WEB_CONTROL.md)

## 连接方式

```text
WiFi名称: sdevil-WRobot
密码:     wrobot123
网址:     http://192.168.8.1
本地域名: http://wrobot.local
```

ESP32 启动时保留 AP 热点，同时可在后台连接家庭 WiFi。连接机器人热点后，captive portal 会自动打开控制页；也可以访问 `http://wrobot.local`。如果系统没有弹出 portal，使用固定地址 `http://192.168.8.1`。

## 当前控制功能

- 起立和坐下
- 恢复机身姿态调整
- 前进、后退、左转和右转
- 10 到 100 的速度限制
- 立即停止
- 显示机器人、Gamepad、平衡角、电量百分比、电池电压、维护模式和 OTA 状态
- 基础设置：机器人名称、控制模式、家庭 WiFi、摄像头流地址和分辨率
- 高级维护中提供 OTA 固件选择、上传进度和固件回滚操作
- 原地、前、后、左、右跳跃

## WebSocket 实时控制

Web UI 会自动连接 `ws://<机器人地址>:81/ws`。摇杆、云台和动作命令优先通过 WebSocket 发送；连接尚未建立或断开时自动回退到保持命令顺序的 HTTP 通道。网络状态卡会显示当前实际通道。第二台控制设备无法取得唯一实时连接时，会在 1.5 秒后自动回退，而不是一直卡在连接中。

客户端消息格式：

```json
{"type":"drive","x":25,"y":60}
{"type":"gimbal","yaw":20,"pitchDelta":-2}
{"type":"action","name":"stand"}
```

摇杆位置变化时立即发送命令，按住期间每 32ms 刷新一次当前值，松手立即发送零值。ESP32 超过 1 秒未收到运动刷新时自动清除命令；WebSocket 断开时也会立即清除运动命令。较宽的看门狗可避免 HTTP fallback 抖动造成电机走停脉冲，同时保留明确的停车机制。状态和设置仍通过 HTTP 获取。

移动和云台控制默认使用高速档。Web 与 Gamepad 输入使用相同的归一化轴范围，并进入同一条 `MotionCommand` 控制路径。

按住两个摇杆之间的上/下按钮可同时伸腿或缩腿，松开后保持当前高度。按住移动摇杆下方的左/右按钮可向对应方向侧身，松开后平滑回正。这些控制每 100ms 刷新一次；超过 350ms 未刷新、WebSocket 断开或页面进入后台时会自动清除。

## 摄像头画面

网页不会通过 ESP32 中转视频。MaixCam 在端口 `8080` 提供：

- `/stream.mjpg`：低延迟 MJPEG 连续视频
- `/snapshot.jpg`：单帧快照回退
- 160×120 远程操控：40ms 编码间隔、JPEG 质量 62，优先降低带宽和延迟
- 320×240 低延迟：45ms 编码间隔、JPEG 质量 80，目标约 22 FPS
- 640×480（默认）：75ms 编码间隔、JPEG 质量 86，目标约 13 FPS
- 1280×720：140ms 编码间隔、JPEG 质量 82，目标约 7 FPS

视频发送使用非阻塞分片，网络暂时变慢时只保留最新等待帧，避免旧帧堆积造成越来越大的延迟。实际帧率仍取决于 MaixCam 检测模型负载和 WiFi 状况。

在设置中更改分辨率并保存后，ESP32 会通过 UART 下发配置。只有分辨率确实发生变化时，MaixCam 才会保存配置并自动重启，重连家庭 WiFi 后使用新分辨率恢复视频。

## WiFi 隐私

- 仓库中的 `maixcam-code/config.py` 的 `WIFI_SSID` 和 `WIFI_PASSWORD` 必须保持空字符串，不要填写真实凭据
- 用户只需在网页保存一次家庭 WiFi；ESP32 在本地保存并同步到 MaixCam
- MaixCam 会把真实凭据保存在设备本地 `/root/wrobot_wifi.json`
- MaixCam 以原子方式写入该文件并设置为仅所有者可读写，WiFi 命令日志会隐藏密码
- 该文件不在仓库中，不应被导出、打包或提交
- ESP32 网页端可以保存相机 `SSID`，但真实密码应视为运行期设备数据，而不是源码内容

## OTA 安全策略

### OTA 升级流程

1. 使用 PlatformIO 编译主板固件。
2. OTA 文件使用 `robot-code/.pio/build/esp32dev/wrobot_firmware_<版本号>.bin`，
   例如 `wrobot_firmware_3.2.51.bin`。
3. 打开机器人 Web UI，进入**设置**，再进入**维护模式**。
4. 确认机器人已经坐下，并处于停机状态。
5. 确认电量不低于 `30%`；如果使用 USB 或外部稳定供电，需要在页面中确认。
6. 在**高级维护**中选择生成的 `wrobot_firmware_<版本号>.bin` 并上传。
7. OTA 过程中不要关闭或刷新页面。页面会保持锁定，直到 ESP32 重启、上报新的 boot ID，并重新可访问。
8. 重连后，在设置页底部确认固件版本已经更新。

- OTA 只能在维护模式下进行
- 未进入维护模式时不能选择固件，所有 OTA 操作均保持禁用；设置页使用独立边框区域明确标识维护功能
- 升级前要求机器人处于坐下/停机状态
- 升级前要求网页显示的电量百分比不低于设定值，默认 `30%`
- OTA 上传期间网页运动和动作命令会被锁定
- 浏览器在上传前发送固件的准确字节数，ESP32 据此计算进度，并在重启前拒绝不完整的固件
- 如果重启导致 HTTP 响应中断，网页会验证新的 `boot_id`，而不是立即显示连接失败
- `/api/status` 同时提供主板与 MaixCam 的版本和构建元数据；设置页最底部只显示 `Firmware <版本>` 和 `Camera <版本>`
- 摄像头版本通过 UART 上报；主板和 MaixCam 程序都更新后才会显示实际版本，否则显示 `unknown`
- 固件使用双 OTA 分区，上传写入的是非当前运行分区
- 如果上传中断电，设备仍会保留原先可启动固件
- 新固件首次启动后会确认运行状态；若首次启动异常，ESP32 OTA 回滚机制可退回到旧版本
- 设置 API 只返回密码是否已保存，绝不会返回已保存的 WiFi 密码

## HTTP API

| 方法 | 地址 | 用途 |
| --- | --- | --- |
| `GET` | `/` | 内置控制页面 |
| `GET` | `/api/status` | 机器人状态 JSON |
| `GET` | `/api/settings` | 当前设置 JSON |
| `POST` | `/api/settings` | 保存基础设置 |
| `POST` | `/api/drive?x=-100..100&y=-100..100` | 运动命令 |
| `POST` | `/api/legs/height?direction=-1..1` | 按住/松开双腿高度方向 |
| `POST` | `/api/legs/lean?percent=-100..100` | 按住/松开侧身控制 |
| `POST` | `/api/action?name=stand` | 起立 |
| `POST` | `/api/action?name=sit` | 坐下 |
| `POST` | `/api/action?name=reset` | 恢复姿态调整 |
| `POST` | `/api/action?name=maintenance_on` | 进入维护模式 |
| `POST` | `/api/action?name=maintenance_off` | 退出维护模式 |
| `POST` | `/api/ota/upload` | 上传固件 |
| `POST` | `/api/ota/rollback` | 回滚到上一个固件 |

状态返回示例：

```json
{"enabled":true,"sitting":false,"gamepad":true,"maintenance":false,"ota_running":false,"angle":1.5,"battery_voltage":8.12,"battery_percent":83,"ota_progress":0,"ota_message":"待机","clients":1}
```

## 后续扩展规则

新增页面动作时，应通过经过校验的 API 和高级动作枚举接入，并在 ESP32 主循环中
消费动作。禁止在 HTTP 回调函数中直接操作电机或舵机驱动。

当前固件已使用默认应用分区约 90% 的 Flash。后续较大的网页资源应压缩、放入
文件系统，或在重新评估分区布局后由外部伴随设备提供。

## 类别化追踪

1. 确认页面已经显示实时摄像头画面。
2. 在“常用动作”中点击“追踪”。
3. 选择 Face、Animal 或 Ping-pong ball。
4. 直接在视频中的目标外拖动一个方框，松手完成选择。
5. 再次点击“追踪”即可停止。

MaixCam 必须在框内检测到用户选择的类别后才会开始追踪；如果没有检测到对应类别，机器人保持静止。浏览器会排除视频留黑区域并发送归一化目标框，因此切换摄像头分辨率或手机布局后仍能正确选择。ESP32 把 TRACKROI 坐标转发给 MaixCam，取消时发送 TRACKSTOP。
