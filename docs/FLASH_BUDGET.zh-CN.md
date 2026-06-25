# Flash 与内存预算

中文 | [English](FLASH_BUDGET.md)

## 哪些内容会进入 ESP32

只有经过编译并链接到固件的内容才会占用 ESP32 应用分区：

- C/C++ 代码和实际链接到的库代码
- `WebController.cpp` 中的常量字符串和内置网页
- 被固件明确引用的静态数据

`README.md`、`docs/` 中的文档、Word/PDF 文件和源码注释都不会被 PlatformIO
烧录进 ESP32，因此不会占用芯片 Flash。

## 分区布局

固件使用 Arduino ESP32 的 `min_spiffs.csv`：

| 分区 | 大小 | 用途 |
| --- | ---: | --- |
| 应用槽 0 | 1,966,080 字节 | 当前或 OTA 固件 |
| 应用槽 1 | 1,966,080 字节 | 备用 OTA 固件 |
| SPIFFS | 131,072 字节 | 可选的小型数据或资源 |
| NVS 与系统数据 | 保留 | 参数和 ESP32 系统信息 |

该方案保留未来 OTA 能力，同时比默认的 1,310,720 字节应用槽增加约 50% 空间。

## 当前基线

当前主体固件在保留 OTA 双分区、Web UI、WiFi、WebSocket、MaixCam 通信和 Gamepad 蓝牙代码后的测量结果：

```text
固件 bin:        1,400,048 字节
应用分区占用:     1,393,473 / 1,966,080 字节（70.9%）
剩余应用空间:     约 559KB
RAM:             63,132 / 327,680 字节（19.3%）
React Web UI JS: 148,153 字节 gzip / 476,501 字节 raw
```

旧版内嵌 HTML 已删除。Markdown 文档、`docs/`、`node_modules/`、`.pio/` 不会被 push 或烧录进 ESP32。

本地体积检查命令：

```powershell
powershell -ExecutionPolicy Bypass -File robot-code\scripts\size-report.ps1
```

## 空间预算规则

1. 在可行情况下将发布固件控制在应用分区的 70% 以下。
2. 每个主要功能都记录 Flash 和 RAM 变化。
3. 添加新依赖前优先复用已有库。
4. AI 模型、图片、音频和大型 UI 资源优先放在 MaixCam 或外部 Agent。
5. 占用明显的可选功能应提供编译期开关。
6. ESP32 只保留紧凑控制页，复杂仪表盘交给伴随设备。
7. 发布前使用分级日志替代大量串口调试字符串。
8. 单次增长超过 20KB 时检查 linker map 和最大符号。

## 优化重点

当前最大自有静态数据是 `WEB_UI_APP_JS_GZ`，约 146KB。后续优化优先级：

1. 保持动作、PID、运动控制优先走新控制器，删除冲突旧逻辑。
2. Web UI 继续压缩：减少 MUI/MUI Icons 依赖，必要时改为轻量 CSS + inline icons。
3. 可选功能使用编译期开关：如纯 WiFi 构建、Gamepad 构建、维护/诊断构建。
4. 串口调试字符串按日志等级编译，发布版关闭详细日志。
5. AI 模型、图片、音频和复杂仪表盘不放 ESP32 固件。
6. 单次功能增长超过 20KB 必须检查 `firmware.map` 和最大符号。

当前不建议直接编译掉 Gamepad 蓝牙，因为项目仍需要 WiFi/手柄控制切换。若空间继续紧张，再做双固件环境：`robot_wifi` 和 `robot_gamepad`。