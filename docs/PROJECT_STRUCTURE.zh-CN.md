# 项目结构

[English](PROJECT_STRUCTURE.md) | 中文

## 目录职责

| 路径 | 职责 |
| --- | --- |
| robot-code/src | ESP32 正式业务代码 |
| robot-code/src/generated | Web UI 自动生成的固件资源，不可手工修改 |
| robot-code/scripts/build_web_ui.py | PlatformIO 预构建脚本，仅在网页源码变化时更新嵌入式 Web UI |
| robot-code/lib/SCServo | 项目内保存的飞特智能舵机库 |
| robot-code/lib/GamepadController | 项目内保存的 Gamepad BLE 支持库 |
| robot-code/scripts | 构建与空间检查脚本 |
| maixcam-code | 作为一个应用整体部署的 MaixCam 代码 |
| servo-code | 仅用于 STS3032 初始设置的 UART 桥固件 |
| web-ui-react/src | React 控制页面源码 |
| docs | 项目文档，不会烧录到任何设备 |

## 命名规则

- Python 模块和函数使用 snake_case。
- C++ 类和模块使用 PascalCase，函数和变量沿用 lower camel case。
- 自动生成的文件统一放入 generated 目录。
- 第三方库保留上游文件名，确保来源和未来升级路径可追踪。
- 实验固件和本地工具不得混入正式业务源码目录。

## 本次整合结论

MaixCam 重复的自动启动脚本已经删除，自动启动统一由 main.py 处理。ESP32 中从未被调用的旧动作序列引擎和已停用的诊断阶段已从正式源码移除。Web、供电、RGB、设置和串口解析具有清晰的硬件职责，强行合并不会减小固件，反而会降低可测试性与可维护性，因此继续保持独立模块。
