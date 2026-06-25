# MaixCam 视觉模型

中文 | [English](VISION_MODELS.md)

## 类别化追踪引擎

当前 MaixCam 固件只支持类别化追踪：

- 人脸：优先尝试 `yolov8n_face.mud`，然后是 `retinaface.mud`，最后是 `face_detector.mud`。
- 动物：使用已安装的 COCO 检测模型，优先 `yolo11n.mud`，失败时回退到 `yolov8n.mud`。支持 bird、cat、dog、horse、sheep、cow、elephant、bear、zebra、giraffe。
- 乒乓球 / 小球：如果安装了专用乒乓球模型则优先使用，否则使用 COCO 的 `sports ball` 类，并上报为 `PING_PONG_BALL`。

Web UI 框选目标后，MaixCam 必须在框内检测到用户指定类别，才会开始追踪。比如用户选择 Face，但框内没有检测到人脸，MaixCam 会拒绝本次选择，而不是回退到通用视觉追踪器。

## 为什么暂时禁用任意物体追踪

当前 MaixCam 硬件支持 NanoTrack，但项目测试中它容易从选中目标漂移到附近背景物体。Sipeed 文档也显示 MixFormerV2 不支持 MaixCAM / MaixCAM-Pro，它是 MaixCAM2 的路线。

因此当前版本禁用任意物体追踪，避免暴露一个不稳定的控制模式。未来如果要恢复任意物体追踪，应使用经过验证的自定义目标检测模型，或升级到支持更强追踪模型的硬件。

## 运行方式

检测模型不会每帧运行。MaixCam 会周期性校验目标类别，并把归一化目标偏差、距离误差、置信度和标签发送给 ESP32。ESP32 仍然负责运动安全，并把目标误差转换为受限的高层运动命令。
