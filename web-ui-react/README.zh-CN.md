# WRobot Web UI

[English](README.md) | 中文

该 React 应用是 WRobot - sdevil Enhanced 嵌入主板固件的控制界面。

## 常用命令

    npm install
    npm run dev
    npm run lint
    npm run build:esp

build:esp 会构建正式版、压缩 JavaScript，并把生成的固件头文件写入 ../robot-code/src/generated/WebUiBundle.h。

ESP32 在 / 提供页面外壳，在 /app.js 提供压缩后的应用，通过 HTTP 提供遥测与设置，并通过 81 端口的 WebSocket 提供低延迟控制。

生成的 dist、本地 node_modules 和 PlatformIO 构建输出不会提交到 Git。