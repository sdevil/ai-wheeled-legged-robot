# WRobot Web UI

English | [中文](README.zh-CN.md)

This React application is the embedded control interface for WRobot - sdevil Enhanced.

## Commands

    npm install
    npm run dev
    npm run lint
    npm run build:esp

The build:esp command builds the production bundle, compresses the JavaScript, and writes the generated firmware header to ../robot-code/src/generated/WebUiBundle.h.

The ESP32 serves the UI shell at /, the compressed application at /app.js, telemetry and settings over HTTP, and low-latency controls over WebSocket port 81.

Generated dist, local node_modules, and PlatformIO outputs are intentionally excluded from Git.