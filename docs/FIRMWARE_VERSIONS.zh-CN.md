# 固件版本记录

## 主板固件 3.2.82 回滚版

- 文件：`robot-code/.pio/build/esp32dev/wrobot_firmware_3.2.82_rollback_3.2.73.bin`
- 备份副本：`local_firmware_backup/wrobot_firmware_3.2.82_rollback_3.2.73.bin`
- 行为基线：`wrobot_firmware_3.2.73.bin`
- 新上报版本号：`3.2.82`
- SHA256：`6E9E3B26099C9C042BC5DE3E3BB71D0833F673A3324AFF3E251245E99BA6B284`

这是基于已知稳定 3.2.73 二进制生成的回滚固件产物。
原始 3.2.73 文件没有被覆盖。
更新上报版本号后，已经重新生成 ESP32 image checksum 和 appended digest。
