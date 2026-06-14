# 嵌入式调试记录

## [2026-06-14] [ESP32/ESP-IDF] - OTA HTTP 连接失败
**标签**：ESP32、ESP-IDF、OTA、HTTP、MQTT
**现象**：MQTT 已收到 OTA 命令，日志出现 `OTA task started`，但随后报错 `Failed to open HTTP connection: ESP_ERR_HTTP_CONNECT`，OLED 无变化。
**根因**：ESP32 无法访问 OTA URL 对应的 HTTP 文件服务器，常见原因包括电脑未开启 `http.server`、IP/端口不匹配、电脑与 ESP32 不在同一局域网、Windows 防火墙拦截。
**解决**：先在电脑浏览器访问 `http://<电脑IP>:8080/test.bin` 验证可下载，再触发 OTA；确认 HTTP 服务在线且 ESP32 与电脑在同一网络。
**教训**：OTA 代码触发成功不代表升级成功，必须看串口中是否出现下载、写分区、重启日志。
**预防规则**：现场 OTA 演示前，先用浏览器验证固件 URL 可直接下载，再用串口确认 `Starting OTA`、`Writing to <ota_x>`、`OTA upgrade successful` 三个关键日志。

## [2026-06-14] [ESP32/ESP-IDF] - 单 OTA 分区导致连续升级冲突
**标签**：ESP32、ESP-IDF、OTA、分区表
**现象**：第一次 OTA 后再次触发 OTA，日志报错 `esp_ota_begin failed (ESP_ERR_OTA_PARTITION_CONFLICT)`，OLED 无变化。
**根因**：分区表只有 `factory + ota_0`，设备运行在 `ota_0` 后，再次 OTA 仍只能选择 `ota_0` 作为更新分区，导致当前运行分区与写入目标冲突。
**解决**：将分区表改为 `factory + ota_0 + ota_1`，每个 app 分区 1MB；擦除 Flash 后重新烧录 bootloader、partition table、ota_data_initial 和 version1 到 factory。
**教训**：如果需要连续 OTA 两次演示，必须至少有两个 OTA app 分区。
**预防规则**：设计 OTA 演示流程前先检查 `partitions.csv`，确认存在 `ota_0` 和 `ota_1`，并确认固件大小小于最小 app 分区。

## [2026-06-14] [ESP32/ESP-IDF] - HTTP 服务目录错误导致 OTA 固件版本不符
**标签**：ESP32、ESP-IDF、OTA、HTTP、演示
**现象**：本地已替换项目目录下的 `test.bin` 为 version2，但设备 OTA 后启动为 version3。
**根因**：8080 端口已有旧的 Python HTTP 服务在运行，实际服务目录不是当前项目目录，ESP32 下载的是旧目录里的 `test.bin`。
**解决**：停止旧 8080 服务，在项目目录重新启动 `python -m http.server 8080`；用浏览器或脚本读取 `http://127.0.0.1:8080/test.bin`，确认其中包含目标版本标识。
**教训**：同名 `test.bin` 可能存在多个目录，不能只看当前项目文件是否替换。
**预防规则**：每次 OTA 前确认 HTTP 服务进程和服务目录，必要时重新启动 HTTP 服务；升级前用版本字符串确认当前 URL 服务的固件版本。
