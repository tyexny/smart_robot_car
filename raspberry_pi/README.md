# 树莓派本地识别与录像

YOLO 与 VLM 在树莓派运行，完整安装和启动步骤见工程根目录 [STARTUP_GUIDE.md](../STARTUP_GUIDE.md)。本目录使用本地模型入口，无需电脑推理服务或 DeepSeek API。网页语音功能按用户要求保留浏览器原有实现。

YOLO 权重在 `best_ncnn_model`，采用固定 320×320 输入。`model_config.py` 将 13 个模型类别映射到 FPGA 的 COCO 协议编号。跟踪来源、版本与许可证见 [UPSTREAM.md](UPSTREAM.md)。

本地 VLM 的模型与视觉投影权重在 `models/lfm2.5-vl-450m`，许可证在该目录的 `LICENSE`。先按启动指南运行本机 `llama-server`，再从工程根目录运行 `bash start_local_pi.sh --record`；仅 YOLO 可运行 `bash start_yolo_pi.sh --record`。

只运行一个图像接收进程；录像在同一个启动命令后添加 `--record`。按一次 Ctrl+C 后等待保存完成。录像及对应 JSONL 存在 `recordings`；纯录像可在本目录运行 `bash launch.sh --no-inference --record`。

VLM 的英文标签与目标框通过 FBOX v3 返回 FPGA，供 HDMI 和带框录像显示。温检继续使用锁定目标 3 秒自动开启规则，以及网页手动开关和 40/50/60°C 阈值。

有线地址为 Pi `192.168.10.1/24`、FPGA `192.168.10.2`，视频 UDP 5000、回传 UDP 5001；网页 TCP 8088、本机 VLM 服务 TCP 8080。网页使用日志打印的完整地址，Wi-Fi 地址以树莓派当前网络为准。

当前代码与固件复查状态见 [BUILD_RESULTS.md](../docs/BUILD_RESULTS.md)。代码测试、RTL 仿真和实机验证分别记录，尚未通过的构建不会作为时序通过版交付。
