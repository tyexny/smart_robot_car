# 树莓派本地版启动

FPGA 工程为 `ov5640_ddr3_hdmi.gprj`。建图保持任务四之前的原逻辑；避障与温检保留当前整合版本。当前固件为 `impl/pnr/ov5640_ddr3_hdmi.fs`，使用 place 3 / route 0，最差内部 setup 仍差 0.088 ns，尚未全部时序通过。已按用户要求结束本轮并暂停；具体状态见 `docs/BUILD_RESULTS.md`。

树莓派与 FPGA 的有线地址分别为 `192.168.10.1/24`、`192.168.10.2`；电脑/手机通过树莓派实际 Wi-Fi 地址访问网页。

首次安装：

```bash
cd ~/yolo_project/raspberry_pi
bash install.sh
make -C mlx90640
sudo apt install ffmpeg
```

纯 YOLO：

```bash
cd ~/yolo_project/raspberry_pi
source .venv/bin/activate
bash launch.sh --web-port 8088 --task-console --record
```

如果沿用已有 `~/yolo_project/venv`，激活该环境后启动即可。

也可在工程根目录运行 `bash start_yolo_pi.sh --record`。本地版的启动脚本不再要求电脑服务、`TASK6_PC_URL` 或 `TASK6_TOKEN`。

执行本地 VLM 寻找任务时，另一个终端启动模型服务：

```bash
cd ~/yolo_project/raspberry_pi
~/llama.cpp/build/bin/llama-server \
  -m ./models/lfm2.5-vl-450m/LFM2.5-VL-450M-Q4_K_M.gguf \
  --mmproj ./models/lfm2.5-vl-450m/mmproj-LFM2.5-VL-450m-Q8_0.gguf \
  --alias task6-vlm --host 127.0.0.1 --port 8080 -c 2048 -t 3
```

然后把前面的纯 YOLO 接收命令换为以下命令，接收程序仍只启动一份：

```bash
bash launch.sh --vlm-url http://127.0.0.1:8080/v1/chat/completions \
  --web-port 8088 --task-console --record
```

仅用 VLM 时再加 `--vlm-only`；纯录屏用 `bash launch.sh --no-inference --record`。模型权重已保留，`llama-server` 使用本机安装的版本。

本地模型服务启动后，也可在工程根目录运行 `bash start_local_pi.sh --record`，同时提供本地 YOLO 和 VLM 功能。`start_vision_pi.sh`、`start_target_pi.sh` 和 `start_task6_pi.sh` 在本提交目录内均为这个本地入口的兼容名称。

网页使用启动日志打印的完整地址，不能只填写 IP。打开 FPGA 的 SW4 后才发送用于识别/录屏的图像。SW2 显示前方距离曲线；切换到其他 HDMI 页面时录像来源自动跟随。接收程序只运行一份，停止录屏按一次 Ctrl+C 并等待收尾信息。

热成像：MLX90640 SDA 接 FPGA E13、SCL 接 E14，电源和地按模块资料连接。由 FPGA 通过网络转发原始寄存器，树莓派完成校准和测温。网页支持手动开关及 40/50/60°C 阈值，默认 40°C。按用户当前设定，带温检语义的任务连续锁定目标 3 秒后自动开启检测，也保留 FPGA 到达确认通道；新任务清除前一任务状态，丢锁会重置锁定计时。网页仍可手动开启。示例：“寻找蓝色小球并移动到它附近，并施行温度检测，判断是否异常发热”。

建图：已整套恢复到任务四之前的连续采集、匹配、写图和回环调度，撤回新增的运动写图门控、转弯后停稳等待和停稳重对齐要求；原版初始关键帧等条件保持原规则。实际效果还取决于雷达安装、时间同步、轮速及航向标定。
