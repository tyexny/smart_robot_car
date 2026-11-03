# 智能小车：FPGA 与树莓派本地模型版

YOLO（NCNN）与 VLM 图像模型在树莓派运行。本目录的启动入口无需电脑推理服务或云端 API 密钥。模型文件保留在 `raspberry_pi/best_ncnn_model` 与 `raspberry_pi/models`。

从 GitHub 获取完整工程时先安装 Git LFS，并下载实际模型及固件文件：

```bash
git lfs install
git clone https://github.com/tyexny/smart_robot_car.git
cd smart_robot_car
git lfs pull
```

模型和固件通过 Git LFS 保存；网页中的指针文本不能代替实际文件。部署目录可自行选择，启动指南中的 `~/yolo_project` 是示例路径。

安装、模型服务、网页与热成像的使用见 [STARTUP_GUIDE.md](STARTUP_GUIDE.md)。纯 YOLO 可用 `bash start_yolo_pi.sh --record`；本地 VLM 服务启动后，用 `bash start_local_pi.sh --record` 启动组合模式。

建图保持任务四前版本。温检保留用户的锁定 3 秒自动开启设定、40/50/60°C 网页阈值及手动开关。

当前随包固件是本轮已完成方案中最好的一组，最差内部 setup 为 **−0.088 ns，尚未全部时序通过**。本轮已按用户要求结束并暂停。验证结果、固件哈希与已知限制见 [BUILD_RESULTS.md](docs/BUILD_RESULTS.md)。
