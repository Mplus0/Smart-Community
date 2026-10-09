# P2-A 代码与环境检查记录

> 以下为历史 P2-A 记录，不代表新增交通灯代码已验证。2026-10-09 的交通灯接入仅完成文件编辑和回归准备；按用户要求未运行新增代码、容器回归或 Gazebo。模型复制也待执行。当前状态及命令见 [traffic_light_validation.md](traffic_light_validation.md)。

用户后续明确要求只完成代码编写和环境检查，识别效果由用户自行测试。因此本轮不启动人物/车牌识别节点，不执行真实模型推理，不报告识别准确率、实际识别帧率或推理延迟。

## 已完成的静态检查

全部 Python/ROS/catkin 执行位于 `smart-community-dev` 容器，Linux 用户 `developer`；宿主机仅编辑文件、Git/哈希/字节比较及 Docker 操作。

- `/usr/bin/python3 tests/check_static.py`：退出 0，脚本 AST 语法、执行权限、package/三个 launch XML、五模型大小与 SHA256 均通过。
- `catkin_make -j2`：退出 0，新包与既有工作空间构建成功，日志 `/tmp/p2a-build.log`。
- `rospack find robot_perception`：退出 0，定位 `/workspace/car_2026/src/robot_perception`。
- `roslaunch --nodes robot_perception perception.launch`：仅 `/person_detection`、`/plate_recognition`。
- `enable_person:=false`：仅 `/plate_recognition`；`enable_plate:=false`：仅 `/person_detection`。
- `enable_person:=false show_images:=true`：仅 `/plate_recognition`、`/plates_preview`。
- 上述都是 launch 解析，不会启动推理节点。
- 宿主机 `cmp` 对比五个模型与用户解压目录：全部相同；`git diff --check` 通过。

| 模型 | 字节数 | SHA256 |
|---|---:|---|
| person_best.pt | 5375301 | 4d3076a9ca0afd9a35f9d393d0726d9e4207a45155348fb768fccd3d10e28806 |
| y5fu_320x_sim.onnx | 2343161 | 2a985dc63a5cc947ec36d18503d6fc0fd54525b9dba17f2fe29a71e86b44456d |
| y5fu_640x_sim.onnx | 3930791 | 0306de937471b87f56eb3f5620815e7e7058f8ab7428e0734fc64627cc4d716c |
| rpv3_mdict_160_r3.onnx | 10258843 | 8fb08b5db2adeccf43b05006bbbf409e4659d08d72e46a62631c00ff751eaeb3 |
| litemodel_cls_96x_r1.onnx | 1604473 | fe123688d3bf08b9ef029fcd57f3ac4644ac2e5ef8b9e7a676aacd5fad152142 |

ONNX 清单 `models_manifest.json` 原样保留。没有获取替代模型或更改权重。

## 环境基线

系统 Python 3.8.10：`rospy`、`sensor_msgs.msg`、`std_msgs.msg` 导入通过；NumPy 1.17.4、OpenCV 4.2.0、Pillow 7.0.0、PyYAML 5.3.1。原系统无 ultralytics、torch、torchvision、hyperlpr3、onnxruntime。因此新建隔离环境，不覆盖系统包。

`nvidia-smi` 可见 RTX 4060 系列显卡，驱动 595.91.07、8 GiB 显存；这仅证明容器设备可见，不等于所选 Python 已有 CUDA 推理能力。本轮采用 CPU Torch 环境，未配置/验证 GPU 推理。

容器未发现可用中文字体；车牌标注图按原逻辑降级为 `plate#编号`，JSON 保留完整 Unicode 号码。用户可自行指定已安装的中文字体路径。

在用户缩小测试范围之前，曾只读订阅现有 `/camera/color/image_raw`：收到两个时间戳递增的 640×480 `rgb8` 帧。当时图像只有道路/墙体；未进行识别或操控运动。已有 Gazebo、AMCL、move_base 均直接复用，未重复启动。

## 待用户自行验证

- 本地模型实际加载与人物/车牌正样本识别，包括 C/NC 映射、中文完整号码、目标框/文字与图像一一对应。
- 当前 Gazebo 视角、距离、遮挡和光照条件下的漏检/误检，特别是灯具误判为人物。
- 目标移出视野、相机输入中断和节点故障后的消费端超时语义。
- 双路实际帧率、推理时延、长期 CPU/内存占用，以及未来 GPU 环境。

没有执行真实模型推理或假输出替代识别。库导入、哈希、适配器测试和构建通过不能替代这些运行验收。

## Git 范围

本轮新增仅 `car_2026/src/robot_perception/**`，更新根 README 的包说明。未修改 P1、导航、底盘、URDF、地图和 Gazebo 场景。

任务开始时已有未跟踪的 `car_2026/route_p1_test.yaml`、`.lock`、交接 ZIP 与解压目录，均保留原状，未暂存或提交。没有使用 `git add`；提交时只选择新包及根 README，勿 `git add .` 将交接副本和测试路线纳入。模型缓存、虚拟环境、下载缓存及测试日志均位于容器用户目录或 `/tmp`，不在源码树内。

## 安装布局检查

`cmake -DCMAKE_INSTALL_PREFIX=/tmp/p2a-install -P build/robot_perception/cmake_install.cmake` 退出 0；安装的可执行文件只有人物、车牌和模型准备脚本，资源包含三个 launch、五个模型、模型清单与文档。使用 `install(PROGRAMS)` 保留 env shebang；launch-prefix 显式解释器适用于 source/install 两种布局。临时安装目录不纳入 Git。

## 最终隔离环境检查（2026-10-09）

解释器：`/home/developer/.venvs/robot-perception/bin/python`，Python **3.10.18**。实际用户为 `developer`，HOME 为 `/home/developer`。

| 依赖 | 实际版本 | 检查结果 |
|---|---|---|
| ultralytics | 8.4.159 | 导入通过，ONLINE=false、AUTOINSTALL=false |
| torch | 2.14.0+cpu | 导入通过；cuda.is_available()=false，torch.version.cuda=None |
| torchvision | 0.29.0+cpu | 导入通过 |
| numpy | 2.2.6 | 导入通过 |
| opencv-python | 5.0.0.93（cv2 5.0.0） | 导入通过 |
| Pillow | 12.3.0 | 导入通过 |
| PyYAML | 6.0.3 | 导入通过 |
| hyperlpr3 | 0.1.3 | 缓存准备后离线导入通过，未创建 LicensePlateCatcher |
| onnxruntime | 1.23.2 | 导入通过，可用 provider 为 AzureExecutionProvider、CPUExecutionProvider；未创建推理 Session |
| rospkg / catkin-pkg | 1.6.3 / 1.1.1 | 安装及依赖一致性检查通过 |
| rospy / sensor_msgs.msg / std_msgs.msg | 现有 ROS Noetic | 在上述 Python 3.10 中导入通过 |

`uv pip check --python /home/developer/.venvs/robot-perception/bin/python`：退出 0，62 个包依赖一致。完整固定版本见 `requirements-cpu.lock.txt`，直接依赖约束见 `requirements-cpu.txt`。依赖导入报告位于容器 `/tmp/p2a-environment.json`；Ultralytics 首次导入仅在 `/home/developer/.config/Ultralytics/` 创建设置文件。

缓存准备命令与只读验证均退出 0：

```bash
/home/developer/.venvs/robot-perception/bin/python /workspace/car_2026/src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3
/home/developer/.venvs/robot-perception/bin/python /workspace/car_2026/src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3 --verify-only
```

四个模型安装到 `/home/developer/.hyperlpr3/20230229/onnx/`，大小及 SHA256 全部一致，无模型联网下载。静态检查了 HyperLPR3 原库的缓存初始化逻辑，再执行缓存验证后的导入检查；未执行识别。

安装结束后重新检查系统 Python：仍为 3.8.10，NumPy 1.17.4、OpenCV 4.2.0、Pillow 7.0.0、PyYAML 5.3.1，ROS 消息仍可正常导入。系统包未被替换。

## 代码适配器检查

在独立 Python 中执行 `python tests/test_adapters.py`：退出 0，5 项测试通过。覆盖五种图像编码、行 padding、非法输入拒绝、C/NC 固定映射、标注函数及缓存冲突保护。Header 测试实际调用 Noetic `serialize_message`，确认输出 seq 改写后输入源 seq 仍保持不变。

测试只使用内存中的合成像素和元数据验证代码，不调用 YOLO.predict、HyperLPR catcher 或 ONNX 推理，不能作为识别效果证据。临时安装目录中的模型准备脚本也完成 `--verify-only` 检查，退出 0。
