# OpenCV 指针仪表读数

Smart-Community 的独立离线模块，用于主表盘标定、指针检测和整数读数。量程为 **-30～50，每格 1**；无需 ROS、YOLO、GPU 或训练数据。

当前使用固定构图的标定，支持整幅图片等比例缩放。相机位置、角度或焦距变化后需要重新标定。自动定位、透视对齐、副表盘读数及 ROS 任务接入尚未实现。

## 快速开始

**以下命令都从 Smart-Community 仓库根目录执行**，不要先进入 `opencv/src`。

```bash
git clone https://github.com/Mplus0/Smart-Community.git
cd Smart-Community
```

已有本地仓库时，直接打开仓库根目录终端。需要 Python 3.8 或以上；离线开发推荐独立虚拟环境，不要将本依赖直接安装到 ROS 系统环境。

### Windows PowerShell

```powershell
python -m venv opencv/.venv
.\opencv\.venv\Scripts\python.exe -m pip install -r opencv/requirements.txt
.\opencv\.venv\Scripts\python.exe -m opencv read opencv/photo/clear3.png --config opencv/configs/competition.json
```

### Ubuntu / Linux 图形桌面

```bash
python3 -m venv opencv/.venv
opencv/.venv/bin/python -m pip install -r opencv/requirements.txt
opencv/.venv/bin/python -m opencv read opencv/photo/clear3.png --config opencv/configs/competition.json
```

Ubuntu 缺少 venv 时先安装 `python3-venv`。标定需要图形桌面和带 GUI 的 OpenCV；无窗口服务器可运行 `read`，但需具备 OpenCV 所需系统库。

成功运行后打开：

- `opencv/outputs/read/clear3/result.json`：连续读数、整数读数、状态和失败原因。
- `opencv/outputs/read/clear3/debug.png`：圆心、刻度圈和候选指针方向。

随附 `competition.json` 对应 `clear3.png`。这是启动示例，不能代表比赛准确率已达标，也不能直接套用到构图不同的其他照片。

## 常用命令

下面的 `python` 指**已安装本模块依赖的解释器**：Windows 可替换为 `.\opencv\.venv\Scripts\python.exe`，Linux 可替换为 `opencv/.venv/bin/python`。在编辑器里选中该环境，或激活该环境后，也可以直接使用 `python`。

```bash
python -m opencv --help
python -m opencv read opencv/photo/clear3.png --config opencv/configs/competition.json
```

只有检查候选指针方向时才使用演示命令：

```bash
python -m opencv demo
```

`demo` 使用不含刻度标定的几何配置，返回 `needs_calibration`，读数为 `null`，结果放在 `opencv/outputs/demo/`。

## 标定自己的照片

把原始照片放入 `opencv/photo/`，下方的 `your_photo.jpg` 替换成实际文件名：

```bash
python -m opencv calibrate opencv/photo/your_photo.jpg
```

1. 按住左键框选完整大表盘，可以反复拖框替换，Enter 确认。
2. 点击大指针旋转中心，可以重复点击调整，Enter 确认并进入下一步。
3. 点击外圈刻度上的一点确定半径，检查圆圈位置，Enter 确认。
4. 依次点击 **-30、-20、-10、0、10、20、30、40、50** 对应的刻度线，每个点都按 Enter 确认。
5. 总览时再按 Enter 保存；Backspace 返回修改，R 清空当前候选，Esc 取消。

点击数字对应的刻度线，保持各点位置选择方式一致。只需标定这 9 个点，程序在相邻点之间插值。

默认保存到 `opencv/configs/meter.json`，同时生成 `meter.reference.png`；最终确认前不会更新文件。随后读取：

```bash
python -m opencv read opencv/photo/your_photo.jpg --config opencv/configs/meter.json
```

参数：

- `--output opencv/configs/my_meter.json`：标定时指定另一个配置文件，避免更新默认配置。
- `--values=-30,-10,10,30,50`：自选中间标定点，须按整数递增并包含 -30 和 50。
- `--direction counterclockwise`：刻度数值沿逆时针增大时使用。
- `--roi x,y,width,height`：指定原图框选区域，跳过拖框。
- `--exclude x,y,r`：屏蔽圆，可重复指定；坐标相对于裁剪后的表盘。不要覆盖大指针必需的采样区域。

## 结果与配置

| 字段 / 状态 | 含义 |
| --- | --- |
| `reading` | 连续估计读数，例如 26.78 |
| `rounded` | 最终整数，例如 27 |
| `status=ok` | 通过当前检测检查，仍需核对调试图与实拍真值 |
| `status=rejected` | 清晰度不足、指针不明确等；读数为 null，应重拍或检查标定 |
| `status=needs_calibration` | 有候选方向但没有刻度标定，不能作为实际读数 |
| `metrics` | 检测指标，不是正确率或概率 |
| `resolution` | 标定 / 输入尺寸及是否做坐标缩放 |

`read` 默认按图片名更新 `opencv/outputs/read/<图片名>/`，重复运行保留最近一次结果。使用不同配置做对比，或图片同名时，可指定：

```bash
python -m opencv read opencv/photo/clear3.png --config opencv/configs/competition.json --output opencv/outputs/read/clear3_comparison
```

`image`、`config`、`processed_at_utc` 记录原图、配置位置和 UTC 处理时间。输出可以删除，下次运行会创建；Git 不提交生成结果。

退出码：0 为流程完成（包括待标定），2 为拒绝读数，1 为输入或配置错误。上层调用应检查 `status`，不能只看退出码。

[配置说明](configs/README.md) · [输出说明](outputs/README.md)

## 开发

```text
opencv/
├── __main__.py          # python -m opencv 的入口
├── __init__.py          # 提供 load_config / read_frame / read_image
├── src/
│   ├── cli.py           # 命令行和输出保存
│   ├── core.py          # 指针检测、尺寸适配和读数
│   ├── calibration.py   # 标定窗口
│   └── selection.py     # 框选与点击状态
├── configs/             # 标定 JSON 与参考图
├── photo/               # clear1、clear2、clear3 和自行采集的原图
├── outputs/             # 识别结果，按需自动创建
├── tests/               # 自动测试，不依赖真实鼠标操作
└── requirements.txt
```

从仓库根目录运行测试：

```bash
python -m unittest discover -s opencv/tests -t . -v
```

其他程序可调用（`frame` 为 uint8 BGR 三通道图像）：

```python
from opencv import load_config, read_frame

config = load_config("opencv/configs/competition.json")
result, debug = read_frame(frame, config)
```

## 常见问题

- `No module named opencv`：确认终端在 Smart-Community 仓库根目录。
- `No module named cv2` 或 `numpy`：使用安装依赖时的同一个 Python 解释器。
- 找不到照片或配置：使用上方以 `opencv/` 开头的路径，并替换为实际文件名。
- 图片尺寸变化：仅同一构图的整图等比例缩放可自动换算；裁剪、旋转和机位变化需要重新标定。
- 绿色刻度圈或红色圆心没有落在实际表盘上：当前配置不适用，应先重新标定，不能仅降低检测阈值。
- 照片存在反光或模糊：不能保证识别；用真实比赛距离和不同光照的原图验收，分别记录错误报数与拒绝读数。

算法方法参考 [Pointer-meter-identification-and-reading](https://github.com/CherryXuan/Pointer-meter-identification-and-reading)。本模块使用径向检测和角度插值，不依赖该仓库的固定直线排序。
