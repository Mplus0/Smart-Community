# YOLO26 人员检测训练

使用独立的 Python 3.10+ 环境，按训练机器的 CUDA 环境安装 PyTorch，然后安装支持 YOLO26 的 Ultralytics：

```bash
python -m pip install -U "ultralytics>=8.4.0"
```

在仓库根目录执行：

```bash
python yolo_train/scripts/person/train.py
# 指定第一张 GPU；没有 GPU 时可使用 --device cpu
python yolo_train/scripts/person/train.py --device 0
```

默认从仓库根目录的 `models/yolo26n.pt` 加载官方检测预训练权重，文件缺失时 Ultralytics 会尝试联网下载到该位置。也可手动下载后放入该目录，避免训练时下载。自动选择 CUDA GPU 或 CPU。Windows 使用了主入口保护，可使用多进程数据加载。

配置中的 `model: ../models/yolo26n.pt` 以 `yolo_train/` 为基准，也支持绝对路径。可通过命令行临时指定权重：

```bash
python yolo_train/scripts/person/train.py --model D:/Desktop/智慧社区/Smart-Community/models/yolo26n.pt
```

修改同目录 `config.yaml` 可调整模型、轮数、图片尺寸、批次大小和数据加载进程数。可将模型改为 `../models/yolo26s.pt`、`../models/yolo26m.pt` 等 YOLO26 检测权重，修改模型时也应调整输出名称。显存不足时减小 `batch`。`--config` 接受自定义配置文件路径；配置内的数据和输出路径以 `yolo_train/` 为基准，与运行目录无关。

训练读取 `datasets/person/data.yaml` 中的训练集 313 张和验证集 35 张，类别为 `community_person`、`outsider`。不设置测试集，最终效果通过真机测试。

结果默认位于 `yolo_train/results/person/yolo26n/`，包括训练曲线、验证指标、`weights/best.pt` 和 `weights/last.pt`。重复运行会自动递增目录名称。脚本不会覆盖部署目录的现有模型，真机部署时选择本次训练的 `best.pt`。

接口参考：[Ultralytics YOLO26](https://docs.ultralytics.com/models/yolo26/)。
