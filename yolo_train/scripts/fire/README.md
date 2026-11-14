# YOLO26 火焰检测训练

环境安装沿用 [人员训练说明](../person/README.md)。默认加载 `models/yolo26n.pt`，使用 YOLO26n 检测模型；权重缺失时 Ultralytics 会尝试下载。

在仓库根目录执行：

```bash
python yolo_train/scripts/fire/train.py
# 指定 GPU；使用 CPU 时改为 --device cpu
python yolo_train/scripts/fire/train.py --device 0
```

读取 `yolo_train/datasets/fire/data.yaml`，训练集 209 张、验证集 23 张，单类别 `0: fire`。默认训练 100 轮，图片尺寸 640，批次大小 16，随机种子 42，自动选择 CUDA GPU 或 CPU。

修改同目录 `config.yaml` 可调整训练参数。`--config` 接受自定义配置文件路径，`--model` 可临时指定权重。配置中的模型、数据和输出路径以及 `--model` 的相对路径均以 `yolo_train/` 为基准。

结果保存在 `yolo_train/results/fire/yolo26n/`，包含验证指标、训练曲线和 `weights/best.pt`、`weights/last.pt`。重复运行自动递增目录名称，部署权重需另行选择和复制。
