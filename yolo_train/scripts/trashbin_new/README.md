# YOLO26 垃圾桶颜色与开闭状态检测训练

环境安装沿用 [人员训练说明](../person/README.md)。默认使用 `models/yolo26n.pt`；权重缺失时 Ultralytics 会尝试下载。

在仓库根目录执行：

```bash
python yolo_train/scripts/trashbin_new/train.py --device 0
# 使用 CPU
python yolo_train/scripts/trashbin_new/train.py --device cpu
```

读取 `datasets/trashbin/data.yaml`，使用训练集 444 张、验证集 48 张，检测 8 类垃圾桶颜色与开闭状态。默认最多训练 150 轮，图片尺寸 640，batch 8，随机种子 42。验证指标连续 40 轮没有改善时提前停止。

同目录 `config.yaml` 可调整参数；支持 `--config`、`--model`、`--device`。配置中的模型、数据和输出相对路径及 `--model` 的相对路径以 `yolo_train/` 为基准。不传 `--device` 时自动选择 CUDA GPU 或 CPU。

结果保存到 `yolo_train/results/trashbin_new/yolo26n/`，包含验证指标、训练曲线和 `weights/best.pt`、`weights/last.pt`，重复运行自动递增目录名称。
