# YOLO26 垃圾分类检测训练

环境安装沿用 [人员训练说明](../person/README.md)。默认使用 `models/yolo26n.pt`；权重缺失时 Ultralytics 会尝试下载。

在仓库根目录执行：

```bash
python yolo_train/scripts/trash_sorting/train.py --device 0
# 使用 CPU
python yolo_train/scripts/trash_sorting/train.py --device cpu
```

当前训练读取 `datasets/trash_sorting/data.yaml`，包含 24 类具体垃圾，训练集 258 张、验证集 29 张。`images/` 存放原图和 X-AnyLabeling JSON，`train/` 和 `val/` 使用 YOLO 标签。含四分类旧标签或未确认标签的 6 张图片留在 `images/` 待复核。默认训练 100 轮、长边尺寸 1920、batch 2、随机种子 42；`patience: 30` 允许提前停止，需完整训练 100 轮时设为 `0`。

启用 `rect: true` 等比例矩形训练；关闭 Mosaic、随机缩放、随机平移和多尺度训练，避免高分辨率原图再次被缩小或裁切。1920×1080 图片保留原尺寸并按网络步长补边，通常输入为宽 1920、高 1088；同一批次按最大宽高比补齐，混合批次的补边高度可能增加。数据中 53 张原始 640×480 图片会放大到 1920×1440，不会产生新的真实细节。显存不足时先将 `batch` 减为 `1`。

同目录 `config.yaml` 可调整参数；支持 `--config`、`--model`、`--device`。配置中的模型、数据和输出相对路径及 `--model` 的相对路径以 `yolo_train/` 为基准。不传 `--device` 时自动选择 CUDA GPU 或 CPU。

结果保存到 `yolo_train/results/trash_sorting/yolo26n/`，包含验证指标、训练曲线和 `weights/best.pt`、`weights/last.pt`，重复运行自动递增目录名称。

实车 `waste_detector` 当前使用四分类模型。部署新训练的 24 类模型时，需要同步更新检测类别和物品到四大类的映射。
