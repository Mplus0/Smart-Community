# 垃圾桶颜色与开闭状态检测数据集

共 261 张图片，按原文件名排序编号为 `0001` 至 `0261`，`images/` 保留图片与 JSON 标注。`split_manifest.csv` 记录原文件名、新文件名和所属集合。

固定随机种子 42，按约 9:1 划分训练集 235 张、验证集 26 张。`train/`、`val/` 下分别包含 `images/` 和 YOLO 格式的 `labels/`。2 张无目标框图片保留为空标签样本。

类别编号沿用原配置：0 `blue_close`、1 `blue_open`、2 `grey_close`、3 `grey_open`、4 `green_close`、5 `green_open`、6 `red_close`、7 `red_open`。

`data.yaml` 的数据路径以仓库根目录为基准。训练方式见 [训练说明](../../scripts/trashbin_new/README.md)。`training_cpu_v2.json` 保留为原标注工具配置，当前 YOLO26 训练脚本使用自身的 `config.yaml`。
