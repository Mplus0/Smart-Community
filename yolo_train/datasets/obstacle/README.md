# 障碍物检测数据集

数据共 120 张，按原文件名排序统一编号为 `0001` 至 `0120`。`images/` 保留图片和 JSON 标注，JSON 的 `imagePath` 与新文件名一致。

固定随机种子 42，按约 9:1 划分：`train/images`、`train/labels` 为训练集（108 张），`val/images`、`val/labels` 为验证集（12 张）。标签为 YOLO 检测格式，类别 0 为 `obstacle`。

`split_manifest.csv` 记录原文件名、新文件名和所属集合。`data.yaml` 提供数据集配置，路径以仓库根目录为基准。仅划分训练集和验证集，最终效果通过部署真机测试。
