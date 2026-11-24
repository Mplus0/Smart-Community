# YOLO26 火焰检测训练

重新标注并进行光照增强的数据集使用 `fire_aug.yaml`：1579 张训练原图各增加两个增强版本，训练集共 4737 张；验证集为 79 张原图。原始数据位于 `datasets/fire/images/`，增强与划分记录位于 `datasets/fire_aug/`。

```bash
python yolo_train/scripts/fire/train.py --config yolo_train/scripts/fire/fire_aug.yaml --device 0
```

该配置默认训练 100 轮、输入 640、batch 16，结果保存到 `yolo_train/results/fire_aug/yolo26n/`。以下为原训练配置及已部署模型的记录；原 `datasets/fire/` 当前不保留旧划分，继续训练请使用上面的增强配置。

96GB RTX PRO 6000 云端使用 `fire_aug_cloud.yaml`：输入 640、batch 256、workers 16、内存缓存、AMP，最多 200 轮，连续 50 轮未改善时早停。关闭严格确定性以优先考虑吞吐量。数据位于服务器 `/root/autodl-tmp/Smart-Community/`，由用户手动启动训练：

```bash
cd /root/autodl-tmp/Smart-Community
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 nohup /root/miniconda3/bin/python -u yolo_train/scripts/fire/train.py --config yolo_train/scripts/fire/fire_aug_cloud.yaml --device 0 > fire_aug_train.log 2>&1 &
tail -f fire_aug_train.log
```

该配置结果保存在 `yolo_train/results/fire_aug/yolo26n_640_cloud/`，重复训练会自动递增目录名。

环境安装沿用 [人员训练说明](../person/README.md)。默认加载 `models/yolo26n.pt`，使用 YOLO26n 检测模型；权重缺失时 Ultralytics 会尝试下载。

在仓库根目录执行：

```bash
python yolo_train/scripts/fire/train.py
# 指定 GPU；使用 CPU 时改为 --device cpu
python yolo_train/scripts/fire/train.py --device 0
```

读取 `yolo_train/datasets/fire/data.yaml`，训练集 972 张、验证集 49 张，按约 20:1 划分，单类别 `0: fire`。默认训练 100 轮，图片尺寸 640，批次大小 16，随机种子 42，自动选择 CUDA GPU 或 CPU。

修改同目录 `config.yaml` 可调整训练参数。`--config` 接受自定义配置文件路径，`--model` 可临时指定权重。配置中的模型、数据和输出路径以及 `--model` 的相对路径均以 `yolo_train/` 为基准。

结果保存在 `yolo_train/results/fire/yolo26n/`，包含验证指标、训练曲线和 `weights/best.pt`、`weights/last.pt`。重复运行自动递增目录名称，部署权重需另行选择和复制。

2026-11-24 云端数据、脚本和预训练权重已放置到 `/root/autodl-tmp/Smart-Community/`。使用服务器现有 Conda 环境，在 SSH 终端执行以下命令启动后台训练：

```bash
cd /root/autodl-tmp/Smart-Community
nohup /root/miniconda3/bin/python -u yolo_train/scripts/fire/train.py --device 0 > fire_train.log 2>&1 &
tail -f fire_train.log
```

2026-11-25 光照增强训练在第 130 轮早停，最佳模型来自第 80 轮。79 张验证图上的精确率为 0.990、召回率为 1.000、mAP50 为 0.995、mAP50-95 为 0.957。云端 `yolo26n_640_cloud/weights/best.pt` 已下载为 `yolo_train/model/fire_aug_20261125_best.pt`，并覆盖 `yolo_train/model/fire.pt`。

导出的 `yolo_train/model/fire.onnx` 已同步到 `robot_ws/src/community_perception/models/fire.onnx`，X-AnyLabeling 也通过原配置引用新模型：静态 FP32 输入 `[1,3,640,640]`，原始输出 `[1,5,8400]`，opset 17，关闭端到端后处理和内置 NMS。实车火焰配置启用等比例缩放补边（`letterbox: true`），与训练及标注工具一致，置信度阈值不变。已通过 ONNX 结构检查与现有检测代码的样例推理，尚未在实车 ROS 服务上验证。
