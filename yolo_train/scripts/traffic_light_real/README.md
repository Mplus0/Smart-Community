# 实车红绿灯目标检测训练

使用 `datasets/traffic_light_real/images/` 中的图片及同名 JSON 矩形框标注，训练 YOLO26 检测模型。类别固定为 `0: stop`、`1: go`；当前没有黄灯样本，不训练黄灯类别。原始图片和 JSON 保持不变。

环境安装沿用 [人员训练说明](../person/README.md)，使用独立 Python 3.10+ 训练环境，安装适配训练机器的 PyTorch，以及支持 YOLO26 的 Ultralytics：

```bash
python -m pip install -U "ultralytics>=8.4.0"
```

在仓库根目录执行：

```bash
# 只检查标注和划分统计，不生成文件
python yolo_train/scripts/traffic_light_real/prepare_dataset.py --check-only
# 转换为 YOLO 标签，生成训练集、验证集及 data.yaml
python yolo_train/scripts/traffic_light_real/prepare_dataset.py
# 自动选择 CUDA GPU 或 CPU；也可指定 --device 0 或 --device cpu
python yolo_train/scripts/traffic_light_real/train.py
```

准备脚本支持两点或四点矩形标注，检查类别、坐标范围、图片尺寸及缺失标注；空标注生成空 TXT，作为背景图片保留。按图片包含的类别分组，以种子 42 划分约 90% 训练集、10% 验证集。当前 225 张图片划分为训练 202 张、验证 23 张；无测试集。连续拍摄的相近图片可能分布在两侧，验证指标不能代替独立场景的实车测试。

转换结果位于 `yolo_train/datasets/traffic_light_real_yolo/`。目标目录已存在时拒绝覆盖；如需重新划分，可通过 `--output datasets/traffic_light_real_yolo_v2` 指定新目录，并同步修改训练配置的 `data`。准备脚本支持 `--source`、`--val-ratio` 和 `--seed`，相对路径以 `yolo_train/` 为基准。

训练配置见同目录 `config.yaml`，默认使用 `models/yolo26n.pt`，100 轮、640 输入尺寸、batch 16。关闭色相及饱和度增强，保留少量亮度变化。显存不足时减小 batch。支持 `--config`、`--model`、`--device`；配置内相对路径及 `--model` 相对路径以 `yolo_train/` 为基准，与工作目录无关。预训练权重缺失时 Ultralytics 会尝试下载。

训练输出至 `yolo_train/results/traffic_light_real/yolo26n/`，包括指标、曲线、`weights/best.pt` 和 `weights/last.pt`，重复训练自动递增目录名。

实车当前使用 `stop/go` 两分类，类别顺序为 `0: stop`、`1: go`。2026-11-25 已使用 U 盘 `trained_models/traffic_light/best.pt` 更新 `yolo_train/model/traffic_light.pt/.onnx` 和实车 `community_perception/models/traffic_light.onnx`；该权重来自服务器 YOLO26n 训练，54 轮结束、最佳第 24 轮。已核对源文件 SHA-256、ONNX 结构和部署文件一致性，未进行实车推理评测。导出脚本本身不会自动替换部署模型。

## 导出 ONNX

当前训练好的权重保存在 `yolo_train/model/traffic_light.pt`，导出命令为：

```bash
python yolo_train/scripts/traffic_light_real/export_onnx.py
# 也可指定其他轮次训练的最佳权重
python yolo_train/scripts/traffic_light_real/export_onnx.py --weights results/traffic_light_real/yolo26n/weights/best.pt
```

需安装 `onnx`；运行推理验证时另需 `onnxruntime`。导出文件与输入权重同目录、同名（`.onnx`），默认是 `yolo_train/model/traffic_light.onnx`。使用 FP32、固定输入 `1×3×640×640`、opset 17，关闭端到端输出和内置 NMS，输出为原始检测预测，调用方需完成置信度过滤和 NMS。脚本会执行 ONNX 结构检查。
