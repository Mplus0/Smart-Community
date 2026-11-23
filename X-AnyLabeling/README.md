# X-AnyLabeling

用于智慧社区项目的 YOLO 目标检测数据集标注。

## 环境配置

使用 Conda 创建独立环境：

conda create -n x-anylabeling python=3.12 -y
conda activate x-anylabeling

安装依赖：

pip install -r requirements.txt

## 启动软件

conda activate x-anylabeling
xanylabeling

## 使用说明

1. 导入待标注图片。
2. 创建类别并进行目标框标注。
3. 保存原始 JSON 标注文件。
4. 导出 YOLO Detection 格式标签。
5. 将标注完成的数据集用于 YOLO 模型训练。

在自动标注面板加载自定义模型时，选择 `person.yaml` 使用原版人员模型，或选择 `person_half.yaml` 使用半身人员模型。两个配置都从 `yolo_train/model/` 加载对应的 ONNX 模型，并识别 `community_person` 和 `outsider`。

选择 `traffic_light.yaml` 加载实车红绿灯模型，输出 `stop` 和 `go` 矩形框。
