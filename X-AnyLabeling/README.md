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