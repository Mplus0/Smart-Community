# 标定配置

- `competition.json`：随附 `opencv/photo/clear3.png` 的人工标定，用于首次启动示例。
- `competition_geometry.json`：不含刻度标定，仅供 `demo` 检查候选方向。
- `meter.json`：默认标定输出，取决于最近一次保存内容，不会按照片名称自动关联。
- `*.reference.png`：标定时自动保存的裁剪参考图，便于核对位置；读取时主要使用 JSON。

在 Smart-Community 仓库根目录运行：

```bash
python -m opencv calibrate opencv/photo/clear3.png --output opencv/configs/my_meter.json
python -m opencv read opencv/photo/clear3.png --config opencv/configs/my_meter.json
```

JSON 包含原图宽高 `frame_size`、原图 ROI、大指针圆心、半径、刻度数值与位置、增大方向，以及可选屏蔽圆和检测参数。
圆心、刻度点、屏蔽圆的坐标都相对于 ROI 左上角，ROI 本身相对于原图。

配置和生成的参考图可以提交 Git；最终确认标定后才更新。识别时不会写回尺寸适配结果。
同一构图的等比例缩放可复用配置；机位、角度或裁剪改变后需重新标定。

详细操作见 [模块 README](../README.md)。
