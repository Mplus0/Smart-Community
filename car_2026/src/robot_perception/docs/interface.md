# P2-A 人物 / 车牌接口

两个节点仅识别，不发布速度，不调用导航。默认输入 `/camera/color/image_raw`，类型 `sensor_msgs/Image`，支持 `rgb8/bgr8/mono8/rgba8/bgra8` 及每行 padding。保留交接版图像解码、BGR 预处理与推理调用。默认最多每路处理 5 Hz，只取最新图像，不保证每个输入帧都被处理。

| 功能 | std_msgs/String JSON 输出 | sensor_msgs/Image 标注图 |
|---|---|---|
| 人物 | `/perception/person_detections_json` | `/perception/person_image` |
| 车牌 | `/perception/plates_json` | `/perception/plates_image` |

`header` 为 `{seq, stamp: {secs, nsecs}, frame_id}`；JSON 保留源图 Header 全部字段。输出图为 `bgr8`，保留源 `stamp/frame_id`；ROS 的 Publisher 序列化会给图像 `header.seq` 分配该输出流自己的序号，因此跨话题应以源时间戳及 frame_id 关联，而非以图像 seq 相等为准。两个节点异步执行，最后收到的两条 JSON 不一定来自同一源帧。

## 人物 JSON（schema_version=1）

- `engine: yolo26_person_detector`
- `header`、`width`、`height`：源帧信息；尚无图像的 invalid 通知可以为 `header: null`。
- `frame_valid`、`reason`：`true/ok` 为成功处理；异常为 `false/decode_error`、`false/inference_error`；输入墙上时钟超时为 `false/camera_stale`。
- `processing_ms`：解码、推理及标注的处理耗时；错误/断流通知为 0。
- `score_kind: yolo_model_confidence`
- `detections`：每项为 `class_id`、`label`、`display_label`、`confidence`、`bbox_xyxy`。
- 标签固定：`0 / community_person / C`；`1 / non_community_person / NC`。启动时检查模型 names，映射不符则拒绝启动。
- `bbox_xyxy`：原图像素 `[x1,y1,x2,y2]`，浮点数，边界裁剪至图像范围；不是地图坐标。

有效空数组只表示当前帧未检出符合阈值的目标；invalid 空数组不能解读为“无人”。`camera_stale` 通常只通知一次，不是周期心跳；消费者不能只依赖 invalid 消息，因为节点退出时不会继续发布。

人物没有永久 ID、街区归属或任务关联。灯具/背景误检风险继承交接模型，调高阈值不能替代困难负样本验证。每帧检测个数不能累加为社区总人数。

## 车牌 JSON（schema_version=1）

- `engine: hyperlpr3`
- `header`、`width`、`height`：源帧信息。
- `inference_ms`、`processing_ms`：模型推理、整帧处理耗时。
- `plates`：每项为 `text`（完整 Unicode 中文/字母/数字）、`confidence`、`plate_type_id`、整数像素 `bbox_xyxy`。
- 类型编号保持 HyperLPR3 0.1.3 定义；本轮不加业务解释。
- 默认 `min_confidence=0.5`，检测等级保留 `DETECT_LEVEL_HIGH`。

**此接口没有 `frame_valid/reason`。** 成功处理且未检出时发布 `plates: []`；相机断流、排队帧过旧、解码或推理失败时保持沉默，不补发错误 JSON，也不重新发布旧号码。

缺少中文字体时标注图显示 `plate#编号 + confidence`，JSON 仍保留原始完整号码。可用 `plate_font_path` 指定已有中文字体；不分发 Windows 字体。

## 下一阶段消费者必须满足的条件（本轮未实现）

同时检查源 ROS 时间戳和自收到消息起的 monotonic 墙上时钟超时；拒绝旧帧、重复/倒退时间戳、无效人物帧，以及过期车牌结果。`/clock` 暂停时 ROS 时间可能不变，不能只使用 ROS 时间差。节点收到重复时间戳的输入仍可能执行推理，不能把重复源帧当作新证据。

应按各自流的有效新帧关联任务窗口，不持续复用最后一条 JSON；航点/街区绑定、跨帧去重、车牌多帧融合、终端统计与结果落盘由 P2-B 处理。

## 交通灯仅预留名称

后续拟用 `/perception/traffic_light_json` 与 `/perception/traffic_light_image`。P2-A **没有这些话题的发布器或消息内容**，不发布 RED/GREEN/UNKNOWN，不移植 HSV，不改变 P1 默认禁止自动越线的保护。未来 YOLO 接口字段另行确认。
