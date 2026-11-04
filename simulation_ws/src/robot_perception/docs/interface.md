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

## 交通灯 ROI 分类（可选，schema_version=1）

`traffic_light_classification_node.py` 订阅同一原图，只在自身副本中裁剪；人物和车牌节点仍接收完整原图。
默认使用 Python 3.10 隔离环境、CPU、`imgsz=224`，模型为包内 `models/traffic_light_best.pt`。
启动时从 checkpoint 读取任务类型，必须为 `classify`；名称必须恰好是 `red/yellow/green`，通过 `model.names` 映射 ID，不固定 ID 顺序。模型不存在或不匹配时直接拒绝启动，不下载替代权重。

| 话题 | 类型 | 内容 |
|---|---|---|
| `/perception/traffic_light_json` | `std_msgs/String` | 下述分类 JSON |
| `/perception/traffic_light_image` | `sensor_msgs/Image` | 原始分辨率 BGR 图、ROI 框、标签和置信度 |

- `schema_version: 1`，`engine: yolo11n_traffic_light_classifier`。
- `header`：源图的 seq/stamp/frame_id；启动后从未收到图像时为 null。标注图复制 Header，保持源 stamp/frame_id，图像 seq 允许 ROS 序列化重新分配。
- `roi_xyxy`：原图像素整数 `[left, top, right, bottom]`，right/bottom 为排他边界；未解码/未计算 ROI 或断流通知时为 null。它是人工配置的分类区域，**不是模型检测框**。
- `label`：`RED/YELLOW/GREEN/UNKNOWN`。
- `confidence`：模型在三类中最大概率，低置信度拒绝时仍保留该概率；非推理结果的异常/过期通知为 0。不是经过校准的交通灯存在概率。
- `frame_valid`：仅当帧新鲜、推理完成且概率达到阈值时为 true；表示分类结果通过上述检查，**不证明场景存在交通灯**。
- `reason`：成功为 `ok`；拒绝为 `low_confidence/decode_error/inference_error/camera_stale/invalid_stamp/future_stamp/non_increasing_stamp`。拒绝时固定 `frame_valid=false, label=UNKNOWN`。
- `processing_ms`：从开始处理到分类后检查结束的墙钟耗时，不含发布与标注；未执行处理的拒绝/断流通知为 0。
- `score_kind: conditional_class_probability`、`presence_verified: false`：明确声明分类器不能判断无灯背景。

配置 `config/traffic_light_classification.yaml` 的 `roi` 默认为 `[0.0, 0.0, 1.0, 0.5]`。
坐标乘以原图宽高后向下取整，默认恰好为 `image[0:height//2, 0:width]`；零面积、越界或非法配置拒绝。
ROI 副本交给 Ultralytics 官方分类预处理，最终 `imgsz=224`。官方预处理仍可能在该 ROI 内做中心裁剪；ROI 框展示的是传入区域，并不表示模型保留了框内所有边缘像素。应结合实际训练图构图与场景验收，不擅自改变训练时的归一化或模型结构。

私有参数：`model/device/input_topic/image_topic/json_topic/confidence/max_rate/stale_seconds`，默认置信度阈值 0.8、最多 5 Hz、过期时间 1.5 秒。`imgsz` 固定 224；修改 ROI 可通过单路 `config:=...` 或统一入口 `traffic_light_config:=...` 传入自用 YAML。

回调仅保存最新帧（订阅 queue_size=1）；单工作循环限频，不建立待推理队列。推理前后均检查源 ROS 时间年龄与接收后的 monotonic 墙钟年龄。零时间戳、未来时间戳、重复或倒退源时间戳均拒绝；/clock 暂停时墙钟仍可判过期。仿真重置导致源时间回退后，需要重启该节点以重置时间戳水位。

断流发布一次 UNKNOWN 通知，不重复发布旧图。解码失败没有可用标注图；可解码的低置信度或推理失败仍发布原尺寸 UNKNOWN 标注图。消费者必须自行实施消息接收超时，不能依赖节点退出后继续发通知。

无灯画面也可能高置信度输出 GREEN。所有颜色仅供视觉测试；当前 `robot_competition` **不消费这个新话题**，原交通灯安全拦截保持不变，不实现停车、放行或自动越线。
