import io
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
import zipfile
from functools import wraps
from contextlib import asynccontextmanager

import cv2
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from typing import List
from .processes import Processes, ROOT
from .ros_bridge import RosBridge

WS = ROOT.parent
sys.path.insert(0, str(WS / 'tools'))
sys.path.insert(0, str(WS / 'src/community_perception/scripts'))
from test_detector_model import ONNXDetector, load_detector, DEFAULT_CONFIG

config = json.loads((ROOT/'config.json').read_text(encoding='utf-8-sig'))
processes = Processes(config)
ros = None
media_lock = threading.RLock()
vision_lock = threading.Lock()
vision = dict(jpeg=None, result={}, raw=None)
writer = None
capture_interval, next_capture = 0, 0
dataset = 'default'
media_root = Path(config['media_root']).expanduser().resolve()


@asynccontextmanager
async def lifespan(app):
    global ros
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.update(ROS_MASTER_URI=config['master_uri'], ROS_IP=config['ros_ip'])
    ros = RosBridge()
    worker_stop.clear()
    threading.Thread(target=media_worker, daemon=True).start()
    yield
    worker_stop.set()
    if ros.mode == 'manual':
        ros.halt()
    ros.closed = True
    with media_lock:
        if writer:
            writer.release()


app = FastAPI(title='Smart Community Infra', lifespan=lifespan)
worker_stop = threading.Event()


@app.exception_handler(Exception)
async def failure(request, exc):
    from fastapi.responses import JSONResponse
    return JSONResponse(status_code=503, content={'detail':str(exc)})


@app.exception_handler(ValueError)
async def invalid(request, exc):
    from fastapi.responses import JSONResponse
    return JSONResponse(status_code=409, content={'detail':str(exc)})


class Settings(BaseModel):
    master_uri: str
    ros_ip: str


@app.get('/api/config')
def settings():
    return config


@app.post('/api/config')
def save_settings(body: Settings):
    from urllib.parse import urlparse
    import ipaddress
    uri = urlparse(body.master_uri)
    if uri.scheme != 'http' or not uri.hostname or uri.port != 11311:
        raise ValueError('ROS Master 格式：http://IP:11311')
    ipaddress.ip_address(body.ros_ip)
    updated = dict(config, **body.dict())
    (ROOT/'config.json').write_text(json.dumps(updated, indent=2),encoding='utf-8')
    return {'message':'已保存；ROS 环境在重启后生效，请先释放手动控制'}


@app.get('/api/state')
def state():
    return ros.snapshot()


@app.get('/api/processes')
def process_status():
    try:
        return {'processes':processes.status()}
    except Exception as exc:
        return {'processes':{},'error':str(exc)}


class Start(BaseModel):
    algorithm: str = 'cartographer'
    map: str = ''
    route: str = ''
    navigation_only: bool = False


operation_lock = threading.Lock()


def serialized(fn):
    @wraps(fn)
    def call(*args, **kwargs):
        with operation_lock:
            return fn(*args, **kwargs)
    return call


@app.post('/api/process/{name}/{action}')
@serialized
def process_action(name: str, action: str, body: Start):
    if action == 'stop':
        processes.stop(name)
        if name == 'mission' and ros.ready:
            ros.cancel.publish(ros.GoalID())
        return {'ok':True}
    if action != 'start' or name not in ['hardware','mapping','navigation','services','mission']:
        raise ValueError('未知服务或操作')
    if not ros.online:
        raise ValueError('ROS Master 离线')
    nodes = ros.nodes
    groups = dict(
        hardware=['/community_serial','/usb_cam','/imu','/odometry_publisher',
                  '/lslidar_driver_node','/base_link_to_laser4','/configure_camera'],
        mapping=['/slam_gmapping','/cartographer_node','/cartographer_occupancy_grid_node',
                 '/robot_state_publisher','/ekf_localization'],
        navigation=['/move_base','/amcl','/map_server','/robot_state_publisher','/ekf_localization'],
        mission=['/community_mission'],
        services=['/people_detector','/traffic_light_detector','/fire_detector',
                  '/trash_bin_detector','/waste_detector','/bicycle_detector',
                  '/license_plate_ocr_server','/fall_counter_server','/speech_server','/video_recorder_node'])
    if any(n in nodes for n in groups[name]):
        raise ValueError('已有相同功能节点运行，禁止重复启动')
    if name in ['mapping','navigation']:
        opposite = 'navigation' if name == 'mapping' else 'mapping'
        if any(n in nodes for n in groups[opposite]) or processes.status().get(opposite,{}).get('running'):
            raise ValueError('建图与导航互斥，请先停止另一服务')
    if name in ['navigation','mission'] and (ros.mode != 'auto' or ros.estop):
        raise ValueError('请先选择自动模式并解除软件急停')
    args = ['roslaunch','community_mission' if name=='mission' else 'community_bringup',name+'.launch']
    if name == 'hardware':
        args += ['base_protocol:=aa_bb']
    if name == 'mapping':
        if body.algorithm not in ['gmapping','cartographer']:
            raise ValueError('未知建图算法')
        args += ['algorithm:='+body.algorithm]
    if name == 'navigation' and body.map:
        args += ['map:='+body.map]
    if name == 'mission':
        if '/move_base' not in nodes:
            raise ValueError('请先启动导航')
        args += ['navigation_only:='+str(body.navigation_only).lower()]
        if body.route:
            args += ['route:='+body.route]
    return processes.start(name,args)


class Control(BaseModel):
    mode: str = 'auto'
    locked: bool = True


@app.post('/api/control')
@serialized
def control(body: Control):
    ros.halt()
    if body.mode == 'manual':
        active = processes.status()
        if any(active.get(k,{}).get('running') for k in ['navigation','mission']):
            raise ValueError('请先停止 Infra 导航和巡检进程')
    with ros.lock:
        if not ros.online and not body.locked:
            raise ValueError('ROS 离线，不能解除控制锁定')
        if body.mode not in ['manual','auto']:
            raise ValueError('未知控制模式')
        if body.mode == 'manual' and (ros.conflicts() or '/community_mission' in ros.nodes or '/move_base' in ros.nodes):
            raise ValueError('请先停止导航、巡检和其他速度发布者')
        if not body.locked and '/community_mission' in ros.nodes and ros.estop:
            raise ValueError('解除急停前请停止巡检，防止其重发目标')
        ros.mode, ros.estop = body.mode, body.locked
        if not body.locked:
            ros.emergency_active = False
    return {'ok':True}


@app.post('/api/estop')
def estop():
    ros.emergency()
    # Only stop an owned mission. External controllers require their own operator.
    try:
        if processes.status().get('mission',{}).get('running'):
            processes.stop('mission')
    except Exception as exc:
        return {'ok':True,'message':'停车已发出，SSH 停止任务失败：'+str(exc)}
    return {'ok':True,'message':'已取消导航、发送零速并锁定 Infra；外部控制器仍需单独停止'}


@app.websocket('/api/control/ws')
async def control_socket(ws: WebSocket):
    await ws.accept()
    owner = id(ws)
    try:
        while True:
            body = await ws.receive_json()
            if body.get('stop'):
                if ros.owner == owner:
                    ros.halt()
                continue
            values = [float(body.get(k,0)) for k in ['x','y','z']]
            if any(not math.isfinite(v) for v in values) or abs(values[0])>.5 or abs(values[1])>.5 or abs(values[2])>1:
                raise ValueError('速度超限：X/Y ≤ 0.5 m/s，Yaw ≤ 1 rad/s')
            ros.manual(owner,*values)
    except (WebSocketDisconnect, ValueError) as exc:
        if isinstance(exc, ValueError):
            await ws.send_json({'error':str(exc)})
            await ws.close()
    finally:
        if ros.owner == owner:
            ros.halt()


class Pose(BaseModel):
    x: float
    y: float
    yaw: float = 0


@app.post('/api/pose/{kind}')
def pose(kind: str, body: Pose):
    if kind == 'cancel':
        if ros.ready:
            ros.cancel.publish(ros.GoalID())
    elif kind in ['initial','goal']:
        if '/community_mission' in ros.nodes:
            raise ValueError('巡检运行中，禁止插入手动导航目标')
        if not all(math.isfinite(v) for v in [body.x,body.y,body.yaw]):
            raise ValueError('坐标必须为有限数值')
        ros.navigate(kind,body.x,body.y,body.yaw)
    else:
        raise ValueError('未知操作')
    return {'ok':True}


@app.get('/api/map')
def get_map():
    return ros.map


class MapSave(BaseModel):
    path: str


@app.post('/api/map/save')
def save_map(body: MapSave):
    if not ros.online or ros.map is None:
        raise ValueError('没有可用地图')
    return processes.start('map_save',['rosrun','map_server','map_saver','-f',body.path])


@app.get('/api/mission')
def mission():
    return processes.results()


@app.get('/api/models')
def models():
    import yaml
    return yaml.safe_load(DEFAULT_CONFIG.read_text(encoding='utf-8'))['detectors']


class Camera(BaseModel):
    topic: str


@app.post('/api/camera')
def camera(body: Camera):
    if not ros.ready:
        raise ValueError('ROS 未连接')
    with media_lock:
        if writer or capture_interval:
            raise ValueError('请先停止录像和连续采集')
        ros.select_camera(body.topic)
    return {'ok':True}


@app.get('/api/image')
def image():
    frame = ros.frame()
    return Response(cv2.imencode('.jpg',frame)[1].tobytes(),media_type='image/jpeg',headers={'Cache-Control':'no-store'})


class Vision(BaseModel):
    model: str = 'people_detector'
    mode: str = 'onnx'
    confidence: float = Field(.5,ge=0,le=1)
    iou: float = Field(.4,ge=0,le=1)


cached_detector = None
cached_key = None


@app.post('/api/vision')
def detect(body: Vision):
    global cached_detector,cached_key
    with vision_lock:
        ros.frame()  # Reject stale camera data before taking one shared snapshot.
        source = ros.image
        raw = ros.cv.imgmsg_to_cv2(source, 'bgr8')
        original = raw.copy()
        cfg, path = load_detector(DEFAULT_CONFIG,body.model)
        start = time.monotonic()
        result = {}
        if body.mode == 'onnx':
            key = (body.model,body.confidence,body.iou)
            if cached_key != key:
                cached_detector = ONNXDetector(path,cfg['class_names'],body.confidence,body.iou)
                cached_key = key
            # Reuse deployed preprocessing/postprocessing; ONNXDetector validates model shape.
            from yolo_detector_service import YOLOv8DetectorService
            adapter = YOLOv8DetectorService.__new__(YOLOv8DetectorService)
            adapter.input_width,adapter.input_height = cached_detector.width,cached_detector.height
            adapter.nc,adapter.class_names = len(cfg['class_names']),dict(enumerate(cfg['class_names']))
            adapter.conf_threshold,adapter.iou_threshold = body.confidence,body.iou
            output = cached_detector.session.run(None,{cached_detector.input.name:adapter.preprocess(raw)})[0]
            detections = adapter.postprocess(output,raw.shape)
            records = []
            for d in detections:
                x1,y1,x2,y2 = d['bbox']
                cv2.rectangle(raw,(x1,y1),(x2,y2),(0,220,0),2)
                cv2.putText(raw,f"{d['class_name']} {d['score']:.2f}",(x1,max(20,y1)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,220,0),2)
                records.append(dict(class_name=d['class_name'],score=float(d['score']),bbox=d['bbox']))
            jpeg = cv2.imencode('.jpg',raw)[1].tobytes()
            result = {'model':body.model,'confidence':body.confidence,'iou':body.iou,'detections':records}
        else:
            from community_perception.srv import ObjectDetection, LicensePlateOCR

            service = '/license_plate_ocr' if body.mode == 'ocr' else cfg['service_name']
            if body.mode not in ['ocr','service']:
                raise ValueError('未知测试模式')
            ros.ros.wait_for_service(service,timeout=2)
            response = ros.ros.ServiceProxy(service,LicensePlateOCR if body.mode=='ocr' else ObjectDetection)(source)
            if body.mode == 'ocr':
                result = dict(texts=list(response.plate_texts),confidences=list(response.confidences),message=response.status_message)
            else:
                result = dict(success=response.success,message=response.message,classes=list(response.detected_classes),confidences=list(response.confidences),bboxes=list(response.bboxes))
                for i,(label,score) in enumerate(zip(response.detected_classes,response.confidences)):
                    x1,y1,x2,y2 = response.bboxes[i*4:i*4+4]
                    cv2.rectangle(raw,(x1,y1),(x2,y2),(0,220,0),2)
                    cv2.putText(raw,f'{label} {score:.2f}',(x1,max(20,y1)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,220,0),2)
            jpeg = cv2.imencode('.jpg',raw)[1].tobytes()
        elapsed = time.monotonic()-start
        result.update(ms=round(elapsed*1000,1),fps=round(1/max(elapsed,.001),1),mode=body.mode)
        vision.update(jpeg=jpeg,result=result,raw=original)
        return result


@app.get('/api/vision/image')
def vision_image():
    if not vision['jpeg']:
        raise ValueError('尚无识别结果')
    return Response(vision['jpeg'],media_type='image/jpeg')


def folder():
    p = media_root/dataset
    p.mkdir(parents=True,exist_ok=True)
    return p


def save_photo(frame):
    path = folder()/(str(time.time_ns())+'.jpg')
    if not cv2.imwrite(str(path),frame):
        raise ValueError('图片保存失败')
    return str(path.relative_to(media_root))


class MediaSettings(BaseModel):
    dataset: str = 'default'
    root: str
    interval: float = Field(0,ge=0,le=3600)


@app.post('/api/media/settings')
def media_settings(body: MediaSettings):
    global media_root,dataset,capture_interval
    if not re.fullmatch(r'[\w-]+',body.dataset):
        raise ValueError('数据集名称仅支持字母、数字、中文、下划线和短横线')
    with media_lock:
        if writer:
            raise ValueError('请先停止录像')
        media_root,dataset = Path(body.root).expanduser().resolve(),body.dataset
        capture_interval = max(.2,body.interval) if body.interval else 0
        folder()
    return {'ok':True}


@app.post('/api/media/photo')
def photo():
    with media_lock:
        return {'file':save_photo(ros.frame())}


@app.post('/api/vision/save')
def save_vision():
    with vision_lock, media_lock:
        if vision['raw'] is None:
            raise ValueError('尚无识别结果')
        p = folder()/str(time.time_ns())
        cv2.imwrite(str(p)+'_raw.jpg',vision['raw'])
        Path(str(p)+'_result.jpg').write_bytes(vision['jpeg'])
        Path(str(p)+'.json').write_text(json.dumps(vision['result'],ensure_ascii=False),encoding='utf-8')
    return {'ok':True}


@app.post('/api/media/video/{action}')
def video(action: str):
    global writer
    with media_lock:
        if action == 'stop':
            if writer:
                writer.release()
            writer = None
        elif action == 'start':
            if writer:
                raise ValueError('正在录像')
            frame = ros.frame()
            writer = cv2.VideoWriter(str(folder()/(str(time.time_ns())+'.avi')),cv2.VideoWriter_fourcc(*'MJPG'),10,(frame.shape[1],frame.shape[0]))
            if not writer.isOpened():
                writer.release()
                writer = None
                raise ValueError('无法创建视频')
        else:
            raise ValueError('未知操作')
    return {'ok':True}


media_error = ''


def media_worker():
    global next_capture,media_error
    while not worker_stop.wait(.1):
        with media_lock:
            if not writer and not capture_interval:
                continue
            try:
                frame = ros.frame()
                if writer:
                    writer.write(frame)
                if capture_interval and time.monotonic() >= next_capture:
                    save_photo(frame)
                    next_capture = time.monotonic()+capture_interval
                media_error = ''
            except Exception as exc:
                media_error = str(exc)


@app.get('/api/media')
def files():
    with media_lock:
        return dict(root=str(media_root),dataset=dataset,interval=capture_interval,recording=writer is not None,error=media_error,
                    files=[str(p.relative_to(media_root)) for p in sorted(media_root.rglob('*')) if p.is_file()] if media_root.exists() else [])


def safe_file(name):
    p = (media_root/name).resolve()
    if media_root not in p.parents or not p.is_file():
        raise HTTPException(404,'文件不存在')
    return p


@app.get('/api/media/file')
def download(path: str):
    return FileResponse(safe_file(path))


class Export(BaseModel):
    files: List[str]


@app.post('/api/media/export')
def export(body: Export):
    data = io.BytesIO()
    with zipfile.ZipFile(data,'w',zipfile.ZIP_DEFLATED) as archive:
        for name in body.files:
            archive.write(safe_file(name),name)
    return Response(data.getvalue(),media_type='application/zip',headers={'Content-Disposition':'attachment; filename="media.zip"'})


build = ROOT/'frontend/dist'
if build.exists():
    app.mount('/',StaticFiles(directory=build,html=True),name='frontend')
