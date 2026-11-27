"""Per-run JSON results and a plain text event log; no ROS or decision logic."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid


def timestamp():
    return datetime.now(timezone.utc).isoformat()


def task_summary(kind, result):
    """Keep terminal summaries short; full detector data stays in the JSON file."""
    if kind in ('record_start', 'record_stop'):
        return '录像已开始' if kind == 'record_start' else '录像已停止'
    if not isinstance(result, dict):
        return json.dumps(result, ensure_ascii=False)
    required = {
        'person': ('area', 'people', 'outside', 'illegal_parking'),
        'plate': ('parking_lot', 'plate'), 'fire': ('fire_count',),
        'trash': ('bins', 'waste', 'placement_status'),
        'vehicle_video': ('stand_count', 'fall_count'),
        'vehicle_photo': ('total_count', 'stand_count', 'fall_count'),
        'traffic_light': ('released', 'green_streak'), 'obstacle': ('mode', 'center_x_ratio'),
        'meter': ('temperature_c', 'humidity_percent', 'electricity_kwh'),
        'temperature': ('abnormal_floors', 'max_temperatures_c'),
    }
    if kind not in required or any(key not in result for key in required[kind]):
        text = json.dumps(result, ensure_ascii=False, separators=(',', ':'))
        return text if len(text) <= 360 else text[:360] + '…（完整数据见结果文件）'
    if kind == 'person':
        return (f"区域={result['area']} 人数={result['people']} "
                f"外来人员={result['outside']} 违停={result['illegal_parking']}")
    if kind == 'plate':
        return f"停车位={result['parking_lot']} 车牌={result['plate'] or '未检出'}"
    if kind == 'fire':
        return f"火情数={result['fire_count']}" + (
            f" 楼宇={result.get('building', '未确认')}" if result['fire_count'] else '')
    if kind == 'trash':
        return (f"垃圾桶={len(result['bins'])} 垃圾={len(result['waste'])} "
                f"投放判定={result['placement_status']}")
    if kind == 'vehicle_video':
        return f"正常车辆={result['stand_count']} 倒伏车辆={result['fall_count']}"
    if kind == 'vehicle_photo':
        return (f"车辆总数={result['total_count']} 正常车辆={result['stand_count']} "
                f"倒伏车辆={result['fall_count']}")
    if kind == 'traffic_light':
        return f"放行={result['released']} 连续绿灯帧={result['green_streak']}"
    if kind == 'obstacle':
        return (f"{result.get('announcement', '')} 分支={result['mode']} "
                f"中心横坐标比例={result['center_x_ratio']:.3f}").strip()
    if kind == 'meter':
        return (f"温度={result['temperature_c']}℃ 湿度={result['humidity_percent']}% "
                f"电量={result['electricity_kwh']}kWh")
    if kind == 'temperature':
        return f"异常楼层={result['abnormal_floors']} 最高温度℃={result['max_temperatures_c']}"


class ResultManager:
    def __init__(self, root, points, report=print):
        self.report = report
        self.on_result = None
        self.started = time.monotonic()
        self.run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_') + uuid.uuid4().hex
        self.path = Path(root).expanduser().resolve() / self.run_id
        for folder in ('photo', 'videos'):
            (self.path / folder).mkdir(parents=True, exist_ok=False)
        self.data = dict(run_id=self.run_id, status='RUNNING', started_at=timestamp(),
                         ended_at=None, duration_sec=0.0, current_waypoint=None, error=None,
                         waypoints=[dict(id=p['id'], status='NOT_EXECUTED') for p in points],
                         planned_tasks=[dict(waypoint_id=p['id'], task_type=p['task'], status='NOT_EXECUTED')
                                        for p in points if p['task'] != 'navigation'],
                         operations=[], recording_errors=[])
        self.event('开始巡检 ' + self.run_id)
        self.save()

    def recording_error(self, error):
        # A recording failure must not alter navigation or recognition decisions.
        self.data['recording_errors'].append(str(error))
        self.report('[结果写入失败] ' + str(error))

    def log(self, message):
        self.event(message)
        self.report(message)

    def task_started(self, point, kind):
        tag = '导航开始' if kind == 'navigation' else '任务开始'
        self.log(f"[{tag}] 航点={point['id']} 类型={kind}")

    def task_completed(self, record):
        kind, status = record['task_type'], record['status']
        detail = record['reason'] if status == 'FAILED' else task_summary(kind, record['result'])
        if status != 'FAILED' and record.get('reason'):
            detail += ' 说明=' + record['reason']
        tag = '导航结果' if kind == 'navigation' else '任务结果'
        # A service exception can contain newlines; each outcome remains one terminal line.
        message = (f"[{tag}] 航点={record['waypoint_id']} 类型={kind} 状态={status} "
                   f"耗时={record['duration_sec']:.2f}s 结果={' '.join(detail.splitlines())}")
        self.log(message)
        paths = record.get('paths', [])
        if paths:
            existing = [path for path in paths if Path(path).is_file()]
            if existing:
                self.log(f"[任务证据] 航点={record['waypoint_id']} 文件=" + ' | '.join(existing))
            missing = [path for path in paths if path not in existing]
            if missing:
                self.log(f"[证据未生成] 航点={record['waypoint_id']} 文件=" + ' | '.join(missing))
        if self.on_result is not None:
            try:
                self.on_result(record)
            except Exception as error:
                self.log('[裁判结果入队失败] ' + str(error))

    def event(self, message):
        try:
            with (self.path / 'mission.log').open('a', encoding='utf-8') as stream:
                stream.write(timestamp() + ' ' + message + '\n')
        except OSError as error:
            self.recording_error(error)

    def save(self):
        self.data['duration_sec'] = time.monotonic() - self.started
        try:
            temporary = self.path / 'mission_results.json.tmp'
            temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2,
                                             allow_nan=False), encoding='utf-8')
            temporary.replace(self.path / 'mission_results.json')
        except (OSError, ValueError, TypeError) as error:
            self.recording_error(error)

    def state(self, state, point, reason):
        self.data['current_waypoint'] = point['id']
        self.data['state'] = state
        self.event(f"[状态] {state} 航点={point['id']} 原因={reason}")
        self.save()

    def waypoint(self, point, status):
        record = next(p for p in self.data['waypoints'] if p['id'] == point['id'])
        record['status'] = status
        if status == 'RUNNING':
            record['started_at'] = timestamp()
            record['started_elapsed_sec'] = time.monotonic() - self.started
        else:
            record['ended_at'] = timestamp()
            record['duration_sec'] = time.monotonic() - self.started - record['started_elapsed_sec']
        self.save()

    def execute(self, kind, point, callback, *args, paths=()):
        started = time.monotonic()
        record = dict(waypoint_id=point['id'], task_type=kind, status='RUNNING',
                      started_at=timestamp(), ended_at=None, duration_sec=0.0,
                      result=None, reason='', paths=[str(p) for p in paths])
        planned = next((p for p in self.data['planned_tasks']
                        if p['waypoint_id'] == point['id'] and p['task_type'] == kind), None)
        if planned is not None:
            planned['status'] = 'RUNNING'
        self.data['operations'].append(record)
        self.task_started(point, kind)
        self.save()
        try:
            result = callback(*args)
        except Exception as error:
            record.update(status='FAILED', reason=str(error))
            raise
        else:
            skipped = kind == 'meter' and isinstance(result, dict) and result.get('skipped') is True
            record.update(status='SKIPPED' if skipped else 'SUCCESS', result=result,
                          reason=result.get('reason', '') if skipped else '')
            return result
        finally:
            record.update(ended_at=timestamp(), duration_sec=time.monotonic()-started)
            if planned is not None:
                planned['status'] = record['status']
            self.save()
            self.task_completed(record)

    def record_task(self, result, paths=()):
        """Persist the recognition decision unchanged; never invoke recognition."""
        record = dict(result, paths=[str(p) for p in paths])
        self.data['operations'].append(record)
        for planned in self.data['planned_tasks']:
            if (planned['waypoint_id'], planned['task_type']) == (result['waypoint_id'], result['task_type']):
                planned['status'] = result['status']
        self.save()
        self.task_completed(record)

    def finish(self, status, error=None):
        if self.data['status'] != 'RUNNING':
            return
        for point in self.data['waypoints']:
            if point['status'] == 'RUNNING':
                self.waypoint(point, 'FAILED')
        tasks = [r for r in self.data['operations'] if r['task_type'] != 'navigation']
        summary = dict(completed_waypoints=sum(p['status'] == 'SUCCESS' for p in self.data['waypoints']),
                       successful_tasks=sum(t['status'] in ('SUCCESS', 'EMPTY_VALID') for t in tasks),
                       empty_tasks=sum(t['status'] == 'EMPTY_VALID' for t in tasks),
                       failed_tasks=sum(t['status'] == 'FAILED' for t in tasks),
                       skipped_tasks=sum(t['status'] == 'SKIPPED' for t in tasks))
        self.data.update(status=status, error=error, ended_at=timestamp(), summary=summary)
        self.save()
        mode = ('仅任务' if self.data.get('task_only') else
                '仅导航' if self.data.get('navigation_only') else '导航与任务')
        self.log(f"[巡检结束] 状态={status} 模式={mode} 完成航点={summary['completed_waypoints']} "
                 f"任务成功={summary['successful_tasks']}（其中空结果={summary['empty_tasks']}） "
                 f"任务失败={summary['failed_tasks']} 耗时={self.data['duration_sec']:.1f}s")
        if error:
            self.log('[结束原因] ' + ' '.join(str(error).splitlines()))
        self.log(f"[结果文件] {self.path / 'mission_results.json'}")
        self.log(f"[事件日志] {self.path / 'mission.log'}")
