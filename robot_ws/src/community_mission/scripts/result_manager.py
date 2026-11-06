"""Per-run JSON results and a plain text event log; no ROS or decision logic."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid


def timestamp():
    return datetime.now(timezone.utc).isoformat()


class ResultManager:
    def __init__(self, root, points, report=print):
        self.report = report
        self.started = time.monotonic()
        self.run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_') + uuid.uuid4().hex
        self.path = Path(root).expanduser().resolve() / self.run_id
        for folder in ('photo', 'videos'):
            (self.path / folder).mkdir(parents=True, exist_ok=False)
        self.data = dict(run_id=self.run_id, status='RUNNING', started_at=timestamp(),
                         ended_at=None, duration_sec=0.0, current_waypoint=None, error=None,
                         waypoints=[dict(id=p['id'], status='NOT_EXECUTED') for p in points],
                         planned_tasks=[dict(waypoint_id=p['id'], task_type=t, status='NOT_EXECUTED')
                                        for p in points for t in p.get('tasks', [])],
                         operations=[], recording_errors=[])
        self.event('开始巡检 ' + self.run_id)
        self.save()

    def recording_error(self, error):
        # A recording failure must not alter navigation or recognition decisions.
        self.data['recording_errors'].append(str(error))
        self.report('[结果] 写入失败：' + str(error))

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
        self.event(f"[任务] {point['id']} {kind} 开始")
        self.save()
        try:
            result = callback(*args)
        except Exception as error:
            record.update(status='FAILED', reason=str(error))
            raise
        else:
            record.update(status='SUCCESS', result=result)
            return result
        finally:
            record.update(ended_at=timestamp(), duration_sec=time.monotonic()-started)
            if planned is not None:
                planned['status'] = record['status']
            self.save()
            detail = record['reason'] or json.dumps(record['result'], ensure_ascii=False)
            message = f"[任务] {point['id']} {kind}: {record['status']} {detail}"
            self.event(message)
            self.report(message)

    def record_task(self, result, paths=()):
        """Persist the recognition decision unchanged; never invoke recognition."""
        record = dict(result, paths=[str(p) for p in paths])
        self.data['operations'].append(record)
        for planned in self.data['planned_tasks']:
            if (planned['waypoint_id'], planned['task_type']) == (result['waypoint_id'], result['task_type']):
                planned['status'] = result['status']
        self.save()
        detail = result['reason'] if result['status'] == 'FAILED' else json.dumps(result['result'], ensure_ascii=False)
        message = f"[任务] {result['waypoint_id']} {result['task_type']}: {result['status']} {detail}"
        self.event(message)
        self.report(message)

    def finish(self, status, error=None):
        if self.data['status'] != 'RUNNING':
            return
        for point in self.data['waypoints']:
            if point['status'] == 'RUNNING':
                self.waypoint(point, 'FAILED')
        tasks = [r for r in self.data['operations'] if r['task_type'] != 'navigation']
        summary = dict(completed_waypoints=sum(p['status'] == 'SUCCESS' for p in self.data['waypoints']),
                       successful_tasks=sum(t['status'] in ('SUCCESS', 'EMPTY_VALID') for t in tasks),
                       failed_tasks=sum(t['status'] == 'FAILED' for t in tasks))
        self.data.update(status=status, error=error, ended_at=timestamp(), summary=summary)
        self.save()
        message = (f"[汇总] {status} 完成航点={summary['completed_waypoints']} "
                   f"任务成功={summary['successful_tasks']} 失败={summary['failed_tasks']} "
                   f"耗时={self.data['duration_sec']:.1f}s 结果={self.path}")
        self.event(message)
        self.report(message)
