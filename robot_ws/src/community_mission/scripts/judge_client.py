"""Offline-LAN referee reporting; one worker, no ROS or navigation dependencies."""
import json
import mimetypes
from pathlib import Path
import queue
import threading
from urllib.request import Request, build_opener, ProxyHandler
from urllib.parse import urlsplit
import uuid


class JudgeClient:
    def __init__(self, url, team, directory, log=print, timeout=3.0):
        self.url = url.rstrip('/')
        if urlsplit(self.url).scheme not in ('http', 'https') or not urlsplit(self.url).hostname:
            raise ValueError('judge_url must be an HTTP(S) address')
        if not isinstance(team, str) or not team.strip():
            raise ValueError('judge_team must not be empty')
        self.team, self.timeout, self.log = team.strip(), timeout, log
        self.journal = Path(directory) / 'judge_reports.jsonl'
        self.queue = queue.Queue(maxsize=128)
        self.stopping = threading.Event()
        self.people = {}
        self.vehicles = {'stand_count': 0, 'fall_count': 0}
        self.vehicle_images = []
        # LAN requests must not follow a desktop/system Internet proxy.
        self.http = build_opener(ProxyHandler({}))
        self.worker = threading.Thread(target=self._work, name='judge-report', daemon=True)
        self.worker.start()

    def _post(self, path, body, content_type):
        request = Request(self.url + path, data=body, headers={'Content-Type': content_type})
        with self.http.open(request, timeout=self.timeout) as response:
            result = json.load(response)
        if result.get('ok') is not True:
            raise RuntimeError('Referee rejected report: ' + str(result))
        return result

    def _multipart(self, fields, paths):
        boundary = uuid.uuid4().hex
        body = bytearray()
        for name, value in fields.items():
            body.extend(f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode('utf-8'))
        for index, path in enumerate(paths):
            path = Path(path)
            if not path.is_file():
                continue
            content = path.read_bytes()
            if len(content) > 30 * 1024 * 1024:
                raise ValueError('Referee image exceeds 30 MB: ' + str(path))
            mime = mimetypes.guess_type(path.name)[0] or 'image/jpeg'
            body.extend(f'--{boundary}\r\nContent-Disposition: form-data; name="images"; filename="evidence{index}{path.suffix}"\r\nContent-Type: {mime}\r\n\r\n'.encode())
            body.extend(content)
            body.extend(b'\r\n')
        body.extend(f'--{boundary}--\r\n'.encode())
        if len(body) > 100 * 1024 * 1024:
            raise ValueError('Referee upload exceeds 100 MB')
        return bytes(body), 'multipart/form-data; boundary=' + boundary

    def _save(self, entry):
        try:
            with self.journal.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(entry, ensure_ascii=False) + '\n')
        except OSError as error:
            self.log('[裁判日志写入失败] ' + str(error))

    def submit(self, record):
        if record['status'] not in ('SUCCESS', 'EMPTY_VALID'):
            return
        # Snapshot small metadata only; image reading and HTTP happen in the worker.
        snapshot = json.loads(json.dumps(record, ensure_ascii=False))
        if self.stopping.is_set():
            self._save(dict(status='not_sent', reason='client_closed', record=snapshot))
            return
        try:
            self.queue.put_nowait(snapshot)
        except queue.Full:
            self._save(dict(status='not_sent', reason='queue_full', record=snapshot))
            self.log('[裁判上报失败] 队列已满，任务结果已保留')

    @staticmethod
    def _images(record):
        paths = [p for p in record.get('paths', []) if Path(p).suffix.lower() in ('.jpg', '.jpeg', '.png')]
        annotated = [p for p in paths if p.endswith('_annotated.jpg') or p.endswith('_trash.jpg') or p.endswith('_overlay.jpg')]
        return annotated or paths

    def reports(self, record):
        kind, result = record['task_type'], record.get('result') or {}
        images = self._images(record)
        waypoint = record['waypoint_id']
        def report(task, text, sub_id=waypoint):
            return dict(task=task, result=text, sub_id=sub_id, paths=images,
                        extra=json.dumps(dict(waypoint_id=waypoint, task_type=kind), ensure_ascii=False))
        if kind == 'plate':
            lot = result['parking_lot']
            return [report('plate', f"{lot}号停车场车牌号为{result['plate'] or '未检测到车牌'}", f'{lot}号停车位')]
        if kind == 'person':
            self.people[waypoint] = dict(result, paths=images)
            totals = {area: {key: sum(p[key] for p in self.people.values() if p['area'] == area)
                             for key in ('people', 'outside', 'illegal_parking')} for area in ('A', 'B')}
            crowd = []
            for area in ('A', 'B'):
                if not any(p['area'] == area for p in self.people.values()):
                    continue
                text = (f"社区内共有{totals['A']['people'] + totals['B']['people']}人，"
                        f"其中A街人数{totals['A']['people']}人，B街人数{totals['B']['people']}人，"
                        f"发现{totals[area]['outside']}名非社区人员在{area}街，图片已保存")
                item = report('crowd', text, area + '街区')
                item['paths'] = list(dict.fromkeys(p for r in self.people.values() if r['area'] == area for p in r['paths']))
                crowd.append(item)
            return crowd
        if kind == 'fire':
            building = result.get('building') or waypoint
            text = (f"{building}发生火灾，图片已保存" if result['fire_count'] else '未发现火情，图片已保存')
            return [report('fire', text, building)]
        if kind == 'trash':
            reports = []
            for i, match in enumerate(result.get('matches', []), 1):
                waste = '、'.join(g['type'] for g in match['garbage']) or '未识别到对应垃圾'
                verdict = {'CORRECT': '投放正确', 'INCORRECT': '投放错误'}.get(match['status'], '投放情况无法确认')
                reports.append(report('trash', f"{match['bin_type']}垃圾桶{match['bin_state']}，{waste}，{verdict}", f'{i}号桶'))
            return reports or [report('trash', result.get('announcement', '未识别到垃圾桶'), waypoint)]
        if kind == 'meter':
            return [report('meter', f"温度为{result['temperature_c']}摄氏度，湿度为{result['humidity_percent']}%，电表读数为{result['electricity_kwh']}千瓦时")]
        if kind == 'temperature':
            floors = '、'.join(str(v) for v in result['abnormal_floors'])
            return [report('temperature', f'D座大厦在{floors}楼处发现高温情况' if floors else 'D座大厦未发现高温情况', 'D座大厦')]
        if kind == 'vehicle_photo':
            self.vehicle_images.extend(p for p in images if p not in self.vehicle_images)
            return []
        if kind == 'vehicle_video':
            self.vehicles = result
            a = sum(p['illegal_parking'] for p in self.people.values() if p['area'] == 'A')
            b = sum(p['illegal_parking'] for p in self.people.values() if p['area'] == 'B')
            text = (f"A街区有{a}辆电动车违停，B街区有{b}辆电动车违停。"
                    f"停车区内电动车正常{self.vehicles['stand_count']}辆，倒伏{self.vehicles['fall_count']}辆")
            # Video files are not accepted by the referee image API.
            item = report('ebike', text, '电动车统计')
            item['paths'] = self.vehicle_images + [p for p in images if p not in self.vehicle_images]
            return [item]
        if kind == 'announcement' and result.get('queued_text'):
            return [report('sign', result['queued_text'])]
        return []

    def _work(self):
        try:
            self._post('/api/hello', json.dumps({'team': self.team}).encode(), 'application/json')
            self.log('[裁判通信] 通信正常 ' + self.url)
        except Exception as error:
            self.log('[裁判通信失败] ' + str(error))
        while not self.stopping.is_set() or not self.queue.empty():
            try:
                record = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                for item in self.reports(record):
                    try:
                        fields = dict(team=self.team, **{k: v for k, v in item.items() if k != 'paths'})
                        body, content_type = self._multipart(fields, item['paths'])
                        response = self._post('/api/report', body, content_type)
                        self._save(dict(status='sent', report=item, response=response))
                        self.log(f"[裁判上报成功] {item['task']} {item['sub_id']}")
                    except Exception as error:
                        self._save(dict(status='failed', report=item, error=str(error)))
                        self.log('[裁判上报失败] ' + str(error))
            except Exception as error:
                self._save(dict(status='failed', record=record, error=str(error)))
                self.log('[裁判结果转换失败] ' + str(error))
            finally:
                self.queue.task_done()

    def close(self, timeout=3.0):
        self.stopping.set()
        self.worker.join(timeout)
        if self.worker.is_alive():
            self.log('[裁判通信] 退出等待超时，请检查 judge_reports.jsonl 与 mission_results.json')
