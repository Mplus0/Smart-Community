#!/usr/bin/env python3
"""Smart-Community PyQt5 dashboard (ROS Noetic / Docker X11).

Left: controls, live camera, completed-person and completed-plate evidence.
Right: competition-only console and mission summary.

All recognition snapshots come from mission_results.json -> tasks[].evidence.
They are NOT continuous subscriptions to perception/person_image or plates_image.
"""

import codecs
from collections import deque
import errno
import html
import http.client
import json
import os
from pathlib import Path
import pty
import re
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPushButton, QSplitter, QTextEdit, QVBoxLayout, QWidget,
)

WORKSPACE = Path('/workspace/car_2026')
TOOLS = WORKSPACE / 'src' / 'tools'
RESULTS = WORKSPACE / 'results'
CAMERA_TOPIC = '/camera/color/image_raw'
ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
ROS_PREFIX_RE = re.compile(r'^\[(INFO|WARN|ERROR|FATAL)\]\s+\[[^\]]+\]:\s*')
BUSINESS_TAGS = ('[比赛]', '[导航]', '[红绿灯]', '[人员]', '[车牌]',
                 '[状态]', '[耗时]', '[结果]', '==========')
OK_TASK_STATUS = ('SUCCESS', 'EMPTY_VALID')


def is_launch_running(launch_name):
    """Check a launch started by GUI or in a separate terminal."""
    pattern = r'[r]oslaunch .*' + re.escape(launch_name)
    try:
        return subprocess.run(
            ['pgrep', '-u', str(os.getuid()), '-f', pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2,
            check=False,
        ).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def environment_running():
    """Support both the combined environment.launch and older tmux startup."""
    if is_launch_running('environment.launch'):
        return True
    try:
        return subprocess.run(
            ['tmux', 'has-session', '-t', '=smart-community'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=1, check=False,
        ).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


class _FastXmlRpcTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=0.4)


def ros_master_available():
    """Quick ROS Master RPC probe; do not initialize rospy while Master is down."""
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    try:
        master = xmlrpc.client.ServerProxy(
            uri, transport=_FastXmlRpcTransport(), allow_none=True)
        return master.getPid('/smart_community_gui')[0] == 1
    except (OSError, ValueError, xmlrpc.client.Error, IndexError, TypeError):
        return False


def summarize_mission(data):
    status = data.get('status', '未知')
    names = {'RUNNING': '执行中', 'FINISH': '路线执行结束', 'ERROR': '异常终止'}
    latest = {t.get('waypoint_id'): t for t in data.get('tasks', [])
              if isinstance(t, dict) and t.get('waypoint_id')}
    success = sum(x.get('status') in OK_TASK_STATUS for x in latest.values())
    failed = sum(x.get('status') not in OK_TASK_STATUS for x in latest.values())
    duration = data.get('mission_duration_sec')
    lines = [f"比赛状态：{names.get(status, status)}",
             f"任务结果：成功 {success}，失败 {failed}"]
    if isinstance(duration, (int, float)):
        lines.append(f'运行耗时：{duration:.1f} 秒')

    total = (data.get('person_summary') or {}).get('total') or {}
    counts = total.get('counts')
    if isinstance(counts, dict):
        community = counts.get('community_count', 0)
        other = counts.get('non_community_count', 0)
        lines.append(f'人员统计：社区 {community} / 非社区 {other} / 总计 {community + other}')
    else:
        observed = total.get('observed_counts_not_unique') or {}
        if observed:
            lines.append('人员总数：未形成有效汇总')
            lines.append('航点观察计数：社区 {} / 非社区 {}'.format(
                observed.get('community_count', 0), observed.get('non_community_count', 0)))
        else:
            lines.append('人员总数：暂无记录')

    plate_records = data.get('plates') or []
    plate_texts = [str(plate['text']) for rec in plate_records
                   if rec.get('status') == 'SUCCESS'
                   for plate in rec.get('recognized_plates') or []
                   if plate.get('text')]
    lines.append('有效车牌：' + ('、'.join(plate_texts) if plate_texts else '暂无'))
    if data.get('error'):
        lines.append('异常：' + str(data['error']))
    return '\n'.join(lines)


def task_display(task):
    """Summarize a completed task without re-running recognition."""
    point = task.get('waypoint_id', '-')
    status = task.get('status', '未知')
    succeeded = status in OK_TASK_STATUS
    kind = task.get('task_type')
    if kind == 'person':
        if succeeded:
            detections = task.get('detections') or []
            community = sum(x.get('label') == 'community_person' for x in detections)
            other = sum(x.get('label') == 'non_community_person' for x in detections)
            return f'{point}  |  社区 {community}，非社区 {other}  |  成功'
        return f'{point}  |  识别失败：{status}'
    if kind == 'plate':
        plates = '、'.join(str(x.get('text')) for x in task.get('plates') or []
                          if x.get('text'))
        if succeeded:
            return f'{point}  |  {plates or "未检测到车牌"}  |  成功'
        if plates:
            return f'{point}  |  {plates}  |  未通过：{status}'
        return f'{point}  |  识别失败：{status}'
    return point


def evidence_paths(run_dir, task):
    """Choose only images saved for THIS completed task, not raw model frames."""
    root = run_dir.resolve()
    paths = []
    for e in task.get('evidence') or []:
        relative = e.get('image_path') if isinstance(e, dict) else None
        if not isinstance(relative, str):
            continue
        filename = (root / relative).resolve()
        # Do not load files outside this mission's evidence directory.
        try:
            filename.relative_to(root / 'images')
        except ValueError:
            continue
        if filename.is_file():
            paths.append(filename)
    return paths[:3]


class UiSignals(QObject):
    log_line = pyqtSignal(str, str)
    notice = pyqtSignal(str)
    env_failure = pyqtSignal(str)
    stop_finished = pyqtSignal(bool, str)
    camera_frame = pyqtSignal(QImage)


class ImageArea(QFrame):
    """One panel for either live camera or a finished task's saved evidence."""
    def __init__(self, title, placeholder, max_images=1):
        super().__init__()
        self.max_images = max_images
        self.pixmaps = []
        self.setObjectName('imageArea')
        self.setStyleSheet('#imageArea { background: #111c2c; border-radius: 8px; }')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 9, 10, 9)
        layout.setSpacing(7)

        heading = QLabel(title)
        heading.setObjectName('imageTitle')
        heading.setStyleSheet('color: #e2e8f0; font-weight: 600; font-size: 14px;')
        layout.addWidget(heading)

        self.visual_holder = QWidget()
        images_layout = QHBoxLayout(self.visual_holder)
        images_layout.setContentsMargins(0, 0, 0, 0)
        images_layout.setSpacing(7)
        self.images = []
        for _ in range(max_images):
            label = QLabel(placeholder)
            label.setAlignment(Qt.AlignCenter)
            label.setWordWrap(True)
            label.setStyleSheet('background: #07111f; color: #8494a9; border-radius: 5px;')
            label.setMinimumHeight(115 if max_images > 1 else 185)
            images_layout.addWidget(label, 1)
            self.images.append(label)
        layout.addWidget(self.visual_holder, 1)

        self.details = QLabel('等待数据…')
        self.details.setStyleSheet('color: #cad5e2; font-size: 12px;')
        self.details.setWordWrap(True)
        self.details.setMinimumHeight(30)
        layout.addWidget(self.details)
        self.placeholder = placeholder

        for extra in self.images[1:]:
            extra.hide()

    def set_frame(self, frame):
        self.pixmaps = [QPixmap.fromImage(frame)]
        self.details.setText('实时画面：/camera/color/image_raw')
        self._render()

    def set_evidence(self, paths, details):
        # Called ONLY after a completed task has appeared in mission_results.json.
        self.pixmaps = []
        for path in paths[:self.max_images]:
            img = QPixmap(str(path))
            if not img.isNull():
                self.pixmaps.append(img)
        if not self.pixmaps:
            self.images[0].setText('本次任务未保存识别证据图片')
        self.details.setText(details)
        self._render()

    def _render(self):
        n = max(1, len(self.pixmaps))
        usable_width = max(120, self.visual_holder.width() - (n - 1) * 7)
        each_width = max(110, usable_width // n - 4)
        max_height = max(100, self.visual_holder.height() - 6)
        for i, label in enumerate(self.images):
            if i < n:
                label.show()
                if i < len(self.pixmaps):
                    label.setPixmap(self.pixmaps[i].scaled(
                        each_width, max_height, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                else:
                    label.clear()
                    label.setText('本次任务未保存识别证据图片')
            else:
                label.hide()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.pixmaps:
            self._render()


class CameraSubscriber:
    """Subscribes only to the raw camera topic; never runs inference."""
    def __init__(self, signals):
        self.signals = signals
        self.started = False
        self.closed = False
        self.last_frame = 0.0
        self.rospy = None

    def start(self):
        if self.started:
            return
        self.started = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            # Environment launch can exist before ROS Master is ready.
            while not self.closed and not ros_master_available():
                time.sleep(0.4)
            if self.closed:
                return
            ros_python = '/opt/ros/noetic/lib/python3/dist-packages'
            if ros_python not in sys.path:
                sys.path.insert(0, ros_python)
            import rospy
            from sensor_msgs.msg import Image
            self.rospy = rospy
            rospy.init_node('smart_community_gui', anonymous=True, disable_signals=True)
            rospy.Subscriber(CAMERA_TOPIC, Image, self._on_image,
                             queue_size=1, buff_size=4 * 1024 * 1024)
            self.signals.notice.emit('已连接 ROS 摄像头订阅，等待实时画面。')
            rospy.spin()
        except Exception as exc:
            self.signals.notice.emit('摄像头订阅失败：' + str(exc))

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self.last_frame < 0.12:  # Limit GUI redraw to ~8 fps.
            return
        self.last_frame = now
        try:
            w, h, step = int(msg.width), int(msg.height), int(msg.step)
            if w <= 0 or h <= 0 or step <= 0:
                return
            encoding = msg.encoding.lower()
            if encoding in ('rgb8', 'bgr8'):
                if step < w * 3:
                    return
                image = QImage(msg.data, w, h, step, QImage.Format_RGB888)
                image = image.rgbSwapped() if encoding == 'bgr8' else image.copy()
            elif encoding == 'mono8':
                if step < w:
                    return
                image = QImage(msg.data, w, h, step, QImage.Format_Grayscale8).copy()
            elif encoding in ('rgba8', 'bgra8'):
                if step < w * 4:
                    return
                image = QImage(msg.data, w, h, step, QImage.Format_RGBA8888)
                image = image.rgbSwapped() if encoding == 'bgra8' else image.copy()
            else:
                return  # Gazebo color camera commonly uses bgr8/rgb8.
            self.signals.camera_frame.emit(image)
        except (TypeError, ValueError, RuntimeError):
            pass

    def close(self):
        self.closed = True
        if self.rospy is not None:
            try:
                self.rospy.signal_shutdown('GUI closed')
            except Exception:
                pass


class CompetitionGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('智慧社区 · 比赛控制台（PyQt5）')
        self.resize(1480, 870)
        self.setMinimumSize(1050, 640)
        self.signals = UiSignals()
        self.signals.log_line.connect(self.append_log)
        self.signals.notice.connect(self.show_notice)
        self.signals.env_failure.connect(self.on_env_failure)
        self.signals.stop_finished.connect(self.on_stop_finished)
        self.signals.camera_frame.connect(self.camera_updated)

        self.env_proc = None
        self.comp_proc = None
        self.env_tail = deque(maxlen=18)
        self.stopping = False
        self.user_requested_stop = False
        self.last_json_stamp = None
        self.latest_task_keys = {'person': None, 'plate': None}
        self.camera = CameraSubscriber(self.signals)

        self._build_ui()
        self.check_results(force=True)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(700)
        self.tick()

    def _build_ui(self):
        self.setStyleSheet('''
            QMainWindow, QWidget#root { background: #101827; color: #e2e8f0; }
            QLabel { color: #e2e8f0; }
            QGroupBox { color: #e2e8f0; border: 1px solid #2b3b50; border-radius: 8px;
                        margin-top: 14px; padding-top: 8px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QPushButton { background: #263951; color: #f8fafc; border: 1px solid #40536c;
                          border-radius: 6px; padding: 10px 15px; font-size: 13px; }
            QPushButton:hover { background: #35506d; }
            QPushButton:disabled { background: #202b3b; color: #738399; }
            QTextEdit { background: #07111f; color: #e2e8f0; border: 1px solid #26364c;
                        border-radius: 7px; padding: 9px; }
            QSplitter::handle { background: #1c2a3b; }
        ''')
        root = QWidget()
        root.setObjectName('root')
        self.setCentralWidget(root)
        frame = QHBoxLayout(root)
        frame.setContentsMargins(14, 14, 14, 10)
        frame.setSpacing(12)
        horizontal = QSplitter(Qt.Horizontal)
        horizontal.setChildrenCollapsible(False)
        frame.addWidget(horizontal)

        # LEFT: buttons and three VISUAL panes.
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        title = QLabel('智慧社区 · 比赛控制台')
        title.setFont(QFont('Sans', 19, QFont.Bold))
        left_layout.addWidget(title)
        controls = QHBoxLayout()
        controls.setSpacing(7)
        self.start_env_button = QPushButton('启动环境')
        self.start_comp_button = QPushButton('开始比赛')
        self.stop_button = QPushButton('停止全部')
        self.refresh_button = QPushButton('刷新结果')
        for b in (self.start_env_button, self.start_comp_button,
                  self.stop_button, self.refresh_button):
            controls.addWidget(b)
        left_layout.addLayout(controls)
        self.start_env_button.clicked.connect(self.start_environment)
        self.start_comp_button.clicked.connect(self.start_competition)
        self.stop_button.clicked.connect(self.stop_all)
        self.refresh_button.clicked.connect(lambda: self.check_results(force=True))

        statuses = QHBoxLayout()
        self.env_status = QLabel('基础环境：检测中')
        self.comp_status = QLabel('比赛程序：检测中')
        statuses.addWidget(self.env_status)
        statuses.addWidget(self.comp_status)
        statuses.addStretch(1)
        left_layout.addLayout(statuses)

        visuals = QSplitter(Qt.Vertical)
        visuals.setChildrenCollapsible(False)
        self.camera_panel = ImageArea('实时摄像头', '等待摄像头画面…')
        self.person_panel = ImageArea('人员识别 · 最近完成任务', '等待人员任务完成…', max_images=3)
        self.plate_panel = ImageArea('车牌识别 · 最近完成任务', '等待车牌任务完成…', max_images=3)
        sub_visuals = QSplitter(Qt.Horizontal)
        sub_visuals.setChildrenCollapsible(False)
        sub_visuals.addWidget(self.person_panel)
        sub_visuals.addWidget(self.plate_panel)
        visuals.addWidget(self.camera_panel)
        visuals.addWidget(sub_visuals)
        visuals.setStretchFactor(0, 3)
        visuals.setStretchFactor(1, 2)
        visuals.setSizes([420, 290])
        left_layout.addWidget(visuals, 1)
        horizontal.addWidget(left)

        # RIGHT: only start_competition.sh output, plus latest JSON result.
        right = QSplitter(Qt.Vertical)
        right.setChildrenCollapsible(False)
        log_box = QGroupBox('比赛实时日志（仅 start_competition.sh）')
        log_layout = QVBoxLayout(log_box)
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.setFont(QFont('DejaVu Sans Mono', 10))
        self.log_widget.document().setMaximumBlockCount(1400)
        log_layout.addWidget(self.log_widget)
        result_box = QGroupBox('运行结果（mission_results.json）')
        result_layout = QVBoxLayout(result_box)
        self.result_widget = QTextEdit()
        self.result_widget.setReadOnly(True)
        self.result_widget.setFont(QFont('Sans', 11))
        result_layout.addWidget(self.result_widget)
        right.addWidget(log_box)
        right.addWidget(result_box)
        right.setStretchFactor(0, 3)
        right.setStretchFactor(1, 2)
        right.setSizes([440, 280])
        horizontal.addWidget(right)
        horizontal.setSizes([900, 560])
        self.statusBar().showMessage('就绪。任务证据图片仅在对应任务完成后更新。')

    def show_notice(self, message):
        self.statusBar().showMessage(message, 12000)

    def append_log(self, message, severity='normal'):
        color = {'normal': '#e2e8f0', 'warning': '#facc15', 'error': '#fb7185'}[severity]
        self.log_widget.append('<span style="color: {}">{}</span>'.format(
            color, html.escape(message)))
        self.log_widget.moveCursor(QTextCursor.End)

    def camera_updated(self, image):
        if not image.isNull():
            self.camera_panel.set_frame(image)

    def start_environment(self):
        if environment_running() or (self.env_proc and self.env_proc.poll() is None):
            QMessageBox.information(self, '提示', '基础环境已经运行，请勿重复启动。')
            return
        path = TOOLS / 'start.sh'
        if not path.is_file():
            QMessageBox.critical(self, '启动失败', f'找不到 {path}')
            return
        self.user_requested_stop = False
        self.env_tail.clear()
        try:
            self.env_proc = subprocess.Popen(
                ['bash', str(path)], cwd=str(WORKSPACE), stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding='utf-8',
                errors='replace', text=True, bufsize=1, start_new_session=True)
        except OSError as exc:
            QMessageBox.critical(self, '启动失败', str(exc))
            return
        threading.Thread(target=self._drain_env, args=(self.env_proc,), daemon=True).start()
        self.show_notice('正在启动基础环境（环境输出不会进入比赛日志）。')

    def _drain_env(self, proc):
        try:
            for line in proc.stdout:
                self.env_tail.append(line.rstrip())
        except (OSError, ValueError):
            pass
        finally:
            if proc.stdout:
                proc.stdout.close()

    def start_competition(self):
        if is_launch_running('competition.launch') or (self.comp_proc and self.comp_proc.poll() is None):
            QMessageBox.information(self, '提示', '比赛主程序已经运行。')
            return
        if not environment_running():
            QMessageBox.warning(self, '无法启动', '请先启动基础环境。')
            return
        if not ros_master_available():
            QMessageBox.warning(self, '无法启动', 'ROS Master 尚未就绪，请检查基础环境。')
            return
        ok = QMessageBox.question(
            self, '开始比赛',
            '请先在 RViz 中使用 2D Pose Estimate 确认 AMCL 定位。\n\n确认机器人位置、朝向和路线安全，开始正式比赛？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ok != QMessageBox.Yes:
            return
        path = TOOLS / 'start_competition.sh'
        if not path.is_file():
            QMessageBox.critical(self, '启动失败', f'找不到 {path}')
            return
        self.user_requested_stop = False
        master_fd, slave_fd = pty.openpty()
        try:
            proc = subprocess.Popen(
                ['bash', str(path)], cwd=str(WORKSPACE),
                stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
                start_new_session=True, close_fds=True)
        except OSError as exc:
            os.close(master_fd)
            QMessageBox.critical(self, '启动失败', str(exc))
            return
        finally:
            os.close(slave_fd)
        self.comp_proc = proc
        self.log_widget.clear()
        threading.Thread(target=self._read_competition_pty, args=(master_fd,), daemon=True).start()
        self.show_notice('比赛启动中，日志仅来自 start_competition.sh。')

    def _read_competition_pty(self, master_fd):
        """PTY keeps interactive terminal checks working; GUI approval sends yes."""
        decode = codecs.getincrementaldecoder('utf-8')('replace')
        buffer = ''
        answered = False
        try:
            while True:
                try:
                    piece = os.read(master_fd, 8192)
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        break
                    raise
                if not piece:
                    break
                buffer += decode.decode(piece).replace('\r', '\n')
                if not answered and ('输入 yes' in buffer or '输入yes' in buffer):
                    os.write(master_fd, b'yes\n')
                    answered = True
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self._handle_competition_output(line)
                buffer = buffer[-8192:]
            if buffer.strip():
                self._handle_competition_output(buffer)
        except OSError as exc:
            self.signals.notice.emit('比赛日志读取失败：' + str(exc))
        finally:
            os.close(master_fd)

    def _handle_competition_output(self, raw):
        line = ANSI_RE.sub('', raw.strip())
        if not line:
            return
        prefix = ROS_PREFIX_RE.match(line)
        severity = prefix.group(1) if prefix else 'INFO'
        clean = line[prefix.end():] if prefix else line
        useful = (any(tag in clean for tag in BUSINESS_TAGS) or
                  severity in ('WARN', 'ERROR', 'FATAL') or
                  'Traceback' in clean or 'ModuleNotFoundError' in clean or
                  'process has died' in clean or clean.startswith('[START]'))
        if useful:
            level = ('error' if severity in ('ERROR', 'FATAL') else
                     'warning' if severity == 'WARN' else 'normal')
            self.signals.log_line.emit(clean, level)

    def stop_all(self):
        if self.stopping:
            return
        path = TOOLS / 'stop.sh'
        if not path.is_file():
            QMessageBox.critical(self, '停止失败', f'找不到 {path}')
            return
        ok = QMessageBox.question(
            self, '停止确认', '将关闭比赛、导航、视觉和 Gazebo。\n请确认机器人已安全停止。',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ok != QMessageBox.Yes:
            return
        self.stopping = True
        self.user_requested_stop = True
        self.show_notice('正在执行 stop.sh …')
        threading.Thread(target=self._stop_worker, daemon=True).start()

    def _stop_worker(self):
        try:
            result = subprocess.run(
                ['bash', str(TOOLS / 'stop.sh')], cwd=str(WORKSPACE),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                encoding='utf-8', errors='replace', text=True, check=False)
            summary = '\n'.join(result.stdout.splitlines()[-6:])
            self.signals.stop_finished.emit(result.returncode == 0, summary)
        except OSError as exc:
            self.signals.stop_finished.emit(False, str(exc))

    def on_stop_finished(self, success, detail):
        self.stopping = False
        if success:
            self.show_notice('停止操作完成。')
        else:
            QMessageBox.warning(self, '停止未完成', detail or '请检查 ROS 进程。')
            self.show_notice('停止失败，请检查后台进程。')
        self.tick()

    def on_env_failure(self, text):
        QMessageBox.critical(self, '基础环境启动失败', text)

    def check_results(self, force=False):
        try:
            candidates = list(RESULTS.glob('*/mission_results.json'))
            if not candidates:
                if force:
                    self.result_widget.setPlainText('暂无比赛记录。')
                return
            path = max(candidates, key=lambda f: f.parent.name)
            stamp = (str(path), path.stat().st_mtime_ns)
            if stamp == self.last_json_stamp and not force:
                return
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError, TypeError) as exc:
            self.show_notice('读取比赛结果失败：' + str(exc))
            return
        self.last_json_stamp = stamp
        self.result_widget.setPlainText(summarize_mission(data) + '\n\n结果文件：\n' + str(path))

        # Update snapshots ONLY when mission_results.json contains a new,
        # completed corresponding task; do NOT subscribe to raw inference images.
        for task in data.get('tasks') or []:
            if not isinstance(task, dict):
                continue
            kind = task.get('task_type')
            if kind not in self.latest_task_keys:
                continue
            if task.get('status') in ('RUNNING', None):
                continue
            key = (data.get('run_id'), task.get('task_execution_id'))
            if key == self.latest_task_keys[kind]:
                continue
            self.latest_task_keys[kind] = key
            panel = self.person_panel if kind == 'person' else self.plate_panel
            panel.set_evidence(evidence_paths(path.parent, task), task_display(task))

    def tick(self):
        env = environment_running()
        comp = is_launch_running('competition.launch')
        env_starting = self.env_proc is not None and self.env_proc.poll() is None
        comp_starting = self.comp_proc is not None and self.comp_proc.poll() is None
        master_ready = ros_master_available() if env or env_starting else False

        # Only connect camera after ROS Master responds to XML-RPC.
        if env and master_ready:
            self.camera.start()
        if env and master_ready:
            env_text = '运行中'
        elif env or env_starting:
            env_text = '启动中 / 等待 ROS Master'
        else:
            env_text = '未运行'
        self.env_status.setText('基础环境：' + env_text)
        self.comp_status.setText('比赛程序：' + ('运行中' if comp else '未运行'))
        self.start_env_button.setEnabled(not self.stopping and not env and not env_starting)
        # Important: start.sh uses "exec roslaunch" and legitimately stays alive.
        # Do NOT disable the competition button merely because env_proc is running.
        self.start_comp_button.setEnabled(not self.stopping and env and master_ready
                                          and not comp and not comp_starting)
        self.stop_button.setEnabled(not self.stopping)

        if self.env_proc is not None and self.env_proc.poll() is not None:
            code = self.env_proc.returncode
            self.env_proc = None
            if code != 0 and not self.stopping and not self.user_requested_stop:
                self.signals.env_failure.emit('\n'.join(self.env_tail) or f'退出码 {code}')
        if self.comp_proc is not None and self.comp_proc.poll() is not None:
            code = self.comp_proc.returncode
            self.comp_proc = None
            if code != 0 and not self.stopping and not self.user_requested_stop:
                self.show_notice(f'比赛程序异常退出（退出码 {code}），请查看右侧日志。')
            else:
                self.show_notice('比赛程序已结束。')
        self.check_results()

    def closeEvent(self, event):
        if (environment_running() or is_launch_running('competition.launch') or
            self.stopping or (self.env_proc and self.env_proc.poll() is None) or
            (self.comp_proc and self.comp_proc.poll() is None)):
            QMessageBox.warning(self, '程序仍在运行',
                                '请先点击“停止全部”，确保比赛与仿真停止后再退出 GUI。')
            event.ignore()
            return
        self.camera.close()
        event.accept()


def main():
    if not WORKSPACE.is_dir():
        raise SystemExit('请在 Smart-Community 的 Docker 容器内运行（缺少 /workspace/car_2026）。')
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    gui = CompetitionGUI()
    gui.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
