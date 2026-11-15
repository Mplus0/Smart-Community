"""无需相机的录像回归检查：python -m unittest discover -s robot_ws/tools -p test_collect_dataset.py。"""
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np

from collect_dataset import VideoRecorder, start_web_preview


class Source:
    def __init__(self):
        self.frame = np.full((48, 64, 3), 120, dtype=np.uint8)
        self.failed = False

    def image(self, generation=None):
        if self.failed:
            raise RuntimeError('No fresh frame')
        return self.frame, 1, 'ros'


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name) / 'videos'
        self.source = Source()
        self.recorder = VideoRecorder(self.source, self.folder, 10)

    def tearDown(self):
        self.recorder.close()
        self.temp.cleanup()

    def read_video(self):
        video = cv2.VideoCapture(str(self.folder / self.recorder.filename))
        try:
            self.assertTrue(video.isOpened())
            self.assertEqual(video.get(cv2.CAP_PROP_FPS), 10)
            frames = 0
            while True:
                ok, image = video.read()
                if not ok:
                    break
                self.assertEqual(image.shape[:2], (48, 64))
                frames += 1
            return frames
        finally:
            video.release()

    def test_record_stop_restart_and_exit_finalize_playable_files(self):
        self.recorder.start()
        first = self.recorder.filename
        self.recorder.start()  # Duplicate start must not replace the file.
        self.assertEqual(self.recorder.filename, first)
        time.sleep(.35)
        self.recorder.stop()
        self.recorder.stop()
        self.assertGreaterEqual(self.read_video(), 2)
        self.recorder.start()
        self.assertNotEqual(first, self.recorder.filename)
        self.recorder.close()
        self.assertGreaterEqual(self.read_video(), 1)

    def test_stale_source_and_resolution_change_stop_recording(self):
        for stale in (True, False):
            self.source.frame = np.zeros((48, 64, 3), dtype=np.uint8)
            self.source.failed = False
            self.recorder.start()
            self.source.failed = stale
            if not stale:
                self.source.frame = np.zeros((24, 32, 3), dtype=np.uint8)
            deadline = time.monotonic() + 2
            while self.recorder.status()['recording'] and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertFalse(self.recorder.status()['recording'])
            self.assertTrue(self.recorder.status()['record_error'])
            self.assertGreaterEqual(self.read_video(), 1)

    def test_web_recording_and_photo_can_coexist(self):
        args = SimpleNamespace(web_host='127.0.0.1', web_port=0, camera='ros')
        photos = []
        def save(generation):
            photos.append(generation)
            return {'count': len(photos)}
        server = start_web_preview(args, self.source, save, self.recorder.status,
                                   threading.Event(), self.recorder)
        def post(path):
            request = Request('http://127.0.0.1:%s%s' % (server.server_port, path),
                              data=b'{"generation":0}',
                              headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=3) as response:
                return json.load(response)
        try:
            self.assertTrue(post('/record/start')['recording'])
            self.assertEqual(post('/capture')['count'], 1)
            self.assertTrue(self.recorder.status()['recording'])
            self.assertFalse(post('/record/stop')['recording'])
            self.assertGreaterEqual(self.read_video(), 1)
            self.source.failed = True
            with self.assertRaises(HTTPError) as failure:
                post('/record/start')
            self.assertEqual(failure.exception.code, 503)
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
