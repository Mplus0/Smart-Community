"""Inspection state and task-facing perception rules."""
from collections import Counter
from datetime import datetime, timezone
import time

from rules import (Detection, detections, people_counts, illegal_parking_count,
                   trash_roi, trash_result)


class Inspection:
    def __init__(self, route, services, speech, settings, bridge, ocr_interval=3.0):
        self.route, self.services, self.speech = route, services, speech
        self.settings, self.ocr_interval = settings, ocr_interval
        self.bridge = bridge
        self.people = Counter(A=0, B=0)
        self.outside = Counter(A=0, B=0)
        self.illegal = Counter(A=0, B=0)
        self.plates = {}
        self.people_points = set()
        self.vehicle_frames = []
        self.last_ocr = 0.0

    def inspect_people(self, point, image):
        street = point['area']
        objects = detections(self.services.detect('people_detector', image))
        total, outside, bounds = people_counts(objects, image.width, image.height, self.settings)
        self.people[street] += total
        self.outside[street] += outside
        # Same image for both services; a missing person ROI cannot leak from a previous point.
        vehicles = detections(self.services.detect('bicycle_detector', image))
        illegal = illegal_parking_count(vehicles, bounds, image.width, image.height, self.settings)
        self.illegal[street] += illegal
        self.people_points.add(point['id'])

        return dict(area=street, people=total, outside=outside, illegal_parking=illegal)

    def inspect_plate(self, point, image):
        # Preserve the original OCR request spacing.
        time.sleep(max(0, self.ocr_interval - (time.monotonic()-self.last_ocr)))
        response = self.services.ocr(image)
        self.last_ocr = time.monotonic()
        lot = point['parking_lot']
        plate = response.plate_texts[0].strip() if response.plate_texts else ''
        self.plates[lot] = plate or '未检测到车牌'

        return dict(parking_lot=lot, plate=plate)

    def inspect_fire(self, point, image):
        objects = detections(self.services.detect('fire_detector', image))
        count = sum(d.label == 'fire' for d in objects)
        if not count:
            return dict(fire_count=0)
        # A failed building-name OCR must not suppress the fire report.
        building = '大厦'
        ocr_error = ''
        try:
            response = self.services.ocr(image)
            building = response.plate_texts[0] if response.plate_texts else building
        except Exception as error:
            ocr_error = str(error)

        return dict(fire_count=count, building=building, ocr_error=ocr_error)

    def inspect_trash(self, point, image):
        bins = detections(self.services.detect('trash_bin_detector', image))
        roi = trash_roi(bins, image.width, image.height, self.settings)
        waste = []
        if roi is not None:
            x1, y1, x2, y2 = roi
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            cropped = self.bridge.cv2_to_imgmsg(frame[y1:y2, x1:x2].copy(), encoding='bgr8')
            cropped.header = image.header
            objects = detections(self.services.detect('waste_detector', cropped))
            waste = [Detection(d.label, (d.box[0] + x1, d.box[1] + y1,
                                         d.box[2] + x1, d.box[3] + y1), d.confidence)
                     for d in objects]
        return dict(trash_result(bins, waste, self.settings), roi=roi)

    def inspect_vehicle_photo(self, point, image):
        objects = detections(self.services.detect('bicycle_detector', image))
        self.vehicle_frames.append((objects, image.width, image.height))

        return dict(detected_classes=[d.label for d in objects])

    def inspect_video(self, point, path):
        result = self.services.count_video(path)

        return dict(stand_count=result.stand_count, fall_count=result.fall_count)

    def execute(self, point, task_type, source):
        """Run one recognition task; source is an image or a video path.

        Empty observations are valid results. Failures are returned for the mission
        to persist and terminate, never retried here.
        """
        started = time.monotonic()
        record = dict(waypoint_id=point['id'], task_type=task_type, status='FAILED',
                      reason='', result={}, duration_sec=0.0,
                      started_at=datetime.now(timezone.utc).isoformat())
        handlers = {'person': self.inspect_people, 'plate': self.inspect_plate,
                    'fire': self.inspect_fire, 'trash': self.inspect_trash,
                    'vehicle_photo': self.inspect_vehicle_photo, 'vehicle_video': self.inspect_video}
        empty = {'person': lambda r: r['people'] == 0,
                 'plate': lambda r: not r['plate'],
                 'fire': lambda r: r['fire_count'] == 0,
                 'trash': lambda r: not r['bins'] or not r['waste'],
                 'vehicle_photo': lambda r: not any(c in ('stand', 'fall') for c in r['detected_classes']),
                 'vehicle_video': lambda r: r['stand_count'] + r['fall_count'] == 0}
        try:
            if task_type not in handlers:
                raise ValueError('Unknown inspection task: ' + task_type)
            result = handlers[task_type](point, source)
            is_empty = empty[task_type](result)
            record.update(result=result, status='EMPTY_VALID' if is_empty else 'SUCCESS',
                          reason='未检测到有效目标' if is_empty else '识别完成')
            if result.get('ocr_error'):
                record['reason'] = '已识别火情；楼名 OCR 异常，沿用默认楼名：' + result['ocr_error']
        except Exception as error:
            record['reason'] = f'{type(error).__name__}: {error}'
        record.update(duration_sec=time.monotonic()-started,
                      ended_at=datetime.now(timezone.utc).isoformat())
        return record
