"""Inspection state and task-facing perception rules."""
from collections import Counter
import time

from .rules import (detections, people_counts, illegal_parking_count,
                    trash_announcement, vehicle_counts, vehicle_announcement)


class Inspection:
    def __init__(self, route, services, speech):
        self.route, self.services, self.speech = route, services, speech
        self.people = Counter(A=0, B=0)
        self.outside = Counter(A=0, B=0)
        self.illegal = Counter(A=0, B=0)
        self.plates = {}
        self.people_points = set()
        self.vehicle_frames = []
        self.last_ocr = 0.0

    def inspect_people(self, index, image):
        street = 'A' if index in self.route['photo_idx'][:3] else 'B'
        objects = detections(self.services.detect('people_detector', image))
        total, outside, bounds = people_counts(objects, image.width, image.height)
        self.people[street] += total
        self.outside[street] += outside
        # Same image for both services; a missing person ROI cannot leak from a previous point.
        vehicles = detections(self.services.detect('bicycle_detector', image))
        self.illegal[street] += illegal_parking_count(vehicles, bounds, image.width, image.height)
        self.people_points.add(index)
        if self.people_points == set(self.route['photo_idx']):
            self.speech.put(f'社区内共有{sum(self.people.values())}人，其中A街人数{self.people["A"]}人，'
                            f'B街人数{self.people["B"]}人，发现{self.outside["A"]}名非社区人员在A街，'
                            f'发现{self.outside["B"]}名非社区人员在B街，图片已保存')

    def inspect_plate(self, index, image):
        # Preserve the original OCR request spacing.
        time.sleep(max(0, 3.0 - (time.monotonic()-self.last_ocr)))
        response = self.services.ocr(image)
        self.last_ocr = time.monotonic()
        lot = self.route['license_plate_idx'].index(index)+1
        plate = response.plate_texts[0].strip() if response.plate_texts else ''
        self.plates[lot] = plate or '未检测到车牌'
        if len(plate) >= 2:
            self.speech.put(f'{lot}号停车场车牌为'+' '.join(plate))

    def inspect_fire(self, image):
        objects = detections(self.services.detect('fire_detector', image))
        count = sum(d.label == 'fire' for d in objects)
        if not count:
            return
        # A failed building-name OCR must not suppress the fire report.
        building = '大厦'
        try:
            response = self.services.ocr(image)
            building = response.plate_texts[0] if response.plate_texts else building
        except Exception:
            pass
        self.speech.put(f'{building}发现火灾隐患{count}处')

    def inspect_trash(self, image):
        bins = detections(self.services.detect('trash_bin_detector', image))
        waste = detections(self.services.detect('waste_detector', image))
        self.speech.put(trash_announcement(bins, waste, image.width))

    def inspect_vehicle_photo(self, image):
        objects = detections(self.services.detect('bicycle_detector', image))
        self.vehicle_frames.append((objects, image.width, image.height))
        if len(self.vehicle_frames) == len(self.route['electrocar_idx']):
            stand, fall = vehicle_counts(self.vehicle_frames)
            self.speech.put(vehicle_announcement(self.illegal, stand, fall))

    def inspect_video(self, path):
        result = self.services.count_video(path)
        self.speech.put(vehicle_announcement(self.illegal, result.stand_count, result.fall_count))

    def run_at(self, index, image):
        if index in self.route['photo_idx']:
            self.inspect_people(index, image)
        if index in self.route['license_plate_idx']:
            self.inspect_plate(index, image)
        if index in self.route['fire_idx']:
            self.inspect_fire(image)
        if index in self.route['trash_can_idx']:
            self.inspect_trash(image)
        if index in self.route['electrocar_idx']:
            self.inspect_vehicle_photo(image)
