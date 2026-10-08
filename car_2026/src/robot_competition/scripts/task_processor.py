#!/usr/bin/env python3
"""Finite task windows and lightweight, conservative temporal fusion."""
from collections import Counter
import time
import unicodedata

from perception_client import finite, integer

DEFAULTS = {
    'task_timeout_sec': 12.0, 'settle_sec': 0.5, 'observation_sec': 2.0,
    'min_valid_frames': 4, 'max_valid_frames': 24, 'min_support_frames': 3,
    'max_source_age_sec': 2.0, 'image_match_timeout_sec': 1.5,
    'task_retry_count': 1, 'min_confidence': 0.5, 'match_iou': 0.3,
    'match_center_distance': 0.08, 'max_track_gap_frames': 2,
    'vote_fraction': 0.67, 'max_evidence_images': 3,
    'plate_text_annotation_confirmed': False,
}


def task_settings(config, kind):
    settings = dict(DEFAULTS)
    for section in ('defaults', kind):
        overrides = config.get(section, {})
        if not isinstance(overrides, dict) or set(overrides) - set(DEFAULTS):
            raise ValueError('invalid task_config.' + section)
        settings.update(overrides)
    for key in ('task_timeout_sec', 'observation_sec', 'max_source_age_sec', 'image_match_timeout_sec'):
        finite(settings[key], key, 0.001)
    finite(settings['settle_sec'], 'settle_sec', 0)
    for key in ('min_valid_frames', 'max_valid_frames', 'min_support_frames', 'max_evidence_images'):
        integer(settings[key], key, 1)
    for key in ('task_retry_count', 'max_track_gap_frames'):
        integer(settings[key], key, 0)
    for key in ('min_confidence', 'match_iou', 'match_center_distance', 'vote_fraction'):
        finite(settings[key], key, 0, 1)
    if not 1 <= settings['min_support_frames'] <= settings['min_valid_frames'] <= settings['max_valid_frames'] <= 128:
        raise ValueError('require support <= min_valid <= max_valid <= 128')
    if settings['observation_sec'] > settings['task_timeout_sec']:
        raise ValueError('observation_sec exceeds task_timeout_sec')
    if settings['max_evidence_images'] > 8:
        raise ValueError('max_evidence_images must be <= 8')
    if type(settings['plate_text_annotation_confirmed']) is not bool:
        raise ValueError('plate_text_annotation_confirmed must be boolean')
    return settings


def overlap(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection / union if union else 0


def associate(frames, kind, settings):
    """One-to-one greedy assignment within ONE fixed observation window only."""
    tracks = []
    multiple_in_frame = False
    field = 'detections' if kind == 'person' else 'plates'
    for index, frame in enumerate(frames):
        payload = frame['payload']
        objects = [item for item in payload[field] if item['confidence'] >= settings['min_confidence']]
        multiple_in_frame |= len(objects) > 1
        pairs = []
        for oi, item in enumerate(objects):
            box = item['bbox_xyxy']
            for ti, track in enumerate(tracks):
                if index - track['last_index'] > settings['max_track_gap_frames'] + 1:
                    continue
                if kind == 'person' and item['class_id'] != track['samples'][-1][1]['class_id']:
                    continue
                previous = track['samples'][-1][1]['bbox_xyxy']
                iou = overlap(box, previous)
                distance = (((box[0]+box[2]-previous[0]-previous[2])/(2*payload['width']))**2 +
                            ((box[1]+box[3]-previous[1]-previous[3])/(2*payload['height']))**2)**0.5
                if iou >= settings['match_iou'] or distance <= settings['match_center_distance']:
                    pairs.append((iou - distance, oi, ti))
        assigned_objects, assigned_tracks = set(), set()
        for score, oi, ti in sorted(pairs, reverse=True):
            if oi in assigned_objects or ti in assigned_tracks:
                continue
            tracks[ti]['samples'].append((index, objects[oi]))
            tracks[ti]['last_index'] = index
            assigned_objects.add(oi)
            assigned_tracks.add(ti)
        for oi, item in enumerate(objects):
            if oi not in assigned_objects:
                tracks.append({'samples': [(index, item)], 'last_index': index})
    return tracks, multiple_in_frame


def normalized_plate(text):
    return ''.join(unicodedata.normalize('NFKC', text).upper().split())


def fuse(frames, kind, settings):
    field = 'detections' if kind == 'person' else 'plates'
    if not any(frame['payload'][field] for frame in frames):
        return {'status': 'EMPTY_VALID', 'reason': 'enough valid empty frames',
                'detections': [], 'plates': [], 'evidence_options': [[len(frames)-1]]}
    tracks, multiple = associate(frames, kind, settings)
    stable = [t for t in tracks if len(t['samples']) >= settings['min_support_frames']]
    if kind == 'person':
        detections, options = [], []
        for track in stable:
            samples = track['samples']
            best_index, best = max(samples, key=lambda sample: (sample[1]['confidence'], sample[0]))
            detections.append({
                'class_id': best['class_id'], 'label': best['label'], 'display_label': best['display_label'],
                'confidence': sum(item['confidence'] for _, item in samples) / len(samples),
                'support_frames': len(samples), 'bbox_xyxy': best['bbox_xyxy'],
                'representative_frame_index': best_index,
                'support_observations': [{'frame_index': i, **item} for i, item in samples],
            })
            # A representative bbox is tied to its exact source frame, not an average box.
            options.append([i for i, _ in sorted(samples, key=lambda s: (s[1]['confidence'], s[0]), reverse=True)])
        return {'status': 'SUCCESS' if stable else 'FAILED_UNSTABLE',
                'reason': 'stable local person tracks' if stable else 'nonempty frames but insufficient high-confidence support',
                'detections': detections, 'plates': [], 'evidence_options': options or [[len(frames)-1]]}
    # Without an expected target ROI/identity, never assign an arbitrary vehicle.
    if multiple or len(tracks) > 1:
        indices = sorted({i for t in tracks for i, _ in t['samples']}, reverse=True)
        return {'status': 'AMBIGUOUS', 'reason': 'multiple spatial plate candidates; no vehicle association',
                'detections': [], 'plates': [], 'evidence_options': [indices or [len(frames)-1]]}
    if not stable:
        return {'status': 'FAILED_UNSTABLE', 'reason': 'insufficient high-confidence plate support',
                'detections': [], 'plates': [], 'evidence_options': [[len(frames)-1]]}
    samples = stable[0]['samples']
    votes = Counter(normalized_plate(item['text']) for _, item in samples)
    ranking = votes.most_common()
    winner, support = ranking[0]
    if (not winner or support < settings['min_support_frames'] or support/len(samples) < settings['vote_fraction'] or
            (len(ranking) > 1 and ranking[1][1] == support)):
        return {'status': 'AMBIGUOUS', 'reason': 'conflicting full plate strings',
                'detections': [], 'plates': [], 'votes': dict(votes),
                'evidence_options': [[i for i, _ in reversed(samples)]]}
    supporters = [(i, item) for i, item in samples if normalized_plate(item['text']) == winner]
    best_index, best = max(supporters, key=lambda pair: (pair[1]['confidence'], pair[0]))
    return {'status': 'SUCCESS', 'reason': 'full-string majority on one local plate track', 'detections': [],
            'plates': [{'text': winner, 'raw_text': best['text'], 'confidence': sum(s['confidence'] for _, s in supporters)/support,
                        'support_frames': support, 'vote_fraction': support/len(samples), 'plate_type_id': best['plate_type_id'],
                        'bbox_xyxy': best['bbox_xyxy'], 'representative_frame_index': best_index,
                        'support_observations': [{'frame_index': i, **item} for i, item in supporters]}],
            'votes': dict(votes), 'evidence_options': [[i for i, _ in sorted(supporters, key=lambda s: (s[1]['confidence'], s[0]), reverse=True)]]}


class TaskProcessor:
    def __init__(self, client, results, check_running, config=None, sleep=time.sleep):
        self.client, self.results, self.check_running = client, results, check_running
        self.config = config or {}
        self.settings = {kind: task_settings(self.config, kind) for kind in ('person', 'plate')}
        self.sleep = sleep

    def _wait_until(self, deadline):
        while self.client.wall() < deadline:
            self.check_running()
            self.sleep(min(0.02, max(0, deadline - self.client.wall())))
        self.check_running()

    def execute(self, point):
        settings = self.settings[point['task']]
        for attempt in range(settings['task_retry_count'] + 1):
            result = self._attempt(point, attempt + 1, settings)
            if result['status'] in ('SUCCESS', 'EMPTY_VALID', 'FAILED_IMAGE_ANNOTATION'):
                break
        return result

    def _attempt(self, point, attempt, settings):
        result = self.results.begin_task(point, attempt, settings)
        started = self.client.wall()
        frames, window = [], None
        try:
            self._wait_until(started + settings['settle_sec'])
            window = self.client.begin_window(point['task'], settings['max_source_age_sec'])
            result['window_start'] = {'ros_ns': window.start_ros_ns, 'wall_monotonic': window.start_wall}
            deadline = window.start_wall + settings['task_timeout_sec']
            while True:
                self.check_running()
                frames.extend(self.client.drain(window))
                frames = frames[:settings['max_valid_frames']]
                elapsed = self.client.wall() - window.start_wall
                if ((len(frames) >= settings['min_valid_frames'] and elapsed >= settings['observation_sec']) or
                        len(frames) >= settings['max_valid_frames'] or self.client.wall() >= deadline):
                    break
                self.sleep(0.02)
            result['valid_frames'] = len(frames)
            result['raw_frames'] = [frame['payload'] for frame in frames]
            result['diagnostics'] = self.client.diagnostics(window)
            if len(frames) < settings['min_valid_frames']:
                result['status'] = 'FAILED_INVALID' if result['diagnostics'].get('invalid_json', 0) else 'FAILED_TIMEOUT'
                result['reason'] = 'insufficient valid new frames before wall-time deadline'
            else:
                result.update(fuse(frames, point['task'], settings))
                options = result.pop('evidence_options')
                selected, chosen, missing = {}, {}, list(range(len(options)))
                image_deadline = self.client.wall() + settings['image_match_timeout_sec']
                while missing:
                    self.check_running()
                    # Also checks the epoch while waiting for an asynchronously published image.
                    self.client.drain(window)
                    for index in list(missing):
                        for fi in options[index]:
                            if fi in selected:
                                chosen[index] = fi
                                missing.remove(index)
                                break
                            if len(selected) >= settings['max_evidence_images']:
                                continue
                            image = self.client.matching_image(window, frames[fi])
                            if image is not None:
                                selected[fi] = image
                                chosen[index] = fi
                                missing.remove(index)
                                break
                    if not missing or self.client.wall() >= image_deadline:
                        break
                    self.sleep(0.02)
                result['decision_status'] = result['status']
                result['image_missing'] = bool(missing)
                result['uncovered_evidence_groups'] = missing
                for fi, image in selected.items():
                    self.results.save_evidence(result, frames[fi]['payload']['header'], image, fi)
                objects = result['detections'] if point['task'] == 'person' else result['plates']
                for index, obj in enumerate(objects):
                    obj['evidence_image_path'] = None
                    if index in chosen:
                        fi = chosen[index]
                        sample = next(s for s in obj['support_observations'] if s['frame_index'] == fi)
                        obj['representative_frame_index'] = fi
                        obj['bbox_xyxy'] = sample['bbox_xyxy']
                        obj['representative_confidence'] = sample['confidence']
                        obj['evidence_image_path'] = next(e['image_path'] for e in result['evidence'] if e['frame_index'] == fi)
                        if point['task'] == 'plate':
                            obj['raw_text'] = sample['text']
                if missing:
                    result['status'] = 'FAILED_IMAGE_MISSING'
                    result['reason'] += '; matching annotated images missing or evidence limit exceeded'
                elif point['task'] == 'plate' and result['status'] == 'SUCCESS' and not settings['plate_text_annotation_confirmed']:
                    result['status'] = 'FAILED_IMAGE_ANNOTATION'
                    result['reason'] += '; Chinese image text has not been confirmed (plate# is insufficient)'
            self.check_running()
        except Exception as error:
            result.update(status='FAILED_INTERRUPTED', reason=str(error), valid_frames=len(frames),
                          raw_frames=[frame['payload'] for frame in frames])
            result['task_duration_sec'] = self.client.wall() - started
            self.results.record_task(result)
            raise
        result['task_duration_sec'] = self.client.wall() - started
        self.results.record_task(result)
        return result
