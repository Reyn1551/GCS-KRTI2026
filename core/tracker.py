"""
KP2026 SimpleTracker — IoU matching + EMA smoothing.

Shared by gate_mission and video render tools.
"""

from typing import Dict, List, Tuple


class SimpleTracker:
    """Lightweight tracker: IoU matching + EMA smoothing. Prevents track loss."""

    def __init__(
        self,
        max_miss: int = 8,
        iou_thres: float = 0.3,
        smooth_alpha: float = 0.5,
    ):
        self.tracks: Dict[int, Dict] = {}
        self.next_id = 0
        self.max_miss = max_miss
        self.iou_thres = iou_thres
        self.smooth_alpha = smooth_alpha

    def _iou(self, a: Tuple, b: Tuple) -> float:
        x1 = max(a[0], b[0])
        y1 = max(a[1], b[1])
        x2 = min(a[2], b[2])
        y2 = min(a[3], b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[2] - b[0]) * (b[3] - b[1])
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0

    def update(self, detections: List[Dict]) -> List[Tuple]:
        """
        Update tracks with new detections.
        Returns: list of (track_id, bbox, cls_id, score, age)
        """
        for tid in self.tracks:
            self.tracks[tid]["missed"] += 1

        matched_tids: set = set()
        matched_dids: set = set()

        for did, det in enumerate(detections):
            best_iou = self.iou_thres
            best_tid = None
            for tid, track in self.tracks.items():
                if tid in matched_tids:
                    continue
                iou = self._iou(track["bbox"], det["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    best_tid = tid
            if best_tid is not None:
                matched_tids.add(best_tid)
                matched_dids.add(did)
                track = self.tracks[best_tid]
                a = self.smooth_alpha
                b = det["bbox"]
                t = track["bbox"]
                track["bbox"] = (
                    int(a * b[0] + (1 - a) * t[0]),
                    int(a * b[1] + (1 - a) * t[1]),
                    int(a * b[2] + (1 - a) * t[2]),
                    int(a * b[3] + (1 - a) * t[3]),
                )
                track["score"] = a * det["score"] + (1 - a) * track["score"]
                track["cls"] = det["class_id"]
                track["age"] += 1
                track["missed"] = 0

        for did, det in enumerate(detections):
            if did not in matched_dids:
                tid = self.next_id
                self.next_id += 1
                self.tracks[tid] = {
                    "bbox": tuple(map(int, det["bbox"])),
                    "cls": det["class_id"],
                    "score": det["score"],
                    "age": 1,
                    "missed": 0,
                }

        stale = [tid for tid, t in self.tracks.items() if t["missed"] > self.max_miss]
        for tid in stale:
            del self.tracks[tid]

        results: List[Tuple] = []
        for tid, t in self.tracks.items():
            if t["age"] >= 2:
                results.append((tid, t["bbox"], t["cls"], t["score"], t["age"]))
        return results
