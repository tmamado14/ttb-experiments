#!/usr/bin/env python3
"""
Open-vocabulary object detection for the seek behaviour.
========================================================
Wraps YOLO-World, which takes its class list as plain text at inference time
(`set_classes(["bottle"])`). That is the whole reason this is a detector and
not a classifier: "go to the X" has to work for an X nobody enumerated when the
server started, and a fixed COCO head would cap us at its 80 nouns.

Deliberately NOT a vision-language model. A VLM is the obvious-looking answer
and the wrong one here: it answers in prose rather than pixel coordinates, and
takes seconds per frame. An approach loop needs a box position at close to
camera rate, and the robot is moving the whole time it waits.

This module is pure perception -- it decides what is in a frame and where.
Whether a detection is trustworthy enough to drive at is a behaviour question,
and lives in seek.py.
"""

import threading
import time
from collections import namedtuple

import cv2

# cx, cy, w, h are pixels in the frame as published; conf is 0..1.
Box = namedtuple("Box", "cx cy w h conf")


class Detector:
    """YOLO-World behind a lock, with the target class swapped per call.

    One instance is shared by the seek loop and the /detect endpoint, which run
    on different threads. set_classes() mutates model state, so an unguarded
    /detect?target=chair during a "go to the bottle" would repoint the model
    mid-approach and the robot would start driving at a chair. The lock plus
    per-call target makes interleaving safe; the cost is only that alternating
    targets re-encode the text prompt.
    """

    # Far below ultralytics' 0.25 default, and measured rather than guessed.
    # Sampled over 10 live frames of this camera (which is dim -- mean luma
    # 63/255):
    #
    #   present:  chair  0.134-0.241 (median 0.166)   tv  0.50-0.68
    #   absent:   bottle 0.028-0.064                  backpack 0.000
    #
    # So the gap between "in the room" and "not in the room" sits around 0.10,
    # and the stock 0.25 is above EVERY reading a real chair produced. Even
    # 0.15 catches a present chair in only 6 frames out of 10, which through
    # 2-of-3 confirmation is a ~65% chance of ever noticing furniture the robot
    # is staring at. At 0.10 it is 10/10, with absent objects still a clear
    # factor below.
    #
    # False positives are held off where they belong -- by requiring the target
    # in k of n consecutive frames before driving at it (see seek.py) -- not by
    # a threshold set high enough to also reject the true positives.
    def __init__(self, weights, conf=0.10, device=None, imgsz=640):
        # Imported here, not at module scope: ultralytics pulls in torch, which
        # costs seconds and a few hundred MB. Someone running the server without
        # --enable-seek should pay none of that.
        from ultralytics import YOLOWorld
        import torch

        if device is None:
            device = 0 if torch.cuda.is_available() else "cpu"
        self.device = device
        self.conf = conf
        self.imgsz = imgsz
        self.weights = weights
        self._model = YOLOWorld(weights)
        self._lock = threading.Lock()
        self._classes = None
        self.last_ms = None

    def warmup(self, target="object"):
        """Pay the first-call cost off the control loop.

        Measured on this laptop's RTX 3060: 722 ms for the first predict() and
        ~19 ms for every one after. Left until the first real command, that
        would land as most of a second of the robot rotating blind.
        """
        import numpy as np
        blank = np.zeros((640, 480, 3), dtype="uint8")
        t = time.monotonic()
        self.detect(blank, target)
        return time.monotonic() - t

    def _set_target(self, target):
        """Caller must hold the lock."""
        if self._classes != [target]:
            self._model.set_classes([target])
            self._classes = [target]

    def detect_all(self, frame, target):
        """All boxes for `target` in `frame`, best confidence first."""
        with self._lock:
            self._set_target(target)
            t = time.monotonic()
            res = self._model.predict(frame, verbose=False, device=self.device,
                                      imgsz=self.imgsz, conf=self.conf)
            self.last_ms = round((time.monotonic() - t) * 1000)

        boxes = []
        for r in res:
            for b in r.boxes:
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                boxes.append(Box(cx=(x1 + x2) / 2.0, cy=(y1 + y2) / 2.0,
                                 w=x2 - x1, h=y2 - y1, conf=float(b.conf[0])))
        boxes.sort(key=lambda b: b.conf, reverse=True)
        return boxes

    def detect(self, frame, target):
        """The single most confident box for `target`, or None.

        Most confident rather than largest: a partially occluded target the
        model is sure about is a better thing to drive at than a big vague blob.
        """
        boxes = self.detect_all(frame, target)
        return boxes[0] if boxes else None

    @staticmethod
    def annotate(frame, boxes, target, best_only=False):
        """Draw boxes on a copy of `frame`, for the /detect preview.

        Exists so detection can be checked with the robot standing still --
        the first verification stage, before anything is allowed to move.
        """
        out = frame.copy()
        h, w = out.shape[:2]
        # Centre line: the CENTER state drives the box onto exactly this.
        cv2.line(out, (w // 2, 0), (w // 2, h), (255, 200, 0), 1)
        for i, b in enumerate(boxes):
            if best_only and i:
                break
            colour = (0, 230, 0) if i == 0 else (0, 140, 255)
            x1, y1 = int(b.cx - b.w / 2), int(b.cy - b.h / 2)
            x2, y2 = int(b.cx + b.w / 2), int(b.cy + b.h / 2)
            cv2.rectangle(out, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(out, f"{target} {b.conf:.2f}", (x1, max(14, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
        if not boxes:
            cv2.putText(out, f"no {target}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return out
