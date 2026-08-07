#!/usr/bin/env python3
"""
Seek-and-approach: turn "go to the bottle" into motion.
=======================================================
Two separable jobs live here.

SeekParser pulls the *target noun* out of a sentence using the same local
ollama model the driving commands use. It is an intent parser and nothing more
-- it never sees a pixel and never decides when to stop.

SeekBehaviour is the control loop: rotate until the camera finds the target,
turn to face it, drive at it, stop short of it. Bearing comes from vision,
range comes from the lidar, and the two are deliberately decoupled -- centre
the target first, then read straight ahead -- which is what lets the whole
thing work without ever calibrating the camera's field of view.

It runs on MotionExecutor's single motion thread, under MotionExecutor's cancel
event. That is not incidental: the executor owns the "one bounded, cancellable
motion at a time" invariant that makes STOP mean stop, and a behaviour that
commanded velocity from its own thread would quietly break it.
"""

import json
import math
import re
import time

import httpx
from pydantic import BaseModel


# --------------------------------------------------------------------------- #
# Language: sentence -> target noun
# --------------------------------------------------------------------------- #

# Deterministic router. A seek command is recognised here, in Python, rather
# than by widening the driving schema, because the driving schema is
# constrained-decoded: every property in it is required, so adding a `target`
# would make the model emit a target for "move forward 1 meter" too, and a 3B
# model handed a slot it must fill will invent something to fill it with. The
# existing prompt and schema therefore stay byte-identical and this pattern
# decides which of the two parsers a step goes to.
SEEK_PATTERN = re.compile(
    r"\b(?:go\s+to|goto|go\s+towards?|head\s+(?:to|towards?)|drive\s+(?:to|towards?)|"
    r"move\s+(?:to|towards?)|approach|find|look\s+for|search\s+for|seek)\b",
    re.I)

# One field. Nothing about distance or speed: how far to go is what the lidar
# is for, and letting the model volunteer a number here would be inviting it to
# make one up.
SEEK_SCHEMA = {
    "type": "object",
    "properties": {"target": {"type": "string"}},
    "required": ["target"],
}

SEEK_SYSTEM_PROMPT = """\
You extract the object a robot has been asked to drive to.

Return JSON with one field, "target": the object, as a short lowercase noun
phrase, with no articles and no leading verb.

Keep any word that changes which object is meant (a colour, a material) and
drop any word that does not (speed, politeness, urgency).
If no physical object is named, return an empty string.

Examples:
  "go to the bottle"              -> bottle
  "find the red backpack"         -> red backpack
  "approach the chair slowly"     -> chair
  "head towards that potted plant"-> potted plant
  "drive to the person"           -> person
  "go forward 2 meters"           -> (empty string)
"""


class SeekIntent(BaseModel):
    target: str = ""


class SeekParser:
    """Sentence -> target noun, via the same ollama server as NLParser."""

    def __init__(self, url, model, timeout=30.0):
        self.url = url.rstrip("/")
        self.model = model
        self._client = httpx.Client(timeout=timeout)

    @staticmethod
    def looks_like_seek(text):
        return bool(SEEK_PATTERN.search(text or ""))

    def parse(self, text):
        """Return (target, None) or (None, error_message)."""
        body = {
            "model": self.model,
            "stream": False,
            "keep_alive": "10m",
            "options": {"temperature": 0},
            "format": SEEK_SCHEMA,
            "messages": [
                {"role": "system", "content": SEEK_SYSTEM_PROMPT},
                {"role": "user", "content": text},
            ],
        }
        try:
            r = self._client.post(f"{self.url}/api/chat", json=body)
            r.raise_for_status()
            intent = SeekIntent(**json.loads(r.json()["message"]["content"]))
        except httpx.HTTPError as e:
            return None, (f"cannot reach the language model at {self.url} "
                          f"({type(e).__name__})")
        except Exception as e:  # noqa: BLE001 - malformed model output
            return None, ("the language model returned something unusable "
                          f"({type(e).__name__})")

        target = " ".join((intent.target or "").lower().split())
        target = re.sub(r"^(?:the|a|an)\s+", "", target).strip(" .,")
        if not target:
            return None, "I couldn't tell what object you want me to go to"
        if len(target) > 40:
            return None, "that target doesn't look like an object name"
        return target, None


# --------------------------------------------------------------------------- #
# Behaviour: target noun -> motion
# --------------------------------------------------------------------------- #
class SeekConfig:
    """Tunables and caps. Caps are refused, never clamped -- same as chains."""

    def __init__(self, stop_distance=0.35, max_travel=2.5, search_step_deg=25.0,
                 search_max_deg=400.0, total_timeout=60.0, conf=0.15,
                 confirm_k=2, confirm_n=3):
        # Floor well clear of the LDS-01's 0.12 m minimum: below its range_min
        # the lidar reports a dropout, which front_range() correctly discards --
        # so a stop distance set too low is not "stop later", it is "stop blind".
        self.stop_distance = max(0.25, float(stop_distance))
        self.max_travel = float(max_travel)
        self.search_step_deg = float(search_step_deg)
        self.search_max_deg = float(search_max_deg)
        self.total_timeout = float(total_timeout)
        self.conf = float(conf)
        self.confirm_k = int(confirm_k)
        self.confirm_n = int(confirm_n)


class SeekBehaviour:
    """SEARCH -> CENTER -> APPROACH -> done, on the executor's thread."""

    TICK = 0.05                  # matches the executor's closed-loop period

    # Pause after each search step before believing what the camera shows.
    #
    # This has to exceed the camera's end-to-end LATENCY, not merely its frame
    # interval -- a frame that has just arrived still describes where the robot
    # was pointing when it was captured. Measured on this link at 0.38 s (with
    # the publisher at 5 FPS; it was 2.10 s at 30 FPS, which no plausible settle
    # could have covered -- see scripts/robot_start.sh).
    #
    # Set too short, the search inspects the view from the PREVIOUS step and
    # rotates straight past a target that is plainly visible on screen. That
    # reads as "the detector can't see it" and is really "it isn't looking yet".
    SETTLE_AFTER_STEP = 0.60

    # Centring. The deadband is a fraction of half-width, so it is an angle
    # regardless of resolution; 6% of half a frame is a couple of degrees.
    CENTER_DEADBAND = 0.06
    CENTER_GAIN = 1.2            # rad/s per unit of normalised pixel error

    # Ceiling on how fast centring may turn -- set by camera LATENCY, not by
    # what the motors can do.
    #
    # Centring is a closed loop on a feed that is ~0.38 s behind reality, so the
    # robot keeps turning blind for that long after the target reaches centre.
    # Uncapped (the executor's 2.84 rad/s) that blind arc is over 60 degrees,
    # against a frame barely 44 degrees wide -- the target is thrown clean out of
    # view and the loop reports "lost" while staring straight at it. Observed
    # exactly that on "approach the bag": found at 100 degrees swept, then lost
    # during centring.
    #
    # At 0.35 rad/s (20 deg/s) the blind arc is under 8 degrees, comfortably
    # inside the frame. Correcting a target at the edge takes about a second,
    # which is the right trade: slow and converging beats fast and losing it.
    CENTER_MAX_ANG = 0.35

    CENTER_TIMEOUT = 12.0

    # Grace periods are in seconds but what they really buy is FRAMES, so they
    # had to grow when the camera went from ~7 FPS to 5 to fix the latency: the
    # old 1.0 s was 7 frames and became 5. Marginal targets drop out for runs of
    # several frames -- a bag scoring 0.12 was measured missing from 5 of 12
    # consecutive stationary frames -- so a grace of a handful of frames trips
    # on a target that is sitting still in plain view.
    CENTER_LOST_GRACE = 2.5      # s without a detection before giving up centring

    APPROACH_GAIN = 0.5          # rad/s per unit error, while driving

    # Bearing error past which the approach stops and re-aims in place instead
    # of trying to steer and drive at once.
    #
    # Simultaneous drive-and-steer is the obvious design and it does not work on
    # this robot. Closing on a 7 cm bottle, the lidar sector that measures range
    # is only +/-10 degrees wide, and any steering swings it off the target --
    # so the range vanishes exactly when the robot is moving. Measured: bearing
    # error swinging 0.09 -> 0.59 -> 0.09 while the range dropped out entirely
    # and the approach aborted after 10 cm of travel.
    #
    # Stopping to re-aim keeps the sector pointed at the target the whole time
    # the robot is moving, and costs a fraction of a second. It is the same
    # step-and-look reasoning the search uses, for the same reason: this robot
    # measures well when still and badly when turning.
    RECENTER_ERR = 0.15
    # Hard cap on steering while moving. Centring may turn at full speed because
    # it turns in place; an approach may not. At 0.11 m/s a 0.8 rad/s correction
    # is a turn radius of 14 cm -- the robot pirouettes, the target sweeps out of
    # a narrow frame, and the error signal that caused it inverts. Observed
    # directly: approach error oscillating between -0.95 and +0.89.
    APPROACH_MAX_ANG = 0.4

    # Largest believable INCREASE in range between two readings. Closing on a
    # stationary target, range only shrinks; scans arrive ~217 ms apart and the
    # robot covers ~2.4 cm in that time, so a reading that jumps outward by more
    # than this is not the target -- it is the sector having swung onto the
    # background. Measured: readings alternating 0.65 m / 3.30 m during a single
    # approach, which made the robot drive at cruise speed while it was actually
    # 30 cm from the bottle, and overshoot the stop by 9 cm.
    #
    # Only growth is filtered. A sudden DROP is either the target or something
    # that just moved into the path, and both of those must still stop us.
    RANGE_JUMP = 0.25

    # How long outward-jumping readings may be rejected before the filter
    # concedes that they, not its baseline, are the truth.
    #
    # Without this the filter LATCHES and can never recover. Rejecting a
    # reading leaves the baseline untouched, so one spurious CLOSE reading
    # poisons it permanently: a cat crossing the path dropped the range from
    # 1.51 m to 0.62 m for a moment (accepted -- drops always are, since
    # something entering the path must still stop us), and once it left, every
    # true reading of ~1.2 m failed the 0.62 + 0.25 test forever. The robot
    # then aborted with "lost range to target" while the raw lidar was reading
    # the target perfectly, which is exactly as confusing as it sounds.
    #
    # 0.3 s is about 1.5 scans at 5 Hz: longer than a single-scan spike, so
    # one-off noise still gets rejected, but it re-syncs 0.4 s into the streak
    # against a 0.6 s RANGE_GRACE -- replayed against the measured failure,
    # that recovers with 0.2 s to spare instead of aborting. It must stay
    # comfortably under RANGE_GRACE or the latch simply returns.
    #
    # Re-baselining is safe: it does not weaken the stop test, which is
    # evaluated against each fresh reading regardless of the baseline.
    RANGE_RESYNC = 0.3
    LOST_GRACE = 2.5             # s without a detection before calling it lost
                                 # (see CENTER_LOST_GRACE -- same frame-rate maths)
    RANGE_GRACE = 0.6            # s of unreadable lidar before refusing to drive

    # Abort if the lidar goes quiet. NOT the 0.5 s used for odometry: the
    # LDS-01 publishes at 4.6 Hz, not 20 Hz, so one scan period is 217 ms.
    # Measured over 25 s on this Wi-Fi link, scan age ran p50 105 ms, p99
    # 366 ms, max 402 ms -- 0.5 s would have left barely 100 ms of headroom on
    # a link already dropping 10% of packets, and tripped on the first
    # double-drop. 0.8 s tolerates three missed scans, and costs at most 9 cm
    # of travel at the approach speed before the abort lands.
    SCAN_STALE = 0.8

    # How long the lidar may stay stale, with the robot halted, before the seek
    # gives up rather than keep waiting.
    SCAN_WAIT = 3.0
    RANGE_SLOWDOWN = 0.6         # m     start easing off this far from the goal

    # Once this close, finish on the lidar alone. The target legitimately
    # leaves a narrow-FOV frame as we close on it -- it grows past the edges,
    # or drops below the camera's tilt -- and treating that as "lost" would
    # abort the approach in the last few centimetres, every time. Safe because
    # by this point we are centred and the stopping test was never visual.
    # Raised from 0.30 after watching an approach fail from 0.70 m -- just
    # outside the old commit band -- when the target was already centred and
    # nothing but a straight 0.35 m drive remained. Inside this band vision has
    # nothing left to contribute: the robot is aimed, the object is not moving,
    # and the stop was always going to be the lidar's call. Continuing to steer
    # on a target that now fills the frame only invites the box-splitting above.
    LIDAR_COMMIT = 0.50          # m beyond stop_distance

    # How far a box may move between frames and still count as the same object,
    # as a fraction of frame width.
    #
    # Tight, and it has to be. A generous gate does not merely admit the wrong
    # object once -- it lets the lock WALK, one accepted step at a time, because
    # each new position becomes the reference for the next. Measured at 0.25:
    # the bearing error drifted -0.01 -> 0.29 -> 0.60 across 9 mm of travel,
    # each hop comfortably inside the gate. Close up, a bottle is detected as
    # several boxes on different parts of it, so those hops are always available.
    TRACK_GATE = 0.12

    def __init__(self, node, detector, executor, target, cfg):
        self.node = node
        self.detector = detector
        self.executor = executor
        self.target = target
        self.cfg = cfg
        self.detections = 0
        self.last_conf = None
        self._track_cx = None     # where the object we locked onto last was

    # --- perception ------------------------------------------------------- #
    def _look(self, seen_seq):
        """Detect on the newest frame. (box|None, seq, is_new).

        Returns is_new=False when the camera has not produced a frame since
        `seen_seq`, so the caller can keep its previous belief instead of
        paying for inference on a frame it has already judged. The feed is
        ~7 Hz over Wi-Fi and this loop ticks at 20 Hz.
        """
        frame, seq = self.node.get_frame()
        if seq == seen_seq:
            return None, seq, False
        box = self._pick(self.detector.detect_all(frame, self.target))
        self.detections += 1
        if box is not None:
            self.last_conf = round(box.conf, 3)
            self._track_cx = box.cx
        return box, seq, True

    def _pick(self, boxes):
        """Choose which instance to follow when the room holds several.

        Highest confidence is the right answer only for the FIRST look. After
        that it is actively wrong: with two chairs in view their scores sit
        within noise of each other, so the "best" box hops between them from
        frame to frame. Observed directly -- a stationary robot watching two
        chairs saw its centring error jump from 0.67 to 0.018 without anything
        moving, because the pick had swapped objects. Centre on one and drive at
        the other and the robot arrives somewhere nobody asked for.

        So once we have locked on, prefer the candidate nearest to where the
        locked one just was, and only fall back to confidence when nothing is
        near enough to be plausibly the same object.
        """
        if not boxes:
            return None
        if self._track_cx is None:
            return boxes[0]           # detect_all sorts by confidence
        gate = self.TRACK_GATE * self.node.frame_width()
        near = [b for b in boxes if abs(b.cx - self._track_cx) <= gate]
        if not near:
            return boxes[0]
        return min(near, key=lambda b: abs(b.cx - self._track_cx))

    def _confirm(self, cancel, deadline):
        """Require the target in k of n frames before committing to it.

        Open-vocabulary detection is looser than a closed-set head -- it will
        happily put a low box on "bottle" in a frame with no bottle -- and the
        consequence of believing one frame is a robot that turns and drives at
        furniture. Costs a fraction of a second at each search step.
        """
        # Start each round unlocked. A lock left over from the previous search
        # step points at wherever that object was BEFORE the robot turned, and
        # would bias this round's pick towards a stale position.
        self._track_cx = None
        hits, seen, seq = 0, 0, -1
        latest = None
        while seen < self.cfg.confirm_n:
            if cancel.is_set() or time.monotonic() > deadline:
                return None
            box, seq, is_new = self._look(seq)
            if not is_new:
                cancel.wait(self.TICK)
                continue
            seen += 1
            if box is not None:
                hits += 1
                # The most recent sighting, not the most confident one: _pick
                # has already kept these to a single object, so the freshest is
                # simply the most accurate statement of where it is now, and
                # that position is what centring starts from.
                latest = box
            # Can't reach k any more -- stop early rather than watch two more
            # frames we already know cannot change the answer.
            if hits + (self.cfg.confirm_n - seen) < self.cfg.confirm_k:
                return None
        return latest if hits >= self.cfg.confirm_k else None

    def _err(self, box, width):
        """Normalised horizontal error, -1 (hard left) .. +1 (hard right).

        Image x is world-horizontal here: the publisher's 90 degree rotation
        corrects a rotated mount, so the frame arrives upright (just portrait,
        480x640). If that rotation is ever turned off, this is the line that
        silently starts steering by the wrong axis.
        """
        return (box.cx - width / 2.0) / (width / 2.0)

    # --- states ----------------------------------------------------------- #
    def _search(self, state, cancel, deadline):
        """Step, settle, look. Returns (box|None, reason)."""
        swept = 0.0
        while swept <= self.cfg.search_max_deg:
            if cancel.is_set():
                return None, "cancelled"
            if time.monotonic() > deadline:
                return None, "timed out"
            self.executor.update_active(
                state, seek_state="search", swept_deg=round(swept),
                target=self.target)

            box = self._confirm(cancel, deadline)
            if box is not None:
                return box, "done"

            # Rotating left by convention: with no information about where the
            # target is, direction is arbitrary, and a fixed one keeps the
            # behaviour reproducible when something goes wrong.
            reason = self.executor.rotate_relative(
                state, self.cfg.search_step_deg, "rotate_left", "slow")
            if reason != "done":
                return None, ("cancelled" if reason == "cancelled" else reason)
            swept += self.cfg.search_step_deg
            self.node.stop()
            # The shared closed-loop turn reports its progress in DEGREES, and
            # this readout is in metres-to-target. Left alone it surfaces as
            # "progress: 23.3 m" on a seek that has not started driving yet.
            self.executor.set_progress(state, 0.0)
            if cancel.wait(self.SETTLE_AFTER_STEP):
                return None, "cancelled"
        return None, "not found"

    def _center(self, state, cancel, deadline):
        """Rotate until the target sits on the frame's centre line."""
        t_seen = time.monotonic()
        seq, err = -1, None
        t_end = min(deadline, time.monotonic() + self.CENTER_TIMEOUT)
        while True:
            if cancel.is_set():
                return "cancelled"
            now = time.monotonic()
            if now > t_end:
                return "timed out" if now > deadline else "could not centre"

            box, seq, is_new = self._look(seq)
            if is_new:
                if box is not None:
                    err = self._err(box, self.node.frame_width())
                    t_seen = now
                elif now - t_seen > self.CENTER_LOST_GRACE:
                    return "lost"

            if err is None:
                self.node.set_velocity(0.0, 0.0)
                cancel.wait(self.TICK)
                continue

            self.executor.update_active(
                state, seek_state="center", err=round(err, 3),
                conf=self.last_conf, target=self.target)

            if abs(err) <= self.CENTER_DEADBAND:
                self.node.stop()
                return "done"

            # Positive error means the target is right of centre, and turning
            # right is a negative angular velocity (REP-103: yaw is CCW).
            mag = min(abs(err) * self.CENTER_GAIN, self.CENTER_MAX_ANG)
            mag = max(mag, self.executor.MIN_ANG)
            self.node.set_velocity(0.0, -math.copysign(mag, err))
            cancel.wait(self.TICK)

    def _approach(self, state, cancel, deadline):
        """Drive at the target, steering to keep it centred, stop on lidar."""
        start = self.node.get_odom()
        if start is None:
            return "no odometry"
        x0, y0 = start[0], start[1]

        rng, _ = self.node.front_range()
        if rng is None:
            # Refusing rather than creeping forward blind. The lidar is the
            # only thing that knows how far away the target is, and "I cannot
            # see anything ahead" is not the same claim as "the way is clear".
            return "no range to target"
        d0 = rng
        self.executor.set_goal(state, max(d0 - self.cfg.stop_distance, 0.01))

        t_seen = time.monotonic()
        t_ranged = t_seen
        seq, err, last_rng = -1, 0.0, rng
        t_scan_ok = None          # when the lidar first went quiet, if it has
        t_reject = None           # when the current jump-rejection streak began
        while True:
            if cancel.is_set():
                return "cancelled"
            now = time.monotonic()
            if now > deadline:
                return "timed out"

            od = self.node.get_odom()
            if od is None or od[3] > self.executor.ODOM_STALE:
                return "odometry stalled"
            if math.hypot(od[0] - x0, od[1] - y0) > self.cfg.max_travel:
                return "went too far without arriving"

            # A stale lidar stops the robot but does not, on its own, end the
            # command. Over this Wi-Fi link (~10% packet loss, bursty) a gap of
            # about a second happens, and one killed an otherwise clean approach
            # 4 cm from its goal. Detector load was ruled out as the cause by
            # measurement: scan age under full inference load was p99 388 ms
            # against 346 ms idle, so this is the network, not the laptop.
            #
            # Standing still costs nothing and is not a safety compromise -- the
            # robot is not moving on data it doesn't have. Only a gap long
            # enough to mean the lidar is really gone ends the seek.
            age, _, _ = self.node.scan_stats()
            if age is None or age > self.SCAN_STALE:
                self.node.set_velocity(0.0, 0.0)
                if t_scan_ok is None:
                    t_scan_ok = now
                elif now - t_scan_ok > self.SCAN_WAIT:
                    return "lidar stalled"
                # No range is read on this path, so the RANGE_GRACE clock must
                # not run through it either -- same reasoning as the aiming
                # branch below. A stalled lidar is already handled, above, by
                # its own SCAN_WAIT budget; letting it also burn down the range
                # grace would abort on the first dropout after recovery.
                t_ranged = now
                cancel.wait(self.TICK)
                continue
            t_scan_ok = None

            # `close` off the last good range, because this tick's has not been
            # taken yet -- and once inside the commit band the target is not
            # re-aimed at anyway, so a tick of staleness cannot change the plan.
            close = last_rng is not None and \
                last_rng <= self.cfg.stop_distance + self.LIDAR_COMMIT

            box, seq, is_new = self._look(seq)
            if is_new:
                if box is not None:
                    err = self._err(box, self.node.frame_width())
                    t_seen = now
                elif not close and now - t_seen > self.LOST_GRACE:
                    return "lost"

            # Aim, then move -- never both at once. See RECENTER_ERR.
            #
            # Critically, this returns BEFORE the range is read. While the robot
            # is turning, the forward sector sweeps across the whole room, and
            # every reading it takes describes something that merely happens to
            # be passing in front of it. Evaluating the stop test on those is
            # how an approach "arrives" at 0.26 m while actually standing 0.69 m
            # from the target -- observed exactly that. Range is only meaningful
            # when the robot is pointed at what it is measuring.
            if not close and abs(err) > self.RECENTER_ERR:
                self.executor.update_active(
                    state, seek_state="aiming", err=round(err, 3),
                    conf=self.last_conf, target=self.target)
                mag = min(abs(err) * self.CENTER_GAIN, self.CENTER_MAX_ANG)
                self.node.set_velocity(0.0, -math.copysign(
                    max(mag, self.executor.MIN_ANG), err))
                # RANGE_GRACE measures how long we have been UNABLE to read a
                # range -- not how long since we last chose to look. Aiming
                # deliberately takes no reading (see above), so without this the
                # clock runs down through the whole turn and the very first
                # dropout after it aborts with no grace left at all. That
                # surfaced as "lost range to target" mid-approach, 1.28 m from a
                # target in plain view, on a robot that had been driving fine.
                #
                # Resetting is safe precisely because this branch turns in
                # place: nothing is moving forward on data we have not read.
                t_ranged = now
                cancel.wait(self.TICK)
                continue

            rng, _ = self.node.front_range()
            # Discard a reading that jumped outward: it describes something
            # other than what we are driving at, and acting on it means driving
            # fast at a target that is actually close. Treated as "unreadable"
            # rather than substituted, so a genuinely lost target still runs
            # down RANGE_GRACE and stops us instead of coasting on a stale value.
            if rng is not None and last_rng is not None and \
                    rng > last_rng + self.RANGE_JUMP:
                if t_reject is None:
                    t_reject = now
                if now - t_reject > self.RANGE_RESYNC:
                    # Persistently disagreeing with the baseline means the
                    # baseline is wrong, not the sensor. Re-sync rather than
                    # reject forever -- see RANGE_RESYNC.
                    t_reject = None
                else:
                    rng = None
            else:
                t_reject = None
            if rng is not None:
                last_rng = rng
                t_ranged = now
                if rng <= self.cfg.stop_distance:
                    self.node.stop()
                    self.executor.update_active(
                        state, seek_state="arrived", range_m=round(rng, 3))
                    return "done"
            elif now - t_ranged > self.RANGE_GRACE:
                return "lost range to target"

            self.executor.update_active(
                state, seek_state="approach",
                range_m=None if rng is None else round(rng, 3),
                err=round(err, 3), conf=self.last_conf, target=self.target)
            # Report against the last GOOD range, not this tick's: on a tick
            # whose reading was rejected, `rng` is None and d0-d0 would collapse
            # the bar to zero mid-approach.
            self.executor.set_progress(state, max(0.0, d0 - (last_rng or d0)))

            # Ease off near the goal for the same reason the distance goals do:
            # decided at cruise speed, the stop lands late by the control lag.
            if rng is None:
                lin = self.executor.MIN_LIN
            else:
                left = max(0.0, rng - self.cfg.stop_distance)
                lin = self.executor.approach_cruise * \
                    min(1.0, left / self.RANGE_SLOWDOWN)
                lin = max(lin, self.executor.MIN_LIN)
            # Residual trim only: anything larger was handled above by aiming.
            ang = 0.0 if close else -err * self.APPROACH_GAIN
            ang = max(-self.APPROACH_MAX_ANG, min(self.APPROACH_MAX_ANG, ang))
            self.node.set_velocity(lin, ang)
            cancel.wait(self.TICK)

    # --- entry point ------------------------------------------------------ #
    def run(self, state, cancel):
        """Returns the reason the behaviour ended, in the executor's vocabulary."""
        deadline = time.monotonic() + self.cfg.total_timeout

        # Refuse up front if the lidar isn't there. Consistent with how a
        # distance goal refuses without /odom: the alternative would be an
        # open-loop approach, which is a robot driving at something it cannot
        # measure.
        age, _, _ = self.node.scan_stats()
        if age is None:
            return "no lidar"
        if age > self.SCAN_STALE:
            return "lidar stalled"

        box, reason = self._search(state, cancel, deadline)
        if box is None:
            return reason

        reason = self._center(state, cancel, deadline)
        if reason != "done":
            return reason

        return self._approach(state, cancel, deadline)
