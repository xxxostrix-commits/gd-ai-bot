#!/usr/bin/env python3
"""
ai-brain/bot.py -- Geometry Dash 2.2074 AI brain (Geode bridge side).

The C++ mod streams the player state plus the filtered hitboxes of the scan
window (-40 .. +520 units around the player) over UDP. This process runs a
receding-horizon beam search on a 240 Hz physics simulation and answers with:

    {"hold": bool,
     "path": [[x,y],...],                 # chosen future trajectory
     "alt_paths": [[[x,y],...],...],      # rejected / dangerous branches
     "corridor": [x, yTop, yBottom, ...], # safe corridor (flat triples)
     "risk": [0..100, ...],               # per path point (neon colour)
     "marks": [x, y, kind, ...],          # predicted press(0)/release(1)
     "hud": "...", "corridor_height": float}

When the visual preset is "off" the answer is reduced to {"hold": ...}.

Anti-spam design (no 100 CPS clickbot):
  * branching only every `macro_step_ticks`, physics still exact per tick,
  * `click_penalty` score cost for every hold change,
  * `min_switch_ticks` minimum hold/release duration (emergency override),
  * `max_cps` sliding-window limiter,
  * `hysteresis_bonus` for the still-valid previous plan,
  * `safe_corridor_deadzone` removes the centering "sawtooth" reward,
  * adaptive click penalty inside corridors thinner than
    `adaptive_tight_threshold`.

Usage:  python ai-brain/bot.py [--host 127.0.0.1] [--port 12345] [--verbose]
Test :  python ai-brain/bot.py --selftest
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import select
import socket
import sys
import time
from collections import deque

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

# ---- wire type codes (must match src/main.cpp) ------------------------------
HAZARD = 1
SOLID = 2
SLOPE = 3
DEFAULT_CONFIG = {
    "network": {"host": "127.0.0.1", "port": 12345, "socket_buffer": 16384,
                "plan_interval_ms": 20.0},
    "visuals": {"show_alt_paths": True, "show_corridor": True, "trajectory_step": 4,
                "corridor_step_ticks": 8, "send_risk": True, "max_alt_paths": 2},
    "profiles": {
        "active": "smooth",
        "smooth": {"max_cps": 10, "min_switch_ticks": 14, "click_penalty": 18.0,
                   "hysteresis_bonus": 25.0, "safe_corridor_deadzone": 22.0,
                   "macro_step_ticks": 8, "vertical_penalty": 0.08,
                   "smoothness_weight": 0.4},
        "tryhard": {"max_cps": 22, "min_switch_ticks": 5, "click_penalty": 4.0,
                    "hysteresis_bonus": 8.0, "safe_corridor_deadzone": 8.0,
                    "macro_step_ticks": 4, "vertical_penalty": 0.02,
                    "smoothness_weight": 0.15},
    },
    "solver": {"lookahead_ticks": 96, "beam_width": 14,
               "adaptive_tight_threshold": 55.0, "emergency_horizon_ticks": 18,
               "time_budget_ms": 6.0, "clearance_alert": 45.0, "risk_scale": 60.0,
               "death_penalty": 1000.0, "progress_weight": 0.05, "hazard_margin": 4.0},
    "physics": {
        "tick_rate": 240, "base_speed": 311.58,
        "speed_multipliers": {"0.5": 0.5, "1": 1.0, "2": 2.0, "3": 3.0, "4": 4.0},
        "gravity": 3153.6, "jump_velocity": 670.8, "max_fall_speed": 1500.0,
        "ufo_jump_velocity": 620.0, "ball_gravity": 3153.6,
        "ball_flip_velocity": 480.0, "robot_gravity": 3153.6,
        "robot_jump_min": 380.0, "robot_jump_max": 700.0, "robot_charge_rate": 1500.0,
        "spider_gravity": 3153.6, "spider_teleport_range": 260.0,
        "ship_gravity": 1150.0, "ship_thrust": 1700.0, "ship_max_vy": 480.0,
        "wave_angle_deg": 45.0, "player_hitbox": 30.0, "hazard_hitbox": 22.0,
        "ground_y": 0.0, "out_of_bounds": 2600.0,
    },
}


def num(value, default=0.0):
    """Safe float conversion (None / NaN / garbage -> default)."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result) or math.isinf(result):
        return default
    return result


def as_int(value, default):
    """Safe int conversion."""
    try:
        return int(num(value, float(default)))
    except (TypeError, ValueError, OverflowError):
        return default


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def deep_merge(base, override):
    """Recursively merge `override` over a copy of `base`."""
    if isinstance(base, dict) and isinstance(override, dict):
        merged = dict(base)
        for key, value in override.items():
            merged[key] = deep_merge(base[key], value) if key in base else value
        return merged
    return override


def load_config(path):
    with open(path, "r", encoding="utf-8") as handle:
        raw = json.load(handle)
    if not isinstance(raw, dict):
        raise ValueError("config.json must contain a JSON object")
    return deep_merge(DEFAULT_CONFIG, raw)


class ConfigStore:
    """Loads config.json and re-reads it at most once per second (hot reload)."""

    def __init__(self, path):
        self.path = path
        self.data = load_config(path) if os.path.exists(path) else deep_merge(DEFAULT_CONFIG, {})
        self.mtime = self._mtime()
        self.last_check = 0.0
        self.error = None

    def _mtime(self):
        try:
            return os.path.getmtime(self.path)
        except OSError:
            return 0.0

    def reload_if_changed(self, now):
        if now - self.last_check < 1.0:
            return False
        self.last_check = now
        mtime = self._mtime()
        if mtime == self.mtime:
            return False
        self.mtime = mtime
        try:
            self.data = load_config(self.path)
            self.error = None
            return True
        except Exception as exc:  # keep the last good config on syntax errors
            self.error = str(exc)
            return False
ORB = 4
PAD = 5
GRAVITY_FLIP = 6
GRAVITY_NORMAL = 7
GRAVITY_TOGGLE = 8
SPEED_PORTALS = {10: 0.5, 11: 1.0, 12: 2.0, 13: 3.0, 14: 4.0}
MODE_PORTALS = {20: "cube", 21: "ship", 22: "ball", 23: "ufo",
                24: "wave", 25: "robot", 26: "spider", 27: "swing"}
BLOCKING = (HAZARD, SOLID, SLOPE)
SOLID_LIKE = (SOLID, SLOPE)
GROUND_MODES = ("cube", "ball", "robot", "spider", "swing")
FLYING_MODES = ("ship", "ufo", "wave", "swing")
# ============================================================================
#  Level model: hitboxes sorted by x
# ============================================================================

class Level:
    """Immutable view of the scanned objects, sorted by x."""

    def __init__(self, objects):
        cleaned = []
        for obj in objects or ():
            try:
                x, y, w, h, kind = (float(obj[0]), float(obj[1]),
                                    float(obj[2]), float(obj[3]), as_int(obj[4], 0))
            except (TypeError, ValueError, IndexError, KeyError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)
                    and math.isfinite(w) and math.isfinite(h)):
                continue
            if w <= 0.0 or h <= 0.0:
                continue
            cleaned.append((x, y, w, h, kind))
        cleaned.sort(key=lambda o: o[0])
        self.objects = tuple(cleaned)
        self.xs = [o[0] for o in cleaned]
        # Nothing is streamed past the scan window, so anything beyond the last
        # streamed object is unknown space (still floored at y = 0 like GD).
        self.scan_x = max((o[0] + o[2] for o in cleaned), default=0.0)

    def __bool__(self):
        return bool(self.objects)

    def near(self, x_min, x_max, margin=0.0):
        """Objects whose x span intersects [x_min - margin, x_max + margin]."""
        if not self.objects:
            return ()
        lo = bisect.bisect_left(self.xs, x_min - margin - 300.0)
        lo = max(0, lo)
        result = []
        for obj in self.objects[lo:]:
            if obj[0] > x_max + margin:
                break
            if obj[0] + obj[2] >= x_min - margin:
                result.append(obj)
        return result

    def corridor_at(self, x, y, ground_y=0.0, ceiling=1200.0):
        """Free vertical band around x -> (bottom, top, width, tight)."""
        bottom = ground_y
        top = max(ceiling, y + 400.0)
        for obj in self.near(x - 1.0, x + 1.0, margin=1.0):
            if obj[4] not in BLOCKING:
                continue
            oy0 = obj[1]
            oy1 = oy0 + obj[3]
            if oy1 <= y:
                bottom = max(bottom, oy1)
            elif oy0 >= y:
                top = min(top, oy0)
        if top < bottom:
            top = bottom
        width = top - bottom
        return bottom, top, width, width < 90.0

    def nearest_vertical(self, x, y, reach=200.0):
        """Distance to the closest blocking object above and below."""
        above = reach
        below = reach
        for obj in self.near(x - 20.0, x + 20.0):
            if obj[4] not in BLOCKING:
                continue
            oy0 = obj[1]
            oy1 = oy0 + obj[3]
            if oy1 <= y:
                below = min(below, y - oy1)
            elif oy0 >= y:
                above = min(above, oy0 - y)
            else:
                return 0.0, 0.0
        return above, below
# ============================================================================
#  Physics (every constant is configurable in config.json -> "physics")
# ============================================================================

class Physics:
    """Per-second constants; the simulation integrates them at 1/tick_rate."""

    def __init__(self, cfg):
        p = cfg.get("physics") or {}
        self.tick_rate = max(30.0, num(p.get("tick_rate"), 240.0))
        self.dt = 1.0 / self.tick_rate
        self.base_speed = max(1.0, num(p.get("base_speed"), 311.58))

        self.gravity = num(p.get("gravity"), 3153.6)
        self.jump = num(p.get("jump_velocity"), 670.8)
        self.max_fall = num(p.get("max_fall_speed"), 1500.0)
        self.ufo_jump = num(p.get("ufo_jump_velocity"), 620.0)

        self.ball_gravity = num(p.get("ball_gravity"), self.gravity)
        self.ball_flip = num(p.get("ball_flip_velocity"), 480.0)

        self.robot_gravity = num(p.get("robot_gravity"), self.gravity)
        self.robot_min = num(p.get("robot_jump_min"), 380.0)
        self.robot_max = num(p.get("robot_jump_max"), 700.0)
        self.robot_charge = num(p.get("robot_charge_rate"), 1500.0)

        self.spider_gravity = num(p.get("spider_gravity"), self.gravity)
        self.spider_range = num(p.get("spider_teleport_range"), 260.0)

        self.ship_gravity = num(p.get("ship_gravity"), 1150.0)
        self.ship_thrust = num(p.get("ship_thrust"), 1700.0)
        self.ship_max_vy = num(p.get("ship_max_vy"), 480.0)

        self.wave_slope = math.tan(math.radians(num(p.get("wave_angle_deg"), 45.0)))

        self.player_hitbox = max(4.0, num(p.get("player_hitbox"), 30.0))
        self.hazard_hitbox = max(2.0, num(p.get("hazard_hitbox"), 22.0))
        self.ground_y = num(p.get("ground_y"), 0.0)
        self.out_of_bounds = max(400.0, num(p.get("out_of_bounds"), 2600.0))

        raw = p.get("speed_multipliers") or {}
        self.speed_multipliers = dict(SPEED_PORTALS)
        try:
            for tier, key in zip((10, 11, 12, 13, 14), sorted(raw, key=float)):
                self.speed_multipliers[tier] = max(0.05, float(raw[key]))
        except (TypeError, ValueError):
            pass


class SimState:
    """Mutable snapshot of one branch inside the search."""

    __slots__ = ("x", "y", "vy", "vx", "upside", "ground", "mode", "hold", "charge")

    def __init__(self, x, y, vy=0.0, vx=311.58, upside=False, ground=False,
                 mode="cube", hold=False):
        self.x = x
        self.y = y
        self.vy = vy
        self.vx = vx
        self.upside = upside
        self.ground = ground
        self.mode = mode
        self.hold = hold
        self.charge = 0.0

    def clone(self):
        other = SimState.__new__(SimState)
        other.x = self.x
        other.y = self.y
        other.vy = self.vy
        other.vx = self.vx
        other.upside = self.upside
        other.ground = self.ground
        other.mode = self.mode
        other.hold = self.hold
        other.charge = self.charge
        return other
class Simulator:
    """Fixed-step (1/240 s) forward simulation used by the beam search."""

    def __init__(self, phys, level):
        self.phys = phys
        self.level = level
        self.half = phys.player_hitbox * 0.5
        self.hazard_half = phys.hazard_hitbox * 0.5
        self.tick_objs = ()

    def prepare(self, x0, vx, ticks):
        """Pre-computes the candidate object list for every future tick."""
        step = vx * self.phys.dt
        x = x0
        window = []
        for _ in range(ticks + 2):
            x += step
            window.append(self.level.near(x - 55.0, x + 55.0))
        self.tick_objs = tuple(window)
        return self.tick_objs

    def objects_at(self, tick):
        if not self.tick_objs:
            return ()
        if tick < 0:
            return self.tick_objs[0]
        if tick >= len(self.tick_objs):
            return self.tick_objs[-1]
        return self.tick_objs[tick]

    @staticmethod
    def overlap(x, y, half, obj):
        return (x + half > obj[0] and x - half < obj[0] + obj[2]
                and y + half > obj[1] and y - half < obj[1] + obj[3])

    def clearance(self, x, y, objects=None):
        """Distance from the player box to the nearest blocking hitbox."""
        best = 220.0
        half = self.half
        if objects is None:
            objects = self.level.near(x - 30.0, x + 30.0)
        for obj in objects:
            if obj[4] not in BLOCKING:
                continue
            oy0 = obj[1]
            oy1 = oy0 + obj[3]
            if oy0 >= y + half:
                best = min(best, oy0 - (y + half))
            elif oy1 <= y - half:
                best = min(best, (y - half) - oy1)
            else:
                return 0.0
        return max(0.0, best)

    def cap_fall(self, st, up):
        cap = self.phys.max_fall
        if up > 0.0:
            if st.vy < -cap:
                st.vy = -cap
        elif st.vy > cap:
            st.vy = cap
    def step(self, st, hold, tick):
        """Advances one tick; returns False if this branch died."""
        phys = self.phys
        dt = phys.dt
        prev_hold = st.hold
        st.hold = hold

        st.x += st.vx * dt
        up = -1.0 if st.upside else 1.0
        grounded = st.ground
        st.ground = False
        mode = st.mode

        if mode == "wave":
            st.vy = st.vx * phys.wave_slope * (1.0 if hold else -1.0) * up
            st.y += st.vy * dt
        elif mode == "ship":
            st.vy += (phys.ship_thrust if hold else -phys.ship_gravity) * up * dt
            st.vy = clamp(st.vy, -phys.ship_max_vy, phys.ship_max_vy)
            st.y += st.vy * dt
        elif mode == "ufo":
            if hold and not prev_hold:
                st.vy = phys.ufo_jump * up
            st.vy -= phys.gravity * up * dt
            self.cap_fall(st, up)
            st.y += st.vy * dt
        elif mode == "ball":
            if hold and not prev_hold:
                st.upside = not st.upside
                up = -1.0 if st.upside else 1.0
                st.vy = phys.ball_flip * up
            st.vy -= phys.ball_gravity * up * dt
            self.cap_fall(st, up)
            st.y += st.vy * dt
        elif mode == "robot":
            if hold:
                st.charge = min(phys.robot_max, st.charge + phys.robot_charge * dt)
            else:
                if prev_hold and st.charge > 0.0:
                    st.vy = max(phys.robot_min, st.charge) * up
                st.charge = 0.0
            st.vy -= phys.robot_gravity * up * dt
            self.cap_fall(st, up)
            st.y += st.vy * dt
        elif mode == "spider":
            if hold and not prev_hold:
                self.spider_jump(st, up)
            st.vy -= phys.spider_gravity * up * dt
            self.cap_fall(st, up)
            st.y += st.vy * dt
        else:  # cube / swing
            if hold and grounded:
                st.vy = phys.jump * up
            st.vy -= phys.gravity * up * dt
            self.cap_fall(st, up)
            st.y += st.vy * dt

        # Level floor. With inverted gravity the player lands on ceilings that
        # arrive through the level data, so the floor is only applied here.
        if not st.upside and st.y - self.half <= phys.ground_y:
            st.y = phys.ground_y + self.half
            if st.vy < 0.0:
                st.vy = 0.0
            st.ground = True

        for obj in self.objects_at(tick):
            code = obj[4]
            if code == HAZARD:
                if self.overlap(st.x, st.y, self.hazard_half, obj):
                    return False
            elif code == SOLID or code == SLOPE:
                if self.overlap(st.x, st.y, self.half, obj):
                    if not self.resolve_solid(st, obj):
                        return False
            elif code == ORB:
                if hold and self.overlap(st.x, st.y, self.hazard_half + 6.0, obj):
                    st.vy = phys.jump * up
                    st.ground = False
            elif code == PAD:
                if self.overlap(st.x, st.y, self.hazard_half + 6.0, obj):
                    st.vy = phys.jump * 1.05 * up
                    st.ground = False
            elif code in (GRAVITY_FLIP, GRAVITY_NORMAL, GRAVITY_TOGGLE):
                if self.overlap(st.x, st.y, self.hazard_half, obj):
                    st.upside = (True if code == GRAVITY_FLIP else
                                 (False if code == GRAVITY_NORMAL else not st.upside))
                    st.ground = False
            elif code in SPEED_PORTALS:
                if self.overlap(st.x, st.y, self.hazard_half, obj):
                    st.vx = phys.base_speed * phys.speed_multipliers.get(
                        code, SPEED_PORTALS[code])
            elif code in MODE_PORTALS:
                if self.overlap(st.x, st.y, self.hazard_half, obj):
                    new_mode = MODE_PORTALS[code]
                    if new_mode != st.mode:
                        st.mode = new_mode
                        st.vy = 0.0
                        st.charge = 0.0
                        st.ground = False

        return abs(st.y) <= phys.out_of_bounds
    def resolve_solid(self, st, obj):
        """Landing is allowed, side impacts kill (GD behaviour)."""
        half = self.half
        ox0, oy0, ow, oh = obj[0], obj[1], obj[2], obj[3]
        ox1 = ox0 + ow
        oy1 = oy0 + oh

        # Distance needed to leave the block through each face. The smallest
        # one is the minimum translation vector; only shallow exits are treated
        # as landings, anything deeper is a lethal side/inside hit.
        exit_up = oy1 - (st.y - half)
        exit_down = (st.y + half) - oy0
        exit_left = (st.x + half) - ox0
        exit_right = ox1 - (st.x - half)

        vertical = exit_up <= exit_left and exit_up <= exit_right
        if vertical and exit_up <= exit_down:
            if obj[4] == SLOPE or (st.vy <= 0.0 and exit_up <= half):
                st.y = oy1 + half
                st.vy = 0.0
                st.ground = True
                return True
            return False
        if exit_down <= exit_left and exit_down <= exit_right:
            if obj[4] == SLOPE or (st.vy >= 0.0 and exit_down <= half):
                st.y = oy0 - half
                st.vy = 0.0
                st.ground = True
                return True
        return False

    def spider_jump(self, st, up):
        """Simplified spider teleport to the nearest surface above/below."""
        half = self.half
        best = None
        for obj in self.level.near(st.x - 20.0, st.x + 20.0):
            if obj[4] not in SOLID_LIKE:
                continue
            oy0 = obj[1]
            oy1 = oy0 + obj[3]
            if up > 0.0:
                if oy0 < st.y + half or oy0 - st.y > self.phys.spider_range:
                    continue
                best = oy0 if best is None else min(best, oy0)
            else:
                if oy1 > st.y - half or st.y - oy1 > self.phys.spider_range:
                    continue
                best = oy1 if best is None else max(best, oy1)
        if best is not None:
            st.y = best - half if up > 0.0 else best + half
            st.vy = 0.0
            st.ground = True


# ============================================================================
#  Beam / plan containers
# ============================================================================

class Beam:
    """One surviving branch: state + action history + accumulated score."""

    __slots__ = ("st", "actions", "score", "switches", "path", "risk", "alive",
                 "death_tick", "hold_len")

    def __init__(self, st, actions=None, score=0.0, switches=0, path=None,
                 risk=None, hold_len=0):
        self.st = st
        self.actions = actions or []
        self.score = score
        self.switches = switches
        self.path = path or []
        self.risk = risk or []
        self.alive = True
        self.death_tick = 0
        self.hold_len = hold_len

    def clone(self):
        other = Beam.__new__(Beam)
        other.st = self.st.clone()
        other.actions = list(self.actions)
        other.score = self.score
        other.switches = self.switches
        other.path = list(self.path)
        other.risk = list(self.risk)
        other.alive = self.alive
        other.death_tick = self.death_tick
        other.hold_len = self.hold_len
        return other


class Plan:
    """Everything that gets sent back to the C++ overlay."""

    __slots__ = ("hold", "path", "risk", "alt_paths", "corridor", "corridor_height",
                 "hud", "state", "clearance", "depth", "marks")

    def __init__(self):
        self.hold = False
        self.path = []
        self.risk = []
        self.alt_paths = []
        self.corridor = []
        self.marks = []
        self.corridor_height = 0.0
        self.hud = ""
        self.state = "idle"
        self.clearance = 0.0
        self.depth = 0
# ============================================================================
#  Brain: receding-horizon beam search with anti-spam shaping
# ============================================================================

class Brain:
    def __init__(self, store):
        self.store = store
        self.phys = Physics(store.data)
        self.level = Level(())
        self.sim = Simulator(self.phys, self.level)
        self.hold = False
        self.hold_age = 0
        self.ticks_since_plan = 0
        self.last_x = None
        self.switch_times = deque()
        self.prev_actions = []
        self.prev_hit = None
        self.plans = 0

    # -- configuration ------------------------------------------------------
    def profile(self, style):
        profiles = self.store.data.get("profiles") or {}
        name = style if style in ("smooth", "tryhard") else None
        if name is None:
            name = "smooth"
            if str(profiles.get("active")) in ("smooth", "tryhard"):
                name = str(profiles.get("active"))
        merged = deep_merge(DEFAULT_CONFIG["profiles"][name], profiles.get(name) or {})
        merged.pop("_doc", None)
        return merged, name

    # -- public entry -------------------------------------------------------
    def update(self, pkt):
        cfg = self.store.data
        visuals = cfg.get("visuals") or {}
        solver = cfg.get("solver") or {}
        prof, style = self.profile(pkt.get("style_profile"))
        preset = str(pkt.get("visual_preset") or "showcase")

        level = Level(pkt.get("objects") or [])
        if level.objects != self.level.objects:
            self.level = level
            self.sim = Simulator(self.phys, level)

        if not bool(pkt.get("enabled", True)):
            self.hold = False
            return self._idle_plan("disabled")

        start = self._start_state(pkt)
        if start is None:
            self.hold = False
            return self._idle_plan("dead")

        # Ticks elapsed since the previous plan, derived from the distance the
        # player travelled (robust to packet jitter and to the self test).
        if self.last_x is None:
            self.ticks_since_plan = 1
        else:
            dx = max(0.0, start.x - self.last_x)
            self.ticks_since_plan = int(clamp(
                dx / max(1.0, start.vx) * self.phys.tick_rate, 0.0, 2000.0))
        self.last_x = start.x

        lookahead = int(clamp(num(solver.get("lookahead_ticks"), 96), 16, 400))
        beam_width = int(clamp(num(solver.get("beam_width"), 14), 2, 64))
        macro = int(clamp(num(prof.get("macro_step_ticks"), 6), 1, 32))
        emergency_h = int(clamp(num(solver.get("emergency_horizon_ticks"), 18), 4, 120))

        self.sim.prepare(start.x, start.vx, lookahead + 2)
        keep_dies = self._death_tick(start, True, emergency_h) < emergency_h
        emergency = keep_dies or self._death_tick(start, False, emergency_h) < emergency_h

        result = self._search(start, prof, solver, lookahead, beam_width, macro, emergency)
        if result is None:
            self.hold = not keep_dies
            plan = self._idle_plan("trapped")
            plan.hold = self.hold
            self.prev_actions = []
            return plan
        best, alive, dead = result

        # ---- real anti-spam gate: minimum hold time + CPS limiter ----
        desired = bool(best.actions[0][0]) if best.actions else self.hold
        now = time.monotonic()
        recent = deque(t for t in self.switch_times if now - t <= 1.0)
        min_switch = int(clamp(num(prof.get("min_switch_ticks"), 14), 0, 240))
        max_cps = int(clamp(num(prof.get("max_cps"), 10), 1, 200))
        # hold_age accumulates the simulated ticks since the last real switch.
        hold_age = self.hold_age + self.ticks_since_plan
        if desired != self.hold and not emergency:
            if hold_age < min_switch or len(recent) >= max_cps:
                desired = self.hold
        if desired != self.hold:
            recent.append(now)
            hold_age = 0
            self.ticks_since_plan = 0
        self.switch_times = recent
        self.hold_age = hold_age
        self.hold = desired

        plan = self._build_plan(start, best, alive, dead, visuals, prof, solver,
                                style, emergency)
        self.prev_actions = list(best.actions)
        self.prev_hit = self._hit_pack(start)
        self.plans += 1

        if preset == "off":
            # Absolute silence: only the input is required.
            plan.path = []
            plan.risk = []
            plan.alt_paths = []
            plan.corridor = []
            plan.marks = []
            plan.corridor_height = 0.0
        return plan

    # -- state helpers ------------------------------------------------------
    def _start_state(self, pkt):
        if pkt.get("x") is None or pkt.get("y") is None:
            return None
        if bool(pkt.get("dead", False)):
            return None
        mode = str(pkt.get("mode") or "cube")
        if mode not in ("cube", "ship", "ball", "ufo", "wave", "robot", "spider", "swing"):
            mode = "cube"
        vx = num(pkt.get("speed_units"), 0.0)
        if vx <= 1.0:
            tier = as_int(pkt.get("speed_tier"), 11)
            vx = self.phys.base_speed * self.phys.speed_multipliers.get(tier, 1.0)
        return SimState(
            num(pkt.get("x")), num(pkt.get("y")),
            vy=num(pkt.get("vy"), 0.0), vx=vx,
            upside=bool(pkt.get("upside_down", False)),
            ground=bool(pkt.get("on_ground", False)),
            mode=mode, hold=self.hold,
        )

    @staticmethod
    def _hit_pack(start):
        return (round(start.x), round(start.y, 1), round(start.vx, 1),
                start.upside, start.ground, start.mode)

    @staticmethod
    def _idle_plan(state):
        plan = Plan()
        plan.hold = False
        plan.state = state
        plan.hud = "idle"
        return plan

    def _death_tick(self, start, hold, horizon):
        st = start.clone()
        for tick in range(horizon):
            if not self.sim.step(st, hold, tick):
                return tick
        return horizon

    def _tight_scale(self, start, solver):
        gap = self.level.corridor_at(start.x, start.y, self.phys.ground_y)[2]
        threshold = max(1.0, num(solver.get("adaptive_tight_threshold"), 55.0))
        if gap >= threshold:
            return 1.0
        return clamp(gap / threshold, 0.2, 1.0)
# -- simulation helpers -------------------------------------------------
    def _advance(self, beam, hold, ticks, tick, shape=None):
        """Steps `beam` verbatim for `ticks` ticks (exact per-tick physics)."""
        survived = True
        for _ in range(ticks):
            if not self.sim.step(beam.st, hold, tick):
                survived = False
                break
            tick += 1
            beam.path.append((beam.st.x, beam.st.y))
            clearance = self.sim.clearance(
                beam.st.x, beam.st.y, self.sim.objects_at(tick - 1))
            beam.risk.append(clamp(100.0 - clearance / 60.0 * 100.0, 0.0, 100.0))
            if shape is not None:
                deadzone, center, alert, vertical_penalty = shape
                if clearance < 8.0:
                    beam.score -= (8.0 - clearance) * 1.5
                else:
                    beam.score += min(clearance, alert) * 0.02
                if beam.st.mode in ("ship", "ufo"):
                    beam.score -= abs(beam.st.vy) * vertical_penalty * 0.001
                # Dead zone: no reward for re-centering while safely inside it.
                if center is not None:
                    distance = abs(beam.st.y - center)
                    if distance > deadzone * 0.5:
                        beam.score -= (distance - deadzone * 0.5) * 0.02
        beam.alive = survived
        beam.death_tick = tick
        return survived, tick

    def _simulate_actions(self, start, actions, limit):
        """Runs a full hold-plan from `start` -> (beam, alive)."""
        beam = Beam(start.clone())
        beam.st.hold = start.hold
        beam.path = [(beam.st.x, beam.st.y)]
        beam.risk = [self._risk(beam.st.x, beam.st.y)]
        tick = 0
        for hold, length in actions:
            hold = bool(hold)
            length = max(1, int(length))
            if hold != beam.st.hold:
                beam.switches += 1
                beam.hold_len = 0
            beam.hold_len += length
            survived, tick = self._advance(beam, hold, min(length, limit - tick), tick)
            beam.actions.append((hold, length))
            if not survived or tick >= limit:
                return beam, survived
        return beam, True

    def _seed_branches(self, start, prof, lookahead):
        """Fresh root + (optionally) a replay of the previous winning plan."""
        beams = [Beam(start.clone(), hold_len=self.hold_age + self.ticks_since_plan)]
        if self.prev_actions and self.prev_hit == self._hit_pack(start):
            beam, alive = self._simulate_actions(start, self.prev_actions, lookahead)
            if alive:
                beam.score += float(prof.get("hysteresis_bonus", 0.0))
                beams.append(beam)
        return beams

    def _risk(self, x, y):
        above, below = self.level.nearest_vertical(x, y)
        return clamp(100.0 - min(above, below) * 2.0, 0.0, 100.0)

    def _orb_ahead(self, beam, ticks=36):
        """Cube rule: no pointless air clicks unless an orb is in reach."""
        for tick in range(ticks):
            for obj in self.sim.objects_at(tick):
                if obj[4] == ORB:
                    if abs((obj[0] + obj[2] * 0.5) - beam.st.x) < 90.0:
                        return True
        return False

    def _allowed_holds(self, beam, emergency, min_switch, max_switches):
        current = beam.st.hold
        if emergency:
            return (False, True)
        if beam.hold_len < min_switch or beam.switches >= max_switches:
            return (current,)
        if (beam.st.mode == "cube" and not current and not beam.st.ground
                and not self._orb_ahead(beam)):
            return (False,)
        return (current, not current)

    def _plan_score(self, beam, prof, solver, start):
        """End-of-horizon shaping (per-tick shaping already ran in _advance)."""
        score = beam.score
        alert = num(solver.get("clearance_alert"), 45.0)
        progress = num(solver.get("progress_weight"), 0.05)
        if beam.path:
            score += (beam.st.x - beam.path[0][0]) * progress
        clearance = self.sim.clearance(beam.st.x, beam.st.y)
        if clearance < alert:
            score -= (alert - clearance) * 0.6
        if beam.st.mode in ("ship", "ufo", "ball", "robot"):
            score -= abs(beam.st.vy) * float(prof.get("vertical_penalty", 0.05))
        if beam.st.mode in GROUND_MODES and beam.st.y < start.y - 40.0:
            score -= (start.y - beam.st.y) * 0.05
        return score
# -- beam search ---------------------------------------------------------
    def _shape(self, prof, start, scale):
        """Per-tick shaping coefficients: dead zone, altitude, vertical speed."""
        bottom, top, gap, _tight = self.level.corridor_at(start.x, start.y,
                                                           self.phys.ground_y)
        deadzone = float(prof.get("safe_corridor_deadzone", 0.0)) * scale
        center = None
        if gap < 900.0:  # only meaningful in closed corridors
            center = (bottom + top) * 0.5
        return (deadzone / 2.0, center,
                num(self.store.data.get("solver", {}).get("clearance_alert"), 45.0),
                float(prof.get("vertical_penalty", 0.05)))

    def _search(self, start, prof, solver, lookahead, beam_width, macro, emergency):
        """Branch every `macro` ticks, exact physics inside every macro step."""
        budget = max(0.5, num(solver.get("time_budget_ms"), 6.0)) / 1000.0
        deadline = time.perf_counter() + budget
        scale = self._tight_scale(start, solver)
        penalty = float(prof.get("click_penalty", 18.0)) * scale
        min_switch = int(clamp(num(prof.get("min_switch_ticks"), 14), 0, 240))
        shape = self._shape(prof, start, scale)
        macro = max(1, macro)

        alive = self._seed_branches(start, prof, lookahead)
        dead = []
        steps = int(math.ceil(lookahead / macro))
        # CPS cap expressed inside the horizon: 10 CPS over 96 ticks = 4 clicks.
        max_switches = max(1, int(num(prof.get("max_cps"), 10)
                                 * lookahead / max(1.0, self.phys.tick_rate)) + 1)

        for _ in range(steps):
            if not alive:
                break
            candidates = []
            for beam in alive:
                tick = len(beam.path) - 1
                remaining = max(0, lookahead - tick)
                if remaining == 0:
                    candidates.append(beam)
                    continue
                span = min(macro, remaining)
                for hold in self._allowed_holds(beam, emergency, min_switch, max_switches):
                    child = beam.clone()
                    if hold != beam.st.hold:
                        child.switches += 1
                        child.hold_len = 0
                        child.score -= penalty
                    survived, _tick = self._advance(child, hold, span, tick, shape)
                    child.hold_len += span
                    if child.actions and child.actions[-1][0] == hold:
                        child.actions[-1] = (hold, child.actions[-1][1] + span)
                    else:
                        child.actions.append((hold, span))
                    if not survived:
                        dead.append(child)
                    else:
                        candidates.append(child)
                if time.perf_counter() > deadline and candidates:
                    break

            if not candidates:
                break
            candidates.sort(key=lambda b: b.score, reverse=True)
            merged = []
            for cand in candidates:
                duplicate = False
                for kept in merged:
                    if (kept.st.hold == cand.st.hold
                            and abs(kept.st.y - cand.st.y) < 3.0
                            and abs(kept.st.vy - cand.st.vy) < 12.0):
                        duplicate = True
                        break
                if not duplicate:
                    merged.append(cand)
                if len(merged) >= beam_width:
                    break
            alive = merged
            if time.perf_counter() > deadline and len(merged) >= 2:
                break

        for beam in alive:
            beam.score = self._plan_score(beam, prof, solver, start)
        alive.sort(key=lambda b: b.score, reverse=True)
        dead.sort(key=lambda b: b.death_tick, reverse=True)
        if not alive:
            # Everything died: return the branch that survived the longest so
            # the bot still produces a sane (if doomed) input.
            if dead:
                survivor = dead[0]
                survivor.alive = True
                return survivor, [survivor], dead[:3]
            return None
        return alive[0], alive, dead[:3]
# -- answer construction ------------------------------------------------
    @staticmethod
    def _sample(points, step):
        if step <= 1 or len(points) <= 2:
            return list(points)
        sampled = [points[i] for i in range(0, len(points), step)]
        if sampled[-1] != points[-1]:
            sampled.append(points[-1])
        return sampled

    def _build_plan(self, start, best, alive, dead, visuals, prof, solver,
                    style, emergency):
        plan = Plan()
        plan.hold = self.hold
        plan.depth = len(best.path) - 1

        step = int(clamp(num(visuals.get("trajectory_step"), 4), 1, 16))
        plan.path = [[round(p[0], 1), round(p[1], 1)]
                     for p in self._sample(best.path, step)]

        if visuals.get("send_risk", True):
            risk = [int(clamp(r, 0, 100)) for r in self._sample(best.risk, step)]
            # Keep the two arrays aligned even if a branch was cut short.
            if len(risk) < len(plan.path):
                risk.extend([0] * (len(plan.path) - len(risk)))
            plan.risk = risk[:len(plan.path)]

        if visuals.get("show_alt_paths", True):
            max_alt = int(clamp(num(visuals.get("max_alt_paths"), 2), 0, 4))
            traces = [b.path for b in alive[1:1 + max_alt]]
            if dead:
                traces.append(dead[0].path)
            if len(traces) < 1 and max_alt > 0:
                # Always give the overlay at least one rejected branch.
                alt, _alive = self._simulate_actions(
                    start, [(not best.st.hold, max(1, plan.depth))], max(1, plan.depth))
                traces.append(alt.path)
            plan.alt_paths = [
                [[round(p[0], 1), round(p[1], 1)] for p in self._sample(trace, step * 2)]
                for trace in traces if len(trace) > 3
            ][:max_alt]

        if visuals.get("show_corridor", True):
            cstep = int(clamp(num(visuals.get("corridor_step_ticks"), 8), 1, 64))
            corridor = []
            heights = []
            for i in range(0, len(best.path), cstep):
                px, py = best.path[i]
                bottom, top, gap, _tight = self.level.corridor_at(
                    px, py, self.phys.ground_y)
                corridor.extend((round(px, 1), round(top, 1), round(bottom, 1)))
                heights.append(gap)
            plan.corridor = corridor
            plan.corridor_height = min(heights) if heights else 0.0

        # Predicted press (0) / release (1) moments of the committed plan.
        marks = []
        tick = 0
        previous = start.hold
        for hold, length in best.actions:
            if hold != previous and tick < len(best.path):
                point = best.path[tick]
                marks.extend((round(point[0], 1), round(point[1], 1), 0 if hold else 1))
                previous = hold
            tick += int(length)
        plan.marks = marks

        plan.clearance = self.sim.clearance(start.x, start.y)
        cps = len([t for t in self.switch_times if time.monotonic() - t <= 1.0])
        if emergency:
            state = "emergency"
        elif plan.corridor_height and plan.corridor_height < 60.0:
            state = "tight"
        else:
            state = "safe"
        plan.state = state
        plan.hud = (f"cps {cps}/{int(clamp(num(prof.get('max_cps'), 10), 1, 200))}"
                    f" | clr {plan.clearance:.0f}"
                    f" | depth {plan.depth}"
                    f" | {style}/{state}")
        return plan

# ============================================================================
#  UDP server
# ============================================================================

class Server:
    def __init__(self, store, host, port, verbose=False):
        self.store = store
        self.brain = Brain(store)
        self.host = host
        self.port = port
        self.verbose = verbose
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        buffer_size = int(num((store.data.get("network") or {}).get("socket_buffer"), 16384))
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buffer_size)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, buffer_size)
        except OSError:
            pass
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        self.plan = None
        self.last_plan_time = 0.0
        self.cached = b'{"hold":false}'
        self.cached_key = None
        self.packets = 0
        self.replans = 0
        self.last_status = 0.0

    # -- response encoding --------------------------------------------------
    def encode(self, plan, preset):
        if preset == "off":
            # Visual preset off: only the input, nothing else on the wire.
            return b'{"hold":true}' if self.brain.hold else b'{"hold":false}'
        payload = {
            "hold": bool(self.brain.hold),
            "hud": plan.hud,
            "corridor_height": round(plan.corridor_height, 1),
        }
        if plan.path:
            payload["path"] = plan.path
        if plan.risk:
            payload["risk"] = plan.risk
        if plan.alt_paths:
            payload["alt_paths"] = plan.alt_paths
        if plan.corridor:
            payload["corridor"] = plan.corridor
        if plan.marks:
            payload["marks"] = plan.marks
        return json.dumps(payload, separators=(",", ":")).encode("ascii")

    # -- packet handling ----------------------------------------------------
    def handle(self, data, addr, now):
        pkt = json.loads(data.decode("utf-8", "ignore"))
        interval = num((self.store.data.get("network") or {}).get("plan_interval_ms"),
                       20.0) / 1000.0
        preset = str(pkt.get("visual_preset") or "showcase")

        if self.plan is None or now - self.last_plan_time >= interval:
            self.plan = self.brain.update(pkt)
            self.last_plan_time = now
            self.replans += 1
        else:
            # Reuse the previous plan between search updates (anti-spam + CPU).
            self.brain.hold_age += 1
            self.plan.hold = self.brain.hold

        key = (preset, self.plan.hold, id(self.plan), len(self.plan.path),
               len(self.plan.alt_paths))
        if key != self.cached_key:
            self.cached = self.encode(self.plan, preset)
            self.cached_key = key
        self.sock.sendto(self.cached, addr)
        self.packets += 1

    def status(self, now, force=False):
        if not force and now - self.last_status < 2.0:
            return
        self.last_status = now
        if self.verbose or force:
            print(f"[brain] packets {self.packets} | plans {self.replans} | "
                  f"hold {'PRESS' if self.brain.hold else 'release'} | "
                  f"payload {len(self.cached)} B")

    # -- main loop ----------------------------------------------------------
    def run(self):
        print(f"[brain] UDP listening on {self.host}:{self.port}")
        print(f"[brain] config: {self.store.path} (hot reload, 1 s)")
        try:
            while True:
                now = time.monotonic()
                if self.store.reload_if_changed(now):
                    self.brain = Brain(self.store)
                    print("[brain] config reloaded")
                if self.store.error and self.verbose:
                    print(f"[brain] config error: {self.store.error}")

                ready, _, _ = select.select([self.sock], [], [], 0.2)
                if not ready:
                    self.status(now)
                    continue
                while True:
                    try:
                        data, addr = self.sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    try:
                        self.handle(data, addr, time.monotonic())
                    except (ValueError, KeyError, TypeError) as exc:
                        if self.verbose:
                            print(f"[brain] discarded packet: {exc}")
                self.status(now)
        except KeyboardInterrupt:
            print("\n[brain] stopped")
        finally:
            self.sock.close()
        return 0
# ============================================================================
#  Self test: closed-loop simulation of a demo level + anti-spam assertions.
#  Run with:  python ai-brain/bot.py --selftest
# ============================================================================

def _demo_level():
    """Short cube level: single spike, spike pair, raised platform."""
    return [
        [600.0, 0.0, 30.0, 30.0, HAZARD],
        [1100.0, 0.0, 30.0, 30.0, HAZARD],
        [1140.0, 0.0, 30.0, 30.0, HAZARD],
        [1500.0, 0.0, 600.0, 45.0, SOLID],
        [2300.0, 90.0, 180.0, 30.0, SOLID],
    ]


def selftest():
    store = ConfigStore(CONFIG_PATH)
    failures = []

    def check(name, condition):
        print(f"  {'ok  ' if condition else 'FAIL'} {name}")
        if not condition:
            failures.append(name)

    print("[selftest] config")
    check("config loaded", isinstance(store.data, dict) and "physics" in store.data)
    phys = Physics(store.data)
    check("jump velocity comes from config",
          abs(phys.jump - num(store.data["physics"]["jump_velocity"], 0.0)) < 1e-6)

    print("[selftest] level index")
    level = Level(_demo_level())
    check("objects indexed", len(level.objects) == len(_demo_level()))
    check("corridor walls found", len(level.near(1490.0, 1510.0)) >= 1)

    print("[selftest] closed-loop cube run (smooth profile)")
    brain = Brain(store)
    brain.level = level
    brain.sim = Simulator(brain.phys, level)
    brain.phys = phys
    state = SimState(60.0, 15.0, ground=True, mode="cube")
    fps_ticks = max(1, int(round(brain.phys.tick_rate / 60.0)))
    switches = 0
    previous = False
    hold_ticks = 0
    min_hold_seen = 1 << 30
    dies = False
    progress = 0.0
    plan = None

    for _ in range(240 * 8):
        for _tick in range(fps_ticks):
            if not brain.sim.step(state, brain.hold, 0):
                dies = True
                break
        if dies:
            break
        plan = brain.update({
            "x": state.x, "y": state.y, "vy": state.vy,
            "speed_units": state.vx, "speed_tier": 11, "mode": state.mode,
            "upside_down": state.upside, "on_ground": state.ground, "dead": False,
            "enabled": True, "visual_preset": "showcase", "style_profile": "smooth",
            "objects": _demo_level(),
        })
        if brain.hold != previous:
            switches += 1
            min_hold_seen = min(min_hold_seen, hold_ticks)
            hold_ticks = 0
            previous = brain.hold
        hold_ticks += fps_ticks
        progress = max(progress, state.x)

    print(f"       reached x={progress:.0f}, switches={switches}, "
          f"min hold={min_hold_seen if min_hold_seen < (1 << 30) else 'n/a'} ticks")
    check("bot survives the demo level", not dies and progress > 700.0)
    check("no click spam (switches <= 3/s)", switches <= 24)
    check("hold duration respects min_switch_ticks",
          min_hold_seen >= int(num(store.data["profiles"]["smooth"]["min_switch_ticks"], 14)) - 2)
    check("plan rendered a path", bool(plan and plan.path))
    check("every path point has a risk value", bool(plan) and len(plan.risk) == len(plan.path))

    print("[selftest] visual presets")
    brain_presets = Brain(store)
    brain_presets.level = level
    brain_presets.sim = Simulator(phys, level)
    base_packet = {
        "x": 200.0, "y": 15.0, "vy": 0.0, "speed_units": 311.58, "speed_tier": 11,
        "mode": "cube", "upside_down": False, "on_ground": True, "dead": False,
        "enabled": True, "style_profile": "smooth", "objects": _demo_level(),
    }
    plan_off = brain_presets.update(dict(base_packet, visual_preset="off"))
    check("off preset carries no arrays",
          not plan_off.path and not plan_off.alt_paths and not plan_off.corridor)

    server = Server.__new__(Server)
    server.brain = brain_presets
    encoded = Server.encode(server, plan_off, "off")
    check("off preset payload is hold only", encoded in (b'{"hold":true}', b'{"hold":false}'))

    plan_show = brain_presets.update(dict(base_packet, visual_preset="showcase"))
    show_bytes = Server.encode(server, plan_show, "showcase")
    print(f"       showcase payload {len(show_bytes)} B, {len(plan_show.path)} path points")
    check("showcase payload fits the UDP buffer", len(show_bytes) < 16000)
    check("showcase sends alt paths", bool(plan_show.alt_paths))

    if failures:
        print(f"[selftest] {len(failures)} failure(s): {', '.join(failures)}")
        return 1
    print("[selftest] all checks passed")
    return 0
# ============================================================================
#  Entry point
# ============================================================================

def main(argv):
    parser = argparse.ArgumentParser(description="Geometry Dash AI brain (UDP)")
    parser.add_argument("--host", default=None, help="bind address (default: config)")
    parser.add_argument("--port", type=int, default=None, help="UDP port (default: config)")
    parser.add_argument("--config", default=CONFIG_PATH, help="path to config.json")
    parser.add_argument("--verbose", action="store_true", help="verbose logging")
    parser.add_argument("--selftest", action="store_true", help="run internal checks and exit")
    args = parser.parse_args(argv)

    store = ConfigStore(args.config)
    if store.error:
        print(f"[brain] config error: {store.error}")
    if args.selftest:
        return selftest()

    network = store.data.get("network") or {}
    host = args.host or str(network.get("host") or "127.0.0.1")
    port = args.port or as_int(network.get("port"), 12345)
    server = Server(store, host, port, args.verbose)
    return server.run()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
