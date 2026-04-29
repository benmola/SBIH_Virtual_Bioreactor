"""
AD Reactor Control Challenge — Digital Twin Dashboard
Visualises the physical reactor, the data pipeline, and the AI Digital Twin.

Usage:
  python main.py              # auto-detect COM port
  python main.py COM3         # specify port explicitly
  python main.py --demo       # demo mode (mouse sliders)
"""

import sys, math, random, time, threading, collections
from dataclasses import dataclass
from typing import Optional, Tuple, List

import pygame
import serial
import serial.tools.list_ports

# ── Screen layout ─────────────────────────────────────────────────────────────
SCREEN_W, SCREEN_H = 1400, 820
FPS         = 60
HEADER_H    = 64
BANNER_H    = 46
MAIN_TOP    = HEADER_H + BANNER_H     # 110
MAIN_BOT    = 700
MAIN_H      = MAIN_BOT - MAIN_TOP     # 590
LEFT_W      = 430                      # reactor panel
RIGHT_X     = 930                      # gauges panel start
FLOW_CX     = (LEFT_W + RIGHT_X) // 2 # 680 — centre of data-flow panel
REACTOR_CX  = LEFT_W  // 2            # 215
REACTOR_CY  = MAIN_TOP + MAIN_H // 2  # 405

# Gauge positions (side-by-side in right panel)
TEMP_GX, PH_GX, GAUGE_GY = 1050, 1275, REACTOR_CY - 40
GAUGE_R  = 90
SLIDER_Y = MAIN_BOT + 20              # below main area — demo sliders
# Biogas time-series chart (right panel, below the two gauges)
BIOGAS_CHART_X = RIGHT_X + 15
BIOGAS_CHART_Y = GAUGE_GY + GAUGE_R + 48
BIOGAS_CHART_W = SCREEN_W - RIGHT_X - 25
BIOGAS_CHART_H = MAIN_BOT - BIOGAS_CHART_Y - 10

# ── Serial constants ──────────────────────────────────────────────────────────
BAUD_RATE     = 115200
SAMPLE_HZ     = 20
DELAY_SECONDS = 2.5
DELAY_SAMPLES = int(DELAY_SECONDS * SAMPLE_HZ)

# ── Colours ───────────────────────────────────────────────────────────────────
BLACK      = (255, 255, 255)   # window fill — white
WHITE      = (255, 255, 255)   # kept for compatibility
TEXT       = (18,  24,  52)    # dark navy for all body text
GREEN      = (0,   170, 65)
RED        = (210, 35,  35)
AMBER      = (230, 140, 0)
BLUE       = (25,  100, 230)
DARK_GRAY  = (170, 174, 195)
MID_GRAY   = (100, 108, 145)
PANEL_BG   = (232, 236, 248)   # light blue-grey panels
PANEL_MID  = (210, 216, 238)   # slightly darker for flow boxes

# ── AD parameter ranges ───────────────────────────────────────────────────────
PH_MIN,   PH_MAX   = 4.0,  9.0
PH_SAFE_LO,  PH_SAFE_HI  = 6.8, 7.2
PH_WARN_LO,  PH_WARN_HI  = 6.0, 7.8

TEMP_MIN, TEMP_MAX = 25.0, 50.0
TEMP_SAFE_LO, TEMP_SAFE_HI = 35.0, 40.0
TEMP_WARN_LO, TEMP_WARN_HI = 33.0, 42.0

OLR_YAW_THRESHOLD = 15.0


def _imu_map(v: float, in_lo: float, in_hi: float, out_lo: float, out_hi: float) -> float:
    """Linear map from one range to another."""
    return out_lo + (v - in_lo) / (in_hi - in_lo) * (out_hi - out_lo)


# ── Data classes ──────────────────────────────────────────────────────────────
@dataclass
class IMUPacket:
    pitch: float; roll: float; yaw: float; switch: int

@dataclass
class ADState:
    temperature:     float = 37.5
    ph:              float = 7.0
    olr_disturbance: float = 0.0


# ── Serial reader ─────────────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    def __init__(self, port: str, queue: collections.deque):
        super().__init__(daemon=True)
        self.port = port; self.queue = queue
        self.running = True; self.connected = False

    def run(self):
        while self.running:
            try:
                with serial.Serial(self.port, BAUD_RATE, timeout=1) as ser:
                    self.connected = True
                    while self.running:
                        raw = ser.readline()
                        if not raw: continue
                        parts = raw.decode("utf-8", errors="ignore").strip().split(",")
                        if len(parts) == 4:
                            try:
                                self.queue.append(IMUPacket(
                                    float(parts[0]), float(parts[1]),
                                    float(parts[2]), int(parts[3])))
                            except ValueError: pass
            except serial.SerialException:
                self.connected = False; time.sleep(2)

    def stop(self): self.running = False


# ── Data processor ────────────────────────────────────────────────────────────
class DataProcessor:
    def __init__(self):
        self._delay_buf = collections.deque(maxlen=DELAY_SAMPLES)
        self._prev_temp = 37.5; self._prev_ph = 7.0
        self._smooth_temp = 37.5; self._smooth_ph = 7.0
        self._prev_yaw = 0.0; self._olr = 0.0

    def _imu_to_raw(self, pkt: IMUPacket) -> Tuple[float, float]:
        return (_imu_map(pkt.pitch, -90.0, 90.0, TEMP_MIN, TEMP_MAX),
                _imu_map(pkt.roll,  -90.0, 90.0, PH_MIN,   PH_MAX))

    def _olr_disturbance(self, pkt, temp, ph):
        yd = abs(pkt.yaw - self._prev_yaw); self._prev_yaw = pkt.yaw
        if yd > OLR_YAW_THRESHOLD: self._olr = min(self._olr + yd * 0.06, 4.0)
        self._olr *= 0.92
        d = self._olr * (random.random() * 2 - 1)
        return temp + d * 2.0, ph + d * 0.4, self._olr

    def process_manual(self, pkt) -> ADState:
        t, p = self._imu_to_raw(pkt)
        t, p, olr = self._olr_disturbance(pkt, t, p)
        self._delay_buf.append((t, p))
        dt, dp = self._delay_buf[0] if len(self._delay_buf) >= DELAY_SAMPLES else (t, p)
        dt += random.gauss(0, 0.35); dp += random.gauss(0, 0.06)
        ot = 0.85 * self._prev_temp + 0.15 * dt
        op = 0.85 * self._prev_ph   + 0.15 * dp
        self._prev_temp = ot; self._prev_ph = op
        return ADState(ot, op, olr)

    def process_ai(self, pkt) -> ADState:
        t, p = self._imu_to_raw(pkt)
        t, p, olr = self._olr_disturbance(pkt, t, p)
        a = 0.15
        self._smooth_temp = a * t + (1 - a) * self._smooth_temp
        self._smooth_ph   = a * p + (1 - a) * self._smooth_ph
        if self._smooth_ph   < PH_SAFE_LO   - 0.25: self._smooth_ph   += 0.05
        if self._smooth_ph   > PH_SAFE_HI   + 0.25: self._smooth_ph   -= 0.05
        if self._smooth_temp < TEMP_SAFE_LO - 1.0:  self._smooth_temp += 0.10
        if self._smooth_temp > TEMP_SAFE_HI + 1.0:  self._smooth_temp -= 0.10
        return ADState(self._smooth_temp, self._smooth_ph, olr)


# ── Feedstock icon helper ─────────────────────────────────────────────────────
def _draw_feedstock_icons(surf: pygame.Surface, panel_cx: int, bottom_y: int,
                          font: pygame.font.Font):
    """Draw maize silage, food waste, cattle slurry horizontally at bottom of reactor panel."""
    feedstocks = [
        ("Maize Silage",  (240, 190,  40), (80, 160,  60), "corn"),
        ("Food Waste",    (220,  80,  50), (60, 140,  40), "apple"),
        ("Cattle Slurry", (160, 110,  50), (90,  65,  30), "cow"),
    ]
    spacing = 128
    r = 18
    n = len(feedstocks)
    start_x = panel_cx - (n - 1) * spacing // 2
    iy = bottom_y - 52   # icon center y

    # Section title
    title = font.render("FEEDSTOCKS", True, MID_GRAY)
    surf.blit(title, title.get_rect(centerx=panel_cx, centery=iy - r - 14))

    for i, (label, body, detail, shape) in enumerate(feedstocks):
        ix = start_x + i * spacing

        if shape == "corn":
            pygame.draw.ellipse(surf, body,   (ix - 8, iy - r,     16, r * 2))
            pygame.draw.polygon(surf, detail, [(ix - 8, iy - r), (ix - 16, iy - r - 8), (ix, iy - r + 6)])
            pygame.draw.polygon(surf, detail, [(ix + 8, iy - r), (ix + 16, iy - r - 8), (ix, iy - r + 6)])
            for row in range(3):
                for col in range(2):
                    pygame.draw.circle(surf, (220, 150, 20),
                                       (ix - 3 + col * 7, iy - 8 + row * 8), 2)

        elif shape == "apple":
            pygame.draw.circle(surf, body,   (ix, iy + 2), r)
            pygame.draw.ellipse(surf, detail, (ix - 1, iy - r - 7, 14, 9))
            pygame.draw.line(surf, (100, 60, 20), (ix + 5, iy - r), (ix + 7, iy - r - 9), 2)
            pygame.draw.circle(surf, (255, 130, 100), (ix - 6, iy - 4), 5)

        elif shape == "cow":
            pygame.draw.ellipse(surf, body, (ix - r, iy - 8, r * 2, 17))
            pygame.draw.circle(surf, body,  (ix + r - 2, iy - 4), 9)
            pygame.draw.ellipse(surf, (200, 150, 100), (ix + r + 3, iy - 11, 6, 8))
            pygame.draw.ellipse(surf, detail, (ix - 8, iy - 5, 10, 6))
            pygame.draw.ellipse(surf, detail, (ix + 2, iy + 1, 7, 5))
            for lx in (ix - 10, ix - 3, ix + 4, ix + 11):
                pygame.draw.line(surf, (120, 80, 30), (lx, iy + 9), (lx, iy + 17), 2)

        # Label below icon
        lbl = font.render(label, True, TEXT)
        surf.blit(lbl, lbl.get_rect(centerx=ix, top=iy + r + 4))


# ── Animated bioreactor ───────────────────────────────────────────────────────
class ReactorViz:
    N_BACT = 16
    N_BUBB = 14

    def __init__(self, cx, cy, w, h):
        self.cx, self.cy, self.w, self.h = cx, cy, w, h
        self.lx = cx - w // 2 + 10
        self.ly = cy - h // 2 + 10
        self.lw = w - 20
        self.lh = h - 20
        # bacteria: [x, y, vx, vy, angle]  (x,y in 0-1 relative to liquid)
        self.bact = [[random.random(), random.random(),
                      random.uniform(-0.003, 0.003), random.uniform(-0.003, 0.003),
                      random.uniform(0, math.tau)] for _ in range(self.N_BACT)]
        # bubbles: [rx, ry, r, speed]
        self.bubb = [[random.uniform(0.1, 0.9), random.random(),
                      random.randint(3, 7), random.uniform(0.003, 0.007)]
                     for _ in range(self.N_BUBB)]
        self.health = 1.0

    def _zone_color_for(self, v, slo, shi, wlo, whi):
        if slo <= v <= shi:   return 1.0   # healthy
        if wlo <= v <= whi:   return 0.5   # warning
        return 0.0                          # danger

    def _health_score(self, v, slo, shi, wlo, whi):
        """1.0 in safe zone, sqrt(0.5) in amber zone (so both-amber × both-amber = 0.50), 0 in danger."""
        if slo <= v <= shi:
            return 1.0
        if wlo < v < slo or shi < v < whi:
            return math.sqrt(0.5)   # both orange → sqrt(0.5)² = 0.5 (50 % health)
        return 0.0

    def update(self, state: ADState):
        ph_h   = self._health_score(state.ph,          PH_SAFE_LO,   PH_SAFE_HI,   PH_WARN_LO,   PH_WARN_HI)
        temp_h = self._health_score(state.temperature, TEMP_SAFE_LO, TEMP_SAFE_HI, TEMP_WARN_LO, TEMP_WARN_HI)
        # Multiplicative — both pH and temperature must be healthy; neither can compensate for the other
        h = ph_h * temp_h
        self.health += (h - self.health) * 0.06

        spd = self.health * 0.005
        for b in self.bact:
            if self.health > 0.2:
                b[2] += random.uniform(-0.0004, 0.0004)
                b[3] += random.uniform(-0.0004, 0.0004)
                mag = math.hypot(b[2], b[3])
                if mag > spd and mag > 0:
                    b[2] = b[2] / mag * spd; b[3] = b[3] / mag * spd
                b[4] = math.atan2(b[3], b[2]) if mag > 0 else b[4]
            else:
                b[2] *= 0.97; b[3] *= 0.97
            b[0] = (b[0] + b[2]) % 1.0
            b[1] = (b[1] + b[3]) % 1.0

        temp_factor = 0.3 + 0.7 * (state.temperature - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)
        for bbl in self.bubb:
            bbl[1] -= bbl[3] * temp_factor
            if bbl[1] < 0:
                bbl[1] = 1.0
                bbl[0] = random.uniform(0.1, 0.9)
                bbl[2] = random.randint(3, 7)
        if state.olr_disturbance > 1.5:
            for bbl in random.sample(self.bubb, 3):
                bbl[1] = 1.0

    def draw(self, surf: pygame.Surface, font_lbl, font_val):
        cx, cy, w, h = self.cx, self.cy, self.w, self.h

        # Liquid colour: amber (healthy) → olive (sick)
        t = self.health
        liq = (int(70 + t * 120), int(60 + t * 100), int(20 + t * 30))

        # Vessel body
        pygame.draw.rect(surf, (58, 65, 102),
                         (cx - w//2, cy - h//2, w, h), border_radius=22)
        # Liquid fill
        pygame.draw.rect(surf, liq,
                         (self.lx, self.ly, self.lw, self.lh), border_radius=16)

        # Bubbles (use a surface for alpha)
        for bbl in self.bubb:
            bx = int(self.lx + bbl[0] * self.lw)
            by = int(self.ly + bbl[1] * self.lh)
            r  = bbl[2]
            bs = pygame.Surface((r*2+2, r*2+2), pygame.SRCALPHA)
            pygame.draw.circle(bs, (200, 230, 255, 150), (r+1, r+1), r)
            surf.blit(bs, (bx - r, by - r))

        # Bacteria
        for b in self.bact:
            bx = int(self.lx + b[0] * self.lw)
            by = int(self.ly + b[1] * self.lh)
            if t > 0.6:   bc = (80,  210, 100)
            elif t > 0.2: bc = (190, 160,  50)
            else:          bc = (180,  70,  40)
            dx = int(math.cos(b[4]) * 6); dy = int(math.sin(b[4]) * 6)
            pygame.draw.line(surf, bc, (bx - dx, by - dy), (bx + dx, by + dy), 3)
            pygame.draw.circle(surf, bc, (bx + dx, by + dy), 3)

        # Foam when OLR spike
        if hasattr(self, '_olr') and self._olr > 1.0:  # type: ignore[attr-defined]
            for i in range(8):
                fx = self.lx + int(i * self.lw / 8) + random.randint(-5, 5)
                pygame.draw.circle(surf, (220, 220, 200),
                                   (fx, self.ly + random.randint(2, 12)), random.randint(4, 10))

        # Vessel outline
        pygame.draw.rect(surf, (150, 165, 210),
                         (cx - w//2, cy - h//2, w, h), 3, border_radius=22)

        pipe_col = (115, 128, 175)

        # Gas riser pipe (vessel top → biogas bag)
        pygame.draw.rect(surf, pipe_col,
                         (cx - 10, cy - h//2 - 46, 20, 47), border_radius=4)

        # Biogas bag — gently inflates when reactor is active
        _tms   = pygame.time.get_ticks()
        _pulse = int(math.sin(_tms * 0.002) * 5) if self.health > 0.3 else 0
        bag_w  = 88 + _pulse
        bag_h  = 48 + max(0, _pulse // 2)
        bag_cy = cy - h//2 - 70
        bag_fill = (45, 185, 110) if self.health > 0.5 else (170, 125, 35)
        pygame.draw.ellipse(surf, bag_fill,
                            (cx - bag_w//2, bag_cy - bag_h//2, bag_w, bag_h))
        pygame.draw.ellipse(surf, (200, 235, 215),
                            (cx - bag_w//2, bag_cy - bag_h//2, bag_w, bag_h), 2)
        surf.blit(font_lbl.render("BIOGAS", True, TEXT),
                  font_lbl.render("BIOGAS", True, TEXT)
                  .get_rect(centerx=cx, centery=bag_cy))

        # Inlet pipe (left — lower third, substrate in)
        pygame.draw.rect(surf, pipe_col,
                         (cx - w//2 - 28, cy + h//4 - 8, 30, 16), border_radius=4)
        surf.blit(font_lbl.render("INLET", True, pipe_col),
                  font_lbl.render("INLET", True, pipe_col)
                  .get_rect(right=cx - w//2 - 33, centery=cy + h//4))
        # ── Feedstock icons at bottom of reactor panel ────────────────────────
        _draw_feedstock_icons(surf, cx, cy + h//2 + 80, font_lbl)

        # Outlet pipe (right — upper third, effluent out)
        pygame.draw.rect(surf, pipe_col,
                         (cx + w//2 - 2, cy - h//4 - 8, 30, 16), border_radius=4)
        surf.blit(font_lbl.render("OUTLET", True, pipe_col),
                  font_lbl.render("OUTLET", True, pipe_col)
                  .get_rect(left=cx + w//2 + 33, centery=cy - h//4))

        # Panel label — above the biogas bag
        surf.blit(font_lbl.render("PHYSICAL AD REACTOR", True, TEXT),
                  font_lbl.render("PHYSICAL AD REACTOR", True, TEXT)
                  .get_rect(centerx=cx, centery=MAIN_TOP + 16))

        # Health bar
        bw = w - 10
        bx = cx - bw // 2
        by = cy + h // 2 + 14
        pygame.draw.rect(surf, DARK_GRAY, (bx, by, bw, 10), border_radius=5)
        if self.health > 0:
            hc = (int(220*(1-self.health)), int(190*self.health), 30)
            pygame.draw.rect(surf, hc, (bx, by, int(bw*self.health), 10), border_radius=5)
        surf.blit(font_lbl.render(f"Reactor Health  {int(self.health*100)} %", True, TEXT),
                  font_lbl.render(f"Reactor Health  {int(self.health*100)} %", True, TEXT)
                  .get_rect(centerx=cx, centery=by + 20))


# ── Arc gauge ─────────────────────────────────────────────────────────────────
class ArcGauge:
    ARC_START = math.radians(150)
    ARC_SPAN  = math.radians(240)

    def __init__(self, cx, cy, r, val_min, val_max,
                 safe_lo, safe_hi, warn_lo, warn_hi, label, unit):
        self.cx, self.cy, self.r = cx, cy, r
        self.val_min, self.val_max = val_min, val_max
        self.safe_lo, self.safe_hi = safe_lo, safe_hi
        self.warn_lo, self.warn_hi = warn_lo, warn_hi
        self.label, self.unit = label, unit
        self._v = (val_min + val_max) / 2

    def update(self, v): self._v = max(self.val_min, min(self.val_max, v))

    def _angle(self, v):
        return self.ARC_START + (v - self.val_min) / (self.val_max - self.val_min) * self.ARC_SPAN

    def _color(self, v):
        if v < self.warn_lo or v > self.warn_hi: return RED
        if v < self.safe_lo or v > self.safe_hi: return AMBER
        return GREEN

    def draw(self, surf, font_val, font_lbl):
        cx, cy, r = self.cx, self.cy, self.r
        STEPS = 300
        THICK = 18

        # ── Dim background ring (all three zone colours at 25% brightness) ──────
        prev_pt = None
        for i in range(STEPS + 1):
            t = i / STEPS
            a = self.ARC_START + t * self.ARC_SPAN
            v = self.val_min + t * (self.val_max - self.val_min)
            col = tuple(c // 3 for c in self._color(v))
            pt = (int(cx + r * math.cos(a)), int(cy + r * math.sin(a)))
            if prev_pt:
                pygame.draw.line(surf, col, prev_pt, pt, THICK)
            prev_pt = pt

        # ── Bright active fill arc (start → current needle position) ──────────
        active_steps = max(0, int(STEPS * (self._v - self.val_min) / (self.val_max - self.val_min)))
        prev_pt = None
        for i in range(active_steps + 1):
            t = i / STEPS
            a = self.ARC_START + t * self.ARC_SPAN
            v = self.val_min + t * (self.val_max - self.val_min)
            pt = (int(cx + r * math.cos(a)), int(cy + r * math.sin(a)))
            if prev_pt:
                pygame.draw.line(surf, self._color(v), prev_pt, pt, THICK)
            prev_pt = pt

        # ── Safe-zone tick marks ───────────────────────────────────────────────
        for mv in (self.safe_lo, self.safe_hi):
            a = self._angle(mv)
            x0 = int(cx + (r - THICK)     * math.cos(a))
            y0 = int(cy + (r - THICK)     * math.sin(a))
            x1 = int(cx + (r + THICK + 4) * math.cos(a))
            y1 = int(cy + (r + THICK + 4) * math.sin(a))
            pygame.draw.line(surf, WHITE, (x0, y0), (x1, y1), 3)

        # ── Tapered speedometer needle ─────────────────────────────────────────
        na  = self._angle(self._v)
        tip = (int(cx + (r - 8) * math.cos(na)), int(cy + (r - 8) * math.sin(na)))
        perp = na + math.pi / 2
        bw  = 7
        b1  = (int(cx + bw * math.cos(perp)), int(cy + bw * math.sin(perp)))
        b2  = (int(cx - bw * math.cos(perp)), int(cy - bw * math.sin(perp)))
        sh  = 3
        pygame.draw.polygon(surf, (0, 0, 0),
                            [(b1[0]+sh, b1[1]+sh), (b2[0]+sh, b2[1]+sh), (tip[0]+sh, tip[1]+sh)])
        pygame.draw.polygon(surf, WHITE, [b1, b2, tip])

        # ── Hub ────────────────────────────────────────────────────────────────
        pygame.draw.circle(surf, DARK_GRAY,          (cx, cy), 14)
        pygame.draw.circle(surf, self._color(self._v),(cx, cy),  8)
        pygame.draw.circle(surf, WHITE,               (cx, cy),  4)

        # ── Value + labels ─────────────────────────────────────────────────────
        vc = self._color(self._v)
        surf.blit(font_val.render(f"{self._v:.2f}", True, vc),
                  font_val.render(f"{self._v:.2f}", True, vc)
                  .get_rect(centerx=cx, centery=cy + r // 3))
        surf.blit(font_lbl.render(self.unit, True, MID_GRAY),
                  font_lbl.render(self.unit, True, MID_GRAY)
                  .get_rect(centerx=cx, centery=cy + r // 3 + 40))
        surf.blit(font_lbl.render(self.label, True, TEXT),
                  font_lbl.render(self.label, True, TEXT)
                  .get_rect(centerx=cx, centery=cy - r - 24))


# ── Slider / Button (unchanged) ───────────────────────────────────────────────
class Slider:
    def __init__(self, x, y, w, lo, hi, label):
        self.track = pygame.Rect(x, y, w, 16)
        self.lo, self.hi, self.label = lo, hi, label
        self.value = (lo + hi) / 2; self.dragging = False

    def handle_event(self, e):
        if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            hx = int(self.track.x + (self.value - self.lo) / (self.hi - self.lo) * self.track.w)
            if math.dist(e.pos, (hx, self.track.centery)) <= 14: self.dragging = True
        elif e.type == pygame.MOUSEBUTTONUP and e.button == 1: self.dragging = False
        elif e.type == pygame.MOUSEMOTION and self.dragging:
            f = (e.pos[0] - self.track.x) / self.track.w
            self.value = self.lo + max(0.0, min(1.0, f)) * (self.hi - self.lo)

    def draw(self, surf, font):
        pygame.draw.rect(surf, MID_GRAY, self.track, border_radius=8)
        hx = int(self.track.x + (self.value - self.lo) / (self.hi - self.lo) * self.track.w)
        pygame.draw.circle(surf, (200, 200, 215), (hx, self.track.centery), 14)
        surf.blit(font.render(f"{self.label}: {self.value:.2f}", True, TEXT),
                  (self.track.x, self.track.y - 26))


class ToggleSwitch:
    """Animated slide toggle — left = MANUAL MODE, right = AI / DIGITALISATION."""
    TRACK_W = 220
    TRACK_H = 54
    THUMB_R = 24

    def __init__(self, cx: int, cy: int):
        self.cx, self.cy = cx, cy
        self.active = False   # False = Manual, True = AI
        self._anim  = 0.0    # smooth interpolation 0 → 1
        self.track  = pygame.Rect(cx - self.TRACK_W // 2,
                                   cy - self.TRACK_H // 2,
                                   self.TRACK_W, self.TRACK_H)

    def handle_event(self, e: pygame.event.Event) -> bool:
        if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            if self.track.inflate(80, 24).collidepoint(e.pos):
                self.active = not self.active
                return True
        return False

    def draw(self, surf: pygame.Surface, font: pygame.font.Font) -> None:
        # Animate thumb position
        target = 1.0 if self.active else 0.0
        self._anim += (target - self._anim) * 0.18

        t  = self._anim
        tw, th = self.TRACK_W, self.TRACK_H
        cx, cy = self.cx, self.cy

        # Track colour: dark-red (manual) → blue (AI)
        track_col = (
            int(180 * (1 - t) + BLUE[0] * t),
            int(30  * (1 - t) + BLUE[1] * t),
            int(30  * (1 - t) + BLUE[2] * t),
        )
        pygame.draw.rect(surf, track_col, self.track, border_radius=th // 2)
        pygame.draw.rect(surf, MID_GRAY,  self.track, 2, border_radius=th // 2)

        # Thumb (sliding circle)
        margin  = self.THUMB_R + 3
        thumb_x = int(cx - tw // 2 + margin + t * (tw - 2 * margin))
        pygame.draw.circle(surf, WHITE,    (thumb_x, cy), self.THUMB_R)
        pygame.draw.circle(surf, MID_GRAY, (thumb_x, cy), self.THUMB_R, 2)

        # Side labels
        lm = font.render("MANUAL MODE", True, RED if not self.active else MID_GRAY)
        la = font.render("AI / DIGITALISATION", True, BLUE if self.active else MID_GRAY)
        surf.blit(lm, lm.get_rect(right=self.track.left - 14, centery=cy))
        surf.blit(la, la.get_rect(left=self.track.right  + 14, centery=cy))


# ── Data-flow / Digital Twin centre panel ─────────────────────────────────────
class FlowPanel:
    """Draws the centre panel: raw sensor readings → (Direct | Digital Twin) → output."""

    def __init__(self):
        # Manual mode: packets that slow and queue in the dead-time delay box
        self.manual_pts = [{'p': i / 8, 'speed': random.uniform(0.003, 0.005)}
                           for i in range(8)]
        self._burst_timer = 0
        # AI mode: fast, evenly-spaced glowing packets
        self.ai_pts = [{'p': i / 6, 'speed': random.uniform(0.010, 0.015)}
                       for i in range(6)]
        self._tick = 0

    def _draw_box(self, surf, rect, title, lines, color, font_t, font_b, pulse=0.0):
        """Draw a labelled rounded box. pulse=0-1 for animated glow border."""
        pygame.draw.rect(surf, PANEL_MID, rect, border_radius=12)
        border_col = tuple(min(255, int(c + pulse * 80)) for c in color)
        pygame.draw.rect(surf, border_col, rect, 2, border_radius=12)
        surf.blit(font_t.render(title, True, color),
                  font_t.render(title, True, color).get_rect(centerx=rect.centerx,
                                                             centery=rect.y + 20))
        for i, ln in enumerate(lines):
            surf.blit(font_b.render(ln, True, TEXT),
                      font_b.render(ln, True, TEXT).get_rect(centerx=rect.centerx,
                                                                  centery=rect.y + 44 + i * 22))

    def _pipe(self, surf, x, y0, y1, color):
        """Styled vertical pipe connector with arrowhead."""
        pw = 5
        pygame.draw.rect(surf, (42, 48, 75), (x - pw, y0, pw * 2, y1 - y0 - 12))
        pygame.draw.line(surf, MID_GRAY, (x - pw, y0), (x - pw, y1 - 12), 1)
        pygame.draw.line(surf, MID_GRAY, (x + pw, y0), (x + pw, y1 - 12), 1)
        pygame.draw.polygon(surf, color, [(x, y1), (x - 9, y1 - 15), (x + 9, y1 - 15)])

    def draw(self, surf, ai_mode: bool, raw: ADState, out: ADState,
             font_title, font_body, font_val, tick: int):
        self._tick = tick
        cx = FLOW_CX
        pulse = (math.sin(tick * 0.08) + 1) / 2   # 0-1 oscillation

        # ── Sensor readings box ────────────────────────────────────────────────
        box1 = pygame.Rect(cx - 170, MAIN_TOP + 20, 340, 78)
        self._draw_box(surf, box1, "SENSOR READINGS",
                       [f"T = {raw.temperature:.1f} °C       pH = {raw.ph:.2f}"],
                       AMBER, font_title, font_body)

        # Pipe: sensor → processing block
        self._pipe(surf, cx, box1.bottom, box1.bottom + 40, MID_GRAY)

        if not ai_mode:
            # ── MANUAL: Operator illustration ─────────────────────────────────
            box2 = pygame.Rect(cx - 170, box1.bottom + 40, 340, 210)
            pygame.draw.rect(surf, PANEL_MID, box2, border_radius=12)
            pygame.draw.rect(surf, RED, box2, 2, border_radius=12)
            surf.blit(font_title.render("OPERATOR CONTROL", True, RED),
                      font_title.render("OPERATOR CONTROL", True, RED)
                      .get_rect(centerx=box2.centerx, centery=box2.y + 20))

            # ── Draw operator person with hard hat ─────────────────────────────
            ox, oy = box2.centerx, box2.y + 118

            # Hard hat brim (yellow flat ellipse)
            pygame.draw.ellipse(surf, (240, 200, 20), (ox - 28, oy - 60, 56, 12))
            # Hard hat dome
            pygame.draw.ellipse(surf, (240, 200, 20), (ox - 20, oy - 72, 40, 26))
            # Hat strap shadow
            pygame.draw.line(surf, (180, 145, 10), (ox - 28, oy - 55), (ox + 28, oy - 55), 2)

            # Head
            pygame.draw.circle(surf, (220, 175, 130), (ox, oy - 42), 16)
            # Eyes
            pygame.draw.circle(surf, (40, 40, 80), (ox - 5, oy - 44), 2)
            pygame.draw.circle(surf, (40, 40, 80), (ox + 5, oy - 44), 2)
            # Smile
            pygame.draw.arc(surf, (160, 100, 60),
                            pygame.Rect(ox - 6, oy - 40, 12, 7), math.pi, 2 * math.pi, 2)

            # Hi-vis vest body (orange)
            body_pts = [(ox - 22, oy - 26), (ox + 22, oy - 26),
                        (ox + 18, oy + 18),  (ox - 18, oy + 18)]
            pygame.draw.polygon(surf, (220, 100, 20), body_pts)
            # Vest reflective stripes (yellow horizontal)
            for sy in (oy - 10, oy + 4):
                pygame.draw.line(surf, (240, 230, 30), (ox - 20, sy), (ox + 20, sy), 3)

            # Left arm
            pygame.draw.line(surf, (220, 100, 20), (ox - 22, oy - 22), (ox - 34, oy + 8), 5)
            pygame.draw.line(surf, (220, 175, 130), (ox - 34, oy + 8), (ox - 38, oy + 20), 4)
            # Right arm — holding clipboard
            pygame.draw.line(surf, (220, 100, 20), (ox + 22, oy - 22), (ox + 32, oy + 4), 5)
            pygame.draw.line(surf, (220, 175, 130), (ox + 32, oy + 4), (ox + 36, oy + 16), 4)
            # Clipboard
            pygame.draw.rect(surf, (240, 235, 215), (ox + 34, oy + 12, 20, 26), border_radius=2)
            pygame.draw.rect(surf, (160, 150, 130), (ox + 34, oy + 12, 20, 26), 1, border_radius=2)
            pygame.draw.rect(surf, (100, 100, 180), (ox + 36, oy + 10, 16, 5), border_radius=2)
            for ly in (oy + 20, oy + 26, oy + 32):
                pygame.draw.line(surf, (160, 150, 130), (ox + 37, ly), (ox + 51, ly), 1)

            # Legs
            pygame.draw.line(surf, (60, 80, 130), (ox - 10, oy + 18), (ox - 12, oy + 44), 6)
            pygame.draw.line(surf, (60, 80, 130), (ox + 10, oy + 18), (ox + 12, oy + 44), 6)
            # Boots
            pygame.draw.rect(surf, (40, 40, 40), (ox - 18, oy + 42, 14, 8), border_radius=3)
            pygame.draw.rect(surf, (40, 40, 40), (ox + 5,  oy + 42, 14, 8), border_radius=3)

            # Sub-labels
            for i, txt in enumerate(["Manual input", "2.5 s delay", "No AI filtering"]):
                surf.blit(font_body.render(txt, True, MID_GRAY),
                          font_body.render(txt, True, MID_GRAY)
                          .get_rect(centerx=box2.centerx, centery=box2.bottom - 52 + i * 18))

            # Slow-packet animation (delay queue)
            self._burst_timer = (self._burst_timer + 1) % 190
            pipe_len = (box2.bottom + 40 + 78) - box1.bottom
            for p in self.manual_pts:
                abs_y = box1.bottom + p['p'] * pipe_len
                in_delay = box2.top < abs_y < box2.bottom
                if in_delay:
                    p['speed'] = max(0.0008, p['speed'] * 0.97)
                    if self._burst_timer < 12:
                        p['speed'] = 0.012
                else:
                    p['speed'] = min(0.005, p['speed'] * 1.02)
                p['p'] = (p['p'] + p['speed']) % 1.0
                py = int(box1.bottom + p['p'] * pipe_len)
                in_box = (box1.top < py < box1.bottom) or (box2.top + 2 < py < box2.bottom - 2)
                if not in_box:
                    bright = min(255, 160 + int(p['speed'] * 15000))
                    pygame.draw.circle(surf, (bright, 50, 50), (cx, py), 5)

            self._pipe(surf, cx, box2.bottom, box2.bottom + 40, MID_GRAY)
            box3 = pygame.Rect(cx - 170, box2.bottom + 40, 340, 78)
            self._draw_box(surf, box3, "GAUGE OUTPUT (DELAYED)",
                           [f"T = {out.temperature:.1f} °C       pH = {out.ph:.2f}"],
                           RED, font_title, font_body)

        else:
            # ── AI: Digital Twin Process diagram ──────────────────────────────
            box2 = pygame.Rect(cx - 180, box1.bottom + 40, 360, 290)
            pygame.draw.rect(surf, PANEL_MID, box2, border_radius=12)
            bc = tuple(min(255, int(c + pulse * 80)) for c in BLUE)
            pygame.draw.rect(surf, bc, box2, 2, border_radius=12)

            # Title
            surf.blit(font_title.render("THE DIGITAL TWIN PROCESS", True, BLUE),
                      font_title.render("THE DIGITAL TWIN PROCESS", True, BLUE)
                      .get_rect(centerx=box2.centerx, centery=box2.y + 18))

            # ── Monitor shell ──────────────────────────────────────────────────
            mon_w, mon_h = 196, 128
            mon_x = box2.centerx - mon_w // 2
            mon_y = box2.y + 34
            pygame.draw.rect(surf, (50, 62, 100), (mon_x, mon_y, mon_w, mon_h), border_radius=8)
            scr = pygame.Rect(mon_x + 5, mon_y + 5, mon_w - 10, mon_h - 10)
            pygame.draw.rect(surf, (18, 24, 50), scr, border_radius=5)
            # Stand + base
            pygame.draw.rect(surf, (70, 82, 120), (box2.centerx - 14, mon_y + mon_h, 28, 11), border_radius=3)
            pygame.draw.rect(surf, (70, 82, 120), (box2.centerx - 36, mon_y + mon_h + 11, 72, 7), border_radius=3)
            pygame.draw.rect(surf, (110, 130, 190), (mon_x, mon_y, mon_w, mon_h), 2, border_radius=8)

            # ── Left screen half: Virtual AD Reactor ──────────────────────────
            lr = pygame.Rect(scr.x + 2, scr.y + 2, scr.w // 2 - 3, scr.h - 4)
            pygame.draw.rect(surf, (50, 60, 98), lr, border_radius=4)
            liq = pygame.Rect(lr.x + 3, lr.y + 3, lr.w - 6, lr.h - 6)
            liq_col = (int(70 + 0.7 * 120), int(60 + 0.7 * 100), int(20 + 0.7 * 30))
            pygame.draw.rect(surf, liq_col, liq, border_radius=3)
            for i in range(5):
                bx2 = liq.x + 5 + i * (liq.w // 5)
                by2 = int(liq.y + 5 + liq.h * (0.15 + 0.7 * ((tick * 0.025 + i * 0.55) % 1.0)))
                by2 = max(liq.y + 3, min(liq.bottom - 3, by2))
                pygame.draw.circle(surf, (180, 220, 255), (bx2, by2), 2)
            pygame.draw.rect(surf, (90, 110, 180), lr, 1, border_radius=4)
            ll = font_body.render("VIRTUAL AD", True, (200, 225, 255))
            surf.blit(ll, ll.get_rect(centerx=lr.centerx, bottom=lr.bottom - 1))

            # ── Right screen half: Biochemical Network ─────────────────────────
            rr = pygame.Rect(scr.x + scr.w // 2 + 1, scr.y + 2, scr.w // 2 - 3, scr.h - 4)
            pygame.draw.rect(surf, (22, 30, 58), rr, border_radius=4)
            ncx2, ncy2 = rr.centerx, rr.y + rr.h // 2 - 6
            nr2 = 20
            node_cols = [(0, 180, 80), (200, 120, 30), (60, 140, 220), (200, 60, 60)]
            nangles = [math.radians(a + tick * 0.5) for a in (0, 90, 180, 270)]
            npos = [(int(ncx2 + nr2 * math.cos(a)), int(ncy2 + nr2 * math.sin(a))) for a in nangles]
            for i in range(4):
                pygame.draw.line(surf, (70, 95, 140), npos[i], npos[(i + 1) % 4], 1)
                pygame.draw.line(surf, (70, 95, 140), npos[i], (ncx2, ncy2), 1)
            for i, (nx2, ny2) in enumerate(npos):
                pygame.draw.circle(surf, node_cols[i], (nx2, ny2), 4)
            pygame.draw.circle(surf, (120, 180, 240), (ncx2, ncy2), 5)
            pygame.draw.rect(surf, (90, 110, 180), rr, 1, border_radius=4)
            rl = font_body.render("BIO NETWORK", True, (200, 225, 255))
            surf.blit(rl, rl.get_rect(centerx=rr.centerx, bottom=rr.bottom - 1))

            # ── Side labels ────────────────────────────────────────────────────
            lbl_left  = [("Hybrid", "AI Model"), ("Real-time", "State Est.")]
            lbl_right = [("Predictive", "Control"), ("Shock-load", "Comp.")]
            lbw = 66
            for i, (l1, l2) in enumerate(lbl_left):
                ly = mon_y + 16 + i * 58
                lbx = box2.left + 4
                pygame.draw.rect(surf, PANEL_MID, (lbx, ly - 13, lbw, 34), border_radius=5)
                pygame.draw.rect(surf, BLUE,      (lbx, ly - 13, lbw, 34), 1, border_radius=5)
                surf.blit(font_body.render(l1, True, TEXT), (lbx + 4, ly - 10))
                surf.blit(font_body.render(l2, True, TEXT), (lbx + 4, ly + 6))
                ax = lbx + lbw; ay = ly + 4
                pygame.draw.line(surf, MID_GRAY, (ax, ay), (mon_x - 3, ay), 1)
                pygame.draw.polygon(surf, MID_GRAY,
                                    [(mon_x, ay), (mon_x - 8, ay - 4), (mon_x - 8, ay + 4)])
            for i, (l1, l2) in enumerate(lbl_right):
                ly = mon_y + 16 + i * 58
                lbx = box2.right - 4 - lbw
                pygame.draw.rect(surf, PANEL_MID, (lbx, ly - 13, lbw, 34), border_radius=5)
                pygame.draw.rect(surf, BLUE,      (lbx, ly - 13, lbw, 34), 1, border_radius=5)
                surf.blit(font_body.render(l1, True, TEXT), (lbx + 4, ly - 10))
                surf.blit(font_body.render(l2, True, TEXT), (lbx + 4, ly + 6))
                ax = lbx; ay = ly + 4
                ex = mon_x + mon_w + 3
                pygame.draw.line(surf, MID_GRAY, (ex, ay), (ax - 3, ay), 1)
                pygame.draw.polygon(surf, MID_GRAY,
                                    [(ax, ay), (ax - 8, ay - 4), (ax - 8, ay + 4)])

            # Sub-labels below monitor
            for i, txt in enumerate(["Hybrid AI ·  Real-time estimation",
                                      "Predictive control ·  OLR compensation"]):
                surf.blit(font_body.render(txt, True, MID_GRAY),
                          font_body.render(txt, True, MID_GRAY)
                          .get_rect(centerx=box2.centerx, centery=box2.bottom - 36 + i * 18))

            # Fast glowing AI packets
            pipe_len = (box2.bottom + 40 + 78) - box1.bottom
            for p in self.ai_pts:
                p['p'] = (p['p'] + p['speed']) % 1.0
                py = int(box1.bottom + p['p'] * pipe_len)
                in_box = (box1.top < py < box1.bottom) or (box2.top + 2 < py < box2.bottom - 2)
                if not in_box:
                    gs = pygame.Surface((18, 18), pygame.SRCALPHA)
                    pygame.draw.circle(gs, (30, 120, 255, 90), (9, 9), 8)
                    surf.blit(gs, (cx - 9, py - 9))
                    pygame.draw.circle(surf, (160, 215, 255), (cx, py), 4)

            self._pipe(surf, cx, box2.bottom, box2.bottom + 40, MID_GRAY)
            box3 = pygame.Rect(cx - 170, box2.bottom + 40, 340, 78)
            self._draw_box(surf, box3, "OPTIMISED OUTPUT",
                           [f"T = {out.temperature:.1f} °C       pH = {out.ph:.2f}"],
                           BLUE, font_title, font_body)

        # ── Horizontal arrows to/from side panels ─────────────────────────────
        arr_y = box3.centery  # type: ignore[possibly-undefined]
        ax0 = box3.right; ax1 = RIGHT_X - 4
        pygame.draw.line(surf, MID_GRAY, (ax0, arr_y), (ax1 - 12, arr_y), 2)
        pygame.draw.polygon(surf, MID_GRAY,
                            [(ax1, arr_y), (ax1 - 12, arr_y - 7), (ax1 - 12, arr_y + 7)])

        lax0 = LEFT_W + 4; lax1 = box1.left
        pygame.draw.line(surf, MID_GRAY, (lax0 + 12, arr_y), (lax1, arr_y), 2)
        pygame.draw.polygon(surf, MID_GRAY,
                            [(lax0, arr_y), (lax0 + 12, arr_y - 7), (lax0 + 12, arr_y + 7)])


# ── Biogas time-series chart ──────────────────────────────────────────────────
class BiogasChart:
    """Rolling line chart for biogas flow rate linked to reactor health."""
    HISTORY    = 300          # frames of rolling history (~5 s at 60 fps)
    FLOW_MIN   = 0.0
    FLOW_MAX   = 10.0
    THRESH_LO  = 3.5          # green band
    THRESH_HI  = 6.5
    WARN_LO    = 2.0          # amber band
    WARN_HI    = 8.0

    def __init__(self, x: int, y: int, w: int, h: int):
        self.rect = pygame.Rect(x, y, w, h)
        self._history: collections.deque = collections.deque(
            [5.0] * self.HISTORY, maxlen=self.HISTORY)
        self._current = 5.0

    def update(self, value: float) -> None:
        self._current = value
        self._history.append(value)

    def draw(self, surf: pygame.Surface, font_lbl: pygame.font.Font,
             font_val: pygame.font.Font) -> None:
        rx, ry, rw, rh = self.rect.x, self.rect.y, self.rect.w, self.rect.h
        PAD_L, PAD_R, PAD_T, PAD_B = 38, 10, 26, 24

        # Background
        pygame.draw.rect(surf, PANEL_MID, self.rect, border_radius=10)
        pygame.draw.rect(surf, DARK_GRAY, self.rect, 1, border_radius=10)

        # Plot area
        px, py = rx + PAD_L, ry + PAD_T
        pw, ph = rw - PAD_L - PAD_R, rh - PAD_T - PAD_B

        def vy(v: float) -> int:
            return py + ph - int((v - self.FLOW_MIN) / (self.FLOW_MAX - self.FLOW_MIN) * ph)

        def vx(i: int) -> int:
            return px + int(i / (self.HISTORY - 1) * pw)

        # Green threshold band (filled)
        t_hi_y, t_lo_y = vy(self.THRESH_HI), vy(self.THRESH_LO)
        band = pygame.Surface((pw, t_lo_y - t_hi_y), pygame.SRCALPHA)
        band.fill((0, 170, 65, 55))
        surf.blit(band, (px, t_hi_y))

        # Threshold lines
        pygame.draw.line(surf, GREEN, (px, t_hi_y), (px + pw, t_hi_y), 1)
        pygame.draw.line(surf, GREEN, (px, t_lo_y), (px + pw, t_lo_y), 1)

        # Warn lines (dashed via segments)
        for sy in (vy(self.WARN_LO), vy(self.WARN_HI)):
            for sx in range(px, px + pw, 10):
                pygame.draw.line(surf, AMBER, (sx, sy), (min(sx + 6, px + pw), sy), 1)

        # Axes
        pygame.draw.line(surf, MID_GRAY, (px, py), (px, py + ph), 2)           # Y
        pygame.draw.line(surf, MID_GRAY, (px, py + ph), (px + pw, py + ph), 2) # X

        # Y-axis ticks + labels
        for v in (0, 2, 4, 6, 8, 10):
            ty = vy(v)
            pygame.draw.line(surf, MID_GRAY, (px - 4, ty), (px, ty), 1)
            lbl = font_lbl.render(str(v), True, MID_GRAY)
            surf.blit(lbl, lbl.get_rect(right=px - 5, centery=ty))

        # X-axis label
        xl = font_lbl.render("Time \u2192", True, MID_GRAY)
        surf.blit(xl, xl.get_rect(right=px + pw, top=py + ph + 3))

        # Y-axis unit
        ul = font_lbl.render("m\u00b3/d", True, MID_GRAY)
        surf.blit(ul, (rx + 2, py))

        # Data line — colour-coded by zone
        pts = list(self._history)
        prev = None
        for i, v in enumerate(pts):
            pt = (vx(i), vy(v))
            if prev:
                if self.THRESH_LO <= v <= self.THRESH_HI:  col = GREEN
                elif self.WARN_LO  <= v <= self.WARN_HI:   col = AMBER
                else:                                       col = RED
                pygame.draw.line(surf, col, prev, pt, 2)
            prev = pt

        # Live value + title
        cv = self._current
        if self.THRESH_LO <= cv <= self.THRESH_HI:   vc = GREEN
        elif self.WARN_LO  <= cv <= self.WARN_HI:    vc = AMBER
        else:                                         vc = RED

        title = font_lbl.render("BIOGAS FLOW RATE", True, TEXT)
        surf.blit(title, title.get_rect(centerx=rx + rw // 2, centery=ry + PAD_T // 2 + 4))

        vt = font_val.render(f"{cv:.1f} m\u00b3/day", True, vc)
        surf.blit(vt, vt.get_rect(right=rx + rw - 4, centery=ry + PAD_T // 2 + 4))


# ── Main dashboard ────────────────────────────────────────────────────────────
class Dashboard:
    def __init__(self, port: Optional[str], demo: bool):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.SCALED | pygame.RESIZABLE)
        pygame.display.set_caption("AD Reactor Control Challenge — Digital Twin")

        self.font_xl  = pygame.font.SysFont("Arial", 48, bold=True)
        self.font_lg  = pygame.font.SysFont("Arial", 32, bold=True)
        self.font_md  = pygame.font.SysFont("Arial", 22)
        self.font_sm  = pygame.font.SysFont("Arial", 17)
        self.clock    = pygame.time.Clock()
        self.tick     = 0

        # Core visualisation objects
        self.reactor = ReactorViz(REACTOR_CX, REACTOR_CY, 200, 265)
        self.flow    = FlowPanel()
        self.temp_gauge = ArcGauge(TEMP_GX, GAUGE_GY, GAUGE_R,
                                   TEMP_MIN, TEMP_MAX, TEMP_SAFE_LO, TEMP_SAFE_HI,
                                   TEMP_WARN_LO, TEMP_WARN_HI, "TEMPERATURE", "°C")
        self.ph_gauge   = ArcGauge(PH_GX,   GAUGE_GY, GAUGE_R,
                                   PH_MIN,   PH_MAX,   PH_SAFE_LO,   PH_SAFE_HI,
                                   PH_WARN_LO,   PH_WARN_HI,   "pH LEVEL",    "pH")
        # Biogas time-series chart (below the two arc gauges)
        self.biogas_chart = BiogasChart(BIOGAS_CHART_X, BIOGAS_CHART_Y,
                                        BIOGAS_CHART_W, BIOGAS_CHART_H)
        self._biogas_flow = 5.0   # smoothed internal value

        # Fallback controls
        self.mode_switch = ToggleSwitch(SCREEN_W//2, SCREEN_H - 26)
        self.temp_slider = Slider(TEMP_GX - 100, SLIDER_Y, 200, TEMP_MIN, TEMP_MAX, "Temp °C")
        self.ph_slider   = Slider(PH_GX   - 100, SLIDER_Y, 200, PH_MIN,   PH_MAX,   "pH")
        self.temp_slider.value = 37.5; self.ph_slider.value = 7.0

        # State
        self.raw_queue: collections.deque[IMUPacket] = collections.deque(maxlen=200)
        self.ad_state  = ADState()
        self.raw_state = ADState()
        self.ai_mode   = False
        self.demo      = demo or (port is None)
        self.show_sliders = self.demo
        self.warn_tick = 0
        self.processor = DataProcessor()

        self.reader: Optional[SerialReader] = None
        if not self.demo and port:
            self.reader = SerialReader(port, self.raw_queue)
            self.reader.start()

    def _effective_ai(self, pkt: Optional[IMUPacket]) -> bool:
        if self.mode_switch.active: return True
        if pkt is not None:         return bool(pkt.switch == 1)
        return False

    def update(self) -> None:
        self.tick += 1
        pkt: Optional[IMUPacket] = self.raw_queue[-1] if self.raw_queue else None

        if self.demo or pkt is None:
            self.ai_mode   = self._effective_ai(None)
            self.raw_state = ADState(self.temp_slider.value, self.ph_slider.value)
            self.ad_state  = self.raw_state
        else:
            # Sync on-screen toggle to match physical hardware switch (visual confirmation)
            self.mode_switch.active = bool(pkt.switch == 1)
            self.ai_mode   = self._effective_ai(pkt)
            self.raw_state = ADState(
                temperature=_imu_map(pkt.pitch, -90.0, 90.0, TEMP_MIN, TEMP_MAX),
                ph=         _imu_map(pkt.roll,  -90.0, 90.0, PH_MIN,   PH_MAX),
            )
            self.ad_state  = (self.processor.process_ai(pkt)
                              if self.ai_mode
                              else self.processor.process_manual(pkt))

        self.reactor.update(self.ad_state)
        self.temp_gauge.update(self.ad_state.temperature)
        self.ph_gauge.update(self.ad_state.ph)
        # Biogas flow tracks reactor health: max 8 m³/day; stays in green when T+pH are healthy
        target_flow = self.reactor.health * 8.0 + random.gauss(0, 0.04)
        self._biogas_flow += (target_flow - self._biogas_flow) * 0.04
        self.biogas_chart.update(self._biogas_flow)
        self.warn_tick = (self.warn_tick + 1) % FPS

    def _stable(self):
        return (PH_SAFE_LO <= self.ad_state.ph <= PH_SAFE_HI and
                TEMP_SAFE_LO <= self.ad_state.temperature <= TEMP_SAFE_HI)

    def _critical(self):
        return (self.ad_state.ph < PH_WARN_LO or self.ad_state.ph > PH_WARN_HI or
                self.ad_state.temperature < TEMP_WARN_LO or self.ad_state.temperature > TEMP_WARN_HI)

    def draw(self):
        self.screen.fill(BLACK)

        # ── Panel backgrounds ──────────────────────────────────────────────────
        pygame.draw.rect(self.screen, PANEL_BG,
                         (0, MAIN_TOP, LEFT_W, MAIN_BOT - MAIN_TOP), border_radius=0)
        pygame.draw.rect(self.screen, PANEL_BG,
                         (RIGHT_X, MAIN_TOP, SCREEN_W - RIGHT_X, MAIN_BOT - MAIN_TOP))
        # Dividers
        pygame.draw.line(self.screen, DARK_GRAY, (LEFT_W, MAIN_TOP), (LEFT_W, MAIN_BOT), 1)
        pygame.draw.line(self.screen, DARK_GRAY, (RIGHT_X, MAIN_TOP), (RIGHT_X, MAIN_BOT), 1)

        # ── Header ────────────────────────────────────────────────────────────
        pygame.draw.rect(self.screen, PANEL_BG, (0, 0, SCREEN_W, HEADER_H))
        self.screen.blit(
            self.font_lg.render("AD REACTOR CONTROL CHALLENGE — DIGITAL TWIN", True, TEXT),
            self.font_lg.render("AD REACTOR CONTROL CHALLENGE — DIGITAL TWIN", True, TEXT)
            .get_rect(centerx=SCREEN_W//2, centery=HEADER_H//2))

        # ── Mode banner ───────────────────────────────────────────────────────
        if self.ai_mode:
            bc, bt = BLUE, "AI PREDICTIVE MODE — Digital Twin Active (Optimised)"
        else:
            bc, bt = RED, "MANUAL MODE — Direct Control (High Latency, No AI)"
        pygame.draw.rect(self.screen, bc, (0, HEADER_H, SCREEN_W, BANNER_H))
        self.screen.blit(self.font_md.render(bt, True, (240, 242, 255)),
                         self.font_md.render(bt, True, (240, 242, 255))
                         .get_rect(centerx=SCREEN_W//2, centery=HEADER_H + BANNER_H//2))

        # ── Right panel title ─────────────────────────────────────────────────
        label = "DIGITAL TWIN OUTPUT" if self.ai_mode else "DIRECT READOUT"
        self.screen.blit(self.font_sm.render(label, True, BLUE if self.ai_mode else RED),
                         self.font_sm.render(label, True, BLUE if self.ai_mode else RED)
                         .get_rect(centerx=(RIGHT_X + SCREEN_W)//2, centery=MAIN_TOP + 16))
        # ── Reactor (left panel) ───────────────────────────────────────────────
        self.reactor.draw(self.screen, self.font_sm, self.font_lg)

        # ── Data flow (centre panel) ───────────────────────────────────────────
        self.flow.draw(self.screen, self.ai_mode, self.raw_state, self.ad_state,
                       self.font_sm, self.font_sm, self.font_md, self.tick)

        # ── Gauges (right panel) ───────────────────────────────────────────────
        self.temp_gauge.draw(self.screen, self.font_lg, self.font_md)
        self.ph_gauge.draw(self.screen,   self.font_lg, self.font_md)
        self.biogas_chart.draw(self.screen, self.font_sm, self.font_md)

        # ── Warnings ──────────────────────────────────────────────────────────
        flash = self.warn_tick < FPS // 2
        warn_y = MAIN_BOT - 18
        if self._critical() and flash:
            msgs = []
            if self.ad_state.ph < PH_WARN_LO:       msgs.append("SYSTEM SOURING — BACTERIA DYING")
            elif self.ad_state.ph > PH_WARN_HI:     msgs.append("ALKALINITY SPIKE — INHIBITION")
            if self.ad_state.temperature > TEMP_WARN_HI: msgs.append("THERMAL RUNAWAY — OVERHEATING")
            elif self.ad_state.temperature < TEMP_WARN_LO: msgs.append("TEMP DROP — METHANOGENS SLOWING")
            for i, m in enumerate(msgs):
                w = self.font_lg.render(f"⚠  {m}  ⚠", True, RED)
                self.screen.blit(w, w.get_rect(centerx=SCREEN_W//2, centery=warn_y + 22 + i*38))
        elif self._stable():
            s = self.font_md.render("✓  REACTOR STABLE — All Parameters in Safe Zone", True, GREEN)
            self.screen.blit(s, s.get_rect(centerx=SCREEN_W//2, centery=warn_y + 18))

        # ── Fallback controls ─────────────────────────────────────────────────
        self.mode_switch.draw(self.screen, self.font_sm)
        if self.show_sliders:
            self.temp_slider.draw(self.screen, self.font_sm)
            self.ph_slider.draw(self.screen, self.font_sm)

        # ── Footer ────────────────────────────────────────────────────────────
        serial_ok = self.reader and self.reader.connected
        if self.demo:     ft, fc = "[DEMO MODE]", AMBER
        elif serial_ok:   ft, fc = "[SERIAL: Connected]", GREEN
        else:             ft, fc = "[SERIAL: Reconnecting…]", AMBER
        self.screen.blit(self.font_sm.render(ft, True, fc), (10, SCREEN_H - 20))
        hint = self.font_sm.render("F2: Toggle sliders   ESC: Quit", True, DARK_GRAY)
        self.screen.blit(hint, hint.get_rect(right=SCREEN_W - 10, bottom=SCREEN_H - 4))

        pygame.display.flip()

    def run(self):
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT: break
                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_ESCAPE: break
                    if e.key == pygame.K_F2: self.show_sliders = not self.show_sliders
                self.mode_switch.handle_event(e)
                if self.show_sliders:
                    self.temp_slider.handle_event(e)
                    self.ph_slider.handle_event(e)
            else:
                self.update(); self.draw(); self.clock.tick(FPS)
                continue
            break
        if self.reader: self.reader.stop()
        pygame.quit()


# ── Entry point ───────────────────────────────────────────────────────────────
def _auto_port():
    for p in serial.tools.list_ports.comports():
        if any(k in p.description.upper() for k in ("USB", "SERIAL", "COM")):
            return p.device
    return None

if __name__ == "__main__":
    args  = sys.argv[1:]
    demo  = "--demo" in args
    port  = next((a for a in args if not a.startswith("--")), None)
    if not demo and not port:
        port = _auto_port()
        if port: print(f"Auto-detected: {port}")
        else:    print("No serial port — DEMO MODE"); demo = True

    Dashboard(port, demo).run()
