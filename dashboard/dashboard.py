"""
Industrial Virtual Bioreactor Dashboard

Run:
  python dashboard.py

Controls:
    Space/M/Tab/Enter: toggle Manual / AI mode
  Up/Down: increase/decrease feed load disturbance
  Left/Right: increase/decrease coolant disturbance
  R      : reset disturbances
  Esc    : quit
"""

import math
import random
import collections
from dataclasses import dataclass

import pygame

# Window
WIDTH, HEIGHT = 1520, 900
FPS = 60

# Theme (industrial, neutral)
BG_TOP = (10, 16, 24)
BG_BOTTOM = (24, 33, 44)
PANEL = (30, 41, 54)
PANEL_EDGE = (86, 107, 126)
PANEL_SOFT = (47, 62, 78)
TEXT = (223, 232, 241)
TEXT_DIM = (145, 162, 178)
CYAN = (66, 192, 220)
GREEN = (72, 192, 122)
AMBER = (224, 170, 78)
RED = (214, 96, 96)
PIPE = (82, 101, 118)
PIPE_GLOW = (136, 163, 186)
STEEL = (129, 145, 160)
LIQUID = (96, 140, 102)

# Layout
HEADER_H = 70
LEFT_W = 500
RIGHT_W = 470
CENTER_W = WIDTH - LEFT_W - RIGHT_W

MAIN_Y = HEADER_H + 16
MAIN_H = HEIGHT - MAIN_Y - 16

PROCESS_RECT = pygame.Rect(20, MAIN_Y, LEFT_W - 30, MAIN_H)
CENTER_RECT = pygame.Rect(LEFT_W + 5, MAIN_Y, CENTER_W - 10, MAIN_H)
RIGHT_RECT = pygame.Rect(WIDTH - RIGHT_W + 5, MAIN_Y, RIGHT_W - 25, MAIN_H)
MODE_TOGGLE_RECT = pygame.Rect(WIDTH - 420, 14, 214, 42)

RNG = random.Random(7)


@dataclass
class PlantState:
    temperature: float = 37.2
    ph: float = 7.05
    gas_flow: float = 5.8
    pressure: float = 1.1
    biomass: float = 74.0


class ProcessModel:
    def __init__(self):
        self.state = PlantState()
        self.mode_ai = True
        self.feed_disturbance = 0.0
        self.cool_disturbance = 0.0
        self.energy_index = 78.0
        self.stability = 86.0
        self._manual_delay = collections.deque([self.state], maxlen=70)

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def step(self, dt: float):
        s = self.state
        noise_t = RNG.gauss(0, 0.03)
        noise_p = RNG.gauss(0, 0.006)

        # Base dynamics near ideal operating point.
        target_t = 37.0 + self.feed_disturbance * 0.5 - self.cool_disturbance * 0.9
        target_ph = 7.0 - self.feed_disturbance * 0.08 + self.cool_disturbance * 0.02

        if self.mode_ai:
            gain_t = 1.55
            gain_ph = 1.7
            damping = 0.18
        else:
            gain_t = 0.55
            gain_ph = 0.52
            damping = 0.06

        s.temperature += (target_t - s.temperature) * gain_t * dt + noise_t
        s.ph += (target_ph - s.ph) * gain_ph * dt + noise_p

        # Secondary derived signals.
        healthy_t = math.exp(-((s.temperature - 37.0) ** 2) / 4.8)
        healthy_ph = math.exp(-((s.ph - 7.0) ** 2) / 0.08)
        health = healthy_t * healthy_ph

        gas_target = 2.2 + 4.5 * health + self.feed_disturbance * 0.35
        s.gas_flow += (gas_target - s.gas_flow) * (0.75 + damping) * dt + RNG.gauss(0, 0.015)

        press_target = 0.85 + 0.11 * s.gas_flow + 0.15 * max(0.0, self.feed_disturbance)
        s.pressure += (press_target - s.pressure) * 0.85 * dt + RNG.gauss(0, 0.007)

        biomass_target = 40.0 + 55.0 * health
        s.biomass += (biomass_target - s.biomass) * 0.22 * dt

        s.temperature = self._clamp(s.temperature, 31.5, 43.5)
        s.ph = self._clamp(s.ph, 5.6, 8.4)
        s.gas_flow = self._clamp(s.gas_flow, 0.4, 9.8)
        s.pressure = self._clamp(s.pressure, 0.5, 2.2)
        s.biomass = self._clamp(s.biomass, 18.0, 96.0)

        # Manual mode includes a visualization delay effect.
        if self.mode_ai:
            self._manual_delay.clear()
            self._manual_delay.extend([PlantState(**vars(s))] * self._manual_delay.maxlen)
            delayed = s
        else:
            self._manual_delay.append(PlantState(**vars(s)))
            delayed = self._manual_delay[0]

        perf = 55.0 + 45.0 * health
        self.energy_index += (perf - self.energy_index) * 0.35 * dt
        st_target = 40.0 + 60.0 * health if self.mode_ai else 28.0 + 52.0 * health
        self.stability += (st_target - self.stability) * 0.28 * dt

        return delayed


class Trend:
    def __init__(self, n=260, value=0.0):
        self.buf = collections.deque([value] * n, maxlen=n)

    def push(self, v):
        self.buf.append(v)

    def values(self):
        return list(self.buf)


def lerp(a, b, t):
    return a + (b - a) * t


def draw_vertical_gradient(surface, rect, c0, c1):
    x, y, w, h = rect
    for i in range(h):
        t = i / max(1, h - 1)
        c = (int(lerp(c0[0], c1[0], t)), int(lerp(c0[1], c1[1], t)), int(lerp(c0[2], c1[2], t)))
        pygame.draw.line(surface, c, (x, y + i), (x + w, y + i))


def draw_soft_shadow(surface, rect, alpha=58, spread=10, radius=12):
    sh = pygame.Surface((rect.w + spread * 2, rect.h + spread * 2), pygame.SRCALPHA)
    pygame.draw.rect(sh, (0, 0, 0, alpha), (spread, spread, rect.w, rect.h), border_radius=radius)
    surface.blit(sh, (rect.x - spread, rect.y - spread))


def draw_panel(surface, rect, title, font_title, font_body):
    draw_soft_shadow(surface, rect, alpha=64, spread=8, radius=12)
    draw_vertical_gradient(surface, rect, (35, 48, 62), (26, 37, 50))
    pygame.draw.rect(surface, PANEL_EDGE, rect, width=2, border_radius=12)

    gloss = pygame.Surface((rect.w - 4, 24), pygame.SRCALPHA)
    pygame.draw.rect(gloss, (255, 255, 255, 14), (0, 0, rect.w - 4, 24), border_radius=8)
    surface.blit(gloss, (rect.x + 2, rect.y + 2))

    t = font_title.render(title, True, TEXT)
    surface.blit(t, (rect.x + 14, rect.y + 10))
    pygame.draw.line(surface, PANEL_SOFT, (rect.x + 12, rect.y + 42), (rect.right - 12, rect.y + 42), 1)
    return pygame.Rect(rect.x + 12, rect.y + 50, rect.w - 24, rect.h - 62)


def zone_color(v, safe_lo, safe_hi, warn_lo, warn_hi):
    if safe_lo <= v <= safe_hi:
        return GREEN
    if warn_lo <= v <= warn_hi:
        return AMBER
    return RED


def draw_metric_card(surface, rect, title, value_text, unit, color, spark_vals, font_title, font_body):
    draw_vertical_gradient(surface, rect, (39, 53, 68), (31, 44, 58))
    pygame.draw.rect(surface, (84, 104, 124), rect, 1, border_radius=10)

    t = font_title.render(title, True, TEXT_DIM)
    surface.blit(t, (rect.x + 10, rect.y + 8))

    v = font_body.render(value_text, True, color)
    surface.blit(v, (rect.x + 10, rect.y + 30))

    u = font_title.render(unit, True, TEXT_DIM)
    surface.blit(u, (rect.right - u.get_width() - 10, rect.y + 34))

    gx, gy, gw, gh = rect.x + 10, rect.bottom - 24, rect.w - 20, 14
    pygame.draw.line(surface, (74, 92, 110), (gx, gy + gh), (gx + gw, gy + gh), 1)

    if len(spark_vals) >= 2:
        lo = min(spark_vals)
        hi = max(spark_vals)
        span = max(1e-6, hi - lo)
        pts = []
        for i, sv in enumerate(spark_vals):
            x = gx + int(i * (gw - 1) / (len(spark_vals) - 1))
            y = gy + gh - int((sv - lo) / span * gh)
            pts.append((x, y))
        fill = pygame.Surface((gw, gh + 1), pygame.SRCALPHA)
        poly = [(0, gh)] + [(p[0] - gx, p[1] - gy) for p in pts] + [(gw - 1, gh)]
        pygame.draw.polygon(fill, (color[0], color[1], color[2], 36), poly)
        surface.blit(fill, (gx, gy))
        pygame.draw.lines(surface, color, False, pts, 2)


def draw_pipe_arrow(surface, p0, p1, color=PIPE_GLOW, width=3):
    pygame.draw.line(surface, (58, 74, 90), p0, p1, width + 8)
    pygame.draw.line(surface, PIPE, p0, p1, width + 4)
    pygame.draw.line(surface, color, p0, p1, width)

    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    mag = max(1e-6, math.hypot(vx, vy))
    ux, uy = vx / mag, vy / mag
    nx, ny = -uy, ux
    tip = p1
    base = (p1[0] - ux * 16, p1[1] - uy * 16)
    l = (base[0] + nx * 8, base[1] + ny * 8)
    r = (base[0] - nx * 8, base[1] - ny * 8)
    pygame.draw.polygon(surface, color, [tip, l, r])


def draw_process_vessel(surface, rect, state: PlantState, tick, font_small):
    shell = pygame.Rect(rect.x + 48, rect.y + 74, rect.w - 96, rect.h - 160)

    # Vessel body and top cap.
    pygame.draw.rect(surface, (54, 69, 85), shell, border_radius=22)
    pygame.draw.rect(surface, (136, 151, 166), shell, width=3, border_radius=22)
    pygame.draw.rect(surface, (48, 63, 79), (shell.x + 16, shell.y - 16, shell.w - 32, 22), border_radius=10)
    pygame.draw.rect(surface, STEEL, (shell.x + 16, shell.y - 16, shell.w - 32, 22), 2, border_radius=10)

    glass = pygame.Surface((shell.w - 20, shell.h - 20), pygame.SRCALPHA)
    pygame.draw.rect(glass, (255, 255, 255, 20), (0, 0, 18, shell.h - 20), border_radius=10)
    pygame.draw.rect(glass, (255, 255, 255, 9), (22, 0, 10, shell.h - 20), border_radius=8)
    surface.blit(glass, (shell.x + 10, shell.y + 10))

    # Liquid fill level from biomass.
    frac = 0.35 + 0.55 * (state.biomass / 100.0)
    liq_h = int(shell.h * frac)
    liq = pygame.Rect(shell.x + 10, shell.bottom - liq_h - 10, shell.w - 20, liq_h)

    liquid_color = (
        int(lerp(68, 122, state.biomass / 100.0)),
        int(lerp(102, 158, state.biomass / 100.0)),
        int(lerp(72, 98, state.biomass / 100.0)),
    )
    pygame.draw.rect(surface, liquid_color, liq, border_radius=16)

    meniscus = pygame.Surface((liq.w, 16), pygame.SRCALPHA)
    pygame.draw.ellipse(meniscus, (224, 245, 225, 60), (0, 0, liq.w, 12))
    surface.blit(meniscus, (liq.x, liq.y + 2))

    # Internal agitator and shaft.
    cx = shell.centerx
    pygame.draw.line(surface, (138, 152, 166), (cx, shell.y + 22), (cx, shell.bottom - 24), 4)
    rot = tick * 0.06
    for off in (-34, 34):
        y = shell.centery + off
        arm = 46
        x1 = cx + math.cos(rot + off * 0.01) * arm
        y1 = y + math.sin(rot + off * 0.01) * arm * 0.35
        x2 = cx - math.cos(rot + off * 0.01) * arm
        y2 = y - math.sin(rot + off * 0.01) * arm * 0.35
        pygame.draw.line(surface, STEEL, (x1, y1), (x2, y2), 6)

    # Bubbles tied to gas flow.
    bubble_n = int(10 + state.gas_flow * 2)
    for i in range(bubble_n):
        phase = (tick * 0.018 + i * 0.41) % 1.0
        bx = liq.x + int((0.12 + (i * 0.19) % 0.76) * liq.w)
        by = liq.bottom - int(phase * max(20, liq.h - 14))
        br = 2 + (i % 3)
        pygame.draw.circle(surface, (197, 226, 214), (bx, by), br, 1)

    # Ports.
    inlet_y = shell.y + shell.h - 86
    outlet_y = shell.y + 64
    pygame.draw.rect(surface, (88, 104, 118), (shell.x - 28, inlet_y - 6, 28, 12), border_radius=4)
    pygame.draw.rect(surface, (88, 104, 118), (shell.right, outlet_y - 6, 28, 12), border_radius=4)

    l1 = font_small.render("Process Vessel", True, TEXT)
    l2 = font_small.render("Mesophilic Digestion", True, TEXT_DIM)
    surface.blit(l1, (rect.x + 14, rect.y + 12))
    surface.blit(l2, (rect.x + 14, rect.y + 34))

    return (shell.x - 28, inlet_y), (shell.right + 28, outlet_y), (shell.centerx, shell.y - 26)


def draw_control_rack(surface, rect, state: PlantState, mode_ai, font_small, font_mono):
    draw_vertical_gradient(surface, rect, (49, 63, 79), (40, 53, 68))
    pygame.draw.rect(surface, (96, 116, 134), rect, 2, border_radius=10)

    title = "Supervisory Control Rack"
    t = font_small.render(title, True, TEXT)
    surface.blit(t, (rect.x + 10, rect.y + 8))

    mode_text = "AI Predictive Control" if mode_ai else "Manual Operator Control"
    mode_col = CYAN if mode_ai else AMBER
    m = font_small.render(mode_text, True, mode_col)
    surface.blit(m, (rect.x + 10, rect.y + 30))

    # Control channels.
    channels = [
        ("Temp Loop", state.temperature, 31.0, 43.0, 37.0),
        ("pH Loop", state.ph, 5.5, 8.5, 7.0),
        ("Gas Backpressure", state.pressure, 0.4, 2.2, 1.1),
    ]
    y = rect.y + 62
    for name, val, lo, hi, sp in channels:
        label = font_small.render(name, True, TEXT_DIM)
        surface.blit(label, (rect.x + 12, y))

        gx = rect.x + 140
        gy = y + 6
        gw = rect.w - 160
        pygame.draw.rect(surface, (52, 67, 82), (gx, gy, gw, 10), border_radius=5)

        pos = int((val - lo) / (hi - lo) * gw)
        spx = int((sp - lo) / (hi - lo) * gw)
        pygame.draw.rect(surface, zone_color(val, sp - (hi - lo) * 0.08, sp + (hi - lo) * 0.08, lo, hi),
                         (gx, gy, max(0, pos), 10), border_radius=5)
        pygame.draw.line(surface, (255, 255, 255), (gx, gy + 1), (gx + max(0, pos), gy + 1), 1)
        pygame.draw.line(surface, TEXT, (gx + spx, gy - 4), (gx + spx, gy + 14), 1)

        value = font_mono.render(f"PV {val:5.2f}", True, TEXT)
        surface.blit(value, (gx + gw - 84, y - 1))
        y += 34


def draw_monitor_screen(surface, rect, state: PlantState, mode_ai, tick, font_small, font_mono):
    # Monitor housing.
    housing = pygame.Rect(rect.x + 18, rect.y + 10, rect.w - 36, rect.h - 36)
    draw_soft_shadow(surface, housing, alpha=72, spread=9, radius=12)
    draw_vertical_gradient(surface, housing, (44, 58, 73), (34, 46, 60))
    pygame.draw.rect(surface, (105, 124, 142), housing, 2, border_radius=12)

    screen = pygame.Rect(housing.x + 16, housing.y + 16, housing.w - 32, housing.h - 58)
    draw_vertical_gradient(surface, screen, (9, 17, 28), (16, 29, 43))
    pygame.draw.rect(surface, (82, 102, 122), screen, 1, border_radius=6)

    # Left: miniature process overview.
    left = pygame.Rect(screen.x + 8, screen.y + 8, screen.w // 2 - 12, screen.h - 16)
    pygame.draw.rect(surface, (18, 36, 52), left, border_radius=6)
    pygame.draw.rect(surface, (67, 96, 115), left, 1, border_radius=6)

    tx = left.x + left.w // 2
    tank = pygame.Rect(tx - 46, left.y + 30, 92, left.h - 52)
    pygame.draw.rect(surface, (60, 76, 92), tank, border_radius=10)
    pygame.draw.rect(surface, (114, 132, 150), tank, 2, border_radius=10)

    level = int((0.35 + 0.55 * (state.biomass / 100.0)) * (tank.h - 10))
    fill = pygame.Rect(tank.x + 5, tank.bottom - level - 5, tank.w - 10, level)
    pygame.draw.rect(surface, (91, 141, 106), fill, border_radius=8)

    # Right: digital twin network.
    right = pygame.Rect(left.right + 8, left.y, screen.right - left.right - 16, left.h)
    pygame.draw.rect(surface, (15, 30, 47), right, border_radius=6)
    pygame.draw.rect(surface, (67, 96, 115), right, 1, border_radius=6)

    cx, cy = right.centerx, right.centery
    orbit = 44
    pts = []
    for i in range(6):
        a = tick * 0.018 + i * (math.tau / 6)
        px = cx + int(math.cos(a) * orbit)
        py = cy + int(math.sin(a) * orbit * 0.72)
        pts.append((px, py))

    for i in range(len(pts)):
        j = (i + 2) % len(pts)
        pygame.draw.line(surface, (72, 119, 152), pts[i], pts[j], 1)

    for p in pts:
        pygame.draw.circle(surface, CYAN if mode_ai else AMBER, p, 4)
    pygame.draw.circle(surface, (210, 228, 245), (cx, cy), 5)

    mode = "AI Twin Active" if mode_ai else "Manual Supervision"
    txt = font_small.render(mode, True, CYAN if mode_ai else AMBER)
    surface.blit(txt, (screen.x + 10, screen.y + 8))

    # Monitor base.
    base_y = housing.bottom - 8
    pygame.draw.rect(surface, (85, 104, 122), (housing.centerx - 24, base_y, 48, 10), border_radius=4)
    pygame.draw.rect(surface, (85, 104, 122), (housing.centerx - 60, base_y + 10, 120, 8), border_radius=4)

    info = [
        f"T {state.temperature:4.1f} degC",
        f"pH {state.ph:4.2f}",
        f"Gas {state.gas_flow:4.2f} m3/h",
        f"P {state.pressure:4.2f} bar",
    ]
    y = housing.bottom - 52
    for line in info:
        t = font_mono.render(line, True, TEXT_DIM)
        surface.blit(t, (housing.x + 16, y))
        y += 14


def draw_trend_plot(surface, rect, series, y_min, y_max, color, title, font_small):
    draw_vertical_gradient(surface, rect, (35, 50, 64), (29, 42, 56))
    pygame.draw.rect(surface, (80, 100, 120), rect, 1, border_radius=8)

    label = font_small.render(title, True, TEXT_DIM)
    surface.blit(label, (rect.x + 8, rect.y + 6))

    plot = pygame.Rect(rect.x + 8, rect.y + 24, rect.w - 16, rect.h - 32)
    draw_vertical_gradient(surface, plot, (14, 25, 38), (12, 21, 32))

    # Guide lines.
    for i in range(1, 4):
        y = plot.y + int(i * plot.h / 4)
        pygame.draw.line(surface, (47, 66, 84), (plot.x + 3, y), (plot.right - 3, y), 1)

    values = list(series)
    if len(values) > 1:
        span = max(1e-6, y_max - y_min)
        pts = []
        for i, v in enumerate(values):
            x = plot.x + int(i * (plot.w - 1) / (len(values) - 1))
            y = plot.bottom - int((v - y_min) / span * plot.h)
            y = max(plot.y + 1, min(plot.bottom - 1, y))
            pts.append((x, y))
        fill = pygame.Surface((plot.w, plot.h), pygame.SRCALPHA)
        poly = [(0, plot.h - 1)] + [(p[0] - plot.x, p[1] - plot.y) for p in pts] + [(plot.w - 1, plot.h - 1)]
        pygame.draw.polygon(fill, (color[0], color[1], color[2], 34), poly)
        surface.blit(fill, (plot.x, plot.y))
        pygame.draw.lines(surface, color, False, pts, 2)


def draw_header(surface, mode_ai, font_title, font_small):
    header_rect = pygame.Rect(0, 0, WIDTH, HEADER_H)
    draw_vertical_gradient(surface, header_rect, (32, 43, 55), (22, 31, 42))
    pygame.draw.line(surface, (91, 109, 128), (0, HEADER_H - 1), (WIDTH, HEADER_H - 1), 1)

    title = font_title.render("Industrial Bioreactor Digital Twin Dashboard", True, TEXT)
    surface.blit(title, (24, 18))

    # Clickable explicit mode switch to avoid hidden keyboard-only interaction.
    pygame.draw.rect(surface, (43, 57, 72), MODE_TOGGLE_RECT, border_radius=10)
    pygame.draw.rect(surface, PANEL_EDGE, MODE_TOGGLE_RECT, 1, border_radius=10)

    left = pygame.Rect(MODE_TOGGLE_RECT.x + 4, MODE_TOGGLE_RECT.y + 4, MODE_TOGGLE_RECT.w // 2 - 6, MODE_TOGGLE_RECT.h - 8)
    right = pygame.Rect(left.right + 6, left.y, MODE_TOGGLE_RECT.w // 2 - 8, left.h)

    ai_active = mode_ai
    man_active = not mode_ai
    pygame.draw.rect(surface, CYAN if ai_active else (58, 73, 88), left, border_radius=8)
    pygame.draw.rect(surface, AMBER if man_active else (58, 73, 88), right, border_radius=8)

    ai = font_small.render("AI", True, (15, 24, 34) if ai_active else TEXT_DIM)
    mn = font_small.render("MANUAL", True, (28, 24, 20) if man_active else TEXT_DIM)
    surface.blit(ai, ai.get_rect(center=left.center))
    surface.blit(mn, mn.get_rect(center=right.center))

    hint = font_small.render("Click or press Space/M", True, TEXT_DIM)
    surface.blit(hint, (MODE_TOGGLE_RECT.x, MODE_TOGGLE_RECT.bottom + 3))


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Industrial Virtual Bioreactor Dashboard")
    clock = pygame.time.Clock()

    font_title = pygame.font.SysFont("consolas", 28, bold=True)
    font_h2 = pygame.font.SysFont("consolas", 20, bold=True)
    font_small = pygame.font.SysFont("consolas", 16)
    font_mono = pygame.font.SysFont("consolas", 15)

    model = ProcessModel()

    trend_t = Trend(260, model.state.temperature)
    trend_ph = Trend(260, model.state.ph)
    trend_gas = Trend(260, model.state.gas_flow)

    tick = 0
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        tick += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in (pygame.K_SPACE, pygame.K_m, pygame.K_TAB, pygame.K_RETURN):
                    model.mode_ai = not model.mode_ai
                elif event.key == pygame.K_UP:
                    model.feed_disturbance = min(1.5, model.feed_disturbance + 0.08)
                elif event.key == pygame.K_DOWN:
                    model.feed_disturbance = max(-1.0, model.feed_disturbance - 0.08)
                elif event.key == pygame.K_LEFT:
                    model.cool_disturbance = max(-1.0, model.cool_disturbance - 0.08)
                elif event.key == pygame.K_RIGHT:
                    model.cool_disturbance = min(1.3, model.cool_disturbance + 0.08)
                elif event.key == pygame.K_r:
                    model.feed_disturbance = 0.0
                    model.cool_disturbance = 0.0
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if MODE_TOGGLE_RECT.collidepoint(event.pos):
                    model.mode_ai = not model.mode_ai

        s = model.step(dt)
        trend_t.push(s.temperature)
        trend_ph.push(s.ph)
        trend_gas.push(s.gas_flow)

        # Background
        draw_vertical_gradient(screen, (0, 0, WIDTH, HEIGHT), BG_TOP, BG_BOTTOM)

        draw_header(screen, model.mode_ai, font_h2, font_small)

        # Major zones
        proc_inner = draw_panel(screen, PROCESS_RECT, "Process", font_h2, font_small)
        center_inner = draw_panel(screen, CENTER_RECT, "Control and Coordination", font_h2, font_small)
        right_inner = draw_panel(screen, RIGHT_RECT, "Monitoring and Analytics", font_h2, font_small)

        inlet, outlet, gas_top = draw_process_vessel(screen, proc_inner, s, tick, font_small)

        # Control rack and monitoring blocks in center/right.
        rack = pygame.Rect(center_inner.x + 20, center_inner.y + 36, center_inner.w - 40, 188)
        draw_control_rack(screen, rack, s, model.mode_ai, font_small, font_mono)

        monitor = pygame.Rect(center_inner.x + 34, rack.bottom + 48, center_inner.w - 68, center_inner.h - rack.h - 90)
        draw_monitor_screen(screen, monitor, s, model.mode_ai, tick, font_small, font_mono)

        # Main closed-loop pipes (similar concept to your sketch, but more industrial HMI style).
        rack_top_center = (rack.centerx, rack.y - 10)
        monitor_right_mid = (monitor.right + 4, monitor.centery)
        process_out = (outlet[0], outlet[1])
        process_in = (inlet[0], inlet[1])
        gas_node = (gas_top[0], gas_top[1] - 12)

        draw_pipe_arrow(screen, process_out, (rack.left - 24, rack.y + 60))
        draw_pipe_arrow(screen, (rack.right + 24, rack.y + 60), (monitor.left - 4, monitor.y + 36))
        draw_pipe_arrow(screen, (monitor.left - 12, monitor.bottom - 26), (process_in[0] + 8, process_in[1] + 2))
        draw_pipe_arrow(screen, gas_node, (rack_top_center[0], rack_top_center[1]))

        # Right KPI cards.
        card_w = right_inner.w - 16
        c1 = pygame.Rect(right_inner.x + 8, right_inner.y + 8, card_w, 84)
        c2 = pygame.Rect(c1.x, c1.bottom + 8, card_w, 84)
        c3 = pygame.Rect(c1.x, c2.bottom + 8, card_w, 84)
        c4 = pygame.Rect(c1.x, c3.bottom + 8, card_w, 84)

        draw_metric_card(
            screen,
            c1,
            "Reactor Temperature",
            f"{s.temperature:4.2f}",
            "degC",
            zone_color(s.temperature, 36.4, 37.6, 35.3, 39.0),
            trend_t.values()[-48:],
            font_small,
            font_h2,
        )
        draw_metric_card(
            screen,
            c2,
            "pH",
            f"{s.ph:4.2f}",
            "pH",
            zone_color(s.ph, 6.85, 7.20, 6.4, 7.8),
            trend_ph.values()[-48:],
            font_small,
            font_h2,
        )
        draw_metric_card(
            screen,
            c3,
            "Biogas Flow",
            f"{s.gas_flow:4.2f}",
            "m3/h",
            zone_color(s.gas_flow, 4.8, 7.6, 3.0, 8.8),
            trend_gas.values()[-48:],
            font_small,
            font_h2,
        )
        draw_metric_card(
            screen,
            c4,
            "Stability Index",
            f"{model.stability:4.1f}",
            "%",
            zone_color(model.stability, 75, 100, 58, 100),
            [model.stability + math.sin((tick + i) * 0.03) * 0.8 for i in range(48)],
            font_small,
            font_h2,
        )

        # Trend plots at bottom-right.
        plot_h = (right_inner.bottom - c4.bottom - 30) // 3
        p1 = pygame.Rect(right_inner.x + 8, c4.bottom + 12, card_w, plot_h)
        p2 = pygame.Rect(right_inner.x + 8, p1.bottom + 8, card_w, plot_h)
        p3 = pygame.Rect(right_inner.x + 8, p2.bottom + 8, card_w, plot_h)

        draw_trend_plot(screen, p1, trend_t.values(), 33.0, 41.0, CYAN, "Temperature Trend", font_small)
        draw_trend_plot(screen, p2, trend_ph.values(), 6.0, 8.2, AMBER, "pH Trend", font_small)
        draw_trend_plot(screen, p3, trend_gas.values(), 1.0, 9.5, GREEN, "Biogas Trend", font_small)

        # Status bar with disturbance knobs.
        status = pygame.Rect(20, HEIGHT - 44, WIDTH - 40, 30)
        pygame.draw.rect(screen, (30, 41, 54), status, border_radius=8)
        pygame.draw.rect(screen, (74, 93, 112), status, 1, border_radius=8)

        mode = "AI" if model.mode_ai else "MANUAL"
        status_text = (
            f"Mode: {mode} | Feed disturbance: {model.feed_disturbance:+.2f} | "
            f"Coolant disturbance: {model.cool_disturbance:+.2f} | "
            "Controls: Click AI/MANUAL or Space/M/Tab/Enter, Arrows tune disturbances, R reset, Esc quit"
        )
        st = font_small.render(status_text, True, TEXT_DIM)
        screen.blit(st, (status.x + 10, status.y + 6))

        # Alarm badge.
        alarm = zone_color(s.temperature, 36.4, 37.6, 35.3, 39.0)
        alarm = RED if alarm == RED or zone_color(s.ph, 6.85, 7.20, 6.4, 7.8) == RED else alarm
        badge = pygame.Rect(WIDTH - 172, 16, 148, 36)
        pygame.draw.rect(screen, (47, 60, 74), badge, border_radius=8)
        pygame.draw.rect(screen, alarm, badge, 2, border_radius=8)
        btxt = "ALARM" if alarm == RED else ("WARNING" if alarm == AMBER else "NORMAL")
        bt = font_h2.render(btxt, True, alarm)
        screen.blit(bt, bt.get_rect(center=badge.center))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
