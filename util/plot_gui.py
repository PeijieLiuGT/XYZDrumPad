import sys
import re
import time
import serial
import serial.tools.list_ports
import pygame

# Expected serial line formats (one per line):
#   void
#   (x, y, velocity)
# x, y, velocity may be int or float. Whitespace is tolerated.
HIT_RE = re.compile(
    r"\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)"
)
VOID_RE = re.compile(r"\bvoid\b", re.IGNORECASE)

# Coordinate space of the physical pad (matches the 3x3 drive/data grid in
# drum_pad_vip.ino: 3 cells per axis -> indices 0..2, possibly fractional if
# the firmware sends centroid/interpolated coords).
X_MIN, X_MAX = 0.0, 2.0
Y_MIN, Y_MAX = 0.0, 2.0
VEL_MAX = 127.0

# Trail of past hits to draw (ring buffer)
TRAIL_LEN = 64
TRAIL_FADE_S = 1.5  # seconds for a trail point to fully fade

def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return []
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device}  ({p.description})")
    return ports

def open_port(port_name: str, baud: int = 115200) -> serial.Serial:
    ser = serial.Serial(port_name, baudrate=baud, timeout=0.05)
    ser.reset_input_buffer()
    return ser

def read_lines(ser: serial.Serial):
    """
    Yield complete text lines from the serial port. Buffers partial input
    between calls so we don't lose bytes when a line spans reads.
    """
    if not hasattr(read_lines, "buf"):
        read_lines.buf = bytearray()

    chunk = ser.read(512)
    if chunk:
        read_lines.buf += chunk

    out = []
    while True:
        nl = read_lines.buf.find(b"\n")
        if nl < 0:
            break
        line = bytes(read_lines.buf[:nl]).decode(errors="replace").strip("\r\n\t ")
        del read_lines.buf[:nl + 1]
        if line:
            out.append(line)
    return out

def parse_line(line: str):
    """
    Returns:
      ("hit", (x, y, velocity)) for a (x,y,vel) reading
      ("void", None)            for a void reading
      None                      if the line doesn't match
    """
    m = HIT_RE.search(line)
    if m:
        x = float(m.group(1))
        y = float(m.group(2))
        v = float(m.group(3))
        return ("hit", (x, y, v))
    if VOID_RE.search(line):
        return ("void", None)
    return None

def vel_color(v: float):
    """Map velocity 0..VEL_MAX to a heat color (black->red->yellow->white)."""
    t = max(0.0, min(1.0, v / VEL_MAX))
    if t < 0.5:
        r = int(255 * (t / 0.5)); g = 0; b = 0
    elif t < 0.85:
        r = 255; g = int(255 * ((t - 0.5) / 0.35)); b = 0
    else:
        r = 255; g = 255; b = int(255 * ((t - 0.85) / 0.15))
    return (r, g, b)

def world_to_screen(x: float, y: float, plot_rect: pygame.Rect):
    """Map (x, y) in pad space to pixel coords inside plot_rect.
    Pad y increases downward on the physical grid (row 0 at top), so we keep
    that convention here too."""
    nx = (x - X_MIN) / (X_MAX - X_MIN) if X_MAX > X_MIN else 0.0
    ny = (y - Y_MIN) / (Y_MAX - Y_MIN) if Y_MAX > Y_MIN else 0.0
    nx = max(0.0, min(1.0, nx))
    ny = max(0.0, min(1.0, ny))
    px = plot_rect.left + int(nx * plot_rect.width)
    py = plot_rect.top + int(ny * plot_rect.height)
    return px, py

def draw_grid(surface: pygame.Surface, plot_rect: pygame.Rect, font: pygame.font.Font):
    pygame.draw.rect(surface, (28, 28, 32), plot_rect)
    pygame.draw.rect(surface, (60, 60, 70), plot_rect, width=2, border_radius=4)

    # 3x3 cell guides matching the physical grid.
    for i in range(1, 3):
        x = plot_rect.left + plot_rect.width * i // 3
        y = plot_rect.top + plot_rect.height * i // 3
        pygame.draw.line(surface, (45, 45, 55),
                         (x, plot_rect.top), (x, plot_rect.bottom), 1)
        pygame.draw.line(surface, (45, 45, 55),
                         (plot_rect.left, y), (plot_rect.right, y), 1)

    # Axis labels (corners of the pad space).
    def label(text, pos, anchor="topleft"):
        s = font.render(text, True, (140, 140, 150))
        rect = s.get_rect(**{anchor: pos})
        surface.blit(s, rect)

    label(f"({X_MIN:g}, {Y_MIN:g})",
          (plot_rect.left + 4, plot_rect.top + 4), "topleft")
    label(f"({X_MAX:g}, {Y_MIN:g})",
          (plot_rect.right - 4, plot_rect.top + 4), "topright")
    label(f"({X_MIN:g}, {Y_MAX:g})",
          (plot_rect.left + 4, plot_rect.bottom - 4), "bottomleft")
    label(f"({X_MAX:g}, {Y_MAX:g})",
          (plot_rect.right - 4, plot_rect.bottom - 4), "bottomright")

def draw_trail(surface: pygame.Surface, trail, plot_rect: pygame.Rect, now: float):
    # Older points first so newer points overlay them.
    for x, y, v, t_hit in trail:
        age = now - t_hit
        if age >= TRAIL_FADE_S:
            continue
        alpha = 1.0 - (age / TRAIL_FADE_S)
        radius = int(6 + (v / VEL_MAX) * 28)
        color = vel_color(v)
        # Fade by blending toward background.
        bg = 28
        faded = (
            int(bg + (color[0] - bg) * alpha),
            int(bg + (color[1] - bg) * alpha),
            int(bg + (color[2] - bg) * alpha),
        )
        cx, cy = world_to_screen(x, y, plot_rect)
        pygame.draw.circle(surface, faded, (cx, cy), radius)
        pygame.draw.circle(surface, (10, 10, 10), (cx, cy), radius, 1)

def draw_current(surface: pygame.Surface, hit, plot_rect: pygame.Rect):
    if hit is None:
        return
    x, y, v = hit
    cx, cy = world_to_screen(x, y, plot_rect)
    radius = int(10 + (v / VEL_MAX) * 36)
    color = vel_color(v)
    # Outer glow ring.
    pygame.draw.circle(surface, color, (cx, cy), radius + 6, 2)
    pygame.draw.circle(surface, color, (cx, cy), radius)
    pygame.draw.circle(surface, (255, 255, 255), (cx, cy), max(2, radius // 6))

def main():
    ports = list_ports()
    if not ports:
        sys.exit(1)

    try:
        choice = input("Select port index (default 0): ").strip()
        idx = int(choice) if choice else 0
    except ValueError:
        idx = 0

    port_name = ports[idx].device
    ser = open_port(port_name, 115200)
    print(f"Opened {port_name} @ 115200")

    pygame.init()
    W, H = 560, 640
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Drum Pad Hit Plot")
    clock = pygame.time.Clock()
    big = pygame.font.SysFont(None, 36)
    mid = pygame.font.SysFont(None, 24)
    small = pygame.font.SysFont(None, 18)

    # Square plot area centered horizontally, with room for HUD below.
    plot_size = min(W - 40, H - 200)
    plot_rect = pygame.Rect(
        (W - plot_size) // 2, 20, plot_size, plot_size
    )

    trail = []  # list of (x, y, v, t_hit)
    current_hit = None  # (x, y, v) or None when void
    last_event_time = time.time()
    hits_per_sec = 0.0
    last_hit_time = None
    state = "void"  # "void" or "hit"

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_c:
                trail.clear()
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_d:
                print("state:", state, "current:", current_hit,
                      "trail_len:", len(trail))

        now = time.time()

        for line in read_lines(ser):
            parsed = parse_line(line)
            if parsed is None:
                continue
            kind, payload = parsed
            last_event_time = now
            if kind == "void":
                state = "void"
                current_hit = None
            else:
                state = "hit"
                current_hit = payload
                trail.append((payload[0], payload[1], payload[2], now))
                if len(trail) > TRAIL_LEN:
                    del trail[:len(trail) - TRAIL_LEN]
                if last_hit_time is not None:
                    dt = now - last_hit_time
                    if dt > 0:
                        hits_per_sec = 0.85 * hits_per_sec + 0.15 * (1.0 / dt)
                last_hit_time = now

        screen.fill((18, 18, 22))
        draw_grid(screen, plot_rect, small)
        draw_trail(screen, trail, plot_rect, now)
        draw_current(screen, current_hit, plot_rect)

        # Status panel under the plot.
        panel_top = plot_rect.bottom + 16
        if state == "hit" and current_hit is not None:
            x, y, v = current_hit
            status_text = f"HIT  x={x:.2f}  y={y:.2f}  vel={v:.0f}"
            status_color = vel_color(v)
        else:
            status_text = "VOID"
            status_color = (90, 90, 110)

        status_surf = big.render(status_text, True, status_color)
        screen.blit(status_surf, (plot_rect.left, panel_top))

        info_lines = [
            f"port: {port_name}   hits/s: {hits_per_sec:.1f}   trail: {len(trail)}",
            f"pad space: x[{X_MIN:g},{X_MAX:g}]  y[{Y_MIN:g},{Y_MAX:g}]  vel[0,{VEL_MAX:g}]",
            "C = clear trail    D = debug print    ESC = quit",
        ]
        for i, text in enumerate(info_lines):
            surf = mid.render(text, True, (200, 200, 210))
            screen.blit(surf, (plot_rect.left, panel_top + 44 + i * 22))

        pygame.display.flip()
        clock.tick(60)

    ser.close()
    pygame.quit()

if __name__ == "__main__":
    main()
