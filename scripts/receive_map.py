import serial
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, FixedLocator

# ==================== Settings ====================
SERIAL_PORT = "/dev/ttyUSB0"   # Linux: /dev/ttyUSB0 or /dev/ttyACM0 | Windows: COM3
BAUDRATE    = 115200
TIMEOUT     = 10               # seconds per read
RESOLUTION  = 0.05             # meters per pixel -> 5 cm/pixel
GRID_CM     = 50               # one grid square every 50 cm
OUTPUT_PNG  = "map_with_cm_grid.png"
REFRESH_MS  = 200              # how often the display checks for a new frame
# ==================================================


# ---------------- Serial PGM reader ----------------
def _read_token(ser):
    token = b""
    while True:
        ch = ser.read(1)
        if ch == b"":
            raise TimeoutError("Serial timeout while reading PGM header")
        if ch == b"#":
            while ch not in (b"\n", b""):
                ch = ser.read(1)
            continue
        if ch.isspace():
            if token:
                return token
            else:
                continue
        token += ch


def read_one_pgm(ser):
    """Read a single PGM image (P2 or P5) from an open serial port."""
    magic = _read_token(ser)
    while magic not in (b"P2", b"P5"):
        magic = _read_token(ser)

    width  = int(_read_token(ser))
    height = int(_read_token(ser))
    maxval = int(_read_token(ser))
    n_pixels = width * height

    if magic == b"P5":
        bytes_per_px = 1 if maxval < 256 else 2
        expected = n_pixels * bytes_per_px
        raw = b""
        while len(raw) < expected:
            chunk = ser.read(expected - len(raw))
            if not chunk:
                raise TimeoutError("Serial timeout while reading pixel data")
            raw += chunk
        dtype = np.uint8 if bytes_per_px == 1 else ">u2"
        arr = np.frombuffer(raw, dtype=dtype).reshape((height, width))
    else:  # P2 ASCII
        vals = []
        while len(vals) < n_pixels:
            vals.append(int(_read_token(ser)))
        dtype = np.uint8 if maxval < 256 else np.uint16
        arr = np.array(vals, dtype=dtype).reshape((height, width))

    return arr.astype(np.uint8)


# ---------------- Shared state between threads ----------------
state = {
    "arr": None,           # latest image
    "new": False,          # flag: a new frame is ready to draw
    "running": True,       # keep the reader thread alive
}
lock = threading.Lock()


def reader_thread():
    """Continuously receive maps and store the latest one."""
    while state["running"]:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        except Exception as e:
            print(f"Could not open serial port: {e}")
            return
        print("Waiting for PGM stream...")
        try:
            while state["running"]:
                arr = read_one_pgm(ser)
                with lock:
                    state["arr"] = arr
                    state["new"] = True
                print(f"Frame received: {arr.shape[1]}x{arr.shape[0]}")
        except Exception as e:
            print(f"Reader error: {e} — reopening port...")
        finally:
            ser.close()


# ---------------- Display setup ----------------
cm_per_pixel = RESOLUTION * 100.0
px_per_grid  = GRID_CM / cm_per_pixel

# Origin in pixel coords; None until first frame sets a sensible default
origin = {"px": None, "py": None}

fig, ax = plt.subplots(figsize=(12, 9))


def make_ticks(origin_px, length):
    ticks = []
    t = origin_px
    while t >= 0:
        ticks.append(t)
        t -= px_per_grid
    t = origin_px + px_per_grid
    while t <= length:
        ticks.append(t)
        t += px_per_grid
    return sorted(ticks)


def draw(arr):
    height, width = arr.shape
    if origin["px"] is None:          # first frame: default origin = bottom-left
        origin["px"], origin["py"] = 0.0, float(height)

    ax.clear()
    ax.imshow(arr, cmap="gray", origin="upper")
    ox, oy = origin["px"], origin["py"]

    ax.xaxis.set_major_locator(FixedLocator(make_ticks(ox, width)))
    ax.yaxis.set_major_locator(FixedLocator(make_ticks(oy, height)))

    ax.xaxis.set_major_formatter(FuncFormatter(lambda px, _: f"{(px - ox) * cm_per_pixel:.0f}"))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda px, _: f"{(oy - px) * cm_per_pixel:.0f}"))

    # --- smaller + vertical x labels for dense grids ---
    ax.tick_params(axis="x", labelrotation=90, labelsize=6)
    ax.tick_params(axis="y", labelsize=6)

    ax.grid(which="major", color="red", linestyle="-", linewidth=0.4, alpha=0.6)

    ax.plot(ox, oy, "o", color="lime", markersize=10, markeredgecolor="black")
    ax.axhline(oy, color="cyan", linewidth=1.0, alpha=0.8)
    ax.axvline(ox, color="cyan", linewidth=1.0, alpha=0.8)

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_title(f"LIVE  |  Left-click: set origin  |  Press 'S': save  |  grid {GRID_CM} cm")
    fig.canvas.draw_idle()


def on_timer():
    """Called periodically; redraw only when a new frame has arrived."""
    with lock:
        if state["new"] and state["arr"] is not None:
            arr = state["arr"]
            state["new"] = False
        else:
            arr = None
    if arr is not None:
        draw(arr)


def on_click(event):
    if event.inaxes != ax or event.button != 1:
        return
    origin["px"], origin["py"] = event.xdata, event.ydata
    print(f"New origin set at pixel ({event.xdata:.1f}, {event.ydata:.1f})")
    with lock:
        arr = state["arr"]
    if arr is not None:
        draw(arr)


def on_key(event):
    if event.key.lower() == "s":
        fig.savefig(OUTPUT_PNG, dpi=200, bbox_inches="tight")
        print(f"Saved to {OUTPUT_PNG}")


def on_close(event):
    state["running"] = False


# ---------------- Start everything ----------------
t = threading.Thread(target=reader_thread, daemon=True)
t.start()

fig.canvas.mpl_connect("button_press_event", on_click)
fig.canvas.mpl_connect("key_press_event", on_key)
fig.canvas.mpl_connect("close_event", on_close)

# Periodic timer drives the live refresh
timer = fig.canvas.new_timer(interval=REFRESH_MS)
timer.add_callback(on_timer)
timer.start()

ax.set_title("Waiting for first frame...")
plt.tight_layout()
plt.show()

state["running"] = False   # stop reader when window closes