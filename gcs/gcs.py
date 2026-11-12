# -*- coding: utf-8 -*-
"""
DHO GOKMAVI ground control station (GCS), RoboBoat 2027: two boats, Azure and Marine.

Plain Python, no ROS. Needs PySide6, numpy, pyserial (and protobuf for RoboCommand); see
gcs/README.md. Everything on the screen is English: the judges see it. Some code comments
still date from the 2026 Turkish version.
"""
from __future__ import annotations
import sys, os, json, csv, time, math
from collections import deque
from typing import Optional, Dict, Any

from PySide6 import QtCore, QtWidgets, QtGui
from PySide6.QtCore import Qt, Signal, QThread

import serial
import serial.tools.list_ports

# The GCS link (RoboBoat 2027 step 5a): the same code as the boat's telemetry_bridge, from
# the repository (ws/src/usv_core/usv_core/gcs_link.py); plain Python, no ROS
REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(REPO, 'ws', 'src', 'usv_core'))
from usv_core import gcs_link  # noqa: E402
from usv_core.allocation import allocate, BoatState  # noqa: E402
from PySide6.QtGui import QImageReader
from collections import OrderedDict
from datetime import datetime
import numpy as np
import socket
import struct

def res_path(name: str) -> str:
    # Images live in gcs/resources (PyInstaller onefile: _MEIPASS)
    base = getattr(sys, "_MEIPASS", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                 "resources"))
    return os.path.join(base, name)


# The two boats (RoboBoat 2027): number on the link -> name, panel colour. Their Ubiquiti
# addresses and UDP ports come from the boat profiles (ws/src/usv_nodes/config/boats).
BOATS = {1: ("AZURE", "#ff1744"), 2: ("MARINE", "#40c4ff")}
GCS_UDP_PORT = 15600


def boat_profiles():
    """Return {number: {name, ip, udp_port}} from the boat profiles (simple key: value lines)."""
    folder = os.path.join(REPO, "ws", "src", "usv_nodes", "config", "boats")
    out = {}
    try:
        files = sorted(f for f in os.listdir(folder) if f.endswith(".yaml"))
    except OSError:
        return out
    for f in files:
        d, turret = {}, False
        with open(os.path.join(folder, f), encoding="utf-8") as fh:
            for line in fh:
                line = line.split("#", 1)[0].rstrip()
                turret = turret or "mount: turret" in line
                if line and not line.startswith(" ") and ":" in line:
                    k, v = line.split(":", 1)
                    d[k.strip()] = v.strip()
        if "number" in d:
            # What the boat can do, as usv_core.boat.BoatProfile.capabilities()
            caps = [c for c, yes in (("water", d.get("water_cannon") == "true"),
                                     ("balls", int(d.get("balls", 0) or 0) > 0),
                                     ("lidar", d.get("lidar") == "true"),
                                     ("turret_camera", turret)) if yes]
            out[int(d["number"])] = {"name": d.get("name", f[:-5]), "ip": d.get("ip", ""),
                                     "udp_port": int(d.get("udp_port", 0) or 0),
                                     "capabilities": tuple(caps)}
    return out

# ---- Map image and its corner coordinates (WGS84) ----
MAP_IMAGE_PATH = res_path("roboboat.jpg")
# Top-left (NW) and bottom-right (SE) corners (WGS84 degrees)
NORTH = 27.381
SOUTH = 27.356
WEST  = -82.455
EAST  = -82.445
# ---- Marker sizes ----
WP_RADIUS_PX = 1.5        # Waypoint circle radius (px)
WP_FONT_PT   = 1.4       # Waypoint label font size (pt)
CURR_RADIUS_PX = 1.0      # Current position dot radius (px)
PATH_WIDTH_PX  = 3      # Track line width (px)
KEEP_PIXEL_SIZE = False   # Keep icons the same pixel size on screen whatever the zoom?

KIRMIZI_CIZGI_KALINLIK = 0.7
GPS_UPDATE_INTERVAL_MS = 5000  # Only GPS1..GPS5 are applied, once every 5 s


def haversine_distance(lat1, lon1, lat2, lon2):
    """Return the distance between two coordinates (metres)."""
    R = 6371000  # Earth radius (metres)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Return the bearing between two coordinates (degrees, 0 = north)."""
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.cos(math.radians(lon2 - lon1))
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def is_point_in_polygon(lat, lon, polygon_points):
    """
    Check with ray casting whether a point is inside a polygon.
    polygon_points: [(lat1, lon1), (lat2, lon2), ...] list
    """
    inside = False
    j = len(polygon_points) - 1
    for i in range(len(polygon_points)):
        xi, yi = polygon_points[i]
        xj, yj = polygon_points[j]

        intersect = ((yi > lon) != (yj > lon)) and \
                    (lat < (xj - xi) * (lon - yi) / (yj - yi + 1e-9) + xi)
        if intersect:
            inside = not inside
        j = i
    return inside

# --- Rope border stripe (draws a tiling) ---
class RopeStripe(QtWidgets.QWidget):
    def __init__(self, side: str, pix_path: str, thickness: int, parent=None):
        """
        side: "top" | "bottom" | "left" | "right"
        pix_path: path of halat.png (2560x51)
        thickness: strip thickness (px)
        """
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)  # must not swallow clicks
        self._side = side
        self._thk = int(thickness)

        pm = QtGui.QPixmap(pix_path)
        if pm.isNull():
            # fallback: grey
            pm = QtGui.QPixmap(512, 51)
            pm.fill(QtGui.QColor("#777"))
        # Vertical edges: turn by 90 deg
        if side in ("left", "right"):
            pm = pm.transformed(QtGui.QTransform().rotate(90), QtCore.Qt.SmoothTransformation)
        self._pm = pm

        # Fix the size (the geometry is still set from outside)
        if side in ("top", "bottom"):
            self.setFixedHeight(self._thk)
        else:
            self.setFixedWidth(self._thk)

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        # Tile the whole rect with the rope texture
        p.drawTiledPixmap(self.rect(), self._pm)

class HugeImageItem(QtWidgets.QGraphicsItem):
    """
    Draws huge full-resolution JPEG/PNGs, loading tiles lazily.
    QImageReader.setClipRect decodes only the needed region from disk.
    A simple LRU cache keeps the tiles on screen.
    """
    def __init__(self, image_path: str, tile_size: int = 2048, cache_limit: int = 128, parent=None):
        super().__init__(parent)
        self.image_path = image_path
        self.tile_size = int(tile_size)
        self.cache_limit = int(cache_limit)  # most tiles kept at once
        self._cache: OrderedDict[tuple[int,int], QtGui.QPixmap] = OrderedDict()

        r = QImageReader(self.image_path)
        r.setAutoTransform(True)
        sz = r.size()
        if not sz.isValid():
            # No size metadata: read once to get the size
            img = r.read()
            if img.isNull():
                raise RuntimeError(f"Image cannot be read: {self.image_path}")
            self._W, self._H = img.width(), img.height()
        else:
            self._W, self._H = sz.width(), sz.height()

        self.setFlag(QtWidgets.QGraphicsItem.ItemUsesExtendedStyleOption, True)

    # overall size
    @property
    def width(self):  return self._W
    @property
    def height(self): return self._H

    def boundingRect(self) -> QtCore.QRectF:
        return QtCore.QRectF(0, 0, self._W, self._H)

    def _load_tile(self, tx: int, ty: int) -> QtGui.QPixmap:
        key = (tx, ty)
        if key in self._cache:
            # update the LRU
            pm = self._cache.pop(key)
            self._cache[key] = pm
            return pm

        x = tx * self.tile_size
        y = ty * self.tile_size
        w = min(self.tile_size, self._W - x)
        h = min(self.tile_size, self._H - y)

        r = QImageReader(self.image_path)
        r.setAutoTransform(True)
        r.setClipRect(QtCore.QRect(x, y, w, h))
        img = r.read()
        if img.isNull():
            # on failure a grey placeholder
            pm = QtGui.QPixmap(w, h)
            pm.fill(QtGui.QColor("#777"))
        else:
            pm = QtGui.QPixmap.fromImage(img)

        # LRU: keep within capacity
        self._cache[key] = pm
        if len(self._cache) > self.cache_limit:
            self._cache.popitem(last=False)  # drop the oldest
        return pm

    def paint(self, painter: QtGui.QPainter, option: QtWidgets.QStyleOptionGraphicsItem, widget=None):
        # Visible (exposed) rect -> which tiles are needed?
        rect = option.exposedRect.intersected(self.boundingRect())
        if rect.isEmpty():
            return

        ts = self.tile_size
        x0 = int(rect.left() // ts)
        y0 = int(rect.top()  // ts)
        x1 = int((rect.right()) // ts)
        y1 = int((rect.bottom()) // ts)

        # fetch a margin of tiles too (smoother pan / zoom)
        pad = 1
        x0 = max(0, x0 - pad)
        y0 = max(0, y0 - pad)
        x1 = min((self._W - 1) // ts, x1 + pad)
        y1 = min((self._H - 1) // ts, y1 + pad)

        for ty in range(y0, y1 + 1):
            for tx in range(x0, x1 + 1):
                pm = self._load_tile(tx, ty)
                painter.drawPixmap(tx * ts, ty * ts, pm)

def normalize_gps_dict(gps_raw):
    """
    Accepted formats, for example:
      {"GPS1":{"lat":..,"lon":..}, ...}
      {"GPS1_enlem":..,"GPS1_boylam":.., ...}
      {"GPS1":[lat,lon], ...}  or {"GPS1":"lat,lon"}
      {"points":[{"name":"GPS1","lat":..,"lon":..}, ...]}
    Returns: {"GPS1":{"lat":float,"lon":float}, ...}
    """
    if not isinstance(gps_raw, dict):
        return None

    out = {}
    # 1) Already a nested dict
    ok_nested = True
    for i in range(1, 6):
        k = f"GPS{i}"
        v = gps_raw.get(k)
        if not (isinstance(v, dict) and "lat" in v and "lon" in v):
            ok_nested = False
            break
    if ok_nested:
        # convert them all to float
        for i in range(1, 6):
            k = f"GPS{i}"
            v = gps_raw.get(k)
            if v is None:
                continue
            try:
                out[k] = {"lat": float(v["lat"]), "lon": float(v["lon"])}
            except Exception:
                pass
        return out if out else None

    # 2) GPS1_enlem / GPS1_boylam
    any_found = False
    for i in range(1, 6):
        ke = f"GPS{i}_enlem"
        kb = f"GPS{i}_boylam"
        if ke in gps_raw and kb in gps_raw:
            try:
                out[f"GPS{i}"] = {"lat": float(gps_raw[ke]), "lon": float(gps_raw[kb])}
                any_found = True
            except Exception:
                pass
    if any_found:
        return out

    # 3) GPS1: [lat,lon]  ya da "lat,lon"
    any_found = False
    for i in range(1, 6):
        k = f"GPS{i}"
        if k in gps_raw:
            v = gps_raw[k]
            try:
                if isinstance(v, (list, tuple)) and len(v) == 2:
                    lat, lon = float(v[0]), float(v[1])
                elif isinstance(v, str) and "," in v:
                    a, b = v.split(",", 1)
                    lat, lon = float(a), float(b)
                else:
                    continue
                out[k] = {"lat": lat, "lon": lon}
                any_found = True
            except Exception:
                pass
    if any_found:
        return out

    # 4) {"points":[{"name":"GPS1","lat":..,"lon":..}, ...]}
    pts = gps_raw.get("points")
    if isinstance(pts, list):
        for p in pts:
            try:
                name = str(p.get("name"))
                if name.upper().startswith("GPS"):
                    out[name.upper()] = {"lat": float(p["lat"]), "lon": float(p["lon"])}
            except Exception:
                pass
        if out:
            return out

    return None


# -------------------- Serial Worker (fixed and faster) --------------------
def _parse_or_recover(line: str):
    """
    Parse a received line. Returns (obj_or_None, leftover_prefix).

    Two different things corrupt a line on this link, and they need different answers.

    The radio runs with Mavlink framing OFF, so it treats the serial stream as opaque bytes
    and cuts wherever its per-frame buffer fills. A packet larger than one frame therefore
    goes out in pieces - and with three nodes sharing the channel, another node's turn can
    fall BETWEEN those pieces. Its bytes land inside our JSON:

        {"i":1,"t":"23:05:16","a":16{"id":3,"drone_color":"KIRMIZI"}
        77,"b":1722,...,"ts":1}

    Note this is not a collision: nobody transmitted at the same time. It is ordinary TDMA
    doing exactly what it should, on a message it was never told the boundaries of. It is
    also why commands kept getting through while telemetry did not - at 38-76 B a command
    never spans two frames, so there is no seam to cut across.

    Both halves are intact, so both are recoverable. The complete object at the END is
    returned; the incomplete PREFIX is handed back so the caller can glue it to the next
    line, which carries the rest of the interrupted packet.

    Recovery looks for a packet START, not merely a later '{'. Every packet on this link
    begins with {"i" (telemetry) or {"id" (drone). A blind search for '{' would happily
    accept a nested object and hand the UI something that is not a packet at all.
    """
    try:
        return json.loads(line), ""
    except json.JSONDecodeError:
        pass

    for marker in ('{"i"', '{"id"'):
        start = line.find(marker, 1)
        while start != -1:
            try:
                return json.loads(line[start:]), line[:start]
            except json.JSONDecodeError:
                start = line.find(marker, start + 1)
    return None, ""


# Wire keys are short to keep a packet inside one radio frame; they are expanded back to
# the long names here, the moment the line is parsed, so on_packet() and everything below
# it still reads 'pwm_L', 'position' and friends exactly as before.
_TASK_NAMES = {
    0: "TASK1_APPROACH", 1: "TASK1_STATE_ENTER", 2: "TASK1_STATE_MID",
    3: "TASK1_STATE_EXIT", 4: "TASK2_START", 5: "TASK2_GO_TO_MID",
    6: "TASK2_GO_TO_END", 7: "T3_START", 8: "T3_MID",
    9: "TASK3_SEARCH_KAMIKAZE", 10: "FINISHED", 11: "TASK_UNKNOWN",
}

_WIRE_KEYS = {
    "i": "id", "t": "t_ms", "a": "pwm_L", "b": "pwm_R", "c": "pwm_FL", "d": "pwm_FR",
    "s": "pwm_STEER", "v": "spd", "h": "hdg", "th": "trg_hdg", "ea": "err_ang",
    "ce": "ctrl_err", "k": "task", "ds": "dist", "m": "mod", "f": "FPS", "r": "relay",
    "tc": "tcol", "kq": "task_queue",
}


# Drone reports colours in Turkish; the boat's config and detector speak English.
COLOR_TR_TO_EN = {"KIRMIZI": "red", "YESIL": "green", "SIYAH": "black", "SARI": "yellow"}
COLOR_EN_TO_TR = {v: k for k, v in COLOR_TR_TO_EN.items()}


class PySerialLink:
    """The RFD over pyserial: for Windows, where gcs_link.SerialLink (termios) does not run."""

    name = "rfd"

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.5)
        self.ser.reset_input_buffer()
        self.limit = int(baud / 10.0 * 0.5)
        self.rx_bytes = self.tx_bytes = self.dropped = 0

    def send(self, data):
        if len(data) > self.limit:
            self.dropped += 1
            return
        try:
            self.ser.write(data)
            self.tx_bytes += len(data)
        except (serial.SerialException, OSError):
            self.dropped += 1

    def read(self):
        n = self.ser.in_waiting
        data = self.ser.read(n) if n else b""
        self.rx_bytes += len(data)
        return data

    def close(self):
        self.ser.close()


class LinkWorker(QThread):
    """
    The GCS's end of the link, over the Ubiquiti network (UDP) and the RFD at once.

    Replaces the 2026 SerialWorker (RoboBoat 2027 step 5b). Every command goes out on every
    link; telemetry arrives on both and only the newest packet of each boat is passed on
    (gcs_link.NewestFilter), with the link that brought it as '_link'. The drone's packets
    (RFD) pass through as before. A link that cannot be opened is retried every second.
    """

    packet = Signal(dict)
    status = Signal(str)
    link = Signal(bool)
    links_state = Signal(dict)        # {"udp": seconds since heard (None = never), "rfd": ...}

    def __init__(self, udp_peers, rfd_port, baud, csv_path=None):
        super().__init__()
        self.udp_peers = udp_peers    # [(host, port)] of the boats; [] = no network link
        self.rfd_port = rfd_port      # '' = no radio
        self.baud = baud
        self.csv_path = csv_path
        self._stop = False
        self._tx_queue = []
        self._newest = gcs_link.NewestFilter()
        self._wp_accum = {}
        self._links = None
        self._last_error = {}

    def _expand(self, d: dict) -> dict:
        """
        Short wire keys -> the long names the UI has always used.

        Packets from the drone arrive with their own long keys already and pass through
        untouched. Doing the translation here rather than in on_packet() means the map,
        the CSV writer and every label below it keep working without a single edit.
        """
        if "i" not in d:
            return d                      # drone packet (or an already-expanded one)

        out = {}
        for k, v in d.items():
            out[_WIRE_KEYS.get(k, k)] = v

        # Position travels flat - two keys instead of a nested object with its own
        # punctuation - and is re-nested here.
        if "la" in d and "lo" in d:
            out["position"] = {"lat": d["la"], "lon": d["lo"]}
            out.pop("la", None); out.pop("lo", None)
        if "xa" in d and "xo" in d:
            out["target"] = {"lat": d["xa"], "lon": d["xo"]}
            out.pop("xa", None); out.pop("xo", None)

        # 'ts' is 0/1 on the wire; the readout compares against these strings.
        if "ts" in d:
            out["tsrc"] = "drone" if int(d.get("ts", 0)) else "cfg"
            out.pop("ts", None)

        # Never transmitted: it has always been the constant "GOOD" on the boat, so paying
        # for it three times a second bought nothing. Filled in so SENSORS reads
        # exactly as it did before.
        out.setdefault("hlth", "GOOD")

        # Task states travel as small integers; unknown values pass through as-is.
        if isinstance(out.get("task"), int):
            out["task"] = _TASK_NAMES.get(out["task"], str(out["task"]))

        # TIME is stamped at ARRIVAL rather than transmitted. The boat's clock cost 15 B
        # per packet to deliver 1-second resolution; the arrival time is what the operator
        # actually wants to know (is this data fresh?) and it is free.
        out.setdefault("t_ms", time.strftime("%H:%M:%S"))

        # One waypoint per packet, accumulated into the full set.
        if "w" in d and "wa" in d and "wo" in d:
            try:
                bid = int(d.get("i", 1))
                store = self._wp_accum.setdefault(bid, {})
                store[int(d["w"])] = (float(d["wa"]), float(d["wo"]))
                out["waypoints"] = {
                    "id": bid,
                    **{f"GPS{n}": {"lat": la, "lon": lo} for n, (la, lo) in store.items()},
                }
            except (TypeError, ValueError):
                pass
            for k in ("w", "wa", "wo"):
                out.pop(k, None)

        return out

    def queue_send(self, obj: Dict[str, Any]):
        self._tx_queue.append(dict(obj))

    def _open(self, kind):
        try:
            if kind == "udp":
                return gcs_link.UdpLink(GCS_UDP_PORT, self.udp_peers)
            if gcs_link.termios is not None:
                return gcs_link.SerialLink(self.rfd_port, self.baud)
            return PySerialLink(self.rfd_port, self.baud)
        except (OSError, serial.SerialException) as e:
            text = f"{kind.upper()} could not be opened: {e} (retrying every second)"
            if self._last_error.get(kind) != text:     # once, not every second
                self._last_error[kind] = text
                self.status.emit(text)
            return None

    def run(self):
        wanted = (["udp"] if self.udp_peers else []) + (["rfd"] if self.rfd_port else [])
        opened = {}
        retry_t = 0.0
        heard_any = False
        state_t = 0.0
        self._links = gcs_link.Links([])
        while not self._stop:
            now = time.monotonic()
            if len(opened) < len(wanted) and now >= retry_t:
                retry_t = now + 1.0
                for kind in wanted:
                    if kind not in opened:
                        lk = self._open(kind)
                        if lk is not None:
                            opened[kind] = lk
                            self._links.links.append(lk)
                            self._links.readers[lk.name] = gcs_link.LineReader()
                            self._links.heard[lk.name] = 0.0
                            self.status.emit(f"{kind.upper()} opened")
            try:
                while self._tx_queue:
                    self._links.send(self._tx_queue.pop(0))
                for name, obj in self._links.read(now):
                    if "i" in obj and not self._newest.is_new(obj["i"], obj.get("q"), now):
                        continue          # the same packet over the other link, or older
                    d = self._expand(obj)
                    d["_link"] = name
                    self.packet.emit(d)
                    if not heard_any:
                        heard_any = True
                        self.link.emit(True)
            except Exception as e:  # noqa: B902 (the link thread must keep running)
                self.status.emit(f"Link error: {e}")
            if now - state_t >= 0.5:
                state_t = now
                self.links_state.emit({k: (now - self._links.heard[k]
                                           if self._links.heard.get(k) else None)
                                       for k in opened})
            time.sleep(0.01)
        self._links.close()

    def stop(self):
        self._stop = True


# ------------- Zoom with the mouse and drag by hand ---------------
class GraphicsView(QtWidgets.QGraphicsView):
    zoomChanged = QtCore.Signal(float)
    clicked = Signal(QtCore.QPointF, QtCore.QPoint)
    moved = Signal(QtCore.QPointF)

    def __init__(self, scene, parent=None, min_zoom=0.25, max_zoom=16.0):
        super().__init__(scene, parent)
        self._min_zoom = float(min_zoom)
        self._max_zoom = float(max_zoom)

        # --- VIEW SETTINGS ---
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        self.setViewportUpdateMode(QtWidgets.QGraphicsView.FullViewportUpdate)

        # Hide the scroll bars (cleaner view)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # --- CURSOR ---
        # HIDE the mouse (only the red cross shows)
        self.setCursor(Qt.BlankCursor)
        # For a thin cross instead of hiding it, delete the line above and use the one below:
        # self.setCursor(Qt.CrossCursor)

        # Drag state
        self._is_panning = False
        self._pan_start = QtCore.QPoint()

    def wheelEvent(self, event: QtGui.QWheelEvent):
        delta = event.pixelDelta().y() if not event.angleDelta().y() else event.angleDelta().y()
        if delta == 0:
            event.ignore();
            return
        step_factor = 1.15 if delta > 0 else (1.0 / 1.15)
        cur = self.transform().m11()
        new_zoom = max(self._min_zoom, min(self._max_zoom, cur * step_factor))
        if new_zoom == cur: return
        factor = new_zoom / cur
        self.scale(factor, factor)
        smooth = new_zoom >= 1.0
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, smooth)
        self.zoomChanged.emit(new_zoom)
        event.accept()

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        # Left button pressed: start dragging
        if event.button() == Qt.LeftButton:
            self._is_panning = True
            self._pan_start = event.position().toPoint()

            # Click signal (Shift etc. is checked outside)
            view_pt = event.position().toPoint()
            self.clicked.emit(self.mapToScene(view_pt), event.globalPosition().toPoint())

            # To change the cursor (e.g. a fist)
            # self.setCursor(Qt.ClosedHandCursor)
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent):
        if event.button() == Qt.LeftButton:
            self._is_panning = False
            # Released: hide again (or a thin cross)
            self.setCursor(Qt.BlankCursor)
        super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        # 1. Dragging
        if self._is_panning:
            delta = event.position().toPoint() - self._pan_start
            self._pan_start = event.position().toPoint()

            # Scroll the scroll bars by hand
            hs = self.horizontalScrollBar()
            vs = self.verticalScrollBar()
            hs.setValue(hs.value() - delta.x())
            vs.setValue(vs.value() - delta.y())
            event.accept()

        # 2. Coordinate signal (for the crosshair)
        self.moved.emit(self.mapToScene(event.position().toPoint()))
        super().mouseMoveEvent(event)


# -------------------- Map Panel --------------------
class MapPanel(QtWidgets.QWidget):
    """Simple image map based on QGraphicsView.
    - Drag: drag with the left button
    - Zoom: +/- buttons, the wheel works too
    - Lat/Lon -> pixel conversion with the WebMercator formulas
    """
    # Sends index (1-5), lat, lon
    gps_selected = Signal(int, float, float, int)

    def __init__(self, image_path: str, north: float, south: float, west: float, east: float, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(420)
        self.setMinimumHeight(400)
        self.image_path = image_path
        self.north, self.south, self.west, self.east = north, south, west, east
        self._R = 6378137.0  # WebMercator radius
        self._max_zoom = 16.0
        self._min_zoom = 0.25

        # Scene & View
        self.scene = QtWidgets.QGraphicsScene(self)
        self.view = GraphicsView(self.scene, self, min_zoom=self._min_zoom, max_zoom=self._max_zoom)

        self.view.setContextMenuPolicy(Qt.CustomContextMenu)
        self.view.customContextMenuRequested.connect(self._on_context_menu)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self.view.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        # Hide the scroll bars (for a clean HUD)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setViewportUpdateMode(QtWidgets.QGraphicsView.FullViewportUpdate)

        # Pixmap (large images scaled down automatically)
        MAX_DIM = 32000  # safe QPixmap / texture limit

        # --- RANGE RINGS ---
        self.ring_group = QtWidgets.QGraphicsItemGroup()
        self.ring_group.setZValue(13)
        self.scene.addItem(self.ring_group)
        self.rings_enabled = False
        self.ring_step_m = 5.0
        self.ring_count = 3

        # --- GEOFENCE ---
        self._fence_mode = False
        self._fence_points_latlon = []
        self.fence_polygon = QtWidgets.QGraphicsPolygonItem()
        pen = QtGui.QPen(QtGui.QColor("#ff1744"))
        pen.setWidth(2)
        pen.setStyle(Qt.DashLine)
        brush = QtGui.QBrush(QtGui.QColor(255, 23, 68, 50))
        self.fence_polygon.setPen(pen)
        self.fence_polygon.setBrush(brush)
        self.fence_polygon.setZValue(14)
        self.scene.addItem(self.fence_polygon)

        # --- GHOST TRAIL ---
        self.ghost_pen = QtGui.QPen(QtGui.QColor(200, 200, 200, 120))
        self.ghost_pen.setWidthF(1.5)
        self.ghost_pen.setStyle(Qt.DashLine)
        self.ghost_path = QtGui.QPainterPath()
        self.ghost_item = self.scene.addPath(self.ghost_path, self.ghost_pen)
        self.ghost_item.setZValue(11)

        # --- SMART RULER ---
        self._ruler_active = False
        self._ruler_start_lat = None
        self._ruler_start_lon = None
        self._ruler_start_scene = None
        self.ruler_line = QtWidgets.QGraphicsLineItem()
        pen = QtGui.QPen(QtGui.QColor("yellow"))
        pen.setWidth(2);
        pen.setStyle(Qt.DashLine)
        self.ruler_line.setPen(pen);
        self.ruler_line.setZValue(900);
        self.ruler_line.setVisible(False)
        self.scene.addItem(self.ruler_line)

        self.ruler_text = QtWidgets.QGraphicsTextItem("")
        self.ruler_text.setDefaultTextColor(QtGui.QColor("yellow"))
        f = self.ruler_text.font();
        f.setBold(True);
        f.setPointSize(10);
        self.ruler_text.setFont(f)
        self.ruler_text.setZValue(901);
        self.ruler_text.setVisible(False)

        self.ruler_bg = QtWidgets.QGraphicsRectItem()
        self.ruler_bg.setBrush(QtGui.QBrush(QtGui.QColor(0, 0, 0, 150)))
        self.ruler_bg.setZValue(900);
        self.ruler_bg.setVisible(False)
        self.scene.addItem(self.ruler_bg);
        self.scene.addItem(self.ruler_text)

        # --- COURSE MANAGEMENT ---
        self._course_mode = None
        self._temp_corners = []
        self._temp_corner_items = []
        self.courses = {
            "A": {"corners": [], "rows": 4, "cols": 4, "visible": True, "color": QtGui.QColor(0, 255, 255)},
            "B": {"corners": [], "rows": 4, "cols": 4, "visible": True, "color": QtGui.QColor(255, 0, 255)},
            "C": {"corners": [], "rows": 4, "cols": 4, "visible": True, "color": QtGui.QColor(255, 255, 0)}
        }
        self.course_groups = {
            "A": self.scene.createItemGroup([]),
            "B": self.scene.createItemGroup([]),
            "C": self.scene.createItemGroup([])
        }
        for k, grp in self.course_groups.items():
            self.scene.addItem(grp);
            grp.setZValue(5)

        # --- IMAGE LOADING ---
        if not os.path.exists(self.image_path):
            pm = QtGui.QPixmap(800, 600);
            pm.fill(QtGui.QColor("#555"))
            self._img_W, self._img_H = pm.width(), pm.height()
            self.image_item = self.scene.addPixmap(pm)
        else:
            self.image_item = HugeImageItem(self.image_path, tile_size=2048, cache_limit=128)
            self.scene.addItem(self.image_item)
            self._img_W, self._img_H = self.image_item.width, self.image_item.height

        # Overlay group
        self.overlay_group = QtWidgets.QGraphicsItemGroup();
        self.overlay_group.setZValue(10)
        self.scene.addItem(self.overlay_group)

        # Current position marker (red)
        r = CURR_RADIUS_PX
        self.curr_marker = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        self.curr_marker.setBrush(QtGui.QBrush(QtGui.QColor("#ff1744")))
        self.curr_marker.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 1.0))
        self.curr_marker.setZValue(20);
        self.curr_marker.setVisible(False)
        if KEEP_PIXEL_SIZE: self.curr_marker.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
        self.scene.addItem(self.curr_marker)

        self.path_pen = QtGui.QPen(QtGui.QColor("#ff5252"));
        self.path_pen.setWidthF(PATH_WIDTH_PX)
        self.path_pen.setCosmetic(True)  # <--- IMPORTANT: must not get thinner when zooming!
        self.path_path = QtGui.QPainterPath()
        self.path_item = self.scene.addPath(self.path_path, self.path_pen);
        self.path_item.setZValue(12)

        # USV 2 (Marine): its own marker and track, in its panel's colour. The 2026 code
        # moved them (update_current_position) but never made them, so boat 2 was not drawn.
        self.curr_marker_2 = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        self.curr_marker_2.setBrush(QtGui.QBrush(QtGui.QColor("#40c4ff")))
        self.curr_marker_2.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 1.0))
        self.curr_marker_2.setZValue(20)
        self.curr_marker_2.setVisible(False)
        if KEEP_PIXEL_SIZE: self.curr_marker_2.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
        self.scene.addItem(self.curr_marker_2)
        self.path_pen_2 = QtGui.QPen(QtGui.QColor("#40c4ff"))
        self.path_pen_2.setWidthF(PATH_WIDTH_PX)
        self.path_pen_2.setCosmetic(True)
        self.path_path_2 = QtGui.QPainterPath()
        self.path_item_2 = self.scene.addPath(self.path_path_2, self.path_pen_2)
        self.path_item_2.setZValue(12)

        # Lists holding the track history (up to 500 m)
        self.trail_points_1 = []
        self.trail_points_2 = []
        # --- SMOOTHING AND INTERPOLATION STATE ---
        self._target_lat_1 = None
        self._target_lon_1 = None
        self._visual_lat_1 = None
        self._visual_lon_1 = None

        self._target_lat_2 = None
        self._target_lon_2 = None
        self._visual_lat_2 = None
        self._visual_lon_2 = None

        # Animation timer updating ~30 frames a second
        self.anim_timer = QtCore.QTimer(self)
        self.anim_timer.timeout.connect(self._animate_markers)
        self.anim_timer.start(33)  # ~30 FPS (1000ms / 30)


        # --- Heading overlays ---
        self._last_lat = None;
        self._last_lon = None;
        self._real_hdg_deg = None;
        self._targ_hdg_deg = None
        self._real_line = QtWidgets.QGraphicsLineItem();
        real_pen = QtGui.QPen(QtGui.QColor("#ff1744"));
        real_pen.setWidthF(2.2)
        self._real_line.setPen(real_pen);
        self._real_line.setZValue(18);
        self.scene.addItem(self._real_line);
        self._real_line.setVisible(False)
        self._real_arrow = QtWidgets.QGraphicsPathItem();
        self._real_arrow.setPen(real_pen);
        self._real_arrow.setZValue(19);
        self.scene.addItem(self._real_arrow);
        self._real_arrow.setVisible(False)
        self._targ_line = QtWidgets.QGraphicsLineItem();
        targ_pen = QtGui.QPen(QtGui.QColor("#00e676"));
        targ_pen.setWidthF(2.0)
        self._targ_line.setPen(targ_pen);
        self._targ_line.setZValue(17);
        self.scene.addItem(self._targ_line);
        self._targ_line.setVisible(False)

        # Waypoint layer
        self.wp_group = QtWidgets.QGraphicsItemGroup();
        self.wp_group.setZValue(15);
        self.scene.addItem(self.wp_group)

        # Layout
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.view)

        # Signals
        self.view.clicked.connect(self._on_map_click)
        self.view.moved.connect(self._on_mouse_move)

        # Labels
        self.click_labels_root = QtWidgets.QGraphicsItemGroup();
        self.click_labels_root.setZValue(500);
        self.scene.addItem(self.click_labels_root)

        # Crosshair
        self.crosshair = QtWidgets.QGraphicsItemGroup()
        pen = QtGui.QPen(QtGui.QColor("#ffff00"));
        pen.setWidthF(1.6)
        l1 = QtWidgets.QGraphicsLineItem(-5, 0, 5, 0);
        l2 = QtWidgets.QGraphicsLineItem(0, -5, 0, 5)
        for ln in (l1, l2):
            ln.setPen(pen);
            ln.setZValue(1000);
            ln.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True);
            ln.setAcceptedMouseButtons(Qt.NoButton)
            self.crosshair.addToGroup(ln)
        self.crosshair.setZValue(1000);
        self.crosshair.setAcceptedMouseButtons(Qt.NoButton)
        self.scene.addItem(self.crosshair);
        self.crosshair.setVisible(True)

        # --- NEW OVERLAY STARTS HERE (the old _btn_box was removed) ---
        # Call the function that adds the buttons as an overlay
        self._setup_overlay_controls()
        self._label_offset = QtCore.QPointF(8, -8)

        # Start
        self.view.setSceneRect(QtCore.QRectF(0, 0, self._img_W, self._img_H))
        self.center_image()
        self.set_zoom(0.2)

        self._click_labels = []
        self.view.zoomChanged.connect(self._relayout_click_labels)
        self._debug_corners()
        self.cached_pts_1 = []
        self.cached_pts_2 = []

    def _setup_overlay_controls(self):
        """Build the military-theme HUD buttons (left: zoom + magnifier, right: pan + compass)."""

        # Shared style
        hud_style = """
            QPushButton {
                background-color: rgba(20, 20, 20, 200);
                color: #00e5ff;
                border: 1px solid #546e7a;
                border-radius: 4px;
                font-weight: bold;
                font-size: 16px;
                margin: 0px;
            }
            QPushButton:hover {
                background-color: rgba(0, 229, 255, 40);
                border: 1px solid #00e5ff;
                color: white;
            }
            QPushButton:pressed {
                background-color: rgba(0, 229, 255, 100);
                border: 1px solid #00e5ff;
                padding-top: 2px;
                padding-left: 2px;
            }
            QLabel {
                color: #00e5ff;
                font-size: 22px;
                background: transparent;
                border: none;
            }
        """

        # --- 1. ZOOM PANEL (TOP LEFT) ---
        self.zoom_widget = QtWidgets.QWidget(self)
        self.zoom_widget.setStyleSheet(hud_style)

        v_zoom = QtWidgets.QVBoxLayout(self.zoom_widget)
        v_zoom.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        v_zoom.setContentsMargins(4, 4, 4, 4)
        v_zoom.setSpacing(4)
        v_zoom.setAlignment(Qt.AlignCenter)

        z_size = 38

        self.btn_in = QtWidgets.QPushButton("+")
        self.btn_in.setFixedSize(z_size, z_size)
        self.btn_in.clicked.connect(lambda: self.zoom(1.25))

        lbl_magnifier = QtWidgets.QLabel("🔍")
        lbl_magnifier.setAlignment(Qt.AlignCenter)

        self.btn_out = QtWidgets.QPushButton("−")
        self.btn_out.setFixedSize(z_size, z_size)
        self.btn_out.clicked.connect(lambda: self.zoom(0.8))

        v_zoom.addWidget(self.btn_in)
        v_zoom.addWidget(lbl_magnifier)
        v_zoom.addWidget(self.btn_out)

        # --- 2. PAN PANEL (TOP RIGHT) ---
        self.pan_widget = QtWidgets.QWidget(self)
        self.pan_widget.setStyleSheet(hud_style)

        g_pan = QtWidgets.QGridLayout(self.pan_widget)
        g_pan.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        g_pan.setContentsMargins(4, 4, 4, 4)
        g_pan.setSpacing(4)

        p_size = 34

        btn_u = QtWidgets.QPushButton("▲");
        btn_u.setFixedSize(p_size, p_size)
        btn_u.clicked.connect(lambda: self._pan_map(0, -150))

        btn_d = QtWidgets.QPushButton("▼");
        btn_d.setFixedSize(p_size, p_size)
        btn_d.clicked.connect(lambda: self._pan_map(0, 150))

        btn_l = QtWidgets.QPushButton("◀");
        btn_l.setFixedSize(p_size, p_size)
        btn_l.clicked.connect(lambda: self._pan_map(-150, 0))

        btn_r = QtWidgets.QPushButton("▶");
        btn_r.setFixedSize(p_size, p_size)
        btn_r.clicked.connect(lambda: self._pan_map(150, 0))

        lbl_nav = QtWidgets.QLabel("🧭")
        lbl_nav.setAlignment(Qt.AlignCenter)

        g_pan.addWidget(btn_u, 0, 1)
        g_pan.addWidget(btn_l, 1, 0)
        g_pan.addWidget(lbl_nav, 1, 1)
        g_pan.addWidget(btn_r, 1, 2)
        g_pan.addWidget(btn_d, 2, 1)

    def _pan_map(self, dx, dy):
        """Pan the map with the buttons."""
        h_bar = self.view.horizontalScrollBar()
        v_bar = self.view.verticalScrollBar()
        h_bar.setValue(h_bar.value() + dx)
        v_bar.setValue(v_bar.value() + dy)

    def resizeEvent(self, e: QtGui.QResizeEvent) -> None:
        """Pin the buttons to the corners when the window is resized."""
        super().resizeEvent(e)

        margin = 15  # Margin

        # 1. ZOOM PANEL -> TOP LEFT CORNER
        # Update its size first
        self.zoom_widget.adjustSize()
        # Align left (X = margin)
        self.zoom_widget.move(margin, margin)

        # 2. PAN PANEL -> TOP RIGHT CORNER
        # Update its size first
        self.pan_widget.adjustSize()

        # RIGHT ALIGNMENT:
        # X = (window width) - (panel width) - (margin)
        pan_width = self.pan_widget.width()
        pan_x = self.width() - pan_width - margin

        self.pan_widget.move(pan_x, margin)
    def snapshot_ghost_trail(self):
        """Freeze the red track into a grey ghost track and clear the live one."""
        self.ghost_path.addPath(self.path_path)
        self.ghost_item.setPath(self.ghost_path)

        # Reset the line
        self.path_path = QtGui.QPainterPath()
        self.path_item.setPath(self.path_path)

        # Clear the live track, keep only the last position so the new track starts there
        if self.trail_points_1:
            self.trail_points_1 = [self.trail_points_1[-1]]

    def change_map(self, image_path: str, north: float, south: float, west: float, east: float):
        """Change the map image and its coordinate bounds at run time."""
        self.image_path = image_path
        self.north, self.south, self.west, self.east = north, south, west, east

        # 1. Remove the old map image from the scene
        if hasattr(self, 'image_item') and self.image_item:
            self.scene.removeItem(self.image_item)
            self.image_item = None

        # 2. Load the new image (HugeImageItem or QPixmap)
        if not os.path.exists(self.image_path):
            # No file: a grey placeholder
            pm = QtGui.QPixmap(800, 600)
            pm.fill(QtGui.QColor("#333"))
            painter = QtGui.QPainter(pm)
            painter.setPen(QtGui.QPen(Qt.white))
            painter.drawText(pm.rect(), Qt.AlignCenter, f"Map could not be loaded:\n{os.path.basename(self.image_path)}")
            painter.end()
            self._img_W, self._img_H = pm.width(), pm.height()
            self.image_item = self.scene.addPixmap(pm)
        else:
            # There is one: use HugeImageItem
            self.image_item = HugeImageItem(self.image_path, tile_size=2048, cache_limit=128)
            self.scene.addItem(self.image_item)
            self._img_W, self._img_H = self.image_item.width, self.image_item.height

        # 3. Send the image to the back (Z=0)
        self.image_item.setZValue(0)

        # 4. Update the scene size and centre it
        self.view.setSceneRect(QtCore.QRectF(0, 0, self._img_W, self._img_H))
        self.center_image()

        # 5. Clear the old drawings (waypoints, route) or move them to the new coordinates
        # (old waypoints mean nothing on a new map, so they are deleted)
        self.path_path = QtGui.QPainterPath()
        self.path_item.setPath(self.path_path)
        self.path_path_2 = QtGui.QPainterPath()
        self.path_item_2.setPath(self.path_path_2)
        # To clear the waypoints as well:
        for item in self.wp_group.childItems(): self.scene.removeItem(item)

    def update_range_rings(self):
        """Draw range rings round the current position."""
        # Clear the old ones first
        for item in self.ring_group.childItems():
            self.scene.removeItem(item)

        if not self.rings_enabled:
            return

        # No boat position: nothing to draw
        if self._last_lat is None or self._last_lon is None:
            return

        # --- MATHS: how many pixels is 1 metre? ---
        # In WebMercator the scale changes with latitude.
        # Simply: how many pixels does going 'step' metres east of the boat move?

        lat = self._last_lat
        lon = self._last_lon

        # Centre (boat) pixel
        center_pt = self._scene_pos_from_latlon(lat, lon)
        if center_pt is None: return

        # Reference point: 'step' metres east
        # (uses offset_latlon, added earlier)
        _, lon_east = self._offset_latlon(lat, lon, 0, self.ring_step_m)
        east_pt = self._scene_pos_from_latlon(lat, lon_east)

        if east_pt is None: return

        # Radius ('step' metres in pixels)
        # The X difference is enough (a circle)
        radius_px_base = abs(east_pt.x() - center_pt.x())

        # --- DRAWING ---
        pen = QtGui.QPen(QtGui.QColor("#00e676"))  # Radar green
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)  # A dashed line looks like a radar

        font = QtGui.QFont("Arial", 8)
        font.setBold(True)

        for i in range(1, self.ring_count + 1):
            r_px = radius_px_base * i
            dist_label = int(self.ring_step_m * i)

            # Ring (ellipse)
            # x, y, w, h -> top-left corner and size
            ellipse = QtWidgets.QGraphicsEllipseItem(
                center_pt.x() - r_px,
                center_pt.y() - r_px,
                r_px * 2,
                r_px * 2
            )
            ellipse.setPen(pen)
            self.ring_group.addToGroup(ellipse)

            # Label (e.g. "10m") - right above the ring
            txt = QtWidgets.QGraphicsTextItem(f"{dist_label}m")
            txt.setFont(font)
            txt.setDefaultTextColor(QtGui.QColor("#00e676"))
            # Centre the text
            br = txt.boundingRect()
            txt.setPos(center_pt.x() - br.width() / 2, center_pt.y() - r_px - br.height())
            self.ring_group.addToGroup(txt)

    def clear_ghost_trail(self):
        """Hayalet izi tamamen siler."""
        self.ghost_path = QtGui.QPainterPath()
        self.ghost_item.setPath(self.ghost_path)

    def start_defining_course(self, course_name):
        """Turn on the corner picking mode."""
        self._course_mode = course_name
        self._temp_corners = []
        # Clear the old ones if any
        self.clear_course_visuals(course_name)
        # Change the mouse cursor
        self.view.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.view.setCursor(Qt.CrossCursor)

    def _latlon_interp(self, p1, p2, ratio):
        """Return the point a fraction of the way between two lat/lons (linear interpolation)."""
        lat = p1[0] + (p2[0] - p1[0]) * ratio
        lon = p1[1] + (p2[1] - p1[1]) * ratio
        return (lat, lon)

    def render_course_grid(self, name):
        """Draw the grid of the named course (more transparent version)."""
        data = self.courses[name]
        corners = data["corners"]
        rows = data["rows"]
        cols = data["cols"]
        base_color = data["color"]  # The pure colour (e.g. cyan)
        grp = self.course_groups[name]

        # Clear the group first
        for item in grp.childItems():
            self.scene.removeItem(item)

        if len(corners) != 4:
            return

        # --- COLOURS (TRANSPARENCY) ---
        # 1. Line colour (half transparent)
        line_color = QtGui.QColor(base_color)
        line_color.setAlpha(40)  # 0-255 (120 = 50% visible) ____120

        # 2. Fill colour (very transparent)
        fill_color = QtGui.QColor(base_color)
        fill_color.setAlpha(5)  # Almost invisible ____ 25

        # 3. Corner point colour (a bit stronger)
        corner_color = QtGui.QColor(base_color)
        corner_color.setAlpha(40) #_______180

        # -------------------------------------

        # 1. Draw the corners (smaller dots)
        for i, (lat, lon) in enumerate(corners):
            p = self._scene_pos_from_latlon(lat, lon)
            if p:
                r = 3  # Radius a little smaller (4->3)
                el = QtWidgets.QGraphicsEllipseItem(p.x() - r, p.y() - r, 2 * r, 2 * r)
                el.setBrush(QtGui.QBrush(corner_color))
                el.setPen(Qt.NoPen)  # No outline
                grp.addToGroup(el)

                # Label (1,2,3,4) - slightly transparent
                txt = QtWidgets.QGraphicsTextItem(f"{name}{i + 1}")
                txt.setDefaultTextColor(line_color)
                f = txt.font();
                f.setPixelSize(9);
                txt.setFont(f)
                txt.setPos(p + QtCore.QPointF(4, -12))
                grp.addToGroup(txt)

        # 2. Draw the outer frame (polygon)
        pts_scene = [self._scene_pos_from_latlon(c[0], c[1]) for c in corners]
        if all(pts_scene):
            poly = QtWidgets.QGraphicsPolygonItem(QtGui.QPolygonF(pts_scene))

            # Frame line
            pen = QtGui.QPen(line_color)
            pen.setWidth(2)
            poly.setPen(pen)

            # Inner fill
            poly.setBrush(QtGui.QBrush(fill_color))
            grp.addToGroup(poly)

        # 3. Grid drawing (bilinear)
        c0, c1, c2, c3 = corners[0], corners[1], corners[2], corners[3]

        grid_pen = QtGui.QPen(line_color)
        grid_pen.setStyle(Qt.DotLine)  # Dotted line
        grid_pen.setWidth(1)

        # Vertical lines (columns)
        if cols > 0:
            for i in range(1, cols + 1):
                ratio = i / (cols + 1)
                top_pt_ll = self._latlon_interp(c0, c1, ratio)
                bot_pt_ll = self._latlon_interp(c3, c2, ratio)

                p_top = self._scene_pos_from_latlon(*top_pt_ll)
                p_bot = self._scene_pos_from_latlon(*bot_pt_ll)

                if p_top and p_bot:
                    l = QtWidgets.QGraphicsLineItem(QtCore.QLineF(p_top, p_bot))
                    l.setPen(grid_pen)
                    grp.addToGroup(l)

        # Horizontal lines (rows)
        if rows > 0:
            for i in range(1, rows + 1):
                ratio = i / (rows + 1)
                left_pt_ll = self._latlon_interp(c0, c3, ratio)
                right_pt_ll = self._latlon_interp(c1, c2, ratio)

                p_left = self._scene_pos_from_latlon(*left_pt_ll)
                p_right = self._scene_pos_from_latlon(*right_pt_ll)

                if p_left and p_right:
                    l = QtWidgets.QGraphicsLineItem(QtCore.QLineF(p_left, p_right))
                    l.setPen(grid_pen)
                    grp.addToGroup(l)

    def clear_course_visuals(self, name):
        # Delete the permanent grid first
        grp = self.course_groups.get(name)
        if grp:
            for item in grp.childItems():
                self.scene.removeItem(item)

        # --- CHANGE: if drawing was left half done, delete those white dots too ---
        if self._course_mode == name:
            self._clear_temp_visuals()
            self._temp_corners = []  # Reset the logical list too

    def _clear_temp_visuals(self):
        """Remove the temporary white dots from the scene."""
        for item in self._temp_corner_items:
            try:
                self.scene.removeItem(item)
            except:
                pass
        self._temp_corner_items = []

    def start_fence_drawing(self):
        """Start drawing mode and clear the old fence."""
        self._fence_mode = True
        self._fence_points_latlon = []
        self.fence_polygon.setPolygon(QtGui.QPolygonF())  # Clear the drawing
        self.view.setDragMode(QtWidgets.QGraphicsView.NoDrag)  # Keep the map still while drawing

    def finish_fence_drawing(self):
        """Finish drawing."""
        self._fence_mode = False
        self.view.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)  # Back to scroll mode

    def clear_fence(self):
        """Delete the fence entirely."""
        self._fence_points_latlon = []
        self.fence_polygon.setPolygon(QtGui.QPolygonF())

    def get_fence_points(self):
        return self._fence_points_latlon

    def _on_context_menu(self, pos: QtCore.QPoint):
        """Open the menu on a right click on the map."""
        scene_pos = self.view.mapToScene(pos)
        lat, lon = self._latlon_from_scene(scene_pos.x(), scene_pos.y())

        menu = QtWidgets.QMenu(self)
        menu.setStyleSheet("""
            QMenu { background-color: #333; color: white; border: 1px solid #555; }
            QMenu::item:selected { background-color: #555; }
        """)

        header = menu.addAction(f"K: {lat:.5f}, {lon:.5f}")
        header.setEnabled(False)
        menu.addSeparator()

        # --- USV 1 MENU ---
        menu_ida1 = menu.addMenu("🔴 For USV 1...")

        # USV 1: add new
        action_add1 = menu_ida1.addAction("➕ Add to the end of the list")
        action_add1.triggered.connect(lambda: self.gps_selected.emit(1, lat, lon, -1))

        menu_ida1.addSeparator()

        # USV 1: change existing points (one entry per point on screen)
        for i in range(len(self.cached_pts_1)):
            action_update = menu_ida1.addAction(f"✏️ Change point {i + 1}")
            # m=i captures i inside the lambda (Python closure trick)
            action_update.triggered.connect(lambda checked=False, m=i: self.gps_selected.emit(1, lat, lon, m))

        menu.exec(self.view.mapToGlobal(pos))

    def _latlon_from_scene(self, px: float, py: float) -> tuple[float, float]:
        x_left = self._lon_to_x(self.west)
        x_right = self._lon_to_x(self.east)
        y_top = self._lat_to_y(self.north)
        y_bot = self._lat_to_y(self.south)

        W = float(self._img_W)
        H = float(self._img_H)

        x_merc = x_left + (px / W) * (x_right - x_left)
        y_merc = y_top - (py / H) * (y_top - y_bot)

        lon = math.degrees(x_merc / self._R)
        lat = math.degrees(2.0 * math.atan(math.exp(y_merc / self._R)) - math.pi / 2.0)
        return lat, lon

    # Add inside the MapPanel class:
    def start_smart_ruler(self):
        """Take the mouse position as the start point when Shift is pressed."""
        if self._ruler_active: return  # Already on: do not start again

        # The mouse's position on the screen (global)
        global_pos = QtGui.QCursor.pos()
        # Turn it into the view's position
        view_pos = self.view.mapFromGlobal(global_pos)

        # Mouse outside the map: do not start
        if not self.view.rect().contains(view_pos):
            return

        # Turn it into scene coordinates
        scene_pos = self.view.mapToScene(view_pos)

        # Set the state
        self._ruler_active = True
        self._ruler_start_scene = scene_pos
        self._ruler_start_lat, self._ruler_start_lon = self._latlon_from_scene(scene_pos.x(), scene_pos.y())

        # Make the drawings visible
        self.ruler_line.setVisible(True)
        self.ruler_text.setVisible(True)
        self.ruler_bg.setVisible(True)

        # First line update (starts as a point)
        self.ruler_line.setLine(scene_pos.x(), scene_pos.y(), scene_pos.x(), scene_pos.y())

    def stop_smart_ruler(self):
        """Close and hide the ruler."""
        self._ruler_active = False
        self.ruler_line.setVisible(False)
        self.ruler_text.setVisible(False)
        self.ruler_bg.setVisible(False)

    def toggle_smart_ruler(self):
        """Close the ruler if open, start it if closed."""
        if self._ruler_active:
            self.stop_smart_ruler()
        else:
            self.start_smart_ruler()

    @QtCore.Slot(QtCore.QPointF)
    def _on_mouse_move(self, scene_pt: QtCore.QPointF):
        # the red + follows the cursor
        self.crosshair.setPos(scene_pt)

        # --- RULER UPDATE ---
        if self._ruler_active and self._ruler_start_lat is not None:
            # 1. Update the line
            line = QtCore.QLineF(self._ruler_start_scene, scene_pt)
            self.ruler_line.setLine(line)

            # 2. Work out the distance
            curr_lat, curr_lon = self._latlon_from_scene(scene_pt.x(), scene_pt.y())
            dist_m = haversine_distance(self._ruler_start_lat, self._ruler_start_lon, curr_lat, curr_lon)
            bearing_deg = calculate_bearing(self._ruler_start_lat, self._ruler_start_lon, curr_lat, curr_lon)

            # 3. Update the text
            txt = f"{dist_m:.2f} m\n{bearing_deg:.1f}°"
            self.ruler_text.setPlainText(txt)

            # 4. Put the text a little beside the cursor
            self.ruler_text.setPos(scene_pt + QtCore.QPointF(15, 15))

            # 5. Fit the background box to the text
            br = self.ruler_text.boundingRect()
            self.ruler_bg.setRect(br)
            self.ruler_bg.setPos(self.ruler_text.pos())

    def _ascend_to_label_group(self, item: QtWidgets.QGraphicsItem | None):
        it = item
        while it is not None:
            if isinstance(it, QtWidgets.QGraphicsItemGroup) and it.data(0) == "click_label":
                return it
            it = it.parentItem()
        return None

    def _relayout_click_labels(self, *_):
        scale = max(1e-9, self.view.transform().m11())
        ofs_scene = QtCore.QPointF(self._label_offset.x() / scale, self._label_offset.y() / scale)
        for it in list(self._click_labels):
            grp = it.get("grp")
            line = it.get("line")
            scene_pt = it.get("scene_pt")
            bg_rect = it.get("bg_rect")
            if not grp or not line:
                continue
            # place the label again
            grp.setPos(scene_pt + ofs_scene)
            # work out the line start in the scene again
            start_scene = grp.pos() + QtCore.QPointF(bg_rect.left() / scale, bg_rect.top() / scale)
            line.setLine(QtCore.QLineF(start_scene, scene_pt))

    def _add_click_label(self, scene_pt: QtCore.QPointF, lat: float, lon: float):
        # --- TEXT ---
        txt = QtWidgets.QGraphicsTextItem(f"{lat:.6f}, {lon:.6f}")
        f = txt.font()
        f.setPointSizeF(8.5)
        f.setBold(True)
        txt.setFont(f)
        txt.setDefaultTextColor(QtGui.QColor("#ffffff"))
        txt.setAcceptedMouseButtons(Qt.NoButton)

        # --- BACKGROUND ---
        br = txt.boundingRect()
        bg_rect = QtCore.QRectF(-4, -2, br.width() + 8, br.height() + 4)

        class _BgRect(QtWidgets.QGraphicsRectItem):
            def __init__(self, rect, closer):
                super().__init__(rect)
                self._closer = closer

            def mousePressEvent(self, e):
                if e.button() == Qt.LeftButton:
                    self._closer()
                else:
                    super().mousePressEvent(e)

        # --- GROUP (label of fixed pixel size) ---
        grp = QtWidgets.QGraphicsItemGroup()
        grp.setOpacity(0.30)
        grp.setZValue(600)
        grp.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)

        bg = _BgRect(bg_rect, closer=lambda: None)
        bg.setBrush(QtGui.QBrush(QtGui.QColor(0, 0, 0)))
        bg.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0)))

        grp.addToGroup(bg)
        grp.addToGroup(txt)

        # --- POSITION: normalise the offset by the scale ---
        scale = max(1e-9, self.view.transform().m11())
        ofs_scene = QtCore.QPointF(self._label_offset.x() / scale, self._label_offset.y() / scale)
        grp.setPos(scene_pt + ofs_scene)
        self.scene.addItem(grp)

        # --- RED LINE: from the top LEFT corner to the pixel ---
        start_scene = grp.pos() + QtCore.QPointF(bg_rect.left() / scale, bg_rect.top() / scale)
        line = self.scene.addLine(QtCore.QLineF(start_scene, scene_pt),
                                  QtGui.QPen(QtGui.QColor("#ff1744"), KIRMIZI_CIZGI_KALINLIK))
        line.setZValue(599)

        # --- CLOSING ---
        def _close():
            # remove from the scene
            try:
                self.scene.removeItem(line)
            except Exception:
                pass
            try:
                self.scene.removeItem(grp)
            except Exception:
                pass
            # drop from the list
            self._click_labels[:] = [it for it in self._click_labels if it.get("grp") is not grp]

        bg._closer = _close  # closes at once when the label is clicked

        # --- removed by itself after 10 s ---
        timer = QtCore.QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(_close)
        timer.start(10_000)  # ms

        # keep it in the list (to lay it out again on zoom)
        self._click_labels.append({
            "grp": grp,
            "bg_rect": QtCore.QRectF(bg_rect),
            "scene_pt": QtCore.QPointF(scene_pt),
            "line": line,
            "timer": timer,
        })

    @QtCore.Slot(QtCore.QPointF, QtCore.QPoint)
    def _on_map_click(self, scene_pt: QtCore.QPointF, global_pos: QtCore.QPoint):

        # 1. CLOSE THE RULER (on a left click)
        if self._ruler_active:
            self.stop_smart_ruler()  # <-- shortened
            return

        # 2. --- GEOFENCE DRAWING ---
        if self._fence_mode:
            # Take the coordinate
            lat, lon = self._latlon_from_scene(scene_pt.x(), scene_pt.y())
            self._fence_points_latlon.append((lat, lon))

            # Update the drawing (lat/lon list to scene points, then draw)
            poly_points = []
            for (flat, flon) in self._fence_points_latlon:
                p = self._scene_pos_from_latlon(flat, flon)
                if p: poly_points.append(p)

            self.fence_polygon.setPolygon(QtGui.QPolygonF(poly_points))
            return  # Do nothing else

        # 3. --- COURSE CORNERS ---
        if self._course_mode:
            lat, lon = self._latlon_from_scene(scene_pt.x(), scene_pt.y())
            self._temp_corners.append((lat, lon))

            # Put a temporary dot (visual feedback)
            r = 3
            dot = QtWidgets.QGraphicsEllipseItem(scene_pt.x() - r, scene_pt.y() - r, 2 * r, 2 * r)
            dot.setBrush(Qt.white)
            dot.setPen(QtGui.QPen(Qt.black))  # Black outline added so it shows better
            self.scene.addItem(dot)

            # --- CHANGE HERE: save the point in the list ---
            self._temp_corner_items.append(dot)

            # All 4 corners done?
            if len(self._temp_corners) == 4:
                # Save
                nm = self._course_mode
                self.courses[nm]["corners"] = list(self._temp_corners)

                # Leave the mode
                self._course_mode = None
                self.view.setDragMode(
                    QtWidgets.QGraphicsView.NoDrag)  # Turn off Qt's own dragging (ours works instead)
                self.view.setCursor(Qt.BlankCursor)  # Hide the cursor again (only the crosshair stays)

                # --- CHANGE HERE: clear the temporary white dots ---
                self._clear_temp_visuals()

                # Draw the grid
                self.render_course_grid(nm)

            return


    def _debug_corners(self):
        """Test the alignment by putting green crosses on the map's BBOX corners."""
        # Clear the old debug items
        try:
            for it in getattr(self, "_debug_items", []):
                self.scene.removeItem(it)
        except Exception:
            pass
        self._debug_items = []

        def cross(p, size=8, color="#00c853"):
            pen = QtGui.QPen(QtGui.QColor(color))
            pen.setWidthF(1.5)
            l1 = QtWidgets.QGraphicsLineItem(p.x() - size, p.y(), p.x() + size, p.y())
            l2 = QtWidgets.QGraphicsLineItem(p.x(), p.y() - size, p.x(), p.y() + size)
            l1.setPen(pen)
            l2.setPen(pen)
            l1.setZValue(30)
            l2.setZValue(30)
            # To keep it independent of the zoom:
            if globals().get("KEEP_PIXEL_SIZE", True):
                l1.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
                l2.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
            self.scene.addItem(l1)
            self.scene.addItem(l2)
            self._debug_items.extend([l1, l2])

        # A cross at the NW, NE, SW, SE corners
        for (lat, lon) in [
            (self.north, self.west), (self.north, self.east),
            (self.south, self.west), (self.south, self.east)
        ]:
            p = self._scene_pos_from_latlon(lat, lon)
            if p is not None:
                cross(p)

    # ---- Zoom / Pan ----
    def current_zoom(self) -> float:
        m = self.view.transform()
        # scaleX
        return m.m11()

    def set_zoom(self, z: float):
        z = max(self._min_zoom, min(self._max_zoom, z))
        cur = self.current_zoom()
        if cur == 0:
            return
        self.view.scale(z / cur, z / cur)

    def zoom(self, factor: float):
        self.set_zoom(self.current_zoom() * factor)

    def center_image(self):
        self.view.setSceneRect(QtCore.QRectF(0, 0, self._img_W, self._img_H))
        self.view.centerOn(self._img_W / 2.0, self._img_H / 2.0)

    # ---- Lat/lon -> scene coordinates ----
    def _lon_to_x(self, lon_deg: float) -> float:
        return self._R * math.radians(lon_deg)

    def _lat_to_y(self, lat_deg: float) -> float:
        # WebMercator (EPSG:3857)
        lat_rad = math.radians(lat_deg)
        return self._R * math.log(math.tan(math.pi/4.0 + lat_rad/2.0))

    def _scene_pos_from_latlon(self, lat: float, lon: float) -> QtCore.QPointF | None:
        # Do not draw what is outside
        if not (self.south <= lat <= self.north and self.west <= lon <= self.east):
            # You may still want a pixel for a point outside the map.
            # Clamp it here so it shows at the edge.
            lat = min(max(lat, self.south), self.north)
            lon = min(max(lon, self.west), self.east)

        x_left  = self._lon_to_x(self.west)
        x_right = self._lon_to_x(self.east)
        y_top   = self._lat_to_y(self.north)
        y_bot   = self._lat_to_y(self.south)

        x = self._lon_to_x(lon)
        y = self._lat_to_y(lat)

        # Pixmap size
        W = float(self._img_W)
        H = float(self._img_H)

        # WebMercator is positive up; QGraphics y is positive down.
        # So y is flipped.
        px = (x - x_left) / (x_right - x_left) * W
        py = (y_top - y)   / (y_top - y_bot)  * H

        return QtCore.QPointF(px, py)

    # ---- Geo helpers ----
    def _offset_latlon(self, lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
        """Small-offset approximation. north_m: +north, east_m: +east (metres)."""
        R = 6378137.0
        d_lat = (north_m / R) * (180.0 / math.pi)
        d_lon = (east_m / (R * math.cos(math.radians(lat_deg)))) * (180.0 / math.pi)
        return lat_deg + d_lat, lon_deg + d_lon

    def _end_latlon_from_heading(self, lat: float, lon: float, heading_deg: float, length_m: float) -> tuple[
        float, float]:
        """0 deg = true north, clockwise (+). The lat/lon after going length_m."""
        theta = math.radians(heading_deg if heading_deg is not None else 0.0)
        north_m = length_m * math.cos(theta)
        east_m = length_m * math.sin(theta)
        return self._offset_latlon(lat, lon, north_m, east_m)

    def _update_heading_items(self):
        lat = self._last_lat;
        lon = self._last_lon
        if lat is None or lon is None:
            self._real_line.setVisible(False)
            self._real_arrow.setVisible(False)
            self._targ_line.setVisible(False)
            return

        # --- Real heading (3 m + arrow) ---
        if self._real_hdg_deg is not None:
            lat2, lon2 = self._end_latlon_from_heading(lat, lon, self._real_hdg_deg, 3.0)
            p1 = self._scene_pos_from_latlon(lat, lon)
            p2 = self._scene_pos_from_latlon(lat2, lon2)
            self._real_line.setLine(p1.x(), p1.y(), p2.x(), p2.y())
            self._real_line.setVisible(True)

            # Arrow head: a V near the tip (+/-25 deg)
            arrow_len = 0.8  # m
            left_dir = (self._real_hdg_deg + 180 - 25) % 360
            right_dir = (self._real_hdg_deg + 180 + 25) % 360
            latL, lonL = self._end_latlon_from_heading(lat2, lon2, left_dir, arrow_len)
            latR, lonR = self._end_latlon_from_heading(lat2, lon2, right_dir, arrow_len)
            pL = self._scene_pos_from_latlon(latL, lonL)
            pR = self._scene_pos_from_latlon(latR, lonR)

            path = QtGui.QPainterPath()
            path.moveTo(pL)
            path.lineTo(p2)
            path.lineTo(pR)
            self._real_arrow.setPath(path)
            self._real_arrow.setVisible(True)
        else:
            self._real_line.setVisible(False)
            self._real_arrow.setVisible(False)

        # --- Target heading (5 m, no arrow) ---
        if self._targ_hdg_deg is not None:
            lat3, lon3 = self._end_latlon_from_heading(lat, lon, self._targ_hdg_deg, 5.0)
            p1 = self._scene_pos_from_latlon(lat, lon)
            p3 = self._scene_pos_from_latlon(lat3, lon3)
            self._targ_line.setLine(p1.x(), p1.y(), p3.x(), p3.y())
            self._targ_line.setVisible(True)
        else:
            self._targ_line.setVisible(False)

    # ---- Public setters ----
    def set_real_heading(self, hdg_deg: float | None):
        try:
            self._real_hdg_deg = float(hdg_deg) if hdg_deg is not None else None
        except Exception:
            self._real_hdg_deg = None
        self._update_heading_items()

    def set_target_heading(self, hdg_deg: float | None):
        try:
            self._targ_hdg_deg = float(hdg_deg) if hdg_deg is not None else None
        except Exception:
            self._targ_hdg_deg = None
        self._update_heading_items()

    def _prune_and_redraw_trail(self, boat_id):
        """Limit the track to 500 m and draw the map again."""
        trail = self.trail_points_1 if boat_id == 1 else self.trail_points_2
        if len(trail) < 2:
            return

        max_dist_m = 400.0  # LONGEST TRACK (METRES)
        total_dist = 0.0
        new_trail = [trail[-1]]

        # Add up the distances from the end back (newest point to oldest)
        for i in range(len(trail) - 2, -1, -1):
            d = haversine_distance(trail[i][0], trail[i][1], trail[i + 1][0], trail[i + 1][1])
            total_dist += d
            new_trail.append(trail[i])
            # Once the total passes 500 m, older points are not taken!
            if total_dist >= max_dist_m:
                break

        new_trail.reverse()  # Put the list back in time order

        if boat_id == 1:
            self.trail_points_1 = new_trail
        else:
            self.trail_points_2 = new_trail

        # Draw the QPainterPath again with the new, trimmed points
        painter_path = QtGui.QPainterPath()
        first = True
        for lat, lon in new_trail:
            p = self._scene_pos_from_latlon(lat, lon)
            if p:
                if first:
                    painter_path.moveTo(p)
                    first = False
                else:
                    painter_path.lineTo(p)

        # Put the line on the map
        if boat_id == 1:
            self.path_path = painter_path
            self.path_item.setPath(self.path_path)
        else:
            self.path_path_2 = painter_path
            self.path_item_2.setPath(self.path_path_2)

    def update_current_position(self, lat: float, lon: float, boat_id=1, draw_tail=True):
        # 1. EMA filter (low-pass) - stops the zigzags
        alpha = 0.3  # Smoothing factor (0 to 1, 1 = no filter, low = smooth but late)
        dist_threshold_m = 50.0  # Jumps over 50 m reset the filter (teleport / first fix)

        if boat_id == 1:
            if getattr(self, '_target_lat_1', None) is None:
                # First position
                self._target_lat_1, self._target_lon_1 = lat, lon
                self._visual_lat_1, self._visual_lon_1 = lat, lon
            else:
                dist = haversine_distance(self._target_lat_1, self._target_lon_1, lat, lon)
                if dist > dist_threshold_m:
                    # Big jump: go straight there
                    self._target_lat_1, self._target_lon_1 = lat, lon
                    self._visual_lat_1, self._visual_lon_1 = lat, lon
                else:
                    # EMA smoothing
                    self._target_lat_1 = (alpha * lat) + ((1.0 - alpha) * self._target_lat_1)
                    self._target_lon_1 = (alpha * lon) + ((1.0 - alpha) * self._target_lon_1)

            self._last_lat, self._last_lon = self._target_lat_1, self._target_lon_1
            self._update_heading_items()
            self.update_range_rings()

        elif boat_id == 2:
            if getattr(self, '_target_lat_2', None) is None:
                self._target_lat_2, self._target_lon_2 = lat, lon
                self._visual_lat_2, self._visual_lon_2 = lat, lon
            else:
                dist = haversine_distance(self._target_lat_2, self._target_lon_2, lat, lon)
                if dist > dist_threshold_m:
                    self._target_lat_2, self._target_lon_2 = lat, lon
                    self._visual_lat_2, self._visual_lon_2 = lat, lon
                else:
                    self._target_lat_2 = (alpha * lat) + ((1.0 - alpha) * self._target_lat_2)
                    self._target_lon_2 = (alpha * lon) + ((1.0 - alpha) * self._target_lon_2)

    def _animate_markers(self):
        """Called 30 times a second by a timer. Slides the marker smoothly to the EMA target position (tweening)."""
        interp_factor = 0.2  # Each animation frame goes 20% of the way to the target

        # Counters for performance (if not there yet)
        if not hasattr(self, 'prune_counter_1'): self.prune_counter_1 = 0
        if not hasattr(self, 'prune_counter_2'): self.prune_counter_2 = 0

        # --- USV 1 ---
        if getattr(self, '_visual_lat_1', None) is not None and getattr(self, '_target_lat_1', None) is not None:
            # Interpolation
            self._visual_lat_1 += (self._target_lat_1 - self._visual_lat_1) * interp_factor
            self._visual_lon_1 += (self._target_lon_1 - self._visual_lon_1) * interp_factor

            p = self._scene_pos_from_latlon(self._visual_lat_1, self._visual_lon_1)
            if p is not None:
                self.curr_marker.setPos(p)
                self.curr_marker.setVisible(True)

                # Track (trail) drawing - now with the smoothed display coordinates
                draw_tail = True  # Could be a parameter from outside
                if draw_tail:
                    if not self.trail_points_1 or self.trail_points_1[-1] != (self._visual_lat_1, self._visual_lon_1):
                        self.trail_points_1.append((self._visual_lat_1, self._visual_lon_1))

                        if self.path_path.isEmpty():
                            self.path_path.moveTo(p)
                        else:
                            self.path_path.lineTo(p)
                        self.path_item.setPath(self.path_path)

                        self.prune_counter_1 += 1
                        if self.prune_counter_1 > 50:
                            self._prune_and_redraw_trail(1)
                            self.prune_counter_1 = 0

        # --- USV 2 ---
        if getattr(self, '_visual_lat_2', None) is not None and getattr(self, '_target_lat_2', None) is not None and hasattr(self, 'curr_marker_2'):
            self._visual_lat_2 += (self._target_lat_2 - self._visual_lat_2) * interp_factor
            self._visual_lon_2 += (self._target_lon_2 - self._visual_lon_2) * interp_factor

            p2 = self._scene_pos_from_latlon(self._visual_lat_2, self._visual_lon_2)
            if p2 is not None:
                self.curr_marker_2.setPos(p2)
                self.curr_marker_2.setVisible(True)

                draw_tail = True
                if draw_tail and hasattr(self, 'path_path_2'):
                    if not self.trail_points_2 or self.trail_points_2[-1] != (self._visual_lat_2, self._visual_lon_2):
                        self.trail_points_2.append((self._visual_lat_2, self._visual_lon_2))

                        if self.path_path_2.isEmpty():
                            self.path_path_2.moveTo(p2)
                        else:
                            self.path_path_2.lineTo(p2)
                        self.path_item_2.setPath(self.path_path_2)

                        self.prune_counter_2 += 1
                        if self.prune_counter_2 > 50:
                            self._prune_and_redraw_trail(2)
                            self.prune_counter_2 = 0

    def clear_ghost_trail_by_id(self, boat_id):
        """Clear the boat's live track (and its ghost track, if any) entirely."""
        if boat_id == 1:
            # Clear the ghost track
            self.ghost_path = QtGui.QPainterPath()
            self.ghost_item.setPath(self.ghost_path)

            # Clear the live track (red)
            self.trail_points_1 = []
            self.path_path = QtGui.QPainterPath()
            self.path_item.setPath(self.path_path)
        elif boat_id == 2:
            # Clear USV 2's live track (blue)
            self.trail_points_2 = []
            self.path_path_2 = QtGui.QPainterPath()
            self.path_item_2.setPath(self.path_path_2)

    def update_mission_waypoints(self, boat_id, local_points, remote_points=[], tags=None):
        # 1. Keep the data that came in
        if boat_id == 1:
            self.cached_pts_1 = local_points
        else:
            self.cached_pts_2 = local_points

        other_list = self.cached_pts_2 if boat_id == 1 else self.cached_pts_1

        # Prepare the groups
        if not hasattr(self, 'mission_grp_1'):
            self.mission_grp_1 = QtWidgets.QGraphicsItemGroup()
            self.scene.addItem(self.mission_grp_1)
            self.mission_grp_1.setZValue(25)
        if not hasattr(self, 'mission_grp_2'):
            self.mission_grp_2 = QtWidgets.QGraphicsItemGroup()
            self.scene.addItem(self.mission_grp_2)
            self.mission_grp_2.setZValue(25)

        grp = self.mission_grp_1 if boat_id == 1 else self.mission_grp_2

        # Clear the group (delete old drawings)
        for child in grp.childItems():
            self.scene.removeItem(child)

        # Colours
        if boat_id == 1:
            def_color = QtGui.QColor("#ff1744")  # Red
            text_color = QtGui.QColor("#ff8a80")  # Light red (text)
            y_offset_factor = -1  # WRITE ABOVE
        else:
            def_color = QtGui.QColor("#2979ff")  # Blue
            text_color = QtGui.QColor("#80d8ff")  # Light blue (text)
            y_offset_factor = 1  # WRITE BELOW

        for i, (lat, lon) in enumerate(local_points):
            p = self._scene_pos_from_latlon(lat, lon)
            if not p: continue

            # --- State check (confirmed / shared) ---
            is_verified = False
            for (r_lat, r_lon) in remote_points:
                if abs(lat - r_lat) < 0.000001 and abs(lon - r_lon) < 0.000001:
                    is_verified = True
                    break

            is_shared = False
            if not is_verified:
                for (o_lat, o_lon) in other_list:
                    if abs(lat - o_lat) < 0.000001 and abs(lon - o_lon) < 0.000001:
                        is_shared = True
                        break

            # Point colour
            if is_verified:
                color = QtGui.QColor("#00e676")  # Green (confirmed)
            elif is_shared:
                color = QtGui.QColor("#ff9100")  # Orange (clash)
            else:
                color = def_color

            # --- 1. Draw the dot ---
            r = 1.5
            dot = QtWidgets.QGraphicsEllipseItem(p.x() - r, p.y() - r, 2 * r, 2 * r)
            dot.setBrush(QtGui.QBrush(color))
            dot.setPen(QtGui.QPen(QtGui.QColor("white"), 1))
            grp.addToGroup(dot)

            # --- 2. Prepare the label ---
            label_text = str(i + 1)
            # Shorten the task name (TASK1_START -> T1_START) to save room
            if tags and i < len(tags):
                short_tag = tags[i].replace("TASK", "T").replace("_", " ")
                label_text += f" {short_tag}"

            txt = QtWidgets.QGraphicsSimpleTextItem(label_text)
            f = txt.font()
            f.setPixelSize(4)  # Readable size
            f.setBold(True)
            txt.setFont(f)
            txt.setBrush(QtGui.QBrush(text_color))

            # --- 3. Position (AVOIDING CLASHES) ---
            br = txt.boundingRect()

            # X: put the text a little right of the point
            txt_x = p.x() + 6

            # Y: up or down by ID
            if boat_id == 1:
                # USV 1: ABOVE the point (its height up + a gap)
                txt_y = p.y() - br.height() - 4
            else:
                # USV 2: BELOW the point (straight down + a gap)
                txt_y = p.y() + 4

            txt.setPos(txt_x, txt_y)

            # --- 4. Background box (for readability) ---
            # A half-transparent black box behind the text
            bg_rect = QtWidgets.QGraphicsRectItem(
                txt_x - 2, txt_y - 1,
                br.width() + 4, br.height() + 2
            )
            bg_rect.setBrush(QtGui.QBrush(QtGui.QColor(0, 0, 0, 30)))  # 60% black
            bg_rect.setPen(Qt.NoPen)

            grp.addToGroup(bg_rect)  # Add the box first (it stays below)
            grp.addToGroup(txt)  # Then the text (on top)

            # Optional: a thin line from the box to the point (leader line)
            line = QtWidgets.QGraphicsLineItem(p.x(), p.y(), txt_x, txt_y + br.height() / 2)
            line.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 100), 1))
            grp.addToGroup(line)

    def set_waypoints(self, gps_dict: dict):
        """gps_dict expected format: {"GPS1":{"lat":..,"lon":..}, ...}"""
        if not isinstance(gps_dict, dict):
            return

        # Clear the old items
        try:
            for item in list(self.wp_group.childItems()):
                self.scene.removeItem(item)
            # Make the group again (in some Qt versions a group misbehaves once its childItems are cleared)
            self.scene.removeItem(self.wp_group)
        except Exception:
            pass

        self.wp_group = QtWidgets.QGraphicsItemGroup()
        self.wp_group.setZValue(15)
        self.scene.addItem(self.wp_group)

        # Sizes / constants (defaults if missing)
        r_px = globals().get("WP_RADIUS_PX", 4.0)
        font_pt = globals().get("WP_FONT_PT", 8.5)
        keep_px = globals().get("KEEP_PIXEL_SIZE", True)

        # Draw GPS1..GPS5 in order
        for i in range(1, 6):
            key = f"GPS{i}"
            entry = gps_dict.get(key)
            if not isinstance(entry, dict):
                continue
            try:
                lat = float(entry.get("lat"))
                lon = float(entry.get("lon"))
            except Exception:
                continue
            if lat is None or lon is None:
                continue

            p = self._scene_pos_from_latlon(lat, lon)
            if p is None:
                continue

            # --- Circle marker ---
            circ = QtWidgets.QGraphicsEllipseItem(-r_px, -r_px, 2 * r_px, 2 * r_px)
            circ.setBrush(QtGui.QBrush(QtGui.QColor("#1e88e5")))
            circ.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 1.0))
            circ.setPos(p)
            circ.setZValue(16)
            if keep_px:
                circ.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
            self.wp_group.addToGroup(circ)

            # --- Label ---
            label = QtWidgets.QGraphicsTextItem(key)
            # Font and style
            f = label.font()
            f.setPointSizeF(font_pt)
            f.setBold(True)
            label.setFont(f)

            # Colour: turquoise
            label.setDefaultTextColor(QtGui.QColor("#90EE90")) #000000 #30d5c8

            # Put the text at the CENTRE OF THE CIRCLE
            br = label.boundingRect()  # measure after the font is set
            label.setPos(p - QtCore.QPointF(br.width() / 2.0, br.height() / 2.0))

            label.setZValue(17)
            if keep_px:
                label.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)

            self.wp_group.addToGroup(label)


# -------------------- Main Window --------------------
# -------------------- Main Window --------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DHO GOKMAVI - GROUND CONTROL (Azure · Marine)")
        self.showMaximized()

        self.MISSION_TAGS_1 = [
            "TASK1_START",
            "TASK1_MID",
            "TASK1_END",
            "TASK2_START",
            "TASK2_MID",
            "TASK2_END",
            "TASK3_START",
            "TASK3_MID",
            "TASK3_RIGHT",
            "TASK3_END",
            "TASK3_LEFT",
            "DOCKING"
        ]


        # Style sheet (STRONGER, BRIGHT WHITE)
        self.setStyleSheet("""
                    QMainWindow {
                        background-color: #2b2b2b;
                        color: #e0e0e0;
                    }

                    /* --- STRONGER BUTTONS --- */
                    /* A more specific selector makes sure the style applies */
                    QWidget QPushButton {
                        background-color: #37474f;       /* Koyu Zemin */
                        color: #eceff1;                  /* Light grey/white text */
                        border: 2px solid #546e7a;       /* THICKER (2px) CLEAR BORDER */
                        border-radius: 4px;
                        padding: 4px;
                        font-weight: bold;
                        font-size: 11px;
                    }
                    /* Hover Efekti - PARLASIN */
                    QWidget QPushButton:hover {
                        background-color: #455a64;
                        border: 2px solid #81d4fa;       /* Bright light blue border */
                        color: #ffffff;                  /* Pure white text */
                    }
                    /* Pressed effect - sinks in */
                    QWidget QPushButton:pressed {
                        background-color: #263238;
                        border: 2px solid #263238;
                        padding-top: 6px;                /* Shift for a click feel */
                        padding-left: 6px;
                    }
                    /* Checked effect */
                    QWidget QPushButton:checked {
                        background-color: #00897b;
                        border: 2px solid #00e676;
                        color: white;
                    }

                    /* --- TASK TABLE (BRIGHT WHITE) --- */
                    QTableWidget {
                        background-color: #323639;       /* Koyu Zemin */
                        color: #ffffff;                  /* BRIGHT WHITE, BOLD TEXT */
                        font-weight: bold;
                        gridline-color: #546e7a;         /* Grid lines */
                        border: none;                    /* No outer border (matches the modern frame) */
                    }
                    /* Table headers */
                    QHeaderView::section {
                        background-color: #37474f;
                        color: #eceff1;
                        padding: 4px;
                        border: 1px solid #546e7a;
                        font-weight: bold;
                    }
                    /* Table selected row */
                    QTableWidget::item:selected {
                        background-color: #00e676;       /* Selected row bright green */
                        color: #000000;                  /* Black text on it so it reads */
                    }

                    /* --- OTHER WIDGETS --- */
                    QGroupBox {
                        border: 2px solid #444;          /* Make the border clear */
                        border-radius: 6px;
                        margin-top: 12px;
                        background-color: #323639;
                        font-weight: bold;
                    }
                    QGroupBox::title {
                        subcontrol-origin: margin;
                        left: 10px;
                        padding: 0 5px;
                        color: #4fc3f7;
                        background-color: #323639;       /* Title background */
                    }
                    QCheckBox {
                        spacing: 8px;
                        color: #b0bec5;
                        font-weight: bold;
                    }
                    QCheckBox::indicator {
                        width: 16px;                     /* A slightly bigger box */
                        height: 16px;
                        border: 2px solid #546e7a;
                        background: #263238;
                        border-radius: 3px;
                    }
                    QCheckBox::indicator:checked {
                        background-color: #00e676;
                        border-color: #00e676;
                    }
                    QLineEdit, QSpinBox, QComboBox {
                        background-color: #263238;
                        color: #ffffff;                  /* Inputs bright white too */
                        border: 2px solid #546e7a;
                        padding: 4px;
                        border-radius: 3px;
                        font-weight: bold;
                    }
                    QSplitter::handle {
                        background-color: #1c1c1c;
                    }
                    QLabel {
                        color: #cfd8dc;
                    }
                    /* Tooltip */
                    QToolTip {
                        background-color: #263238;
                        color: #00e676;
                        border: 2px solid #546e7a;
                        padding: 4px;
                        opacity: 230;
                        font-weight: bold;
                    }
                """)
        # Background image path (whatever the file is called: background.jpg or .png)
        bg_path = res_path("background.png").replace("\\", "/")

        # --- MAIN LAYOUT ---
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        # Main layout
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)  # No margins, full screen
        root.setSpacing(0)

        # Rope thickness (defined here so the code below does not fail)
        ROPE_SIZE = 12
        self._rope_thk = ROPE_SIZE

        # Content frame (the whole UI lives inside it)
        self.content = QtWidgets.QFrame(central)
        root.addWidget(self.content)

        # Content layout (the 3-column splitter goes here)
        content_lay = QtWidgets.QHBoxLayout(self.content)
        # Leave a margin of one rope thickness on each side so the content is not hidden under the rope
        content_lay.setContentsMargins(ROPE_SIZE, ROPE_SIZE, ROPE_SIZE, ROPE_SIZE)
        content_lay.setSpacing(0)

        # Rope decoration (added as an overlay on top of the content)
        # Check the image path; draws grey if missing
        rope_path = res_path("halat.png").replace("\\", "/")

        self.rope_top = RopeStripe("top", rope_path, ROPE_SIZE, parent=self.content)
        self.rope_bottom = RopeStripe("bottom", rope_path, ROPE_SIZE, parent=self.content)
        self.rope_left = RopeStripe("left", rope_path, ROPE_SIZE, parent=self.content)
        self.rope_right = RopeStripe("right", rope_path, ROPE_SIZE, parent=self.content)

        # Filter to line the ropes up again when the window is resized
        self.content.installEventFilter(self)

        # --- SPLITTER (LEFT | CENTRE | RIGHT) ---
        splitter = QtWidgets.QSplitter(Qt.Horizontal)
        content_lay.addWidget(splitter)

        # =========================================================
        # LEFT PANEL: USV 1 (Azure), RIGHT PANEL: USV 2 (Marine)  (_build_boat_panel)
        # =========================================================
        self.relay_lbls, self.link_lbls = {}, {}
        self.focus_group = QtWidgets.QButtonGroup(self)    # one boat on the keyboard
        splitter.addWidget(self._build_boat_panel(1, bg_path))

        # =========================================================
        # CENTRE PANEL: MAP & SHARED
        # =========================================================
        center_widget = QtWidgets.QWidget()
        vc = QtWidgets.QVBoxLayout(center_widget)

        # 1. Shared top (map, COM, connect)
        top_grp = QtWidgets.QGroupBox("System Settings")
        vc.addWidget(top_grp);
        top_lay = QtWidgets.QHBoxLayout(top_grp)

        self.MAP_PRESETS = {
            "Golcuk": {"file": "golcuk_harita18.jpeg", "n": 40.7329, "s": 40.7215, "w": 29.7936, "e": 29.8296},
            "RoboBoat (Florida)": {"file": "roboboat.jpg", "n": 27.381, "s": 27.356, "w": -82.455, "e": -82.445},
            "School (Tuzla)": {"file": "okul.jpg", "n": 40.810072, "s": 40.808099, "w": 29.261345, "e": 29.265630}
        }
        # Only the maps whose image is here (gcs/resources); RoboBoat first
        self.MAP_PRESETS = {k: v for k, v in sorted(
            self.MAP_PRESETS.items(), key=lambda kv: not kv[0].startswith("RoboBoat"))
            if os.path.exists(res_path(v["file"]))}
        self.cmb_maps = QtWidgets.QComboBox();
        self.cmb_maps.addItems(self.MAP_PRESETS.keys())
        self.cmb_maps.currentIndexChanged.connect(self._on_map_changed)
        top_lay.addWidget(QtWidgets.QLabel("Map:"));
        top_lay.addWidget(self.cmb_maps)

        # Ubiquiti (UDP) and RFD: both at once (RoboBoat 2027). "Network" picks where the boats
        # are: the field (their profile addresses) or this computer (simulation)
        self.chk_udp = QtWidgets.QCheckBox("Ubiquiti")
        self.chk_udp.setChecked(True)
        top_lay.addWidget(self.chk_udp)
        self.cmb_net = QtWidgets.QComboBox()
        self.cmb_net.addItems(["Field (profiles)", "Simulation (127.0.0.1)"])
        top_lay.addWidget(self.cmb_net)
        self.chk_rfd = QtWidgets.QCheckBox("RFD")
        self.chk_rfd.setChecked(True)
        top_lay.addWidget(self.chk_rfd)
        self.cmb_port_1 = QtWidgets.QComboBox()
        self.cmb_port_1.setEditable(True)        # the simulation's radio: /tmp/usv_rfd_gcs
        self._refresh_ports()
        top_lay.addWidget(self.cmb_port_1)

        self.cmb_baud = QtWidgets.QComboBox();
        self.cmb_baud.addItems(["57600", "115200"]);
        self.cmb_baud.setCurrentText("57600")
        top_lay.addWidget(self.cmb_baud)
        self.chk_csv = QtWidgets.QCheckBox("CSV");
        top_lay.addWidget(self.chk_csv)

        btn_ref = QtWidgets.QPushButton("R");
        btn_ref.setFixedWidth(30);
        btn_ref.clicked.connect(self._refresh_ports)
        top_lay.addWidget(btn_ref)
        self.btn_connect = QtWidgets.QPushButton("CONNECT");
        self.btn_connect.clicked.connect(self._toggle)
        top_lay.addWidget(self.btn_connect)
        # Each link: when it last brought anything
        self.lbl_links = QtWidgets.QLabel("UBNT -  RFD -")
        self.lbl_links.setStyleSheet("color:#90a4ae; font-weight:bold;")
        top_lay.addWidget(self.lbl_links)

        # 2. Map
        # Default map setting (taken from the combobox)
        current_map_key = self.cmb_maps.currentText()
        if current_map_key in self.MAP_PRESETS:
            first_map = self.MAP_PRESETS[current_map_key]
            self.map = MapPanel(res_path(first_map["file"]), first_map["n"], first_map["s"], first_map["w"],
                                first_map["e"])
        else:
            # Fallback
            self.map = MapPanel("roboboat.jpg", 27.381, 27.356, -82.455, -82.445)

        self.map.gps_selected.connect(self._on_map_gps_selected)
        vc.addWidget(self.map)

        # 3. Shared bottom (geofence, judges, course, UAV)
        bot_lay = QtWidgets.QHBoxLayout();
        vc.addLayout(bot_lay)

        # Geofence
        geo_box = QtWidgets.QGroupBox("Geofence");
        gl = QtWidgets.QGridLayout(geo_box)
        self.btn_fence_draw = QtWidgets.QPushButton("Draw");
        self.btn_fence_draw.setCheckable(True);
        self.btn_fence_draw.clicked.connect(self._toggle_fence_draw)
        self.btn_fence_clear = QtWidgets.QPushButton("Clear");
        self.btn_fence_clear.clicked.connect(lambda: self.map.clear_fence())
        self.chk_fence_active = QtWidgets.QCheckBox("ACTIVE");
        self.chk_fence_active.setStyleSheet("color:red; font-weight:bold;")
        gl.addWidget(self.btn_fence_draw, 0, 0);
        gl.addWidget(self.btn_fence_clear, 0, 1);
        gl.addWidget(self.chk_fence_active, 1, 0, 1, 2)
        bot_lay.addWidget(geo_box)

        # Course grids (A-B-C)
        self.course_ui = {}
        course_grp = QtWidgets.QGroupBox("Course");
        cl = QtWidgets.QVBoxLayout(course_grp)
        self.tabs_course = QtWidgets.QTabWidget();
        self.tabs_course.setFixedHeight(120)
        self.tabs_course.setMinimumWidth(250)
        for name in ["A", "B", "C"]:
            w = QtWidgets.QWidget()
            v_tab = QtWidgets.QVBoxLayout(w)
            v_tab.setContentsMargins(4, 4, 4, 4)

            # Top row
            h_top = QtWidgets.QHBoxLayout()
            b_sel = QtWidgets.QPushButton("Select (4 corners)")
            b_sel.clicked.connect(lambda _, n=name: self._start_course_marking(n))

            sr = QtWidgets.QSpinBox();
            sr.setValue(4)
            sc = QtWidgets.QSpinBox();
            sc.setValue(4)

            b_upd = QtWidgets.QPushButton("Update")
            b_upd.clicked.connect(lambda _, n=name: self._update_grid_params(n))

            h_top.addWidget(b_sel)

            # --- NEW LABELS (black and readable) ---
            l_r = QtWidgets.QLabel("Rows:")
            l_r.setStyleSheet(
                "color: black; font-weight: bold; background-color: #b0bec5; padding: 2px; border-radius: 3px;")
            h_top.addWidget(l_r)
            h_top.addWidget(sr)

            l_c = QtWidgets.QLabel("Cols:")
            l_c.setStyleSheet(
                "color: black; font-weight: bold; background-color: #b0bec5; padding: 2px; border-radius: 3px;")
            h_top.addWidget(l_c)
            h_top.addWidget(sc)

            h_top.addWidget(b_upd)
            v_tab.addLayout(h_top)

            # Bottom row (save/load/delete - unchanged)
            h_bot = QtWidgets.QHBoxLayout()
            b_save = QtWidgets.QPushButton(f"💾 Save {name}");
            b_save.clicked.connect(lambda _, n=name: self._save_single_course(n))
            b_load = QtWidgets.QPushButton(f"📂 Load {name}");
            b_load.clicked.connect(lambda _, n=name: self._load_single_course(n))
            b_del = QtWidgets.QPushButton(f"🗑️ Delete {name}");
            b_del.setStyleSheet("color: #ff5252; font-weight: bold;");
            b_del.clicked.connect(lambda _, n=name: self._clear_single_course(n))
            h_bot.addWidget(b_save);
            h_bot.addWidget(b_load);
            h_bot.addWidget(b_del)
            v_tab.addLayout(h_bot)

            self.tabs_course.addTab(w, f"Course {name}")

            # --- IMPORTANT: the button goes in the dict too, so its colour can be changed ---
            self.course_ui[name] = {"sp_row": sr, "sp_col": sc, "btn": b_sel}
        cl.addWidget(self.tabs_course)
        bot_lay.addWidget(course_grp)

        # UAV (drone) menu
        iha_box = QtWidgets.QGroupBox("UAV")
        il = QtWidgets.QVBoxLayout(iha_box)

        # UAV title and LED
        h_iha = QtWidgets.QHBoxLayout()
        lbl_iha = QtWidgets.QLabel("UAV STATUS")
        lbl_iha.setStyleSheet("""
            color: #ff1744;
            font-weight: bold;
            font-size: 10pt;
            background-color: rgba(40, 40, 40, 160);
            padding: 2px 6px;
            border-radius: 4px;
        """)
        h_iha.addWidget(lbl_iha)

        self.drone_led = QtWidgets.QLabel()
        self.drone_led.setFixedSize(16, 16)
        self.drone_led.setStyleSheet("background-color:#b71c1c; border-radius:8px;") # Red = not connected
        h_iha.addWidget(self.drone_led, alignment=Qt.AlignRight)
        il.addLayout(h_iha)

        # UAV target colour
        h_renk = QtWidgets.QHBoxLayout()
        h_renk.addWidget(QtWidgets.QLabel("Target colour:"))
        self.drone_color_lbl = QtWidgets.QLabel("Waiting")
        self.drone_color_lbl.setStyleSheet("font-weight: bold; color: yellow;")
        h_renk.addWidget(self.drone_color_lbl)
        il.addLayout(h_renk)

        # TASK3 COLOUR - what the BOAT is actually hunting, straight from its telemetry.
        #
        # The line above is what the DRONE sees; this one is what arrived at the other end.
        # They are not the same thing and the difference is the whole point: a colour can be
        # detected perfectly, relayed by this GCS, and still never reach the boat if the
        # radio drops that one packet. Until now nothing on this screen would have shown it.
        h_t3 = QtWidgets.QHBoxLayout()
        h_t3.addWidget(QtWidgets.QLabel("TASK3 colour:"))
        self.task3_color_lbl = QtWidgets.QLabel("—")
        self.task3_color_lbl.setStyleSheet("font-weight: bold; color: #9e9e9e;")
        h_t3.addWidget(self.task3_color_lbl)
        il.addLayout(h_t3)

        # Spacer to fill the gap
        il.addStretch(1)

        bot_lay.addWidget(iha_box)
        # RoboBoat 2027 has no UAV: the box is hidden, not deleted (other competitions)
        iha_box.setVisible(False)

        # RoboCommand (the judges' system): the GCS alone talks to it, over the Ethernet
        # cable in the tent (robocommand/, RoboBoat 2026 protocol until 2027's is out)
        rc_box = QtWidgets.QGroupBox("RoboCommand")
        rl = QtWidgets.QGridLayout(rc_box)
        self.ed_rc_host = QtWidgets.QLineEdit("10.10.10.1")
        self.ed_rc_host.setToolTip("On the course 10.10.10.1; with the fake server 127.0.0.1")
        self.sp_rc_port = QtWidgets.QSpinBox()
        self.sp_rc_port.setRange(1, 65535)
        self.sp_rc_port.setValue(50000)
        self.ed_rc_team = QtWidgets.QLineEdit("TEAM")
        self.ed_rc_team.setToolTip("Team code (given in the handbook)")
        self.chk_rc = QtWidgets.QCheckBox("Connect")
        self.chk_rc.toggled.connect(self._toggle_robocommand)
        btn_rc_test = QtWidgets.QPushButton("Test report")
        btn_rc_test.setToolTip("Sends an entry gate report at USV 1's position")
        btn_rc_test.clicked.connect(self._robocommand_test_report)
        self.lbl_rc = QtWidgets.QLabel("off")
        self.lbl_rc.setStyleSheet("color:#90a4ae; font-weight:bold;")
        self.lbl_rc.setWordWrap(True)
        rl.addWidget(QtWidgets.QLabel("Address:"), 0, 0)
        rl.addWidget(self.ed_rc_host, 0, 1)
        rl.addWidget(self.sp_rc_port, 0, 2)
        rl.addWidget(QtWidgets.QLabel("Team:"), 1, 0)
        rl.addWidget(self.ed_rc_team, 1, 1)
        rl.addWidget(self.chk_rc, 1, 2)
        rl.addWidget(btn_rc_test, 2, 0)
        rl.addWidget(self.lbl_rc, 2, 1, 1, 2)
        # As if RoboCommand had sent a task (simulation only: 2026 has no such message);
        # the GCS picks the boat by usv_core.allocation, the operator does not
        self.cmb_rc_task = QtWidgets.QComboBox()
        self.cmb_rc_task.addItems(["task4 (water delivery)", "rally (20 m north)",
                                   "task1"])
        btn_rc_task = QtWidgets.QPushButton("Judges' task (test)")
        btn_rc_task.setToolTip("A task as if from the judges: the GCS picks the boat itself")
        btn_rc_task.clicked.connect(self._robocommand_test_task)
        rl.addWidget(self.cmb_rc_task, 3, 0, 1, 2)
        rl.addWidget(btn_rc_task, 3, 2)
        bot_lay.addWidget(rc_box)

        # System log: what is about neither boat alone (links, RoboCommand, the judges'
        # tasks and who got them). Each boat's own lines go to its panel's log.
        sys_box = QtWidgets.QGroupBox("SYSTEM LOG")
        sl = QtWidgets.QVBoxLayout(sys_box)
        self.sys_log = QtWidgets.QPlainTextEdit()
        self.sys_log.setReadOnly(True)
        self.sys_log.setMaximumBlockCount(1000)
        self.sys_log.setStyleSheet("QPlainTextEdit { background-color: black; color: #4fc3f7;"
                                   " font-family: Consolas, monospace; font-size: 9pt;"
                                   " border: none; }")
        self.sys_log.setMinimumWidth(320)
        sl.addWidget(self.sys_log)
        bot_lay.addWidget(sys_box, 1)
        self._rc = None
        self._reports_seen = {}
        self._boat_state = {}             # number -> what the last telemetry said
        self._profiles = boat_profiles()
        self._rc_timer = QtCore.QTimer(self)
        self._rc_timer.timeout.connect(self._robocommand_tick)

        splitter.addWidget(center_widget)
        splitter.addWidget(self._build_boat_panel(2, bg_path))

        # Splitter ratios
        splitter.setStretchFactor(0, 1)  # Left: USV 1
        splitter.setStretchFactor(1, 4)  # Centre (wide)
        splitter.setStretchFactor(2, 1)  # Right: USV 2

        # Workers
        self.worker_1 = None
        # --- Reliable command channel (R1) ---
        # Why: telemetry became a broadcast and the drone heartbeats at 3 Hz, which made
        # the GCS the quietest node on a channel where the other two never stop talking.
        # A one-shot command that collides is simply gone - streams self-heal, commands
        # did not. Every command now carries a sequence number (cq), the boat echoes the
        # last processed one in telemetry (aq), and the head of this queue is resent every
        # 600 ms until echoed. One press does the whole job, emergency stop included.
        # The seq is time-seeded so a GCS restart cannot collide with acks of an older
        # session; manual_control stays OUTSIDE the queue - resending stale stick values
        # late would be worse than losing them.
        # 2027: one numbered queue per boat (gcs_link.ReliableSender): the radio is shared
        # and each boat acks only its own commands
        self._senders = {bid: gcs_link.ReliableSender(bid, self._tx) for bid in BOATS}
        self._lk_by = {}
        self._connected = False
        self._hb_timers = {}
        for bid in BOATS:
            timer = QtCore.QTimer(self)
            timer.timeout.connect(lambda b=bid: self._on_hb_timeout(b))
            timer.start(2000)
            self._hb_timers[bid] = timer

        self.drone_hb_timer = QtCore.QTimer(self)
        self.drone_hb_timer.timeout.connect(self._on_drone_hb_timeout)

        # Memory of the "confirmed" points the vehicles sent back
        self.verified_mission_1 = []
        self.verified_mission_2 = []

        # Draw the ropes last (the variables now exist, no error)
        self._layout_rope_stripes()

    def _build_boat_panel(self, bid, bg_path):
        """Return one boat's panel: title and LED, telemetry, mission table, controls, log."""
        name, colour = BOATS[bid]
        side = "right" if bid == 1 else "left"
        widget = QtWidgets.QWidget()
        widget.setObjectName(f"panelIDA{bid}")
        widget.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        widget.setStyleSheet(f"""
                    #panelIDA{bid} {{
                        border-image: url("{bg_path}") 0 0 0 0 stretch stretch;
                        border-{side}: 2px solid {colour};
                    }}
                """)
        v1 = QtWidgets.QVBoxLayout(widget)
        v1.setContentsMargins(4, 4, 4, 4)
        v1.setSpacing(5)

        h1 = QtWidgets.QHBoxLayout()
        h1.setContentsMargins(0, 0, 0, 0)
        lbl = QtWidgets.QLabel(f"USV {bid} · {name}")
        lbl.setStyleSheet(f"""
                    color: {colour};
                    font-weight: bold;
                    font-size: 12pt;
                    background-color: rgba(40, 40, 40, 160);
                    padding: 2px 6px;
                    border-radius: 4px;
                    border: 1px solid {colour};
                """)
        lbl.setFixedHeight(30)
        h1.addWidget(lbl)
        led = QtWidgets.QLabel()
        led.setFixedSize(16, 16)
        led.setStyleSheet("background-color:#37474f; border-radius:8px;")
        h1.addWidget(led, alignment=Qt.AlignRight)
        v1.addLayout(h1)
        setattr(self, f"usv{bid}_led", led)

        setattr(self, f"usv{bid}_ui", self._create_telemetry_grid(v1))
        setattr(self, f"usv{bid}_table", self._create_mission_box(v1, bid))

        if bid == 1:
            # Range rings are drawn round the map: one setting, on the first panel
            l_rng = self._add_modern_frame(v1, "RANGE RINGS")
            r1 = QtWidgets.QHBoxLayout()
            self.chk_rings = QtWidgets.QCheckBox("On")
            self.chk_rings.setStyleSheet("border:none;")
            self.chk_rings.toggled.connect(self._toggle_rings)
            self.sp_ring_step = QtWidgets.QSpinBox()
            self.sp_ring_step.setValue(5)
            self.sp_ring_count = QtWidgets.QSpinBox()
            self.sp_ring_count.setValue(3)
            self.sp_ring_step.valueChanged.connect(self._update_ring_settings)
            self.sp_ring_count.valueChanged.connect(self._update_ring_settings)
            r1.addWidget(self.chk_rings)
            r1.addWidget(QtWidgets.QLabel("Step (m):"))
            r1.addWidget(self.sp_ring_step)
            r1.addWidget(QtWidgets.QLabel("Count:"))
            r1.addWidget(self.sp_ring_count)
            l_rng.addLayout(r1)

        self._create_controls(v1, bid)

        l_log = self._add_modern_frame(v1, "SYSTEM LOG")
        log = QtWidgets.QPlainTextEdit()
        log.setReadOnly(True)
        log.setStyleSheet("""
                    QPlainTextEdit {
                        background-color: black;
                        color: #00e676;
                        font-family: Consolas, monospace;
                        font-size: 9pt;
                        border: none;
                    }
                """)
        l_log.addWidget(log)
        setattr(self, f"usv{bid}_log", log)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(430)      # two panels now: 500 each left the map too narrow
        return scroll

    # --- UI BUILDER FUNCTIONS (to avoid repetition) ---
    def _create_telemetry_grid(self, layout):
        # 1. Main frame (container)
        frame = QtWidgets.QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: rgba(30, 30, 30, 180);
                border: 1px solid #546e7a;
                border-radius: 6px;
            }
        """)
        layout.addWidget(frame)

        # The frame's inner layout
        l_main = QtWidgets.QVBoxLayout(frame)
        l_main.setContentsMargins(0, 0, 0, 0)
        l_main.setSpacing(0)

        # 2. Modern title
        lbl_header = QtWidgets.QLabel("TELEMETRY")
        lbl_header.setAlignment(QtCore.Qt.AlignCenter)
        lbl_header.setStyleSheet("""
            background-color: #37474f;
            color: #eceff1;
            font-size: 11px;
            font-weight: bold;
            letter-spacing: 2px;
            padding: 4px;
            border: none;
            border-bottom: 1px solid #546e7a;
            border-top-left-radius: 5px;
            border-top-right-radius: 5px;
        """)
        l_main.addWidget(lbl_header)

        # 3. Data grid
        grid_widget = QtWidgets.QWidget()
        grid_widget.setStyleSheet("border: none; background: transparent;")
        l_main.addWidget(grid_widget)

        g = QtWidgets.QGridLayout(grid_widget)
        g.setContentsMargins(4, 6, 4, 6)
        g.setSpacing(4)

        ui = {}
        keys = [
            "REAR_L_PWM", "REAR_R_PWM", "FRONT_L_PWM", "FRONT_R_PWM",
            "STEER_PWM", "SPEED", "HEADING", "TARGET_HDG",
            "ANGLE_ERR", "CONTROL_ERR", "DIST_TO_GO", "TASK", "MANUAL",
            "TIME", "FPS", "SENSORS", "POSITION", "TARGET"
        ]

        for i, k in enumerate(keys):
            row = i // 2
            col_base = (i % 2) * 2

            # --- CHANGE HERE: THE TITLES ARE NOW BOLD ---
            lbl_title = QtWidgets.QLabel(k)
            lbl_title.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            # added font-weight: bold and a lighter colour (#cfd8dc)
            lbl_title.setStyleSheet("""
                color: #cfd8dc;
                font-size: 14px;
                font-weight: bold;
                margin-right: 2px;
                border: none;
            """)
            g.addWidget(lbl_title, row, col_base)

            # --- CHANGE HERE: THE VALUES STAND OUT MORE ---
            lbl_val = QtWidgets.QLabel("-")
            lbl_val.setAlignment(QtCore.Qt.AlignCenter)  # Centre the text

            lbl_val.setStyleSheet("""
                            color: #00e5ff;
                            font-weight: bold;
                            font-size: 14px;
                            border: none;
                            background-color: rgba(0, 0, 0, 40);
                            border-radius: 3px;
                        """)
            g.addWidget(lbl_val, row, col_base + 1)

            ui[k] = lbl_val

        # 2027: the tasks still to come (mission manager), across the whole row
        row = len(keys) // 2 + 1
        lbl_title = QtWidgets.QLabel("NEXT")
        lbl_title.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        lbl_title.setStyleSheet("color:#cfd8dc; font-size:14px; font-weight:bold; border:none;")
        g.addWidget(lbl_title, row, 0)
        lbl_val = QtWidgets.QLabel("-")
        lbl_val.setStyleSheet("color:#ffd740; font-weight:bold; font-size:13px; border:none;"
                              " background-color: rgba(0, 0, 0, 40); border-radius: 3px;")
        g.addWidget(lbl_val, row, 1, 1, 3)
        ui["NEXT"] = lbl_val

        return ui

    def _create_mission_box(self, layout, bid):
        l = self._add_modern_frame(layout, f"MISSION PLAN (USV {bid})")

        # Table
        table = QtWidgets.QTableWidget(0, 3)
        # Column width
        table.setColumnWidth(0, 45)
        table.setHorizontalHeaderLabels(["No", "Lat", "Lon"])
        table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)

        # --- COMPACT SETTINGS (UPDATED) ---
        # Row height: 16 -> 18 (breathing room for 8 pt)
        table.verticalHeader().setDefaultSectionSize(18)

        # Header height: 18 -> 20
        table.horizontalHeader().setFixedHeight(20)

        # Font: 7 -> 8 pt
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setBold(True)
        table.setFont(font)

        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        table.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)

        # Height also raised with the rows: 110 -> 115
        table.setFixedHeight(115)

        # --- STYLE ---
        table.setStyleSheet("""
            QTableWidget {
                border: none;
                background-color: #263238;
                color: white;
                gridline-color: #455a64;
            }
            QHeaderView::section {
                background-color: #37474f;
                color: #cfd8dc;
                border: 1px solid #546e7a;
                padding: 0px;
                font-size: 10px;
                height: 20px;
            }
            QTableWidget::item {
                padding-top: -1px;      /* Fine tune to centre the text */
                padding-bottom: 0px;
                border: none;
            }
            QTableWidget::item:selected {
                background-color: #00e676;
                color: black;
            }
        """)
        l.addWidget(table)

        # --- ROW 1 (buttons) ---
        row1 = QtWidgets.QHBoxLayout()
        btn_h = 22

        b_add = QtWidgets.QPushButton("➕")
        b_add.setToolTip("Add a point");
        b_add.setFixedWidth(40);
        b_add.setFixedHeight(btn_h)
        b_add.setStyleSheet(self._get_btn_style("#2e7d32", "#69f0ae", "#00e676"))
        b_add.clicked.connect(lambda: self._add_wp_row(bid))

        b_del_sel = QtWidgets.QPushButton("➖")
        b_del_sel.setToolTip("Delete the selected row");
        b_del_sel.setFixedWidth(40);
        b_del_sel.setFixedHeight(btn_h)
        b_del_sel.setStyleSheet(self._get_btn_style("#ef6c00", "#ff9800", "#ffb74d"))
        b_del_sel.clicked.connect(lambda: self._delete_selected_row(bid))

        # --- NEW DUPLICATE BUTTON ---
        b_dup = QtWidgets.QPushButton("📋")
        b_dup.setToolTip("Duplicate the selected point (copy below)")
        b_dup.setFixedWidth(40)
        b_dup.setFixedHeight(btn_h)
        b_dup.setStyleSheet(self._get_btn_style("#fbc02d", "#fff59d", "#ffee58"))  # Yellow style
        b_dup.clicked.connect(lambda: self._duplicate_selected_row(bid))
        # ------------------------------------

        b_clr = QtWidgets.QPushButton("🗑️")
        b_clr.setToolTip("Clear the WHOLE list");
        b_clr.setFixedWidth(40);
        b_clr.setFixedHeight(btn_h)
        b_clr.setStyleSheet(self._get_btn_style("#c62828", "#ff5252", "#ff1744"))
        b_clr.clicked.connect(lambda: table.setRowCount(0))

        b_snd = QtWidgets.QPushButton("🚀 SEND")
        b_snd.setToolTip("Upload the mission to the boat");
        b_snd.setFixedHeight(btn_h)
        b_snd.setStyleSheet(self._get_btn_style("#00c853", "#00e676", "#69f0ae"))
        b_snd.clicked.connect(lambda: self._send_mission(bid))

        row1.addWidget(b_add);
        row1.addWidget(b_del_sel);
        row1.addWidget(b_dup)  # <--- ADD THIS
        row1.addWidget(b_clr);
        row1.addWidget(b_snd)
        l.addLayout(row1)

        # --- ROW 2 ---
        row2 = QtWidgets.QHBoxLayout()
        style_grey = self._get_btn_style("#455a64", "#90a4ae", "#78909c")
        style_blue = self._get_btn_style("#1565c0", "#448aff", "#2979ff")
        # Purplish style (for the task button)
        style_purple = self._get_btn_style("#7b1fa2", "#e1bee7", "#ea80fc")

        btn_up = QtWidgets.QPushButton("▲");
        btn_up.setFixedWidth(30);
        btn_up.setFixedHeight(btn_h)
        btn_up.setStyleSheet(style_grey);
        btn_up.clicked.connect(lambda: self._move_row(bid, -1))

        btn_down = QtWidgets.QPushButton("▼");
        btn_down.setFixedWidth(30);
        btn_down.setFixedHeight(btn_h)
        btn_down.setStyleSheet(style_grey);
        btn_down.clicked.connect(lambda: self._move_row(bid, 1))

        # --- NEW BUTTON: CHANGE TASK ---
        btn_task = QtWidgets.QPushButton("📝 TASK")
        btn_task.setToolTip("Change the current task by hand")
        btn_task.setFixedHeight(btn_h)
        btn_task.setStyleSheet(style_purple)  # A different colour so it catches the eye
        btn_task.clicked.connect(lambda: self._set_manual_task(bid))
        # ------------------------------------------

        b_s = QtWidgets.QPushButton("💾");
        b_s.setFixedHeight(btn_h);
        b_s.setStyleSheet(style_blue)
        b_s.clicked.connect(lambda: self._save_mission(bid))

        b_l = QtWidgets.QPushButton("📂");
        b_l.setFixedHeight(btn_h);
        b_l.setStyleSheet(style_blue)
        b_l.clicked.connect(lambda: self._load_mission(bid))

        row2.addWidget(btn_up);
        row2.addWidget(btn_down)
        row2.addWidget(btn_task)

        row2.addStretch()

        row2.addWidget(b_s);
        row2.addWidget(b_l)
        l.addLayout(row2)

        table.itemChanged.connect(lambda item: self._refresh_map_mission(bid))
        return table

    def _set_manual_task(self, bid):
        """Ask the user for a task name and send it to the vehicle."""
        # Open the input dialog
        text, ok = QtWidgets.QInputDialog.getText(
            self,
            f"USV {bid} task",
            "New task name (e.g. TASK2_START, FINISHED):",
            QtWidgets.QLineEdit.Normal,
            ""
        )

        if ok and text:
            task_name = text.strip()
            if not task_name: return

            # Build the command and send it
            self._send_reliable({
                "target_id": bid,
                "cmd": "set_task",
                "task_name": task_name
            })

    # Replace this function in MainWindow:
    def _refresh_map_mission(self, bid, sync_other=True):
        """
        bid: whose table changed
        sync_other: update the other boat too (a parameter to stop an endless loop).
        """
        t = self.usv1_table if bid == 1 else self.usv2_table
        local_pts = []
        for i in range(t.rowCount()):
            item_lat = t.item(i, 1)
            item_lon = t.item(i, 2)
            if item_lat is None or item_lon is None: continue
            try:
                lt, ln = float(item_lat.text()), float(item_lon.text())
                local_pts.append((lt, ln))
            except:
                pass

        remote_pts = self.verified_mission_1 if bid == 1 else self.verified_mission_2

        # Label choice
        if bid == 1:
            tags_to_send = self.MISSION_TAGS_1
        else:
            tags_to_send = self.MISSION_TAGS_2

        # Update the map
        self.map.update_mission_waypoints(bid, local_pts, remote_pts, tags=tags_to_send)

        # --- SYNCHRONISATION ---
        # If this update is the main trigger (sync_other=True),
        # update the other boat's map too so the "orange" colours match.
        if sync_other:
            other_id = 2 if bid == 1 else 1
            self._refresh_map_mission(other_id, sync_other=False)

    def keyPressEvent(self, e):
        # Toggle when Shift is pressed
        if e.key() == Qt.Key_Shift:
            if not e.isAutoRepeat():  # Ignore key repeat while held
                self.map.toggle_smart_ruler()  # <-- toggle called instead of start

        # The target ID is whichever radio button is checked
        target_id = 1
        if hasattr(self, 'radio_focus_2') and self.radio_focus_2.isChecked():
            target_id = 2

        if e.key() == Qt.Key_Up:
            self._send_manual_pwm(target_id, 1600, 1600)
        elif e.key() == Qt.Key_Down:
            self._send_manual_pwm(target_id, 1300, 1300)
        elif e.key() == Qt.Key_Left:
            self._send_manual_pwm(target_id, 1300, 1600)  # Left: left motor back, right forward
        elif e.key() == Qt.Key_Right:
            self._send_manual_pwm(target_id, 1600, 1300)  # Right: left forward, right back
        elif e.key() == Qt.Key_Space:
            self._send_manual_pwm(target_id, 1500, 1500)  # STOP
        else:
            super().keyPressEvent(e)

    def _confirm_estop(self, bid):
        """Show a confirmation box for the emergency shutdown."""
        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Critical)
        msg.setWindowTitle("EMERGENCY")
        msg.setText(f"WARNING: USV {bid}'s motors will be stopped at once!")
        msg.setInformativeText("Do you want to go on?")
        msg.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        msg.setDefaultButton(QtWidgets.QMessageBox.No)

        if msg.exec() == QtWidgets.QMessageBox.Yes:
            self._send_cmd(bid, "emergency_stop", True)
            self._boat_log(bid, "⚠️ EMERGENCY STOP SENT!")

    def _add_modern_frame(self, parent_layout, title):
        """
        Build a modern frame and set the style of the BUTTONS INSIDE it.
        """
        # 1. Main frame
        frame = QtWidgets.QFrame()
        # NOTE: the QPushButton style is added here!
        frame.setStyleSheet("""
            QFrame {
                background-color: rgba(30, 30, 30, 180);
                border: 1px solid #546e7a;
                border-radius: 6px;
                margin-top: 4px;
            }
            /* --- BUTTONS INSIDE THE FRAME --- */
            QPushButton {
                background-color: #455a64;       /* Dark grey background */
                color: white;                    /* White text */
                border: 2px solid #90a4ae;       /* CLEAR BORDER (metallic) */
                border-radius: 4px;
                padding: 4px;
                min-width: 24px;
                min-height: 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #546e7a;
                border: 2px solid #00e5ff;       /* NEON BLUE on hover */
                color: #ffffff;
            }
            QPushButton:pressed {
                background-color: #263238;
                border: 2px solid #546e7a;
                padding-top: 6px;
                padding-left: 6px;
            }
            QPushButton:checked {
                background-color: #00897b;
                border: 2px solid #00e676;
            }
            /* Labels */
            QLabel {
                border: none;
                background: transparent;
                color: #cfd8dc;
            }
        """)
        parent_layout.addWidget(frame)

        # Main layout
        l_main = QtWidgets.QVBoxLayout(frame)
        l_main.setContentsMargins(0, 0, 0, 0)
        l_main.setSpacing(0)

        # 2. Modern title
        lbl_header = QtWidgets.QLabel(title)
        lbl_header.setAlignment(QtCore.Qt.AlignCenter)
        # Title style (written specifically so it is not overridden)
        lbl_header.setStyleSheet("""
            background-color: #37474f;
            color: #eceff1;
            font-size: 10px;
            font-weight: bold;
            letter-spacing: 1px;
            padding: 4px;
            border: none;
            border-bottom: 1px solid #546e7a;
            border-top-left-radius: 5px;
            border-top-right-radius: 5px;
        """)
        l_main.addWidget(lbl_header)

        # 3. Content area
        content_widget = QtWidgets.QWidget()
        # Content area transparent so the buttons do not take the frame's colour
        content_widget.setStyleSheet("background: transparent; border: none;")
        l_main.addWidget(content_widget)

        content_layout = QtWidgets.QVBoxLayout(content_widget)
        content_layout.setContentsMargins(4, 4, 4, 4)
        content_layout.setSpacing(4)  # A little more space between the buttons

        return content_layout

    def _create_controls(self, layout, bid):
        container_layout = self._add_modern_frame(layout, f"CONTROL (USV {bid})")

        g = QtWidgets.QGridLayout()
        g.setSpacing(6)
        container_layout.addLayout(g)

        # 1. Keyboard focus
        rb_focus = QtWidgets.QRadioButton("⌨️ Keyboard focus")
        rb_focus.setToolTip("Drive this boat with the keyboard (arrow keys)")  # <--- ADDED
        rb_focus.setStyleSheet(
            "QRadioButton { color: #00e676; font-weight: bold; background: transparent; border: none; }")
        self.focus_group.addButton(rb_focus, bid)
        if bid == 1:
            self.radio_focus_1 = rb_focus;
            self.radio_focus_1.setChecked(True)
        else:
            self.radio_focus_2 = rb_focus
        g.addWidget(rb_focus, 0, 0, 1, 2)

        # 2. Mode buttons
        # AUTO (turquoise)
        b_auto = QtWidgets.QPushButton("🤖 AUTO")
        b_auto.setToolTip("Switch to autonomous mode (the mission starts)")  # <--- ADDED
        b_auto.setStyleSheet(self._get_btn_style("#006064", "#00bcd4", "#18ffff"))
        b_auto.clicked.connect(lambda: self._send_cmd(bid, "set_manual", False))

        # MANUAL (purple)
        b_man = QtWidgets.QPushButton("🎮 MANUAL")
        b_man.setToolTip("Switch to manual mode (remote control)")  # <--- ADDED
        b_man.setStyleSheet(self._get_btn_style("#4a148c", "#ab47bc", "#e040fb"))
        b_man.clicked.connect(lambda: self._send_cmd(bid, "set_manual", True))

        g.addWidget(b_auto, 1, 0);
        g.addWidget(b_man, 1, 1)

        # 3. Track controls (ice blue)
        style_ice = self._get_btn_style("#37474f", "#4fc3f7", "#81d4fa")

        b_fz = QtWidgets.QPushButton("❄️");
        b_fz.setToolTip("Freeze the ghost track (keeps the line for analysis)")  # <--- ADDED
        b_fz.setStyleSheet(style_ice)
        b_fz.clicked.connect(lambda: self.map.snapshot_ghost_trail() if bid == 1 else None)

        b_cl = QtWidgets.QPushButton("🧼");
        b_cl.setToolTip("Clear the track on the screen")  # <--- ADDED
        b_cl.setStyleSheet(style_ice)
        b_cl.clicked.connect(lambda: self.map.clear_ghost_trail_by_id(bid))

        g.addWidget(b_fz, 2, 0);
        g.addWidget(b_cl, 2, 1)

        # 4. Relay (motor power) - flips the SAME relay as the switch on the RC transmitter.
        # ArduPilot's RC auxiliary function is edge triggered and MAVLink DO_SET_RELAY
        # writes the same state; the last writer wins. The RC path is unaffected and,
        # being independent of the Jetson, stays the real emergency stop.
        b_relay_on = QtWidgets.QPushButton("⚡ RELAY ON")
        b_relay_on.setToolTip("Powers the motors (the same relay as the switch on the transmitter)")
        b_relay_on.setStyleSheet(self._get_btn_style("#1b5e20", "#00e676", "#00c853"))
        b_relay_on.clicked.connect(lambda: self._send_relay(bid, 1))

        b_relay_off = QtWidgets.QPushButton("⛔ RELAY OFF")
        b_relay_off.setToolTip("Cuts the motors' power (the same relay as the switch on the transmitter)")
        b_relay_off.setStyleSheet(self._get_btn_style("#e65100", "#ffa726", "#fb8c00"))
        b_relay_off.clicked.connect(lambda: self._send_relay(bid, 0))

        g.addWidget(b_relay_on, 3, 0)
        g.addWidget(b_relay_off, 3, 1)

        # The relay's REAL state (from the vehicle's RELAY_STATUS). "The last command we sent"
        # is not shown, so it stays right when the relay is flipped from the transmitter too.
        relay_lbl = QtWidgets.QLabel("RELAY: unknown")
        relay_lbl.setAlignment(QtCore.Qt.AlignCenter)
        relay_lbl.setStyleSheet("color:#90a4ae; font-weight:bold;")
        g.addWidget(relay_lbl, 4, 0, 1, 2)
        self.relay_lbls[bid] = relay_lbl

        # 5. Emergency shutdown (DARK RED)
        b_estop = QtWidgets.QPushButton("🛑 EMERGENCY STOP")
        b_estop.setToolTip("WARNING: cuts the motors' power at once!")  # <--- ADDED
        b_estop.setStyleSheet(self._get_btn_style("#b71c1c", "#ff1744", "#d50000"))
        b_estop.clicked.connect(lambda: self._confirm_estop(bid))
        g.addWidget(b_estop, 5, 0, 1, 2)

        # Link quality - measured live from the boat's sequence numbers. Green: healthy,
        # orange: stuttering, red: bad. A number in place of a "not smooth" feeling.
        link_lbl = QtWidgets.QLabel("LINK: no data")
        link_lbl.setAlignment(QtCore.Qt.AlignCenter)
        link_lbl.setStyleSheet("color:#90a4ae; font-weight:bold;")
        g.addWidget(link_lbl, 6, 0, 1, 2)
        self.link_lbls[bid] = link_lbl

        # 5. Info
        info_text = "[↑]Ahead  [↓]Astern  [←][→]Turn  [SPACE]Stop"
        lbl_info = QtWidgets.QLabel(info_text)
        lbl_info.setAlignment(QtCore.Qt.AlignCenter)
        lbl_info.setStyleSheet(
            "color:#90a4ae; font-size:9px; border:1px dashed #546e7a; border-radius:4px; padding:2px; background:transparent;")
        g.addWidget(lbl_info, 4, 0, 1, 2)

    def _get_btn_style(self, bg_color, border_color, hover_color):
        """
        Return a modern HUD button style in the given colours.
        """
        return f"""
            QPushButton {{
                background-color: {bg_color};
                color: white;
                font-weight: bold;
                border: 2px solid {border_color};
                border-radius: 4px;
                padding: 4px;
                min-width: 24px;
                min-height: 20px;
            }}
            QPushButton:hover {{
                background-color: {hover_color};
                border: 2px solid white;
                color: white;
            }}
            QPushButton:pressed {{
                background-color: {bg_color};
                border: 2px solid {bg_color};
                padding-top: 6px;
                padding-left: 6px;
            }}
        """

    # --- LOGIC FUNCTIONS ---
    @QtCore.Slot(dict)
    def on_packet(self, d: dict):
        bid = int(d.get("id", 1))  # <--- FIX: always turn it into an integer (1 or 2)!

        # Drone (UAV) telemetry check
        if bid == 3:
            # Drone LED effect
            self.drone_led.setStyleSheet("background-color:#00e676; border-radius:8px;")
            QtCore.QTimer.singleShot(100, lambda: self.drone_led.setStyleSheet("background-color:#1b5e20; border-radius:8px;"))
            # 4000 ms, not 2500. The drone heartbeats at 3 Hz and the boat is polled at
            # 3.3 Hz; against one shared timeout both now need ~12 consecutive losses to
            # be declared dead. At the old 1 Hz / 2500 ms the drone needed only THREE,
            # which is why its panel flashed "Link lost" so much more than the boat's
            # on exactly the same radio link.
            self.drone_hb_timer.start(4000)

            drone_color = d.get("drone_color", "BELIRSIZ")
            self.drone_color_lbl.setText(drone_color)

            # Translate to English and send the command.
            # Send ONLY when the colour CHANGES: the drone sent a packet every frame, and
            # on the same radio link that meant dozens of needless commands a second to the boat.
            # UNCERTAIN is not in the map, so it is not sent - when the drone wavers for a moment
            # the colour the boat holds is not wiped.
            if drone_color in COLOR_TR_TO_EN and drone_color != getattr(self, "_last_drone_color", None):
                self._last_drone_color = drone_color
                self._want_color_en = COLOR_TR_TO_EN[drone_color]
                self._send_target_color()
            return

        if "rp" in d:                      # a task report (RoboBoat 2027 step 6d)
            self._on_report(bid, d)
            return

        task_str = d.get("task", "")

        # --- Link statistics: arrival time + sequence number ---
        # 'q' is a counter that rises by one each packet; a jump = packets lost. The split
        # matters: "lost" and "late" are different illnesses, and until now nothing
        # measured them apart.
        if bid in BOATS:
            st = self._lk_by.get(bid)
            if st is None:
                st = self._lk_by[bid] = {"times": [], "last_q": None, "got": 0, "lost": 0,
                                         "via": ""}
            st["via"] = d.get("_link", "")
            st["times"].append(time.time())
            if len(st["times"]) > 200:
                del st["times"][:100]
            q = d.get("q")
            if q is not None:
                try:
                    q = int(q)
                    if st["last_q"] is not None:
                        delta = (q - st["last_q"]) % 100000
                        if 0 < delta < 500:          # a sane range: not a counter jump
                            st["got"] += 1
                            st["lost"] += delta - 1
                        else:                         # the boat restarted: keep the counters
                            st["got"] += 1
                    else:
                        st["got"] += 1
                    st["last_q"] = q
                except (TypeError, ValueError):
                    pass

        # LED
        led = getattr(self, f"usv{bid}_led", None)
        if led:
            led.setStyleSheet("background-color:#00e676; border-radius:10px;")
            QtCore.QTimer.singleShot(100, lambda: led.setStyleSheet("background-color:#1b5e20; border-radius:10px;"))
            if bid in self._hb_timers:
                self._hb_timers[bid].start(2500)

        # UI Update
        ui = getattr(self, f"usv{bid}_ui", None)
        if ui is None:
            return                    # a boat this GCS has no panel for
        # If the packet carries a task list, keep it and update the map

        # The key name can follow your own protocol: "mission_list", "points" etc.
        # Incoming data: "waypoints": {"GPS1": {"lat":..., "lon":...}, ...}
        if "waypoints" in d:
            incoming_list = []
            try:
                points_dict = d["waypoints"]

                # Take the data from GPS1 to GPS12 in order
                for i in range(1, 13):
                    key = f"GPS{i}"
                    if key in points_dict:
                        pt = points_dict[key]

                        # Is it a dict and does it have coordinates?
                        if isinstance(pt, dict) and "lat" in pt and "lon" in pt:
                            lat = float(pt["lat"])
                            lon = float(pt["lon"])

                            # Only points with coordinates (not 0,0) go in the list
                            if lat != 0.0 or lon != 0.0:
                                incoming_list.append((lat, lon))

                # Update the memory
                if bid == 1:
                    self.verified_mission_1 = incoming_list
                else:
                    self.verified_mission_2 = incoming_list

                # Draw the map again (so the green points show)
                self._refresh_map_mission(bid)

            except Exception as e:
                print(f"Mission list parse error: {e}")

        # --- HELPER UPDATE FUNCTION ---
        def s(k, v="-"):
            # If the field is NOT in the packet at all, leave it: the last value stays on screen.
            # Waypoints now come in their own mini packet without the panel fields - the old
            # behaviour set the whole panel to "-" every 10 seconds and made it flicker. "-" is
            # written only when the field arrives with the value None.
            if v not in d:
                return
            raw_val = d.get(v, "-")

            # No data, or None
            if raw_val is None or raw_val == "-":
                ui[k].setText("-")
                return

            try:
                # Check whether it is a number
                val_float = float(raw_val)

                # For integers such as PWM (e.g. 1500.0 -> 1500)
                if val_float.is_integer():
                    txt = f"{int(val_float)}"
                else:
                    # For floats such as SPEED, heading (e.g. 2.5432 -> 2.54)
                    txt = f"{val_float:.2f}"

                # If it still comes out very long (safety)
                if len(txt) > 8:
                    txt = txt[:8]

                ui[k].setText(txt)

            except (ValueError, TypeError):
                # Not a number (text, e.g. a task name)
                txt = str(raw_val)
                # Shorten a very long task name (TASK_... -> TASK..)
                if len(txt) > 12:
                    txt = txt[:10] + ".."
                ui[k].setText(txt)

        s("REAR_L_PWM", "pwm_L")
        s("REAR_R_PWM", "pwm_R")
        s("FRONT_L_PWM", "pwm_FL")
        s("FRONT_R_PWM", "pwm_FR")
        s("STEER_PWM", "pwm_STEER")
        s("SPEED", "spd")
        s("HEADING", "hdg")
        s("TARGET_HDG", "trg_hdg")
        s("ANGLE_ERR", "err_ang")
        s("CONTROL_ERR", "ctrl_err")
        s("DIST_TO_GO", "dist")
        s("TASK", "task")
        if "task_queue" in d:
            ui["NEXT"].setText(" → ".join(t for t in str(d["task_queue"]).split(">") if t)
                                   or "-")
        s("MANUAL", "mod")
        s("TIME", "t_ms")
        s("FPS", "FPS")
        s("SENSORS", "hlth")

        # Relay state: -1 = the vehicle does not report RELAY_STATUS -> say "unknown", do not guess.
        # R1 ack intake. If the boat remembers a LARGER sequence than anything this
        # session has sent (it survived a GCS restart), jump past it - otherwise our
        # fresh, smaller numbers would appear already-acknowledged.
        if "aq" in d and bid in self._senders:
            try:
                self._senders[bid].on_ack(int(d["aq"]), time.time())
            except (TypeError, ValueError):
                pass

        if self._rc is not None and bid in BOATS:
            self._rc.on_telemetry(bid, d)     # heartbeats for RoboCommand
        if bid in BOATS:                      # for the task allocation
            st = self._boat_state.setdefault(bid, {})
            st["t"] = time.monotonic()
            for key in ("task", "mod", "task_queue", "position"):
                if key in d:
                    st[key] = d[key]

        relay_lbl = self.relay_lbls.get(bid)
        if relay_lbl is not None and "relay" in d:
            try:
                rv = int(d.get("relay", -1))
            except (TypeError, ValueError):
                rv = -1
            if rv == 1:
                relay_lbl.setText("RELAY: ON (motors powered)")
                relay_lbl.setStyleSheet("color:#00e676; font-weight:bold;")
            elif rv == 0:
                relay_lbl.setText("RELAY: OFF")
                relay_lbl.setStyleSheet("color:#ffa726; font-weight:bold;")
            else:
                relay_lbl.setText("RELAY: unknown")
                relay_lbl.setStyleSheet("color:#90a4ae; font-weight:bold;")

        # --- TASK3 target colour: the colour the boat REALLY holds ---
        #
        # 'tsrc' says where it came from: "cfg" = config.py's TASK3_KAMIKAZE_COLOR because
        # DRONE_ACTIVE is off, "drone" = the boat is waiting for / using the drone's colour.
        # Showing the source matters as much as the colour: hunting black because the drone
        # said so, and hunting black because the drone never arrived and config says black,
        # look identical on the water and mean completely different things.
        if hasattr(self, "task3_color_lbl") and "tsrc" in d:
            tsrc = d.get("tsrc", "")
            tcol = (d.get("tcol") or "").lower()
            tr = COLOR_EN_TO_TR.get(tcol, tcol.upper())
            if tsrc == "cfg":
                self.task3_color_lbl.setText(f"{tr} (config)")
                self.task3_color_lbl.setStyleSheet("font-weight:bold; color:#90a4ae;")
            elif tcol:
                self.task3_color_lbl.setText(f"{tr} (drone)")
                self.task3_color_lbl.setStyleSheet("font-weight:bold; color:#00e676;")
            else:
                self.task3_color_lbl.setText("NO COLOUR")
                self.task3_color_lbl.setStyleSheet("font-weight:bold; color:#ffa726;")

            # Closed loop. If the boat is holding something other than what the drone last
            # reported, the command did not land - say it again. Rate limited so a command
            # in flight is given time to arrive and be reported back before we repeat it.
            want = getattr(self, "_want_color_en", None)
            if tsrc == "drone" and want and tcol != want:
                if time.time() - getattr(self, "_last_color_tx", 0.0) >= 2.0:
                    self._send_target_color()
                    self.on_status(f"Boat colour differs ({tcol or 'none'}) -> {want} sent again")

        pos = d.get("position", {})
        lat = pos.get("lat", 0.0)
        lon = pos.get("lon", 0.0)
        if lat and lon:
            ui["POSITION"].setText(f"{lat:.5f},\n{lon:.5f}")

        t_pos = d.get("target", {})
        t_lat = t_pos.get("lat", 0.0)
        t_lon = t_pos.get("lon", 0.0)
        if t_lat and t_lon:
            ui["TARGET"].setText(f"{t_lat:.5f},\n{t_lon:.5f}")

        if lat and lon:

            # Make the text a little smaller so it fits
            self.map.update_current_position(lat, lon, boat_id=bid)
            # Log
            logbox = self.usv1_log if bid == 1 else self.usv2_log
            # logbox.appendPlainText(f"DATA: {lat},{lon}") # uncomment if you want it

        # 3. GEOFENCE PROTECTION LOGIC (for both boats)
        # Works only when "ACTIVE" is ticked and the boat is in AUTONOMOUS mode.
        is_manual = bool(d.get("mod", True))  # Default True (safety)

        if self.chk_fence_active.isChecked() and not is_manual:
            pos = d.get("position")
            if isinstance(pos, dict):
                cur_lat = pos.get("lat")
                cur_lon = pos.get("lon")
                fence_pts = self.map.get_fence_points()

                # A fence needs at least 3 points
                if cur_lat and cur_lon and len(fence_pts) >= 3:
                    # If the point is NOT inside the polygon (outside)
                    if not is_point_in_polygon(cur_lat, cur_lon, fence_pts):
                        msg = f"⚠️ GEOFENCE BREACH! STOPPING USV {bid}!"
                        self._boat_log(bid, msg)
                        self.on_status(msg)

                        # 1. Switch to manual (out of autonomous)
                        self._send_cmd(bid, "set_manual", True)

                        # 2. Stop the motors (PWM 1500)
                        self._send_manual_pwm(bid, 1500, 1500)

                        # 3. Audible warning (optional)
                        QtWidgets.QApplication.beep()

    def _mk_btn(self, t, b, l, r):
        btn = QtWidgets.QPushButton(t)
        btn.clicked.connect(lambda: self._send_manual_pwm(b, l, r))
        return btn

    def _tx(self, obj):
        """Put one command line on every link (the senders' output)."""
        if self.worker_1:
            self.worker_1.queue_send(obj)

    def _send_reliable(self, payload):
        """Queue a command for its boat (payload's target_id): numbered, resent until acked."""
        if not self.worker_1:
            return
        bid = int(payload.get("target_id", 1))
        sender = self._senders.setdefault(bid, gcs_link.ReliableSender(bid, self._tx))
        fields = {k: v for k, v in payload.items() if k not in ("target_id", "cmd", "cq")}
        sender.command(payload["cmd"], time.time(), **fields)

    def _service_cmds(self, force_head=False):
        """Driven by the 300 ms UI timer and by every ack (ReliableSender.on_ack)."""
        now = time.time()
        for bid, sender in self._senders.items():
            sender.service(now)
            while sender.failed:
                dead = sender.failed.pop(0)
                self._boat_log(bid, f"⚠ COMMAND NOT DELIVERED: {dead.get('cmd')} "
                                    f"({gcs_link.MAX_TRIES} tries)")

    def _send_cmd(self, bid, cmd, val):
        self._send_reliable({"target_id": bid, "cmd": cmd, "value": val})
        self._boat_log(bid, f"TX: {cmd} -> {val}")

    def _send_relay(self, bid, state):
        """Motor gucu rolesini ac/kapat. Kumandadaki anahtarla ayni roleye gider."""
        self._send_reliable({"target_id": bid, "cmd": "set_relay", "value": int(state)})
        self._boat_log(bid, f"TX: RELAY -> {'ON' if state else 'OFF'}")

    def _send_target_color(self):
        """
        Push the colour the drone reported down to the boat.

        Called on a fresh drone detection AND from the closed loop in on_packet() when the
        boat's telemetry says it is holding something else. The old code sent once, on
        change, and then recorded the new colour as "done" - so if that single command was
        lost on the radio, the drone could report KIRMIZI for the rest of the mission and
        this GCS would stay silent, because as far as it knew nothing had changed. The
        boat would hunt the stale colour to the end of the run.

        This still does not chatter: it fires on change, and otherwise only when the boat
        actually disagrees.
        """
        if not self.worker_1 or not getattr(self, "_want_color_en", None):
            return
        self._send_reliable({"target_id": 1, "cmd": "set_target_color",
                             "color": self._want_color_en})
        self._last_color_tx = time.time()

    def _send_manual_pwm(self, bid, l, r):
        if self.worker_1: self.worker_1.queue_send({"target_id": bid, "cmd": "manual_control", "left": l, "right": r})

    def _duplicate_selected_row(self, bid):
        """Copy the selected row and insert it as a new row just below."""
        t = self.usv1_table if bid == 1 else self.usv2_table
        current = t.currentRow()

        # No selection: warn and leave
        if current < 0:
            self._boat_log(bid, "WARNING: no row selected to duplicate.")
            return

        # Take the selected row's lat and lon
        lat = t.item(current, 1).text()
        lon = t.item(current, 2).text()

        # Open a new row just below
        target_row = current + 1
        t.insertRow(target_row)

        # Pause the signals for now (so the map is not triggered again and again for nothing)
        t.blockSignals(True)
        t.setItem(target_row, 0, QtWidgets.QTableWidgetItem(""))  # The label part is filled by the function
        t.setItem(target_row, 1, QtWidgets.QTableWidgetItem(lat))
        t.setItem(target_row, 2, QtWidgets.QTableWidgetItem(lon))
        t.blockSignals(False)

        # Fix the table numbers and draw the map again
        self._update_table_labels(bid)
        self._refresh_map_mission(bid)

        # Select the new row so the user can duplicate again and again
        t.selectRow(target_row)

    def _delete_selected_row(self, bid):
        """Delete the selected rows from the table."""
        t = self.usv1_table if bid == 1 else self.usv2_table

        # Take all selected rows in case of multiple selection
        # (Deleted from the end so the indices do not shift)
        rows = sorted(set(index.row() for index in t.selectedIndexes()), reverse=True)

        if not rows:
            # No selection: try deleting the current cell's row
            current = t.currentRow()
            if current >= 0:
                rows = [current]
            else:
                self._boat_log(bid, "WARNING: no row selected to delete.")
                return

        # Delete
        for r in rows:
            t.removeRow(r)

        # Update the numbers and the map
        self._update_table_labels(bid)
        self._refresh_map_mission(bid)


    def _add_wp_row(self, bid):
        t = self.usv1_table if bid == 1 else self.usv2_table
        r = t.rowCount();
        t.insertRow(r)
        t.setItem(r, 0, QtWidgets.QTableWidgetItem(str(r + 1)))
        t.setItem(r, 1, QtWidgets.QTableWidgetItem("0.0"));
        t.setItem(r, 2, QtWidgets.QTableWidgetItem("0.0"))
        self._refresh_map_mission(bid)
        self._update_table_labels(bid)

    def _update_table_labels(self, bid):
        """Write the numbers and labels in the table's 'No' column."""
        t = self.usv1_table if bid == 1 else self.usv2_table

        if bid == 1:
            tags = self.MISSION_TAGS_1
        else:
            tags = self.MISSION_TAGS_2

        # --- UPDATE HERE ---
        font_small = QtGui.QFont()
        font_small.setPointSize(8)  # Font raised from 7 to 8
        font_small.setBold(True)

        for i in range(t.rowCount()):
            label_text = str(i + 1)
            if i < len(tags):
                # Shorten the task name: TASK1_START -> T1_S
                short_tag = tags[i].replace("TASK", "T").replace("START", "S").replace("END", "E").replace("MID", "M")
                label_text += f".{short_tag}"

            item = t.item(i, 0)
            if not item:
                item = QtWidgets.QTableWidgetItem()
                item.setFlags(Qt.ItemIsEnabled)  # Read only
                item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                t.setItem(i, 0, item)

            item.setFont(font_small)
            item.setText(label_text)

            # Row height 18 px too
            t.setRowHeight(i, 18)

    def _move_row(self, bid, direction):
        """
        Move the selected row up (-1) or down (+1).
        """
        table = self.usv1_table if bid == 1 else self.usv2_table
        current_row = table.currentRow()

        # No or invalid selection: leave
        if current_row < 0: return

        # Target row
        target_row = current_row + direction

        # Bounds check (do not leave the list)
        if target_row < 0 or target_row >= table.rowCount():
            return

        # --- DATA SWAP ---
        # Take the lat and lon
        lat_curr = table.item(current_row, 1).text()
        lon_curr = table.item(current_row, 2).text()

        lat_target = table.item(target_row, 1).text()
        lon_target = table.item(target_row, 2).text()

        # Write them back swapped
        table.item(current_row, 1).setText(lat_target)
        table.item(current_row, 2).setText(lon_target)

        table.item(target_row, 1).setText(lat_curr)
        table.item(target_row, 2).setText(lon_curr)

        # Move the selection too, so the button can be pressed again and again
        table.selectRow(target_row)

        # Update the map (the labels must move when the order changes)
        self._refresh_map_mission(bid)

    # Find and replace this function in the MainWindow class:
    def _send_mission(self, bid):
        t = self.usv1_table if bid == 1 else self.usv2_table
        cnt = t.rowCount()

        if cnt == 0:
            self._boat_log(bid, "WARNING: the list is empty, nothing sent.")
            return

        self._boat_log(bid, f"preparing {cnt} points...")

        sent_count = 0
        for i in range(cnt):
            try:
                # Check whether the cells are empty
                item_lat = t.item(i, 1)
                item_lon = t.item(i, 2)

                if item_lat and item_lon and item_lat.text() and item_lon.text():
                    lat = float(item_lat.text())
                    lon = float(item_lon.text())

                    self._send_reliable({
                        "target_id": bid,
                        "cmd": "set_gps",
                        "index": i + 1,
                        "lat": lat,
                        "lon": lon
                    })
                    sent_count += 1
                else:
                    self._boat_log(bid, f"ERROR: row {i + 1} is empty or invalid!")
            except ValueError:
                self._boat_log(bid, f"ERROR: row {i + 1} is not a number!")
            except Exception as e:
                self._boat_log(bid, f"ERROR: {e}")

        if sent_count > 0:
            self._boat_log(bid, f"TX: {sent_count} commands queued.")

    def _save_mission(self, bid):
        t = self.usv1_table if bid == 1 else self.usv2_table
        d = [{"lat": t.item(i, 1).text(), "lon": t.item(i, 2).text()} for i in range(t.rowCount())]
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save", f"usv{bid}.json")
        if path:
            with open(path, "w") as f: json.dump(d, f)

    def _load_mission(self, bid):
        t = self.usv1_table if bid == 1 else self.usv2_table
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load")
        if path:
            with open(path, "r") as f:
                data = json.load(f)
            t.setRowCount(0)
            for p in data:
                r = t.rowCount();
                t.insertRow(r)
                t.setItem(r, 0, QtWidgets.QTableWidgetItem(str(r + 1)))
                t.setItem(r, 1, QtWidgets.QTableWidgetItem(str(p["lat"])));
                t.setItem(r, 2, QtWidgets.QTableWidgetItem(str(p["lon"])))

        self._refresh_map_mission(bid)
        self._update_table_labels(bid)

    @QtCore.Slot(int, float, float, int)
    def _on_map_gps_selected(self, idx, lat, lon, target_row):
        """
        idx: 1 (IDA1), 2 (IDA2), 3 (both)
        target_row: -1 adds a new row at the end of the list;
                    0, 1, 2 etc. update the GPS data in that row.
        """

        targets = []
        if idx == 1:
            targets = [1]
        elif idx == 2:
            targets = [2]
        elif idx == 3:
            targets = [1, 2]

        for target in targets:
            t = self.usv1_table if target == 1 else self.usv2_table

            if target_row == -1:
                # --- NEW ROW MODE ---
                r = t.rowCount()
                t.insertRow(r)
                t.setItem(r, 0, QtWidgets.QTableWidgetItem(str(r + 1)))
                t.setItem(r, 1, QtWidgets.QTableWidgetItem(f"{lat:.7f}"))
                t.setItem(r, 2, QtWidgets.QTableWidgetItem(f"{lon:.7f}"))
                self._boat_log(target, "New point added to the list from the map.")

            else:
                # --- UPDATE EXISTING ROW MODE ---
                if target_row < t.rowCount():
                    # Turn off for now to avoid a signal loop
                    t.blockSignals(True)
                    t.item(target_row, 1).setText(f"{lat:.7f}")
                    t.item(target_row, 2).setText(f"{lon:.7f}")
                    t.blockSignals(False)
                    self._boat_log(target, f"Point {target_row + 1} updated from the map.")

            # Updates (refresh the map and the table numbers)
            self._refresh_map_mission(target)
            self._update_table_labels(target)

    # --- Other helper functions (left from older versions) ---
    def _toggle_fence_draw(self, c):
        if c:
            self.btn_fence_draw.setText("Finish"); self.map.start_fence_drawing()
        else:
            self.btn_fence_draw.setText("Draw"); self.map.finish_fence_drawing()

    def _on_hb_timeout(self, bid=1):
        led = getattr(self, f"usv{bid}_led", None)
        if led is not None:
            led.setStyleSheet("background-color:#b71c1c; border-radius:10px;")

    def _on_drone_hb_timeout(self):
        self.drone_led.setStyleSheet("background-color:#b71c1c; border-radius:8px;")
        self.drone_color_lbl.setText("Link lost")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()] or ["COM3"]
        self.cmb_port_1.clear()
        self.cmb_port_1.addItems(ports)

    def _toggle(self):
        if not self._connected:
            # --- CONNECTING ---
            port1 = self.cmb_port_1.currentText().strip() if self.chk_rfd.isChecked() else ""
            baud = int(self.cmb_baud.currentText())
            csv_path = "telemetry_log.csv" if self.chk_csv.isChecked() else None
            peers = []
            if self.chk_udp.isChecked():
                sim = self.cmb_net.currentIndex() == 1
                for num, prof in sorted(boat_profiles().items()):
                    host = "127.0.0.1" if sim else prof["ip"]
                    if host and prof["udp_port"]:
                        peers.append((host, prof["udp_port"]))
                if not peers:
                    self.on_status("Ubiquiti: no ip/udp_port in the boat profiles")
            if not peers and not port1:
                self.on_status("No link to connect: tick Ubiquiti or RFD")
                return

            # One worker, both links
            self.worker_1 = LinkWorker(peers, port1, baud, csv_path)
            self.worker_1.packet.connect(self.on_packet)
            self.worker_1.status.connect(self.on_status)
            self.worker_1.link.connect(self.on_link)
            self.worker_1.links_state.connect(self._on_links_state)
            self.worker_1.start()

            # POLLING
            # 100 ms (10 Hz) asked the boat for a full telemetry packet faster than a
            # 57600 baud radio link can carry one, so the serial buffers backed up and
            # the shared command channel (emergency stop included) got slower and slower.
            # 500 ms fixed that, but once the packet was trimmed (objects dropped, waypoints
            # sent only on change, floats rounded) it became the only thing limiting the
            # display: measured on the 2026-08-12 run the panel refreshed at 2.3 Hz with a
            # single gap over 2 s in 88 s, i.e. the link was healthy and simply idle between
            # polls. At ~332 B a packet, 5 Hz is about 29% of a 57600 baud link.
            self._polling_timer = QtCore.QTimer(self)
            self._polling_timer.timeout.connect(self._perform_polling)
            # 300 ms, not 200. Three radios share one channel and SiK's TDMA is built for
            # two, so every extra boat transmission is another chance to clobber the
            # drone's - and the drone's packets are the rare ones. Going 5 Hz -> 3.3 Hz
            # takes the link from ~34% to ~24% occupied even after tripling the drone's
            # heartbeat, and a 300 ms panel refresh is indistinguishable from 200 ms.
            self._polling_timer.start(300)

            self._connected = True
            self.btn_connect.setText("CONNECTED")
            self.btn_connect.setStyleSheet("background-color: #00e676; color: black; font-weight: bold;")
            self.on_status(f"CONNECTED: Ubiquiti {len(peers)} boats"
                           + (f", RFD {port1} @ {baud}" if port1 else ""))

        else:
            # --- DISCONNECTING ---
            if self.worker_1:
                try:
                    self.worker_1.packet.disconnect()
                except:
                    pass
                self.worker_1.stop()
                if not self.worker_1.wait(1000): self.worker_1.terminate()
                self.worker_1 = None

            if hasattr(self, '_polling_timer'):
                self._polling_timer.stop()

            self._connected = False
            self.btn_connect.setText("CONNECT")
            self.btn_connect.setStyleSheet("background-color: #455a64; color: white;")
            self.on_status("Disconnected.")

    def _perform_polling(self):
        """
        No longer polls. The boat broadcasts on its own clock now (TELEM_BROADCAST_S),
        which deletes the whole six-hop request round trip - the GCS GUI timer, the air
        uplink, the boat's CommandReceiver sleep, the nav_process flag relay and the
        telem_process flag poll all used to sit between "timer fired" and "packet sent",
        and their combined jitter was as large as the polling interval itself. That is
        what an uneven, stuttering panel actually was. This timer now just refreshes the
        link-quality readout computed from what arrives.
        """
        self._update_link_stats()
        self._service_cmds()

    def _on_report(self, bid, d):
        """A boat's task report: ack it, show it, pass it to RoboCommand once."""
        rep = gcs_link.report_from_packet(d)
        if self.worker_1:                  # the boat resends until it hears this
            self.worker_1.queue_send({"target_id": bid, "cmd": "report_ack", "n": rep["id"]})
        seen = self._reports_seen.setdefault(bid, set())
        if rep["id"] in seen:
            return                         # a resend that crossed our ack
        seen.add(rep["id"])
        fields = " ".join(f"{k}={v}" for k, v in rep["fields"].items())
        where = (f" @ {rep['lat']:.6f},{rep['lon']:.6f}" if rep["lat"] is not None
                 and rep["lon"] is not None else "")
        text = f"REPORT #{rep['id']} {rep['type']} {fields} ({rep['task']}){where}"
        if self._rc is not None:
            flat = {"type": rep["type"], "lat": rep["lat"], "lon": rep["lon"], **rep["fields"]}
            text += (" -> RoboCommand" if self._rc.add_report(bid, flat)
                     else " (no RoboCommand message for this, this year)")
        self._boat_log(bid, text)

    def _toggle_robocommand(self, on):
        """Connect to RoboCommand (it keeps retrying while the cable or server is away)."""
        if self._rc is not None:
            self._rc_timer.stop()
            self._rc.close()
            self._rc = None
        if not on:
            self.lbl_rc.setText("off")
            self.lbl_rc.setStyleSheet("color:#90a4ae; font-weight:bold;")
            return
        try:
            from robocommand.client import Client
        except ImportError as e:
            self.on_status(f"RoboCommand: {e} (~/gcs-venv/bin/pip install protobuf)")
            self.chk_rc.setChecked(False)
            return
        self._rc = Client(self.ed_rc_host.text().strip(), self.sp_rc_port.value(),
                          self.ed_rc_team.text().strip() or "TEAM")
        self._rc_timer.start(100)
        self.on_status(f"RoboCommand: {self.ed_rc_host.text().strip()}:"
                       f"{self.sp_rc_port.value()}")

    def _robocommand_tick(self):
        if self._rc is None:
            return
        self._rc.tick()
        while self._rc.tasks:                 # tasks from the judges (2027)
            self._assign_task(self._rc.tasks.pop(0))
        text = self._rc.status()
        if text != self.lbl_rc.text():
            self.lbl_rc.setText(text)
            ok = self._rc.transport.connected
            self.lbl_rc.setStyleSheet(f"color:{'#00e676' if ok else '#ffa726'}; "
                                      "font-weight:bold;")

    def _boat_states(self):
        """Return the boats as usv_core.allocation sees them, from their telemetry."""
        now, out = time.monotonic(), []
        for number, prof in sorted(self._profiles.items()):
            st = self._boat_state.get(number, {})
            queue = [t for t in str(st.get("task_queue", "")).split(">") if t]
            out.append(BoatState(number, prof["name"], prof.get("capabilities", ()),
                                 heard_s=(now - st["t"]) if "t" in st else None,
                                 manual=bool(int(st.get("mod", 1) or 0)),
                                 task=str(st.get("task", "")), upcoming=queue))
        return out

    def _assign_task(self, task):
        """Give a judges' task to the boat usv_core.allocation picks, and say why."""
        name = task["task"]
        number, reason = allocate(name, self._boat_states(), named=task.get("boat"))
        if number is None:
            self.on_status(f"JUDGES' TASK {name} NOT ASSIGNED: {reason}")
            return
        payload = {"target_id": number, "cmd": "task", "task": name,
                   "urgent": 1 if task.get("urgent") else 0, "src": "robocommand"}
        if task.get("lat") is not None and task.get("lon") is not None:
            payload.update(lat=task["lat"], lon=task["lon"])
        if task.get("params"):
            payload["p"] = dict(task["params"])
        self._send_reliable(payload)
        boat = self._profiles.get(number, {}).get("name", number)
        self.on_status(f"JUDGES' TASK {name} -> USV {number} ({boat})")
        self._boat_log(number, f"JUDGES' TASK: {name}" + (" (URGENT)" if task.get("urgent") else ""))

    def _robocommand_test_task(self):
        """Hand the allocation a task as if RoboCommand had sent it (simulation tests)."""
        name = self.cmb_rc_task.currentText().split()[0]
        task = {"task": name, "urgent": True}
        if name == "rally":
            pts = [st["position"] for st in self._boat_state.values()
                   if st.get("position")]
            if not pts:
                self.on_status("A rally point needs a boat's position first")
                return
            lat = sum(p["lat"] for p in pts) / len(pts) + 20.0 / 111000.0
            lon = sum(p["lon"] for p in pts) / len(pts)
            task.update(lat=lat, lon=lon)
        self._assign_task(task)

    def _robocommand_test_report(self):
        """Send one gate report for USV 1 at its last position (to check the path)."""
        if self._rc is None:
            self.on_status("RoboCommand is not connected")
            return
        tel = self._rc.bridge.telemetry.get(1, (0.0, {}))[1]
        pos = tel.get("position") or {}
        self._rc.add_report(1, {"type": "gate", "gate": "entry", "lat": pos.get("lat", 0.0),
                                "lon": pos.get("lon", 0.0)})
        self.on_status("RoboCommand: test report queued (entry gate, USV 1)")

    def _on_links_state(self, ages):
        """Show when each link last brought anything (both should be fresh)."""
        parts, worst = [], 0.0
        for name, label in (("udp", "UBNT"), ("rfd", "RFD")):
            if name in ages:
                age = ages[name]
                parts.append(f"{label} {'—' if age is None else f'{age:.1f}s'}")
                worst = max(worst, 99.0 if age is None else age)
        self.lbl_links.setText("  ".join(parts) or "no link")
        colour = "#00e676" if worst < 2.0 else ("#ffa726" if worst < 5.0 else "#ff5252")
        self.lbl_links.setStyleSheet(f"color:{colour}; font-weight:bold;")

    def _update_link_stats(self):
        """Live link quality per boat, from its sequence numbers and arrival times."""
        now = time.time()
        for bid, lbl in self.link_lbls.items():
            st = self._lk_by.get(bid)
            if not st or not st["times"]:
                lbl.setText("LINK: no data")
                lbl.setStyleSheet("color:#90a4ae; font-weight:bold;")
                continue
            # Rate and largest arrival gap over the last 10 s; loss over ~100 packets.
            recent = [t for t in st["times"] if now - t <= 10.0]
            rate = len(recent) / 10.0
            gaps = [b - a for a, b in zip(recent, recent[1:])]
            tail_gap = now - st["times"][-1]
            max_gap = max(gaps + [tail_gap]) if gaps else tail_gap
            total = st["got"] + st["lost"]
            loss = (100.0 * st["lost"] / total) if total else 0.0
            lbl.setText(f"LINK: {rate:.1f} Hz · loss {loss:.0f}% · gap {max_gap:.1f}s"
                        f" · {st.get('via', '')}")
            if rate >= 2.5 and loss < 10.0:
                style = "color:#00e676; font-weight:bold;"       # healthy
            elif rate >= 1.5:
                style = "color:#ffa726; font-weight:bold;"       # stuttering
            else:
                style = "color:#ff5252; font-weight:bold;"       # bad
            lbl.setStyleSheet(style)

    def _on_map_changed(self):
        d = self.MAP_PRESETS[self.cmb_maps.currentText()]
        self.map.change_map(res_path(d['file']), d['n'], d['s'], d['w'], d['e'])

    def _update_ring_settings(self):
        self.map.ring_step_m = self.sp_ring_step.value();
        self.map.ring_count = self.sp_ring_count.value()
        if self.map.rings_enabled: self.map.update_range_rings()

    def _toggle_rings(self, c):
        self.map.rings_enabled = c; self.map.update_range_rings()

    def _start_course_marking(self, n):
        self.map.start_defining_course(n)
        self.on_status(f"Course {n}: selecting corners")

        # Find the tab index (A=0, B=1, C=2)
        idx = ["A", "B", "C"].index(n)

        # Switch to that tab (even if the user is elsewhere)
        self.tabs_course.setCurrentIndex(idx)

        # --- DYNAMIC COLOUR CHANGE ---
        # Only the selected tab title (QTabBar::tab:selected) is YELLOW.
        # The others keep the default (dark).

        # Colour codes:
        # Selected (active): #ffeb3b (yellow), text: black
        # Normal: #455a64 (grey), text: white

        self.tabs_course.setStyleSheet(f"""
            QTabBar::tab {{
                background: #455a64;
                color: white;
                padding: 8px 20px;
                margin: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QTabBar::tab:selected {{
                background: #ffeb3b;
                color: black;
                font-weight: bold;
            }}
        """)

    def _update_grid_params(self, n):
        ui = self.course_ui[n];
        self.map.courses[n].update({"rows": ui["sp_row"].value(), "cols": ui["sp_col"].value()})
        self.map.render_course_grid(n)

    def _save_courses(self):
        p, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save", "courses.json")
        if p:
            with open(p, "w") as f:
                d = {k: {"corners": v["corners"], "rows": v["rows"], "cols": v["cols"]} for k, v in
                     self.map.courses.items()}
                json.dump(d, f)

    def _save_single_course(self, name):
        """Save only the selected course (A, B or C)."""
        data = self.map.courses.get(name)
        if not data or not data["corners"]:
            self.on_status(f"WARNING: no data for course {name}, not saved.")
            return

        # Suggested file name: parkur_A.json
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, f"Save course {name}", f"parkur_{name}.json",
                                                        "JSON (*.json)")
        if path:
            to_save = {
                "name": name,
                "corners": data["corners"],
                "rows": self.course_ui[name]["sp_row"].value(),
                "cols": self.course_ui[name]["sp_col"].value()
            }
            try:
                with open(path, "w") as f:
                    json.dump(to_save, f, indent=4)
                self.on_status(f"Course {name} saved: {os.path.basename(path)}")
            except Exception as e:
                self.on_status(f"Save error: {e}")

    def _load_single_course(self, name):
        """Load the selected course from a file."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, f"Load course {name}", "", "JSON (*.json)")
        if path:
            try:
                with open(path, "r") as f:
                    d = json.load(f)

                # Keep the data
                self.map.courses[name]["corners"] = d.get("corners", [])

                # Update the spinboxes
                r = d.get("rows", 4)
                c = d.get("cols", 4)
                self.course_ui[name]["sp_row"].setValue(r)
                self.course_ui[name]["sp_col"].setValue(c)

                # Update the map (corners and grid)
                self.map.courses[name]["rows"] = r
                self.map.courses[name]["cols"] = c
                self.map.render_course_grid(name)

                self.on_status(f"Course {name} loaded.")
            except Exception as e:
                self.on_status(f"Load error: {e}")

    def _clear_single_course(self, name):
        """Delete only the selected course from the map and memory."""
        # 1. Clear the memory
        self.map.courses[name]["corners"] = []

        # 2. Clear the drawing
        self.map.clear_course_visuals(name)

        self.on_status(f"Course {name} deleted.")

    def _load_courses(self):
        p, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load")
        if p:
            with open(p, "r") as f:
                d = json.load(f)
                for k, v in d.items():
                    if k in self.map.courses:
                        self.map.courses[k].update(v)
                        self.course_ui[k]["sp_row"].setValue(v["rows"]);
                        self.course_ui[k]["sp_col"].setValue(v["cols"])
                        self.map.render_course_grid(k)

    @staticmethod
    def _stamp():
        return time.strftime("%H:%M:%S")

    @QtCore.Slot(str)
    def on_status(self, s):
        """Write a line to the system log, with the time."""
        if not hasattr(self, "sys_log"):          # while the window is being built
            print(s)
            return
        self.sys_log.appendPlainText(f"{self._stamp()} {s}")

    def _boat_log(self, bid, s):
        """Write a line to one boat's log (its panel), with the time."""
        log = getattr(self, f"usv{bid}_log", None)
        if log is None:
            self.on_status(f"USV {bid}: {s}")
        else:
            log.appendPlainText(f"{self._stamp()} {s}")

    @QtCore.Slot(bool)
    def on_link(self, b):
        self.on_status("LINK OK" if b else "LINK LOST")

    def _layout_rope_stripes(self):
        w = self.content.width();
        h = self.content.height();
        t = self._rope_thk
        self.rope_top.setGeometry(0, 0, w, t);
        self.rope_bottom.setGeometry(0, h - t, w, t)
        self.rope_left.setGeometry(0, 0, t, h);
        self.rope_right.setGeometry(w - t, 0, t, h)

    def eventFilter(self, o, e):
        if o == self.content and e.type() == QtCore.QEvent.Resize: self._layout_rope_stripes()
        return super().eventFilter(o, e)

    def closeEvent(self, e):
        if self._rc is not None:
            self._rc.close()
        if self.worker_1:
            self.worker_1.stop()
            self.worker_1.wait(1000)
        e.accept()

    def _fmt_float(self, v):
        try:
            return f"{float(v):.1f}"
        except:
            return "-"


def main():
    app = QtWidgets.QApplication(sys.argv)
    icon_path = res_path("cross.ico").replace("\\", "/")
    app.setWindowIcon(QtGui.QIcon(icon_path))

    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()