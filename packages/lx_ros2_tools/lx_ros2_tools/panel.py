"""ipywidgets panels for the notebooks (they replace the old VNC desktop tools).

    ros = RosSession("myrobot")
    ParamPanel(ros, "pid_controller_node", {"kp": (0, 20, 0.1), "v_ref": (0, 0.5, 0.01)})
    ActionButtons(ros, "pid_controller_node", ["start", "stop"])
    ImageView(ros, "camera_node/image/compressed")
    TopicPlot(ros, "lane_filter_node/lane_pose", "duckietown_msgs/msg/LanePose", ["d", "phi"])
    TrajectoryPlot(ros, "encoder_pose_node/odom")

Every panel displays itself when constructed in a notebook cell; keep a reference if you want to
``close()`` it later (closing unsubscribes).
"""
import asyncio
import collections
import importlib
import math
import threading
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import ipywidgets as W
from IPython.display import display

from .session import RosSession

Spec = Union[Tuple[float, float, float], Tuple[float, float], None]


def _resolve_type(msg_type: Union[str, Any]) -> Any:
    """``"duckietown_msgs/msg/LanePose"`` -> class."""
    if not isinstance(msg_type, str):
        return msg_type
    pkg, kind, name = msg_type.split("/")
    return getattr(importlib.import_module(f"{pkg}.{kind}"), name)


def _getattr_path(obj: Any, path: str) -> Any:
    for part in path.split("."):
        obj = getattr(obj, part)
    return obj


def _stamp_seconds(msg: Any) -> Optional[float]:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class _Panel:
    def __init__(self):
        self.widget: Optional[W.Widget] = None

    def _ipython_display_(self):
        display(self.widget)

    def close(self):
        if self.widget is not None:
            self.widget.close()


class ParamPanel(_Panel):
    """Read and set parameters of a node with sliders / text fields.

    ``params``: list of names, or ``{name: (min, max, step)}`` for sliders (``None`` for a text field).
    Omitted → all parameters of the node are listed.
    """

    def __init__(self, ros: RosSession, node: str, params: Union[Sequence[str], Dict[str, Spec], None] = None,
                 title: Optional[str] = None, timeout: float = 5.0):
        super().__init__()
        self.ros, self.node_name, self.timeout = ros, node, timeout
        specs: Dict[str, Spec] = dict(params) if isinstance(params, dict) else {n: None for n in (params or [])}
        if not specs:
            specs = {n: None for n in ros.list_parameters(node, timeout)}
        self.specs = specs
        self.fields: Dict[str, W.Widget] = {}
        self.status = W.HTML()
        self.send = W.Button(description="Send", button_style="primary", icon="paper-plane")
        self.read = W.Button(description="Read from robot", icon="refresh")
        self.send.on_click(lambda _: self.send_values())
        self.read.on_click(lambda _: self.read_values())
        rows = [W.HTML(f"<b>{title or ros.name(node)}</b>")]
        for name, spec in specs.items():
            w = self._make_field(name, spec)
            self.fields[name] = w
            rows.append(w)
        rows.append(W.HBox([self.send, self.read]))
        rows.append(self.status)
        self.widget = W.VBox(rows)
        try:
            self.read_values()
        except Exception as e:  # noqa: BLE001
            self.status.value = f"<span style='color:#b00'>{e}</span>"
        display(self.widget)

    def _make_field(self, name: str, spec: Spec) -> W.Widget:
        if spec is not None and len(spec) >= 2:
            lo, hi = float(spec[0]), float(spec[1])
            step = float(spec[2]) if len(spec) > 2 else (hi - lo) / 100.0
            return W.FloatSlider(description=name, min=lo, max=hi, step=step, value=lo, readout_format=".3f",
                                 continuous_update=False, style={"description_width": "120px"}, layout=W.Layout(width="520px"))
        return W.Text(description=name, value="", style={"description_width": "120px"}, layout=W.Layout(width="420px"))

    def values(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for name, w in self.fields.items():
            v = w.value
            if isinstance(w, W.Text):
                text = v.strip()
                if text.lower() in ("true", "false"):
                    v = text.lower() == "true"
                else:
                    try:
                        v = int(text) if text.lstrip("+-").isdigit() else float(text)
                    except ValueError:
                        v = text
            out[name] = v
        return out

    def read_values(self) -> Dict[str, Any]:
        current = self.ros.get_parameters(self.node_name, list(self.fields.keys()), self.timeout)
        for name, value in current.items():
            w = self.fields[name]
            if isinstance(w, W.FloatSlider):
                try:
                    fv = float(value)
                    w.min, w.max = min(w.min, fv), max(w.max, fv)
                    w.value = fv
                except (TypeError, ValueError):
                    pass
            else:
                w.value = str(value)
        self.status.value = "<span style='color:#080'>values read from the robot</span>"
        return current

    def send_values(self) -> Dict[str, str]:
        try:
            result = self.ros.set_parameters(self.node_name, self.timeout, **self.values())
        except Exception as e:  # noqa: BLE001
            self.status.value = f"<span style='color:#b00'>{e}</span>"
            raise
        bad = {k: v for k, v in result.items() if v != "ok"}
        self.status.value = ("<span style='color:#080'>parameters sent</span>" if not bad
                             else f"<span style='color:#b00'>rejected: {bad}</span>")
        return result


class ActionButtons(_Panel):
    """One button per ``std_srvs/Trigger`` service ``<node>/<action>``."""

    def __init__(self, ros: RosSession, node: str, actions: Iterable[str], timeout: float = 5.0):
        super().__init__()
        self.ros, self.node_name, self.timeout = ros, node, timeout
        self.status = W.HTML()
        buttons = []
        for action in actions:
            b = W.Button(description=action, button_style=("danger" if action in ("stop", "estop", "reset") else "success"))
            b.on_click(lambda _, a=action: self.trigger(a))
            buttons.append(b)
        self.widget = W.VBox([W.HTML(f"<b>{ros.name(node)}</b>"), W.HBox(buttons), self.status])
        display(self.widget)

    def trigger(self, action: str):
        try:
            res = self.ros.call_trigger(f"{self.node_name}/{action}", self.timeout)
        except Exception as e:  # noqa: BLE001
            self.status.value = f"<span style='color:#b00'>{action}: {e}</span>"
            return None
        color = "#080" if res.success else "#b00"
        self.status.value = f"<span style='color:{color}'>{action}: {res.message or ('ok' if res.success else 'failed')}</span>"
        return res


class ImageView(_Panel):
    """Live view of a ``sensor_msgs/CompressedImage`` topic (JPEG), throttled to ``rate_hz``."""

    def __init__(self, ros: RosSession, topic: str, rate_hz: float = 5.0, width: int = 480):
        super().__init__()
        from sensor_msgs.msg import CompressedImage

        self.ros = ros
        self._period = 1.0 / max(rate_hz, 0.1)
        self._last = 0.0
        self.image = W.Image(format="jpeg", width=width)
        self.label = W.HTML(f"<b>{ros.name(topic)}</b> <span style='color:#888'>waiting for images…</span>")
        self.widget = W.VBox([self.label, self.image])
        self._sub = ros.subscribe(topic, CompressedImage, self._cb, sensor=True)
        self._count = 0
        display(self.widget)

    def _cb(self, msg):
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        self._count += 1
        data = bytes(msg.data)
        if "png" in (msg.format or "").lower():
            self.image.format = "png"
        self.image.value = data
        self.label.value = f"<b>{self.ros.name(self._sub.topic_name)}</b> <span style='color:#888'>{self._count} frames</span>"

    def close(self):
        self.ros.unsubscribe(self._sub)
        super().close()


class _LivePlot(_Panel):
    """Base for matplotlib plots refreshed from the notebook event loop (ipympl backend)."""

    def __init__(self, title: str, refresh_hz: float = 4.0, figsize=(7, 3.5)):
        super().__init__()
        import matplotlib

        try:
            matplotlib.use("module://ipympl.backend_nbagg")
        except Exception:  # noqa: BLE001 - backend already selected / not available
            pass
        import matplotlib.pyplot as plt

        self._plt = plt
        self._lock = threading.Lock()
        self._dirty = False
        self._closed = False
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.ax.set_title(title)
        self.ax.grid(True)
        self.widget = getattr(self.fig.canvas, "_ipywidget", None) or self.fig.canvas
        self._task = None
        try:
            loop = asyncio.get_event_loop()
            self._task = loop.create_task(self._refresh_loop(1.0 / refresh_hz))
        except RuntimeError:
            self._task = None
        display(self.widget)

    async def _refresh_loop(self, period: float):
        while not self._closed:
            await asyncio.sleep(period)
            if self._dirty:
                with self._lock:
                    self._dirty = False
                    try:
                        self._redraw()
                    except Exception:  # noqa: BLE001
                        pass
                self.fig.canvas.draw_idle()

    def _redraw(self):
        raise NotImplementedError

    def close(self):
        self._closed = True
        self._plt.close(self.fig)


class TopicPlot(_LivePlot):
    """Plot numeric fields of a topic over time (``fields`` are attribute paths, e.g. ``"pose.x"``)."""

    def __init__(self, ros: RosSession, topic: str, msg_type: Union[str, Any], fields: Sequence[str],
                 window: float = 20.0, title: Optional[str] = None, refresh_hz: float = 4.0, sensor: bool = False):
        self.ros, self.fields, self.window = ros, list(fields), window
        self._t0 = time.monotonic()
        self._buf: Dict[str, collections.deque] = {f: collections.deque(maxlen=5000) for f in fields}
        self._times: collections.deque = collections.deque(maxlen=5000)
        super().__init__(title or ros.name(topic), refresh_hz)
        self.ax.set_xlabel("time [s]")
        self._lines = {f: self.ax.plot([], [], label=f)[0] for f in fields}
        self.ax.legend(loc="upper left")
        self._sub = ros.subscribe(topic, _resolve_type(msg_type), self._cb, sensor=sensor)

    def _cb(self, msg):
        with self._lock:
            t = time.monotonic() - self._t0
            self._times.append(t)
            for f in self.fields:
                try:
                    self._buf[f].append(float(_getattr_path(msg, f)))
                except Exception:  # noqa: BLE001
                    self._buf[f].append(math.nan)
            self._dirty = True

    def _redraw(self):
        if not self._times:
            return
        t_now = self._times[-1]
        ts = list(self._times)
        for f, line in self._lines.items():
            line.set_data(ts, list(self._buf[f]))
        self.ax.set_xlim(max(0.0, t_now - self.window), max(t_now, self.window))
        self.ax.relim()
        self.ax.autoscale_view(scalex=False, scaley=True)

    def close(self):
        self.ros.unsubscribe(self._sub)
        super().close()


class TrajectoryPlot(_LivePlot):
    """Plot the x/y path of ``nav_msgs/Odometry`` (default) or ``duckietown_msgs/Pose2DStamped``."""

    def __init__(self, ros: RosSession, topic: str, msg_type: Union[str, Any] = "nav_msgs/msg/Odometry",
                 title: Optional[str] = None, refresh_hz: float = 4.0, max_points: int = 20000):
        self.ros = ros
        self._xy: collections.deque = collections.deque(maxlen=max_points)
        self._heading: Optional[Tuple[float, float, float]] = None
        super().__init__(title or ros.name(topic), refresh_hz, figsize=(5.5, 5.5))
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_aspect("equal", adjustable="datalim")
        (self._line,) = self.ax.plot([], [], "-", lw=1.5)
        (self._robot,) = self.ax.plot([], [], "o", color="orange")
        self._arrow = None
        self._type = _resolve_type(msg_type)
        self._sub = ros.subscribe(topic, self._type, self._cb)

    def _cb(self, msg):
        try:
            if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):  # nav_msgs/Odometry
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                x, y = p.x, p.y
            elif hasattr(msg, "theta"):  # duckietown_msgs/Pose2DStamped
                x, y, yaw = msg.x, msg.y, msg.theta
            elif hasattr(msg, "position"):  # geometry_msgs/Pose
                x, y, yaw = msg.position.x, msg.position.y, 0.0
            else:
                return
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._xy.append((x, y))
            self._heading = (x, y, yaw)
            self._dirty = True

    def _redraw(self):
        if not self._xy:
            return
        xs, ys = zip(*self._xy)
        self._line.set_data(xs, ys)
        x, y, yaw = self._heading
        self._robot.set_data([x], [y])
        if self._arrow is not None:
            self._arrow.remove()
        self._arrow = self.ax.annotate("", xy=(x + 0.05 * math.cos(yaw), y + 0.05 * math.sin(yaw)), xytext=(x, y),
                                       arrowprops=dict(arrowstyle="->", color="orange"))
        self.ax.relim()
        self.ax.autoscale_view()

    def clear(self):
        with self._lock:
            self._xy.clear()
            self._dirty = True

    def close(self):
        self.ros.unsubscribe(self._sub)
        super().close()
