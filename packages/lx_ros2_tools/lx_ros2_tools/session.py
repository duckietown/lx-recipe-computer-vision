"""``RosSession``: use ROS 2 from a Jupyter notebook (the Duckietown code editor).

    from lx_ros2_tools.session import RosSession
    ros = RosSession("myrobot")               # connects to the robot's zenoh router, spins in a thread
    ros.set_parameters("pid_controller_node", kp=2.0)
    ros.call_trigger("pid_controller_node/start")
    sub = ros.subscribe("lane_filter_node/lane_pose", LanePose, lambda m: print(m.d))
    ros.shutdown()

Topic/node/service names without a leading ``/`` are prefixed with the robot namespace.
"""
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

_ROUTER_PORT = 7447


def _configure_env(robot_name: str, zenoh_endpoint: Optional[str]) -> None:
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    if "ZENOH_CONFIG_OVERRIDE" not in os.environ:
        endpoint = zenoh_endpoint or f"tcp/{robot_name}.local:{_ROUTER_PORT}"
        # client mode: everything is routed by the robot's router (a peer session would try to link
        # directly to the robot's nodes, which is unreachable from a laptop or from inside docker)
        os.environ["ZENOH_CONFIG_OVERRIDE"] = f'mode="client";connect/endpoints=["{endpoint}"]'


class RosSession:
    _counter = 0

    def __init__(self, robot_name: Optional[str] = None, node_name: Optional[str] = None,
                 zenoh_endpoint: Optional[str] = None):
        robot_name = (robot_name or os.environ.get("VEHICLE_NAME", "")).strip().lower()
        if robot_name.endswith(".local"):
            robot_name = robot_name[:-6]
        if not robot_name:
            raise ValueError("RosSession needs the robot name, e.g. RosSession('myrobot')")
        self.robot = robot_name
        _configure_env(robot_name, zenoh_endpoint)

        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        RosSession._counter += 1
        self.node = rclpy.create_node(node_name or f"lx_notebook_{os.getpid()}_{RosSession._counter}")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name="lx-ros-session", daemon=True)
        self._thread.start()
        self._subs: List[Any] = []
        self._pubs: Dict[str, Any] = {}

    # --- lifecycle ------------------------------------------------------------------------------
    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - executor shut down
            pass

    def shutdown(self) -> None:
        """Stop spinning and destroy the node (the rclpy context stays up for a new session)."""
        try:
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass

    # --- names ----------------------------------------------------------------------------------
    def name(self, name: str) -> str:
        """Absolute name of a topic/service/node given a name relative to the robot namespace."""
        name = name.strip()
        if name.startswith("/"):
            return name
        return f"/{self.robot}/{name}"

    # --- pub / sub ------------------------------------------------------------------------------
    def subscribe(self, topic: str, msg_type: Any, callback: Callable[[Any], None], sensor: bool = False):
        from rclpy.qos import qos_profile_sensor_data

        qos = qos_profile_sensor_data if sensor else 10
        sub = self.node.create_subscription(msg_type, self.name(topic), callback, qos)
        self._subs.append(sub)
        return sub

    def unsubscribe(self, sub) -> None:
        if sub in self._subs:
            self._subs.remove(sub)
        self.node.destroy_subscription(sub)

    def publisher(self, topic: str, msg_type: Any):
        key = self.name(topic)
        if key not in self._pubs:
            self._pubs[key] = self.node.create_publisher(msg_type, key, 10)
        return self._pubs[key]

    def publish(self, topic: str, msg: Any) -> None:
        self.publisher(topic, type(msg)).publish(msg)

    def wait_for_message(self, topic: str, msg_type: Any, timeout: float = 5.0, sensor: bool = False):
        """Block until one message arrives on ``topic`` (or raise ``TimeoutError``)."""
        box: List[Any] = []
        event = threading.Event()

        def _cb(m):
            if not box:
                box.append(m)
                event.set()

        sub = self.subscribe(topic, msg_type, _cb, sensor=sensor)
        try:
            if not event.wait(timeout):
                raise TimeoutError(f"no message on {self.name(topic)} within {timeout}s")
            return box[0]
        finally:
            self.unsubscribe(sub)

    # --- parameters -----------------------------------------------------------------------------
    def _client(self, srv_type: Any, service: str, timeout: float):
        client = self.node.create_client(srv_type, service)
        if not client.wait_for_service(timeout_sec=timeout):
            self.node.destroy_client(client)
            raise TimeoutError(f"service {service} not available (is the node running on {self.robot}?)")
        return client

    def _call(self, client, request, timeout: float):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                self.node.destroy_client(client)
                raise TimeoutError("service call timed out")
            time.sleep(0.02)
        self.node.destroy_client(client)
        if future.exception() is not None:
            raise future.exception()
        return future.result()

    def list_parameters(self, node: str, timeout: float = 5.0) -> List[str]:
        from rcl_interfaces.srv import ListParameters

        client = self._client(ListParameters, f"{self.name(node)}/list_parameters", timeout)
        res = self._call(client, ListParameters.Request(), timeout)
        hidden = ("use_sim_time", "qos_overrides", "start_type_description_service")
        return [n for n in res.result.names if not n.startswith(hidden)]

    def get_parameters(self, node: str, names: Sequence[str], timeout: float = 5.0) -> Dict[str, Any]:
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python

        client = self._client(GetParameters, f"{self.name(node)}/get_parameters", timeout)
        res = self._call(client, GetParameters.Request(names=list(names)), timeout)
        return {n: parameter_value_to_python(v) for n, v in zip(names, res.values)}

    def set_parameters(self, node: str, timeout: float = 5.0, **values: Any) -> Dict[str, str]:
        """Set parameters on a node. Values are coerced to the node's declared types.
        Returns ``{name: "ok" | reason}``."""
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter

        current = self.get_parameters(node, list(values.keys()), timeout)
        params = []
        for name, value in values.items():
            cur = current.get(name)
            if isinstance(cur, bool):
                value = bool(value)
            elif isinstance(cur, int):
                value = int(value)
            elif isinstance(cur, float):
                value = float(value)
            elif isinstance(cur, str):
                value = str(value)
            params.append(Parameter(name, value=value).to_parameter_msg())
        client = self._client(SetParameters, f"{self.name(node)}/set_parameters", timeout)
        res = self._call(client, SetParameters.Request(parameters=params), timeout)
        return {n: ("ok" if r.successful else (r.reason or "rejected")) for n, r in zip(values.keys(), res.results)}

    # --- services -------------------------------------------------------------------------------
    def call_trigger(self, service: str, timeout: float = 5.0):
        """Call a ``std_srvs/srv/Trigger`` service, e.g. ``"pid_controller_node/start"``."""
        from std_srvs.srv import Trigger

        client = self._client(Trigger, self.name(service), timeout)
        return self._call(client, Trigger.Request(), timeout)

    def call(self, service: str, srv_type: Any, request: Any, timeout: float = 5.0):
        client = self._client(srv_type, self.name(service), timeout)
        return self._call(client, request, timeout)

    # --- introspection --------------------------------------------------------------------------
    def topics(self) -> Dict[str, List[str]]:
        return {n: t for n, t in self.node.get_topic_names_and_types()}

    def nodes(self) -> List[str]:
        return [f"{ns.rstrip('/')}/{n}" for n, ns in self.node.get_node_names_and_namespaces()]

    def __repr__(self) -> str:
        return f"RosSession(robot={self.robot!r}, node={self.node.get_name()!r})"
