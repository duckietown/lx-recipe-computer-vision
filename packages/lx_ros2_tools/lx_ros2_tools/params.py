"""``LiveParams``: declare ROS 2 parameters on a node and keep their values current.

Usage inside a node::

    self.params = LiveParams(self, {"kp": 1.0, "ki": 0.0, "kd": 0.0, "v_ref": 0.2})
    ...
    u = self.params.kp * error          # always the latest value set with `ros2 param set` or a notebook panel

An optional ``on_change(name, value)`` callback is invoked after every successful update.
"""
from typing import Any, Callable, Dict, Optional

from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class LiveParams:
    def __init__(self, node: Node, defaults: Dict[str, Any], on_change: Optional[Callable[[str, Any], None]] = None):
        object.__setattr__(self, "_node", node)
        object.__setattr__(self, "_values", {})
        object.__setattr__(self, "_on_change", on_change)
        for name, default in defaults.items():
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            self._values[name] = node.get_parameter(name).value
        node.add_on_set_parameters_callback(self._on_set)

    # --- rclpy callback -------------------------------------------------------------------------
    def _on_set(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._values:
                current = self._values[p.name]
                value = p.value
                # keep the declared type (a float parameter set with an int is fine)
                if isinstance(current, float) and isinstance(value, int) and not isinstance(value, bool):
                    value = float(value)
                elif isinstance(current, bool) and not isinstance(value, bool):
                    return SetParametersResult(successful=False, reason=f"{p.name} must be a bool")
                elif isinstance(current, int) and not isinstance(current, bool) and isinstance(value, float):
                    if not float(value).is_integer():
                        return SetParametersResult(successful=False, reason=f"{p.name} must be an int")
                    value = int(value)
                self._values[p.name] = value
                if self._on_change is not None:
                    self._on_change(p.name, value)
        return SetParametersResult(successful=True)

    # --- dict-like / attribute access -----------------------------------------------------------
    def __getattr__(self, name: str) -> Any:
        try:
            return self._values[name]
        except KeyError as e:
            raise AttributeError(name) from e

    def __setattr__(self, name: str, value: Any) -> None:
        # setting from inside the node goes through the ROS parameter API so that external
        # observers (ros2 param get, panels) see the same value
        if name not in self._values:
            raise AttributeError(f"unknown parameter {name!r}")
        self._node.set_parameters([Parameter(name, value=value)])

    def __getitem__(self, name: str) -> Any:
        return self._values[name]

    def as_dict(self) -> Dict[str, Any]:
        return dict(self._values)

    def __repr__(self) -> str:
        return f"LiveParams({self._values})"
