import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TwistMux(Node):
    def __init__(self):
        super().__init__("spacemouse_twist_mux")

        self.declare_parameter("input_spacemouse_topic", "spacemouse_mux/input_spacemouse")
        self.declare_parameter("input_home_topic", "spacemouse_mux/input_home")
        self.declare_parameter(
            "output_topic", "franka_controller/target_cartesian_velocity_percent"
        )
        self.declare_parameter("mode_topic", "spacemouse_mux/mode")
        self.declare_parameter("default_mode", "spacemouse")

        self._input_spacemouse_topic = (
            self.get_parameter("input_spacemouse_topic").get_parameter_value().string_value
        )
        self._input_home_topic = (
            self.get_parameter("input_home_topic").get_parameter_value().string_value
        )
        self._output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self._mode_topic = (
            self.get_parameter("mode_topic").get_parameter_value().string_value
        )
        self._mode = (
            self.get_parameter("default_mode").get_parameter_value().string_value
        )

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self.create_subscription(Twist, self._input_spacemouse_topic, self._on_spacemouse, 10)
        self.create_subscription(Twist, self._input_home_topic, self._on_home, 10)
        self.create_subscription(String, self._mode_topic, self._on_mode, 10)

    def _on_mode(self, msg: String) -> None:
        new_mode = msg.data.strip().lower()
        if new_mode in ("spacemouse", "home"):
            self._mode = new_mode

    def _on_spacemouse(self, msg: Twist) -> None:
        if self._mode == "spacemouse":
            self._pub.publish(msg)

    def _on_home(self, msg: Twist) -> None:
        if self._mode == "home":
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
