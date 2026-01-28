import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import pyspacemouse


class SpaceMousePublisher(Node):
    """
    A ROS2 Node that publishes 3D mouse input as Twist messages.

    This class initializes a ROS2 publisher to publish geometry_msgs/Twist messages
    based on the input from a 3D SpaceMouse device. It uses the pyspacemouse library
    to read the device state and publishes the corresponding linear and angular
    velocities at a fixed rate.
    """

    def __init__(self):
        super().__init__("spacemouse_publisher")
        self.get_logger().info("Initializing SpaceMouse publisher...")

        self.declare_parameter("operator_position_front", True)
        self._operator_position_front = (
            self.get_parameter("operator_position_front").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Operator position front: {self._operator_position_front}")

        self.declare_parameter("device_path", "")
        self._device_path = self.get_parameter("device_path").get_parameter_value().string_value

        self.declare_parameter(
            "twist_topic", "franka_controller/target_cartesian_velocity_percent"
        )
        self._twist_topic = (
            self.get_parameter("twist_topic").get_parameter_value().string_value
        )

        self._twist_publisher = self.create_publisher(Twist, self._twist_topic, 10)
        self._gripper_width_publisher = self.create_publisher(
            Float32, "gripper_client/target_gripper_width_percent", 10
        )
        self._timer = self.create_timer(0.01, self._timer_callback)
        self._device_open_success = pyspacemouse.open(
            dof_callback=None,
            button_callback_arr=[
                pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
                pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            ],
            path=self._device_path,
        )

    def _timer_callback(self):
        if not self._device_open_success:
            return

        state = pyspacemouse.read()

        twist_msg = Twist()
        twist_msg.linear.x = -float(state.y)
        twist_msg.linear.y = float(state.x)
        twist_msg.linear.z = float(state.z)
        twist_msg.angular.x = -float(state.roll)
        twist_msg.angular.y = -float(state.pitch)
        twist_msg.angular.z = -float(state.yaw)

        if not self._operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        self._twist_publisher.publish(twist_msg)

    def _button_callback(self, state, buttons, pressed_buttons):
        target_gripper_width_percent_msg = Float32()
        if 0 in pressed_buttons:
            print("Button 1 pressed")
            target_gripper_width_percent_msg.data = 0.0

        elif 1 in pressed_buttons:
            print("Button 2 pressed")
            target_gripper_width_percent_msg.data = 1.0

        self._gripper_width_publisher.publish(target_gripper_width_percent_msg)


def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = SpaceMousePublisher()
    rclpy.spin(spacemouse_publisher)
    spacemouse_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
