"""
IK node: subscribes to a desired tip position (x, y) and publishes the joint
angles that place the arm's tip there, using the project's tested kinematics.

    in :  /target_point   geometry_msgs/Point       desired tip, mm
    out:  /joint_states   sensor_msgs/JointState     shoulder, elbow, wrist (deg)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from eyedropper_control.kinematics import inverse, wrist_for


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")
        self.sub = self.create_subscription(Point, "target_point",
                                            self.on_target, 10)
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.get_logger().info("IK node ready — waiting for /target_point")

    def on_target(self, msg):
        try:
            t1, t2 = inverse(msg.x, msg.y)        # degrees
            t3 = wrist_for(t1, t2)
        except ValueError as e:
            self.get_logger().warn(
                f"unreachable ({msg.x:.0f}, {msg.y:.0f}): {e}")
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["shoulder", "elbow", "wrist"]
        js.position = [t1, t2, t3]
        self.pub.publish(js)
        self.get_logger().info(
            f"({msg.x:.0f}, {msg.y:.0f}) -> "
            f"sh {t1:.1f}  el {t2:.1f}  wr {t3:.1f}")


def main():
    rclpy.init()
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()