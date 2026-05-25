import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw / 2.0
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def quaternion_to_yaw(z: float, w: float):
    return 2.0 * math.atan2(z, w)


def rotate_point_2d(x: float, y: float, yaw: float):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x * cos_yaw - y * sin_yaw,
        x * sin_yaw + y * cos_yaw,
    )

class OdomOffsetNode(Node):
    def __init__(self):
        super().__init__('odom_offset_node')

        self.declare_parameter('option', 3)
        self.option = self.get_parameter('option').value

        self.offset_configs = {
            0: {
                'q0_x': 0.0,
                'q0_y': 0.0,
                'q0_yaw': 0.0,
                'passthrough': True,
                'position_rotation': 0.0,
            },
            3: {
                'q0_x': 0.75,
                'q0_y': 0.75,
                'q0_yaw': 0.0,
                'passthrough': False,
                'position_rotation': 0.0,
            },
            5: {
                'q0_x': 3.25,
                'q0_y': 0.75,
                'q0_yaw': math.pi,
                'passthrough': False,
                'position_rotation': math.pi / 2.0,
            },
            8: {
                'q0_x': 0.75,
                'q0_y': 0.75,
                'q0_yaw': math.pi / 2.0,
                'passthrough': False,
                'position_rotation': 0.0,
            },
        }

        self.initialized_origin = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        if self.option not in self.offset_configs:
            self.get_logger().warn(
                f"Option {self.option} is not supported. Falling back to option 3."
            )
            self.option = 3

        selected = self.offset_configs[self.option]
        self.get_logger().info(
            f"Using offset option {self.option}: q0=({selected['q0_x']}, {selected['q0_y']}, {selected['q0_yaw']})"
        )
        
        # Subscribe to the original odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # Publish to your new 3rd topic
        self.publisher = self.create_publisher(Odometry, '/odom_offset', 10)
        
    def odom_callback(self, msg: Odometry):
        # 1. Change the frame_id so it doesn't conflict with the old odom frame
        msg.header.frame_id = 'odom_offset'

        config = self.offset_configs[self.option]

        if config['passthrough']:
            self.publisher.publish(msg)
            return

        original_x = msg.pose.pose.position.x
        original_y = msg.pose.pose.position.y
        original_yaw = quaternion_to_yaw(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        if not self.initialized_origin:
            self.initial_x = original_x
            self.initial_y = original_y
            self.initial_yaw = original_yaw
            self.initialized_origin = True

            self.get_logger().info(
                f"Captured real odom origin: ({self.initial_x:.3f}, {self.initial_y:.3f}, {self.initial_yaw:.3f})"
            )

        relative_x = original_x - self.initial_x
        relative_y = original_y - self.initial_y
        relative_yaw = original_yaw - self.initial_yaw

        rotated_x, rotated_y = rotate_point_2d(relative_x, relative_y, config['position_rotation'])
        msg.pose.pose.position.x = config['q0_x'] + rotated_x
        msg.pose.pose.position.y = config['q0_y'] + rotated_y

        world_yaw = config['q0_yaw'] + relative_yaw
        qx, qy, qz, qw = yaw_to_quaternion(world_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # 3. Publish to the new topic
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomOffsetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()