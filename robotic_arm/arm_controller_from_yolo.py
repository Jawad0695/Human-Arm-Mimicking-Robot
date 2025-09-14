import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmFollower(Node):
    def __init__(self):
        super().__init__('arm_follower_node')

        # Subscribe to YOLO detection centers
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/yolo_box_center',
            self.listener_callback,
            10
        )

        # Publisher to joint trajectory controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Assuming 2 joints in your robotic arm: shoulder & elbow
        self.joint_names = ['joint1', 'joint2']  # update based on your URDF

    def listener_callback(self, msg):
        cx, cy = msg.data[0], msg.data[1]
        self.get_logger().info(f'Received center: x={cx:.1f}, y={cy:.1f}')

        # Convert pixel position (cx, cy) to joint angles (this is a dummy mapping)
        # You will need to calibrate this with real arm limits
        joint1 = (cx / 640.0) * 2.0 - 1.0  # maps to [-1, 1] rad
        joint2 = (cy / 480.0) * 2.0 - 1.0

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2]
        point.time_from_start.sec = 1

        trajectory_msg.points.append(point)
        self.publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published joint angles: [{joint1:.2f}, {joint2:.2f}]')


def main(args=None):
    rclpy.init(args=args)
    node = ArmFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

