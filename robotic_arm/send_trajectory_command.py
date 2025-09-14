from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.node import Node

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Publish every 2 seconds until it moves once
        self.timer = self.create_timer(2.0, self.send_command)
        self.moved = False

    def send_command(self):
        if self.moved:
            return

        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_4']  # must match exactly

        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, -0.2]  # adjust values as needed
        point.time_from_start.sec = 2

        traj.points.append(point)

        self.publisher.publish(traj)
        self.get_logger().info("✔️ Sent joint trajectory to robot arm")
        self.moved = True  # Send only once

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

