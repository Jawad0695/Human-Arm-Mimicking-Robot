import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class HandTrackerToArm(Node):
    def __init__(self):
        super().__init__('hand_tracker_arm')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # MediaPipe setup
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils

        # OpenCV camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        # For smooth joint movement
        self.prev_joint_1 = 0.0
        self.prev_joint_2 = 0.0
        self.alpha = 0.2  # smoothing factor

        # Timer to run update loop every 0.05s (20 Hz)
        self.timer = self.create_timer(0.05, self.main_loop)

    def main_loop(self):
        ret, img = self.cap.read()
        if not ret:
            return

        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(imgRGB)
        img_h, img_w, _ = img.shape

        if results.multi_hand_landmarks:
            handLms = results.multi_hand_landmarks[0]
            lm = handLms.landmark[8]  # Index fingertip

            cx, cy = int(lm.x * img_w), int(lm.y * img_h)
            self.send_joint_command(cx, cy, img_w, img_h)

            # Draw
            cv2.circle(img, (cx, cy), 10, (0, 255, 0), cv2.FILLED)
            self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)

        cv2.imshow("Hand Tracker", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def send_joint_command(self, x, y, w, h):
        # Normalize x and y to joint angle ranges
        joint_1 = ((x / w) * 2 - 1) * 1.57  # [-1.57, 1.57]
        joint_2 = ((y / h) * 2 - 1) * 1.0   # [-1.0, 1.0]
        joint_4 = 0.0  # fixed or can be extended later

        # Smooth the joint motion
        joint_1 = self.alpha * joint_1 + (1 - self.alpha) * self.prev_joint_1
        joint_2 = self.alpha * joint_2 + (1 - self.alpha) * self.prev_joint_2
        self.prev_joint_1 = joint_1
        self.prev_joint_2 = joint_2

        # ROS2 message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint_1', 'joint_2', 'joint_4']

        point = JointTrajectoryPoint()
        point.positions = [joint_1, joint_2, joint_4]
        point.time_from_start = Duration(sec=1)

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

        self.get_logger().info(f"Sent: joint_1={joint_1:.2f}, joint_2={joint_2:.2f}, joint_4={joint_4:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackerToArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
