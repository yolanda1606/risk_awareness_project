import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('panda_trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        self.get_logger().info('Connecting to controller...')

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # --- WAYPOINT 1: Look Up (Slowly) ---
        point1 = JointTrajectoryPoint()
        # Joint 4 = -0.5 (Valid position)
        point1.positions = [2.8, 0.0, 0.0, 0.0, 0.0, 1.57, 0.785]
        point1.time_from_start.sec = 0  # Increased from 3s

        # --- WAYPOINT 2: Go to Start Position (Right) ---
        point2 = JointTrajectoryPoint()
        point2.positions = [-2.8, 0.0, 0.0, 0.0, 0.0, 1.57, 0.785]
        point2.time_from_start.sec = 15 # Increased from 6s

        # --- WAYPOINT 3: Super Slow 360 Sweep ---
        point3 = JointTrajectoryPoint()
        # Joint 3 = 1.57 (90 deg turn)
        # Joint 4 = -1.57 (90 deg Elbow)
        point3.positions = [2.8, 0.0, 0.0, 0.0, 0.0, 1.57, 0.785]
        point3.time_from_start.sec = 30 # Increased from 12s (Very slow scan)

        msg.points = [point1, point2, point3]
        self.publisher_.publish(msg)
        self.get_logger().info('Executing: Slow Motion Scan')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    import time
    time.sleep(1) 
    node.send_trajectory()
    rclpy.shutdown()

if __name__ == '__main__':
    main()