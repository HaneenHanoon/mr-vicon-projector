import rclpy
from rclpy.node import Node
from time import sleep
from geometry_msgs.msg import Pose, Quaternion, Point, Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class ManualUpdater(Node):
    def __init__(self):
        super().__init__('manual_set_pose')
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service...')

        self.send_pose_repeatedly()

    def send_pose_repeatedly(self):
        for i in range(10):  # send 10 updates
            req = SetEntityState.Request()
            req.state = EntityState()
            req.state.name = 'my_robot'
            req.state.pose = Pose(
                position=Point(x=0.21, y=0.51, z=0.09),
                orientation=Quaternion(x=0.0, y=0.0, z=-0.48, w=0.87)
            )
            req.state.twist = Twist()
            req.state.reference_frame = 'world'

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info(f"Pose update {i+1} success.")
            else:
                self.get_logger().warn(f"Pose update {i+1} failed.")

            sleep(1)  # wait 1 second before next update

def main():
    rclpy.init()
    node = ManualUpdater()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
