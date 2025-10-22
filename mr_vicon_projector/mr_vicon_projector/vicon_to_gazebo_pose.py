# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseStamped, Twist
# # from gazebo_msgs.srv import SetEntityState
# # from gazebo_msgs.msg import EntityState
# # from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# # import time

# # # class ViconToGazebo(Node):
# # #     def __init__(self):
# # #         super().__init__('vicon_to_gazebo')

# # #         self.declare_parameter('vicon_topic', '/vrpn_mocap/iRobot_create3_te/pose')
# # #         self.declare_parameter('model_name', 'my_robot')

# # #         self.vicon_topic = self.get_parameter('vicon_topic').get_parameter_value().string_value
# # #         self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

# # #         qos = QoSProfile(
# # #             reliability=ReliabilityPolicy.BEST_EFFORT,
# # #             history=HistoryPolicy.KEEP_LAST,
# # #             depth=1
# # #         )

# # #         self.cli = self.create_client(SetEntityState, '/set_entity_state')
# # #         while not self.cli.wait_for_service(timeout_sec=1.0):
# # #             self.get_logger().warn('Waiting for Gazebo service...')

# # #         self.subscription = self.create_subscription(
# # #             PoseStamped,
# # #             self.vicon_topic,
# # #             self.vicon_callback,
# # #             qos
# # #         )

# # #         self.last_update_time = 0.0
# # #         self.update_interval = 0.05 # seconds

# # #     def vicon_callback(self, msg):

# # #         self.get_logger().info(
# # #         f"✅ Got Vicon Pose: x={msg.pose.position.x:.3f}, "
# # #         f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}" 
# # #         )
        
# # #         current_time = time.time()
# # #         if current_time - self.last_update_time < self.update_interval:
# # #             return  # Skip if it's too soon

# # #         self.last_update_time = current_time

# # #         request = SetEntityState.Request()
# # #         request.state = EntityState()
# # #         request.state.name = self.model_name
# # #         request.state.pose = msg.pose
# # #         request.state.twist = Twist()
# # #         request.state.reference_frame = 'world'

# # #         self.get_logger().info(
# # #             f"Publishing to Gazebo: x={msg.pose.position.x:.2f}, "
# # #             f"y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
# # #         )

# # #         future = self.cli.call_async(request)
# # #         future.add_done_callback(self.callback_response)

# # #     def callback_response(self, future):
# # #         try:
# # #             result = future.result()
# # #             if not result.success:
# # #                 self.get_logger().warn("⚠️ Gazebo update returned success=False.")
# # #         except Exception as e:
# # #             self.get_logger().error(f"Service call failed: {e}")

# # # def main():
# # #     rclpy.init()
# # #     node = ViconToGazebo()
# # #     rclpy.spin(node)
# # #     node.destroy_node()
# # #     rclpy.shutdown()

# # # if __name__ == '__main__':
# # #     main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class ViconToGazebo(Node):
    def __init__(self):
        
        
        super().__init__('vicon_to_gazebo')
        self.pub_transformed_pose = self.create_publisher(PoseStamped, '/gazebo_robot_pose', 10)
        self.declare_parameter('vicon_topic', '/vrpn_mocap/iRobot_create3_te/pose')
        self.declare_parameter('model_name', 'my_robot')

        self.vicon_topic = self.get_parameter('vicon_topic').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Gazebo service...')

        self.subscription = self.create_subscription(
            PoseStamped,
            self.vicon_topic,
            self.vicon_callback,
            qos
        )

        self.last_update_time = 0.0
        self.update_interval = 0.001 # seconds
        self.last_pose = None

    def vicon_callback(self, msg):
        self.last_pose = msg.pose

        self.get_logger().info(
            f"✅ Got Vicon Pose: x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
        )

        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return

        self.last_update_time = current_time

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.model_name
        
        
        #request.state.pose = msg.pose
        request.state.pose.position.x = 2.25*(msg.pose.position.y) if msg.pose.position.y >= 0 else 2.25*(msg.pose.position.y)
        request.state.pose.position.y = -2.25*(msg.pose.position.x) if msg.pose.position.x >= 0 else -2.25*(msg.pose.position.x)
        #request.state.pose.position.x = 2.2*(msg.pose.position.y)
        #request.state.pose.position.y = -2.25*(msg.pose.position.x)
        request.state.pose.position.z = 2.2*msg.pose.position.z

        
        
        
        # print("########msg.pos")
        # print(request.state.pose.position.x)
        
        
        request.state.twist = Twist()
        request.state.reference_frame = 'world'
        transformed_pose_msg = PoseStamped()
        transformed_pose_msg.header.stamp = self.get_clock().now().to_msg()
        transformed_pose_msg.header.frame_id = "odom"  # or "world" based on your setup
        transformed_pose_msg.pose = request.state.pose  # the already transformed pose

        self.pub_transformed_pose.publish(transformed_pose_msg)

        
       
        
        self.get_logger().info(
            f"Publishing to Gazebo: x={request.state.pose.position.x :.2f}, "
            f"y={request.state.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
        )

        future = self.cli.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn("⚠️ Gazebo update returned success=False.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    
        
    def get_current_position(self):
        if self.last_pose is not None:
            return (self.last_pose.position.x, self.last_pose.position.y)
        else:
            self.get_logger().warn("No pose received yet.")
            return None

def main():
    rclpy.init()
    node = ViconToGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import PoseStamped, Twist
# # from gazebo_msgs.srv import SetEntityState
# # from gazebo_msgs.msg import EntityState
# # from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# # import time

# # class ViconToGazebo(Node):
# #     def __init__(self):
# #         super().__init__('vicon_to_gazebo')

# #         self.declare_parameter('vicon_topic', '/vrpn_mocap/iRobot_create3_te/pose')
# #         self.declare_parameter('model_name', 'my_robot')

# #         self.vicon_topic = self.get_parameter('vicon_topic').get_parameter_value().string_value
# #         self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

# #         qos = QoSProfile(
# #             reliability=ReliabilityPolicy.BEST_EFFORT,
# #             history=HistoryPolicy.KEEP_LAST,
# #             depth=1
# #         )

# #         self.cli = self.create_client(SetEntityState, '/set_entity_state')
# #         while not self.cli.wait_for_service(timeout_sec=1.0):
# #             self.get_logger().warn('Waiting for Gazebo service...')

# #         self.subscription = self.create_subscription(
# #             PoseStamped,
# #             self.vicon_topic,
# #             self.vicon_callback,
# #             qos
# #         )

# #         self.last_update_time = 0.0
# #         self.update_interval = 0.001 # seconds
# #         self.last_pose = None

# #     def vicon_callback(self, msg):
# #         self.last_pose = msg.pose

# #         self.get_logger().info(
# #             f"✅ Got Vicon Pose: x={msg.pose.position.x:.3f}, "
# #             f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
# #         )

# #         current_time = time.time()
# #         if current_time - self.last_update_time < self.update_interval:
# #             return

# #         self.last_update_time = current_time

# #         request = SetEntityState.Request()
# #         request.state = EntityState()
# #         request.state.name = self.model_name
        
        
# #         #request.state.pose = msg.pose
# #         request.state.pose.position.x = 2.175*(msg.pose.position.y) if msg.pose.position.y >= 0 else 2.2*(msg.pose.position.y)
# #         request.state.pose.position.y = -2.25*(msg.pose.position.x) if msg.pose.position.x >= 0 else -2.22*(msg.pose.position.x)
# #         #request.state.pose.position.x = 2.2*(msg.pose.position.y)
# #         #request.state.pose.position.y = -2.25*(msg.pose.position.x)
# #         request.state.pose.position.z = 2.2*msg.pose.position.z

        
        
        
# #         # print("########msg.pos")
# #         # print(request.state.pose.position.x)
        
        
# #         request.state.twist = Twist()
# #         request.state.reference_frame = 'world'
        
       
        
# #         self.get_logger().info(
# #             f"Publishing to Gazebo: x={ request.state.pose.position.x :.2f}, "
# #             f"y={request.state.pose.position.y :.2f}, z={request.state.pose.position.z:.2f}"
# #         )

# #         future = self.cli.call_async(request)
# #         future.add_done_callback(self.callback_response)

# #     def callback_response(self, future):
# #         try:
# #             result = future.result()
# #             if not result.success:
# #                 self.get_logger().warn("⚠️ Gazebo update returned success=False.")
# #         except Exception as e:
# #             self.get_logger().error(f"Service call failed: {e}")

    
        
# #     def get_current_position(self):
# #         if self.last_pose is not None:
# #             return (self.last_pose.position.x, self.last_pose.position.y)
# #         else:
# #             self.get_logger().warn("No pose received yet.")
# #             return None

# # def main():
# #     rclpy.init()
# #     node = ViconToGazebo()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, Twist
# from gazebo_msgs.srv import SetEntityState
# from gazebo_msgs.msg import EntityState
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import time

# class ViconToGazebo(Node):
#     def __init__(self):
#         super().__init__('vicon_to_gazebo')

#         self.declare_parameter('vicon_topic', '/vrpn_mocap/iRobot_create3_te/pose')
#         self.declare_parameter('model_name', 'my_robot')

#         self.vicon_topic = self.get_parameter('vicon_topic').get_parameter_value().string_value
#         self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         self.cli = self.create_client(SetEntityState, '/set_entity_state')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().warn('Waiting for Gazebo service...')

#         self.subscription = self.create_subscription(
#             PoseStamped,
#             self.vicon_topic,
#             self.vicon_callback,
#             qos
#         )

#         self.last_update_time = 0.0
#         self.update_interval = 0.01  # seconds

#     def vicon_callback(self, msg):
#         current_time = time.time()
#         if current_time - self.last_update_time < self.update_interval:
#             return
#         self.last_update_time = current_time

#         # === YOUR FIXED TRANSFORMATION ===
#         x_gazebo = 2.25 * msg.pose.position.y
#         y_gazebo = -2.25 * msg.pose.position.x
#         z_gazebo = msg.pose.position.z  # (optional: scale z if needed)

#         request = SetEntityState.Request()
#         request.state = EntityState()
#         request.state.name = self.model_name
#         request.state.reference_frame = 'world'
#         request.state.pose.position.x = x_gazebo
#         request.state.pose.position.y = y_gazebo
#         request.state.pose.position.z = z_gazebo
#         request.state.pose.orientation = msg.pose.orientation
#         request.state.twist = Twist()

#         self.get_logger().info(
#             f"Publishing to Gazebo: x={x_gazebo:.3f}, y={y_gazebo:.3f}, z={z_gazebo:.3f}"
#         )

#         future = self.cli.call_async(request)
#         future.add_done_callback(self.callback_response)

#     def callback_response(self, future):
#         try:
#             result = future.result()
#             if not result.success:
#                 self.get_logger().warn("⚠️ Gazebo update returned success=False.")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")

# def main():
#     rclpy.init()
#     node = ViconToGazebo()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
