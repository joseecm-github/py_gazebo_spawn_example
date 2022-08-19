from gazebo_msgs.srv import SpawnEntity
import rclpy
import xacro
import os
from rclpy.node import Node
from ament_index_python import get_package_share_directory


class ServiceClientAsync(Node):

    def __init__(self):
        super().__init__('service_client_async')
        self.client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


    def send_request(self, request_data):
        self.future = self.client.call_async(request_data)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    package_name = "py_gazebo_spawn_example"
    urdf_file_name = "ball_model.urdf"
    urdf_file_path = os.path.join(
        get_package_share_directory(package_name), 'urdf', urdf_file_name)

    request = SpawnEntity.Request()
    request.name = "urdf_ball"
    request.xml = xacro.process_file(urdf_file_path).toxml()
    request.robot_namespace = "ball_namespace"
    request.reference_frame = "world"
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 0.0
    request.initial_pose.orientation.x = 0.0
    request.initial_pose.orientation.y = 0.0
    request.initial_pose.orientation.z = 0.0
    request.initial_pose.orientation.w = 0.0

    service_client = ServiceClientAsync()
    response = service_client.send_request(request)

    service_client.get_logger().info(
        'Result of spawing service: %s , %s' %
        (response.success, response.status_message))

    service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
