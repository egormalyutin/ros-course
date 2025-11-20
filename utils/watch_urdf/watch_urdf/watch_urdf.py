import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as Parameter_

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

import xacro


class WatchUrdf(Node):
    last_mtime = None

    def __init__(self):
        super().__init__("watch_urdf")

        self.declare_parameter("path", Parameter_.Type.STRING)
        self.path = self.get_parameter("path").get_parameter_value().string_value

        self.poller = self.create_timer(0.2, self.poll)

        self.srv = self.create_client(
            SetParameters, "/robot_state_publisher/set_parameters"
        )

        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def poll(self):
        try:
            mtime = os.path.getmtime(self.path)
        except FileNotFoundError:
            self.get_logger().warn(f"{self.path} does not exist")

        if mtime != self.last_mtime:
            if self.last_mtime != None:
                self.get_logger().info(f"{self.path} modified")

            self.update()

        self.last_mtime = mtime

    def update(self):
        try:
            doc = xacro.parse(open(self.path, "r"))
            xacro.process_doc(doc)
            self.publish(doc.toxml())
        except Exception as e:
            self.get_logger().error(str(e))

    def publish(self, content):
        req = SetParameters.Request()
        param = Parameter()
        param.name = "robot_description"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = content
        req.parameters.append(param)

        fut = self.srv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)


def main():
    rclpy.init()
    node = WatchUrdf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
