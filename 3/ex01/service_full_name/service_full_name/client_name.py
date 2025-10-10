import sys
import rclpy
from rclpy.node import Node


from name_op_interfaces.srv import FullNameSum


class ClientName(Node):
    def __init__(self):
        super().__init__("client")

        self.client = self.create_client(FullNameSum, "full_name_sum")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("waiting for service")

    def send_request(self, first_name, name, last_name):
        req = FullNameSum.Request()
        req.first_name = first_name
        req.name = name
        req.last_name = last_name
        return self.client.call_async(req)


def main():
    rclpy.init()
    node = ClientName()

    assert len(sys.argv) == 4
    _, first_name, name, last_name = sys.argv

    fut = node.send_request(first_name, name, last_name)
    rclpy.spin_until_future_complete(node, fut)

    resp = fut.result()

    node.get_logger().info(f"Result: {resp.full_name}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
