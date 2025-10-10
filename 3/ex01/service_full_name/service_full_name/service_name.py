import rclpy
from rclpy.node import Node


from name_op_interfaces.srv import FullNameSum


class ServiceName(Node):
    def __init__(self):
        super().__init__("service_name")
        self.srv = self.create_service(FullNameSum, "full_name_sum", self.cb)

    def cb(self, req, resp):
        resp.full_name = f"{req.last_name} {req.name} {req.first_name}"
        return resp


def main():
    rclpy.init()
    node = ServiceName()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
