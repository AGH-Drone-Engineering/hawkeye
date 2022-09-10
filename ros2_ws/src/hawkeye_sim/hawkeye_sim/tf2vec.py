import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from tf2_ros import TransformListener, Buffer

from geometry_msgs.msg import Vector3


class TF2Vec(Node):
    def __init__(self):
        super().__init__('tf2vec')

        self.declare_parameter('target_frame', '')
        self.target_frame = self.get_parameter('target_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vec_pub = self.create_publisher(
            Vector3,
            'vec',
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        self.timer = self.create_timer(1 / 10, self.timer_callback)

    def timer_callback(self):
        from_frame = 'base_link'
        to_frame = self.target_frame

        try:
            transform = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(f'No transform: {e}')
            return
        
        msg = Vector3()
        msg.x = transform.transform.translation.x
        msg.y = transform.transform.translation.y
        msg.z = transform.transform.translation.z
        norm = (msg.x**2 + msg.y**2 + msg.z**2)**0.5
        msg.x /= norm
        msg.y /= norm
        msg.z /= norm
        self.vec_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TF2Vec()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
