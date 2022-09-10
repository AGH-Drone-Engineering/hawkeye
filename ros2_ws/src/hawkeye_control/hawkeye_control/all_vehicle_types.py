from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from tf2_ros import TransformListener, Buffer

from mavros_msgs.srv import CommandBool, CommandTOL, CommandLong, SetMode
from geometry_msgs.msg import TwistStamped, Vector3


class MavrosUniversalVehicleDriver(Node):

    def __init__(self):
        super().__init__('mavros_universal_driver')

        self.guided_set = False
        self.armed = False
        self.takeoff_sent = False

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Arming service not available, waiting again...')

        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Takeoff service not available, waiting again...')

        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Other service not available, waiting again...')

        self.setmode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Other service not available, waiting again...')

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10,
        )

        self.target_vec_sub = self.create_subscription(
            Vector3,
            'target/vec',
            self.target_vec_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def target_vec_callback(self, vec_body: Vector3):
        if not self.guided_set:
            self.set_guided()
            return

        if not self.armed:
            self.arm()
            return

        if not self.takeoff_sent:
            self.takeoff(2)
            return

        from_frame = 'map'
        to_frame = 'base_link'
        try:
            transform = self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                rclpy.time.Time(),
            )
        except Exception as e:
            self.warn(f'Could not get transform: {e}')
            return

        if transform.transform.translation.z < 1:
            self.info(f"Waiting for takeoff: {transform.transform.translation.z}")
            return

        rot = Rotation.from_quat((
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ))
        vec_map = rot.apply((vec_body.x, vec_body.y, vec_body.z))

        vec_ned = (
            vec_map[1],
            vec_map[0],
            -vec_map[2],
        )

        msg = TwistStamped()
        msg.twist.linear.x = vec_ned[0] * 0.2
        msg.twist.linear.y = vec_ned[1] * 0.2
        msg.twist.linear.z = vec_ned[2] * 0.2
        self.velocity_pub.publish(msg)

    def info(self, msg):
        self.get_logger().info(str(msg))

    def error(self, msg):
        self.get_logger().error(str(msg))

    def warn(self, msg):
        self.get_logger().warn(str(msg))

    def set_guided(self):
        msg = SetMode.Request()
        msg.base_mode = 0
        msg.custom_mode = "GUIDED"
        self.info("Request guided mode")
        fut = self.setmode_client.call_async(msg)
        def on_done(f):
            self.guided_set = True
        fut.add_done_callback(on_done)

    def arm(self):
        msg = CommandBool.Request()
        msg.value = True
        self.info("Arming vehicle")
        fut = self.arm_client.call_async(msg)
        def on_done(f):
            self.armed = True
        fut.add_done_callback(on_done)

    def takeoff(self, alt: float):
        msg = CommandTOL.Request()
        msg.altitude = float(alt)
        self.info("Request takeoff")
        fut = self.takeoff_client.call_async(msg)
        def on_done(f):
            self.takeoff_sent = True
        fut.add_done_callback(on_done)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosUniversalVehicleDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
