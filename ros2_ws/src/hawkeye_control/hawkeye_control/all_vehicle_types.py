import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer

from mavros_msgs.srv import CommandBool, CommandTOL, CommandLong, SetMode
from geometry_msgs.msg import PoseStamped


STATE_INIT = 'init'
STATE_GUIDED_SENT = 'guided_sent'
STATE_GUIDED_OK = 'guided_set'
STATE_ARM_SENT = 'arm_sent'
STATE_ARM_OK = 'arm_set'
STATE_TAKEOFF_SENT = 'takeoff_sent'
STATE_TAKEOFF_OK = 'takeoff_set'


class MavrosUniversalVehicleDriver(Node):

    def __init__(self):
        super().__init__('mavros_universal_driver')

        self.declare_parameter('drone_frame', 'base_link')
        self.drone_frame: str = self.get_parameter('drone_frame').value

        self.declare_parameter('target_frame', 'target')
        self.target_frame: str = self.get_parameter('target_frame').value

        self.declare_parameter('takeoff_altitude', 2.0)
        self.takeoff_alt: float = self.get_parameter('takeoff_altitude').value

        self.declare_parameter('min_altitude', 1)
        self.min_alt: float = self.get_parameter('min_altitude').value

        self.state = STATE_INIT

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

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1 / 10, self.timer_callback)

    def timer_callback(self):
        try:
            target_enu = self.tf_buffer.lookup_transform(
                'map',
                self.target_frame,
                rclpy.time.Time(),
            )
            drone_enu = self.tf_buffer.lookup_transform(
                'map',
                self.drone_frame,
                rclpy.time.Time(),
            )
        except Exception as e:
            self.warn(f'Could not get transform: {e}')
            return

        if drone_enu.transform.translation.z > self.min_alt:
            self.state = STATE_TAKEOFF_OK

        if self.state == STATE_INIT:
            self.send_guided()
            self.state = STATE_GUIDED_SENT
            return

        if self.state == STATE_GUIDED_OK:
            self.send_arm()
            self.state = STATE_ARM_SENT
            return

        if self.state == STATE_ARM_OK:
            self.send_takeoff(self.takeoff_alt)
            self.state = STATE_TAKEOFF_SENT
            return

        if self.state != STATE_TAKEOFF_OK:
            return

        if drone_enu.transform.translation.z < self.min_alt:
            self.info(f'Waiting for takeoff: {drone_enu.transform.translation.z:.2f} < {self.min_alt:.2f}')
            return

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = target_enu.transform.translation.x
        msg.pose.position.y = target_enu.transform.translation.y
        msg.pose.position.z = target_enu.transform.translation.z
        self.setpoint_pub.publish(msg)

    def info(self, msg):
        self.get_logger().info(str(msg))

    def error(self, msg):
        self.get_logger().error(str(msg))

    def warn(self, msg):
        self.get_logger().warn(str(msg))

    def send_guided(self):
        msg = SetMode.Request()
        msg.base_mode = 0
        msg.custom_mode = "GUIDED"
        self.info("Request guided mode")
        fut = self.setmode_client.call_async(msg)
        def on_done(f):
            if f.result().mode_sent:
                self.state = STATE_GUIDED_OK
                self.info("Guided mode set")
            else:
                self.state = STATE_INIT
                self.error("Could not set guided mode")
        fut.add_done_callback(on_done)

    def send_arm(self):
        msg = CommandBool.Request()
        msg.value = True
        self.info("Arming vehicle")
        fut = self.arm_client.call_async(msg)
        def on_done(f):
            if f.result().success:
                self.state = STATE_ARM_OK
                self.info("Vehicle armed")
            else:
                self.state = STATE_GUIDED_OK
                self.error("Could not arm vehicle")
        fut.add_done_callback(on_done)

    def send_takeoff(self, alt: float):
        msg = CommandTOL.Request()
        msg.altitude = float(alt)
        self.info("Request takeoff")
        fut = self.takeoff_client.call_async(msg)
        def on_done(f):
            if f.result().success:
                self.state = STATE_TAKEOFF_OK
                self.info("Takeoff complete")
            else:
                self.state = STATE_ARM_OK
                self.error("Could not takeoff")
        fut.add_done_callback(on_done)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosUniversalVehicleDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
