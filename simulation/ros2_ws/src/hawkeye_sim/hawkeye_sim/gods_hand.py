from typing import Tuple
import os
from time import time as get_time
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


def ign_set_pose(model: str, pos: Tuple[float, float, float]):
    x, y, z = pos
    cmd = 'ign service -s /world/iris_arducopter_runway/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 0 --req \'{}\' > /dev/null'
    req = f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}'
    os.system(cmd.format(req))


class GodsHand(Node):
    def __init__(self):
        super().__init__('gods_hand')

        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 1)

    def tick(self):
        phi = 2 * math.pi * get_time() / 200
        r = 50
        x = r * math.cos(phi)
        y = r * math.sin(phi)
        z = 15.0
        ign_set_pose('iris_dummy', (x, y, z))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gods_hand = GodsHand()

    while rclpy.ok():
        rclpy.spin_once(gods_hand, timeout_sec=0)
        gods_hand.tick()
    
    gods_hand.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
