from typing import List, Tuple
import subprocess
import re

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point, TransformStamped


LOGICAL_CAMERA_REGEX = re.compile(r'model {\n {2}name: "(.*)"\n {2}pose {\n {4}position {\n {6}x: (.*)\n {6}y: (.*)\n {6}z: (.*)\n {4}}(?:\n {4}.*)*\n {2}}\n}')


def extract_target_poses_from_output(output: str) -> List[Tuple[str, Point]]:
    poses = []
    for match in LOGICAL_CAMERA_REGEX.finditer(output):
        pose = Point()
        pose.x = float(match.group(2))
        pose.y = float(match.group(3))
        pose.z = float(match.group(4))
        poses.append((match.group(1), pose))
    return poses


class LogicalCamera(Node):
    def __init__(self):
        super().__init__('logical_camera')

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(1 / 10, self.timer_callback)

    def timer_callback(self):
        output = subprocess.run(['ign', 'topic', '-n', '1', '-e', '-t', '/logical_camera'], stdout=subprocess.PIPE).stdout
        target_poses = extract_target_poses_from_output(output.decode())
        for target, pose in target_poses:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = target
            t.transform.translation.x = pose.x
            t.transform.translation.y = pose.y
            t.transform.translation.z = pose.z
            self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    logical_camera = LogicalCamera()
    rclpy.spin(logical_camera)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
