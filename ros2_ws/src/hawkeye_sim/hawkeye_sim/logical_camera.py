
from typing import List, Tuple
import threading
import subprocess
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Pose, Vector3, TransformStamped


LOGICAL_CAMERA_REGEX = re.compile(r'model {\n {2}name: "(.*)"\n {2}pose {\n {4}position {\n {6}x: (.*)\n {6}y: (.*)\n {6}z: (.*)\n {4}}(?:\n {4}.*)*\n {2}}\n}')


def extract_target_poses_from_output(output: str) -> List[Tuple[str, Pose]]:
    poses = []
    for match in LOGICAL_CAMERA_REGEX.finditer(output):
        pose = Pose()
        pose.position.x = float(match.group(2))
        pose.position.y = float(match.group(3))
        pose.position.z = float(match.group(4))
        poses.append((match.group(1), pose))
    return poses


class LogicalCamera(Node):
    def __init__(self):
        super().__init__('logical_camera')

        self.br = TransformBroadcaster(self)

        self.target_vec_pub = self.create_publisher(
            Vector3,
            'target/vec',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )


def main(args=None):
    rclpy.init(args=args)
    logical_camera = LogicalCamera()

    spinner = threading.Thread(target=rclpy.spin, args=(logical_camera,), daemon=True)
    spinner.start()

    rate = logical_camera.create_rate(10)

    while rclpy.ok():
        output = subprocess.run(['ign', 'topic', '-n', '1', '-e', '-t', '/logical_camera'], stdout=subprocess.PIPE).stdout
        target_poses = extract_target_poses_from_output(output.decode())
        for (target, pose) in target_poses:
            t = TransformStamped()
            t.header.stamp = logical_camera.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = target
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            logical_camera.br.sendTransform(t)

            if target == 'iris_dummy':
                vec = Vector3()
                vec.x = pose.position.x
                vec.y = pose.position.y
                vec.z = pose.position.z
                logical_camera.target_vec_pub.publish(vec)
        rate.sleep()

    rclpy.shutdown()
    spinner.join()


if __name__ == '__main__':
    main()
