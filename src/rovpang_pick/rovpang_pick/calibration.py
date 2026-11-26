
import math

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class HandEyeCalibrator:
    def __init__(self, node, target_frame):
        self.node = node
        self.target_frame = target_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            node,
            spin_thread=False
        )

    def camera_to_arm(self, point_stamped):
        try:
            if not point_stamped.header.frame_id:
                raise ValueError("Missing camera frame")

            p = point_stamped.point
            if not all(
                math.isfinite(v) for v in (p.x, p.y, p.z)
            ):
                raise ValueError("Invalid XYZ")

            stamp = Time.from_msg(
                point_stamped.header.stamp
            )

            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point_stamped.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.5)
            )

            transformed = do_transform_point(
                point_stamped,
                transform
            )

            q = transformed.point
            return True, (q.x, q.y, q.z)

        except Exception as exc:
            self.node.get_logger().error(
                f"TF transform failed: {exc}"
            )
            return False, (None, None, None)
