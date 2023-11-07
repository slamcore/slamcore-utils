__copyright__ = """

    SLAMcore Confidential
    ---------------------

    SLAMcore Limited
    All Rights Reserved.
    (C) Copyright 2023

    NOTICE:

    All information contained herein is, and remains the property of SLAMcore
    Limited and its suppliers, if any. The intellectual and technical concepts
    contained herein are proprietary to SLAMcore Limited and its suppliers and
    may be covered by patents in process, and are protected by trade secret or
    copyright law. Dissemination of this information or reproduction of this
    material is strictly forbidden unless prior written permission is obtained
    from SLAMcore Limited.
"""

__license__ = "SLAMcore Confidential"


import csv
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped

from slamcore_utils import DatasetSubdirWriter, MeasurementType
from slamcore_utils.ros2 import Ros2ConverterPlugin, Ros2PluginInitializationFailureError

plugin_name = Path(__file__).name

try:
    from gps_msgs.msg import GPSFix
except ModuleNotFoundError:
    raise Ros2PluginInitializationFailureError(
        plugin_name=plugin_name,
        msg=(
            "Could not import the gps_msgs module. "
            "On Ubuntu this is shipped with the ros-<ros-version>-gps-msgs package."
            "Please make sure that the latter is installed (or that the gps-msgs are installed"
            " via source) and retry"
        ),
    )


class GPSAsPoseStampedWriter(DatasetSubdirWriter):
    """Convert and save GPSFix messages in the Slamcore dataset format."""

    # constants for the potential conversion of LLA -> XYZ if given GPS data
    EARTH_RADIUS_M = 6378137.0
    FLATTENING_RATIO = 1.0 / 298.257224

    def prepare_write(self) -> None:
        self.ofs_data = (self.directory / "data.csv").open("w", newline="")
        self.csv_writer = csv.writer(self.ofs_data, delimiter=",")

        cols = [
            "#timestamp [ns]",
            "p_RS_R_x [m]",
            "p_RS_R_y [m]",
            "p_RS_R_z [m]",
            "q_RS_w []",
            "q_RS_x []",
            "q_RS_y []",
            "q_RS_z []",
        ]

        self.csv_writer.writerow(cols)

    def _write_pose_stamped(self, msg: PoseStamped):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)
        p = msg.pose.position
        q = msg.pose.orientation
        self.csv_writer.writerow([ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z])

    def write(self, msg: GPSFix) -> None:
        """
        Convert LLA GPS coordinates XYZ coordinates

        Consult the following two resources for the conversion equations:

        https://web.archive.org/web/20181018072749/http://mathforum.org/library/drmath/view/51832.html
        https://stackoverflow.com/a/8982005/2843583
        """

        lat_rad = np.deg2rad(msg.latitude)
        lon_rad = np.deg2rad(msg.longitude)
        alt_m = msg.altitude

        cos_lat = np.cos(lat_rad)
        sin_lat = np.sin(lat_rad)

        cos_lon = np.cos(lon_rad)
        sin_lon = np.sin(lon_rad)

        C = 1.0 / np.sqrt(
            cos_lat * cos_lat
            + (1 - self.FLATTENING_RATIO) * (1 - self.FLATTENING_RATIO) * sin_lat * sin_lat
        )
        S = (1.0 - self.FLATTENING_RATIO) * (1.0 - self.FLATTENING_RATIO) * C

        pose_out: PoseStamped = PoseStamped()
        pose_out.header = msg.header
        pose_out.pose.position.x = (self.EARTH_RADIUS_M * C + alt_m) * cos_lat * cos_lon
        pose_out.pose.position.y = (self.EARTH_RADIUS_M * C + alt_m) * cos_lat * sin_lon
        pose_out.pose.position.z = (self.EARTH_RADIUS_M * S + alt_m) * sin_lat

        # # Cache first pose received and always substract from it so that we start from (0,0,0)
        # if self._first_pose is None:
        #     x = pose_out.pose.position.x
        #     y = pose_out.pose.position.y
        #     z = pose_out.pose.position.z
        #     logger.warning(
        #         f"Setting the first GPS received pose from ({x}, {y}, {z}) to to (0,0,0) ..."
        #     )

        #     self._first_pose = Pose()
        #     self._first_pose.position.x = pose_out.pose.position.x
        #     self._first_pose.position.y = pose_out.pose.position.y
        #     self._first_pose.position.z = pose_out.pose.position.z

        # pose_out.pose.position.x -= self._first_pose.position.x
        # pose_out.pose.position.y -= self._first_pose.position.y
        # pose_out.pose.position.z -= self._first_pose.position.z

        self._write_pose_stamped(pose_out)

    def teardown(self):
        self.ofs_data.close()


# define plugins for converter to use ---------------------------------------------------------
# mandatory key in module namespace
converter_plugins = [
    Ros2ConverterPlugin(
        writer_type=GPSAsPoseStampedWriter,
        measurement_type=MeasurementType(
            name="GNSSGroundTruth", shortname="gnss_groundtruth", is_camera=False
        ),
        msg_type="gps_msgs/msg/GPSFix",
    ),
]
