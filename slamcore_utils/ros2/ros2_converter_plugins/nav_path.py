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

from geometry_msgs.msg import PoseStamped

from slamcore_utils.dataset_subdir_writer import DatasetSubdirWriter
from slamcore_utils.measurement_type import MeasurementType
from slamcore_utils.ros2 import Ros2ConverterPlugin, Ros2PluginInitializationFailureError

plugin_name = Path(__file__).name
try:
    from nav_msgs.msg import Path as NavPath
except ModuleNotFoundError:
    raise Ros2PluginInitializationFailureError(
        plugin_name=plugin_name,
        msg=(
            "Could not import the nav_msgs module. On Ubuntu this is shipped with the "
            "ros-<ros-version>-nav-msgs package. "
            "Please make sure that the latter is installed (or that the nav-msgs are installed"
            " via source) and retry"
        ),
    )


class NavPathWriter(DatasetSubdirWriter):
    """Convert a single nav_msgs/Path msg to a Slamcore-compatible CSV file of poses."""

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

    def write(self, msg: NavPath) -> None:
        """
        Write all the poses of a single nav_msgs/Path instance to CSV.

        In cae of more messages arriving after the first call, we skip them and instead print a
        warning.
        """

        for single_pose in msg.poses:
            self._write_pose_stamped(single_pose)

        # don't attempt to convert another message
        self.write = self._write_nop

    def _write_nop(self, msg) -> None:
        self.logger.warning(
            f"{self.__class__.__name__} expected only a single message to convert. "
            f"Instead it just received additional message(s) - msg: {msg}"
        )

    def teardown(self):
        self.ofs_data.close()


# define plugins for converter to use ---------------------------------------------------------
# mandatory key in module namespace
converter_plugins = [
    Ros2ConverterPlugin(
        writer_type=NavPathWriter,
        measurement_type=MeasurementType(
            name="NavPath", shortname="nav_path", is_camera=False
        ),
        msg_type="nav_msgs/msg/Path",
    ),
]
