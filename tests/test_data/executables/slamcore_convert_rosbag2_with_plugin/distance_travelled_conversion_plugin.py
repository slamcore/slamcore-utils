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

from slamcore_utils import DatasetSubdirWriter, MeasurementType, setup_pkg_logging
from slamcore_utils.ros2 import Ros2ConverterPlugin
from std_msgs.msg import Float64

plugin_name = __file__.replace(".py", "")
logger = setup_pkg_logging(plugin_name)

class DistanceTravelledWriter(DatasetSubdirWriter):
    def __init__(self, directory):
        super().__init__(directory=directory)

        self.ofs_tracking_status = (self.directory / "data.csv").open("w", newline="")
        self.csv_tracking_status = csv.writer(self.ofs_tracking_status, delimiter=",")
        self.csv_tracking_status.writerow(["distance_travelled"])

    def write(self, msg: Float64):
        self.csv_tracking_status.writerow([msg.data])

    def teardown(self):
        self.ofs_tracking_status.close()


# define plugins for converter to use ---------------------------------------------------------
# mandatory key in module namespace
converter_plugins = [
    Ros2ConverterPlugin(
        writer_type=DistanceTravelledWriter,
        measurement_type=MeasurementType(
            name="DistanceTravelled", shortname="distance_travelled", is_camera=False
        ),
        msg_type="std_msgs/msg/Float64",
    ),
]
