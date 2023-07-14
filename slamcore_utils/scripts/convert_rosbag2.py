#!/usr/bin/env python3

__copyright__ = """

    Slamcore Confidential
    ---------------------

    Slamcore Limited
    All Rights Reserved.
    (C) Copyright 2023

    NOTICE:

    All information contained herein is, and remains the property of Slamcore
    Limited and its suppliers, if any. The intellectual and technical concepts
    contained herein are proprietary to Slamcore Limited and its suppliers and
    may be covered by patents in process, and are protected by trade secret or
    copyright law. Dissemination of this information or reproduction of this
    material is strictly forbidden unless prior written permission is obtained
    from Slamcore Limited.
"""

__license__ = "Slamcore Confidential"


import sys

from slamcore_utils.logging import logger
from slamcore_utils.utils import inform_about_app_extras

try:
    from PIL import Image

    from slamcore_utils.ros2 import (
        Ros2ConverterPlugin,
        get_topic_names_to_message_counts,
        get_topic_names_to_types,
        init_rosbag2_reader_handle_exceptions,
    )
except ImportError:
    inform_about_app_extras(["ros2"])

try:
    import rosbag2_py
except ModuleNotFoundError:
    logger.error(
        "rosbag2_py module wasn't found. To make sure the rosbag2_py module is in your path, "
        "source your ROS2 setup.{*sh} file"
    )
    sys.exit(1)


import argparse
import csv
import json
import operator
import queue
import threading
from functools import reduce
from pathlib import Path
from runpy import run_path
from typing import Dict, Mapping, Sequence, Tuple, Type

import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image as RosImage

from slamcore_utils import fs
from slamcore_utils.dataset_subdir_writer import DatasetSubdirWriter
from slamcore_utils.logging import verbotsity_to_logging_lvl
from slamcore_utils.measurement_type import (
    MeasurementType,
    get_measurement_type,
    names_to_measurement_types,
    register_measurement_type,
)
from slamcore_utils.progress_bar import progress_bar
from slamcore_utils.ros_utils import add_parser_args
from slamcore_utils.utils import format_dict, format_list, valid_path
from gps_msgs.msg import GPSFix

"""
Convert rosbag2 to Slamcore Euroc dataset format.

Example usage:

```
convert_rosbag2 \
  -b <path to input unarchived rosbag2> \
  -o <path to output dataset directory> \
  -c <topics_to_dir_config.json> \
  -s <storage_type (e.g. mcap)>
```

The script requires a way to map the ROS topics in the bag to Slamcore dataset directories.
Currently this is specified via a JSON file, for example:

```
{
    "ir0": {
        "topic": "/stereo/left/mono/image_raw"
    },
    "ir1": {
        "topic": "/stereo/right/mono/image_raw"
    },
    "imu0": {
        "topic": "/stereo/imu"
    },
    "odometry0": {
        "topic": "/odom"
    },
    "groundtruth0": {
        "topic": "/gts"
    }
}
```
"""

imgq: "queue.Queue[Tuple[RosImage, Path]]" = queue.Queue()
mutex = threading.Lock()
count = 0


def load_converter_plugins(plugin_path: Path) -> Sequence[Ros2ConverterPlugin]:
    """Load all the ROS 2 Converter Plugins specified in the given plugin python module.

    In case of errors print the right error messages acconrdingly.
    """
    logger.debug(f"Loading ROS 2 converter plugin from {plugin_path} ...")
    try:
        ns_dict = run_path(str(plugin_path))
    except BaseException as e:
        raise RuntimeError(
            f"Failed to load ROS 2 converter plugin from {plugin_path.relative_to(plugin_path.parent)}\n\nError: {e}\n\n"
            "Exiting ..."
        ) from e

    # Sanity check, specified converter_plugins var
    if "converter_plugins" not in ns_dict:
        raise RuntimeError(
            f"No converter plugins were exported in specified plugin -> {plugin_path}\n",
            (
                'Make sure that you have initialized a variable named "converter_plugins" '
                "at the top-level of the said plugin."
            ),
        )

    converter_plugins = ns_dict["converter_plugins"]

    # Sanity check, converter_plugins var is of the right type
    if not isinstance(converter_plugins, Sequence):
        raise RuntimeError(
            f"Found the converter_plugins at the top-level of the plugin ({plugin_path}) "
            "but that variable is not of type Sequence. Its value is "
            f"{converter_plugins} and of type {type(converter_plugins)}"
        )

    return converter_plugins


def img_worker():
    while True:
        item = imgq.get()
        msg = item[0]
        filepath = item[1]
        ba = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        Image.fromarray(ba, mode="L").save(filepath, "PNG")
        imgq.task_done()

        progress_bar_.update(1)


# Writers -------------------------------------------------------------------------------------
class ImuWriter(DatasetSubdirWriter):
    def __init__(self, directory):
        super().__init__(directory=directory)

        self.ofs_acc = (self.directory / "acc.csv").open("w", newline="")
        self.ofs_gyro = (self.directory / "gyro.csv").open("w", newline="")
        self.csv_acc = csv.writer(self.ofs_acc, delimiter=",")
        self.csv_gyro = csv.writer(self.ofs_gyro, delimiter=",")
        self.csv_acc.writerow(
            ["timestamp [ns]", "acc x [m/s^2]", "acc y [m/s^2]", "acc z [m/s^2]"]
        )
        self.csv_gyro.writerow(
            ["timestamp [ns]", "gyro x [rad/s]", "gyro y [rad/s]", "gyro z [rad/s]"]
        )

    def write(self, msg):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)

        if int(msg.linear_acceleration_covariance[0]) != -1:
            a = msg.linear_acceleration  # acc
            self.csv_acc.writerow([ts, float(a.x), float(a.y), float(a.z)])

        if int(msg.angular_velocity_covariance[0]) != -1:
            w = msg.angular_velocity  # gyro
            self.csv_gyro.writerow([ts, float(w.x), float(w.y), float(w.z)])

    def teardown(self):
        self.ofs_acc.close()
        self.ofs_gyro.close()


class CameraWriter(DatasetSubdirWriter):
    def __init__(self, directory):
        super().__init__(directory=directory)
        self.data_dir = self.directory / "data"
        self.data_dir.mkdir(parents=False, exist_ok=False)

        self.ofs_camera = (self.directory / "data.csv").open("w", newline="")
        self.csv_camera = csv.writer(self.ofs_camera, delimiter=",")
        self.csv_camera.writerow(["timestamp [ns]", "filename"])

    def write(self, msg):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)

        if msg.encoding not in ("mono8", "8UC1"):
            raise RuntimeError(f"format {msg.encoding} not supported yet")
        if msg.step != msg.width:
            raise RuntimeError(
                f"Strides other than mono8 width not supported yet. {msg.step} != {msg.width}"
            )

        img_path = self.data_dir / f"{ts}.png"
        imgq.put((msg, img_path))

        self.csv_camera.writerow([ts, f"{ts}.png"])

    def teardown(self) -> None:
        self.ofs_camera.close()


class OdometryWriter(DatasetSubdirWriter):
    def __init__(self, directory):
        super().__init__(directory=directory)

        self.ofs_odometry = (self.directory / "data.csv").open("w", newline="")
        self.csv_odometry = csv.writer(self.ofs_odometry, delimiter=",")
        self.csv_odometry.writerow(
            ["timestamp [ns]", "x [m]", "y [m]", "z [m]", "x", "y", "z", "w"]
        )

    def write(self, msg):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.csv_odometry.writerow([ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def teardown(self):
        self.ofs_odometry.close()


class PoseStampedWriter(DatasetSubdirWriter):
    # constants for the potential conversion of LLA -> XYZ if given GPS data
    EARTH_RADIUS_M = 6378137.0
    FLATTENING_RATIO = 1.0 / 298.257224

    def __init__(self, directory):
        super().__init__(directory=directory)

    def register_ros_msg_type(self, msg_type: str) -> None:
        self._is_pose_with_cov = msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped"
        if msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped":
            self.write = self.write_pose_with_cov_stamped
        elif msg_type == "geometry_msgs/msg/PoseStamped":
            self.write = self.write_pose_stamped
        elif msg_type == "nav_msgs/msg/Odometry":
            self.write = self.write_odom
        elif msg_type == "gps_msgs/msg/GPSFix":
            self.write = self.write_gps_fix
        else:
            NotImplementedError(f"Cannot handle message type - {type(msg_type)}")


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

        if self._is_pose_with_cov is True:
            cols.extend(
                # fmt: off
                ["cov_00", "cov_01", "cov_02", "cov_03", "cov_04", "cov_05", "cov_10",
                 "cov_11", "cov_12", "cov_13", "cov_14", "cov_15", "cov_20", "cov_21",
                 "cov_22", "cov_23", "cov_24", "cov_25", "cov_30", "cov_31", "cov_32",
                 "cov_33", "cov_34", "cov_35", "cov_40", "cov_41", "cov_42", "cov_43",
                 "cov_44", "cov_45", "cov_50", "cov_51", "cov_52", "cov_53", "cov_54",
                 "cov_55"]
                # fmt on
            )

        self.csv_writer.writerow(cols)

    def write_pose_stamped(self, msg: PoseStamped):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)
        p = msg.pose.position
        q = msg.pose.orientation
        self.csv_writer.writerow([ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z])

    def write_pose_with_cov_stamped(self, msg: PoseWithCovarianceStamped):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        cov = msg.pose.covariance
        self.csv_writer.writerow([ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z, *cov])


    def write_odom(self, msg: Odometry):
        proxy = PoseStamped()
        proxy.header = msg.header
        proxy.pose = msg.pose.pose
        self.write_pose_stamped(proxy)

    def write(self, msg) -> None:
        pass

    def write_gps_fix(self, msg: GPSFix):
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

        self.write_pose_stamped(pose_out)

    def teardown(self):
        self.ofs_data.close()


# MeasurementType <-> Writers -----------------------------------------------------------------
measurement_type_to_writer_type: Mapping[MeasurementType, Type[DatasetSubdirWriter]] = {
    names_to_measurement_types["Cam"]: CameraWriter,
    names_to_measurement_types["Imu"]: ImuWriter,
    names_to_measurement_types["Infrared"]: CameraWriter,
    names_to_measurement_types["Odometry"]: OdometryWriter,
    names_to_measurement_types["GroundTruth"]: PoseStampedWriter,
    names_to_measurement_types["SLAMPose"]: PoseStampedWriter,
    names_to_measurement_types["SmoothPose"]: PoseStampedWriter,
}

# MeasurementType <-> Allowed Message Types

slamcore_pose_compatible_msgs = [
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseWithCovarianceStamped",
    "gps_msgs/msg/GPSFix",
    "nav_msgs/msg/Odometry",
]

measurement_to_msg_type: Mapping[MeasurementType, Sequence[str]] = {
    names_to_measurement_types["Infrared"]: ["sensor_msgs/msg/Image"],
    names_to_measurement_types["Cam"]: ["sensor_msgs/msg/Image"],
    names_to_measurement_types["Imu"]: ["sensor_msgs/msg/Imu"],
    names_to_measurement_types["Odometry"]: ["nav_msgs/msg/Odometry"],
    names_to_measurement_types["GroundTruth"]: slamcore_pose_compatible_msgs,
    names_to_measurement_types["SLAMPose"]: slamcore_pose_compatible_msgs,
    names_to_measurement_types["SmoothPose"]: ["nav_msgs/msg/Odometry"],
}


class ArgumentDefaultsHelpAndRawFormatter(
    argparse.RawDescriptionHelpFormatter, argparse.ArgumentDefaultsHelpFormatter
):
    pass


# main ----------------------------------------------------------------------------------------
def main():
    # argument parsing and sanity checks ------------------------------------------------------
    parser = argparse.ArgumentParser(
        description="Convert ROS2 Bags to the Slamcore Dataset Reader format.",
        formatter_class=ArgumentDefaultsHelpAndRawFormatter,
    )
    add_parser_args(parser)

    parser.add_argument(
        "-p",
        "--plugins",
        dest="converter_plugins",
        help="Specify converter plugins for conversions of additional topics",
        type=valid_path,
        nargs="*",
        default=[],
    )

    executable = Path(sys.argv[0]).stem
    usecases = {
        "Convert the sample rosbag2 to the Slamcore dataset format": (
            "-c tests/test_data/trimmed_rosbag2.json -b"
            " tests/tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3 -o output"
        ),
        "Convert the sample rosbag2 to the Slamcore dataset format, in verbose mode + ovewrite if exists": (
            "-c tests/test_data/trimmed_rosbag2.json -b"
            " tests/tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3 -o output -vv"
            " --overwrite"
        ),
        "Convert the sample rosbag2 to the Slamcore dataset format - use sample conversion plugin": (
            "-c tests/test_data/executables/slamcore_convert_rosbag2_with_plugin/cfg.jsonc -b"
            " tests/tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3 -o output"
            " tests/test_data/executables/slamcore_convert_rosbag2_with_plugin/distance_travelled_conversion_plugin.py"
        ),
    }
    parser.epilog = format_list(
        header="Usage examples:",
        items=[f"{k}\n    {executable} {v}\n" for k, v in usecases.items()],
    )

    parser_args = vars(parser.parse_args())

    # converter_plugins
    # aggregate and all of the converter plugins specified
    converter_plugin_paths: Sequence[Path] = parser_args["converter_plugins"]
    converter_plugins: Sequence[Ros2ConverterPlugin]
    if converter_plugin_paths:
        converter_plugins = reduce(
            operator.concat,
            (load_converter_plugins(plugin_path) for plugin_path in converter_plugin_paths),
        )
    else:
        converter_plugins = []

    # Sanity check, each one of converter_plugins var is of the right type
    for cp in converter_plugins:
        if not isinstance(cp, Ros2ConverterPlugin):
            raise RuntimeError(
                "One of the specified converter plugins is not of type "
                "Ros2ConverterPlugin, cannot proceed"
            )

    # rosbag file
    bag_path = Path(parser_args["bag"])
    if not bag_path.exists():
        raise FileNotFoundError(bag_path)
    if not bag_path.is_file():
        raise FileNotFoundError(
            f"Expected path to bag file - found path to directory instead -> {bag_path}"
        )

    # output directory
    out_path = Path(parser_args["output"])

    # overwrite
    overwrite = parser_args["overwrite"]

    # verbosity
    verbosity = parser_args["verbosity"]
    logger.setLevel(verbotsity_to_logging_lvl(verbosity))

    # jobs
    jobs = parser_args["jobs"]

    # storage
    storage = parser_args["storage"]

    if not storage:
        file_extension = bag_path.suffix

        if not file_extension:
            raise RuntimeError(
                "The provided bag file does not have an extension and the storage type has "
                "not explicitly specified in the command line. Cannot proceed."
            )
        elif file_extension == ".mcap":
            storage = "mcap"
        elif file_extension in [".sqlite3", ".db3"]:
            storage = "sqlite3"
        else:
            raise NotImplementedError(f"Unrecognised file extension {file_extension}.")

        logger.info(f"Determined storage type {storage} from file extension {file_extension}")

    # output path already exists - generate a new one
    out_path = fs.get_ready_output_path(out_path, overwrite_path=overwrite)
    out_path.mkdir()

    # config file
    config_path = Path(parser_args["config"])
    if not config_path.is_file():
        raise FileNotFoundError(config_path)
    with config_path.open() as f:
        # skip lines starting with comments `// `, e.g., in .jsonc files
        lines = [li for li in f.readlines() if not li.lstrip().startswith("// ")]
        cfg = json.loads(" ".join(lines))

    # print summary ---------------------------------------------------------------------------
    if converter_plugins:
        plugins_str = " | ".join(str(p) for p in converter_plugins)
    else:
        plugins_str = "None"

    announce_items = {
        "Input bag file": bag_path,
        "Storage": storage,
        "Output directory": out_path,
        f"Converter plugins {len(converter_plugins)}": plugins_str,
        "Overwrite output directory": overwrite,
    }
    logger.warning(
        format_dict(
            header="Configuration",
            items=announce_items,
            prefix="\n\n",
            suffix="\n",
        )
    )

    for cp in converter_plugins:
        register_measurement_type(cp.measurement_type)
        measurement_type_to_writer_type[cp.measurement_type] = cp.writer_type  # type: ignore
        measurement_to_msg_type[cp.measurement_type] = [cp.msg_type]

    rosbag_info, reader = init_rosbag2_reader_handle_exceptions(
        bag_path=bag_path, storage=storage
    )
    rosbag_topics = get_topic_names_to_types(reader)
    message_counts = get_topic_names_to_message_counts(rosbag_info)

    (out_path / "metadata.txt").write_text(str(rosbag_info))
    logger.debug(f"Rosbag metadata:\n\n{rosbag_info}")

    # helper structs and functions ------------------------------------------------------------
    def get_cfg_topics(name: str) -> Sequence[str]:
        """Get a list of all the topics for the key at hand."""
        # handle gyro/accel explicitly
        if "gyro_topic" in cfg[name].keys():
            raise NotImplementedError(
                "Split gyro/accelerometer measurements are not currently supported."
            )
        else:
            return [cfg[name]["topic"]]

    # JSON file based sanity checks -----------------------------------------------------------
    logger.info("Validating input config file and contents of rosbag...")
    key_to_measurement_types: Mapping[str, MeasurementType] = {}
    for key, val in cfg.items():
        measurement_type = get_measurement_type(key, val)
        key_to_measurement_types[key] = measurement_type
        cfg_topics = get_cfg_topics(key)

        # check that the type of the topics in the bag matches that of the JSON ---------------
        for topic_name in cfg_topics:
            rosbag_topic = rosbag_topics.get(topic_name)
            if rosbag_topic is None:
                raise RuntimeError(
                    f'Cannot find topic "{topic_name}" in the given rosbag.'
                    f"Available topics are: {list(rosbag_topics.keys())}"
                )
            rosbag_msg_type: str = rosbag_topics[topic_name]
            if rosbag_msg_type not in measurement_to_msg_type[measurement_type]:
                raise RuntimeError(
                    f"Topic type mismatch for topic {topic_name} - "
                    f'Expected: "{measurement_to_msg_type[measurement_type]}", '
                    f'Actual rosbag topic -> "{rosbag_msg_type}"'
                )

    # populate writers ------------------------------------------------------------------------
    # build a map of topic name -> writer
    # during bag playback, only consider topics in our config
    writers: Dict[str, DatasetSubdirWriter] = {}
    logger.debug("Initializing writers and registering ROS message types ...")
    topics_filter: Sequence[str] = []
    for key in cfg.keys():
        measurement_type = key_to_measurement_types[key]
        cfg_topics = get_cfg_topics(key)
        assert len(cfg_topics) == 1
        topic_name = cfg_topics[0]
        directory = out_path / key
        writers[topic_name] = measurement_type_to_writer_type[measurement_type](
            directory=directory
        )

        logger.debug(f"Mapping {measurement_type.name:15} - {topic_name} -> {key}...")
        topics_filter.append(topic_name)

        for topic_name, writer in writers.items():
            rosbag_msg_type: str = rosbag_topics[topic_name]
            writer.register_ros_msg_type(rosbag_msg_type)
            writer.prepare_write()


        # update total counter based on the number of camera topics
        if measurement_type.is_camera:
            with mutex:
                global count
                count += message_counts[topic_name]

    global progress_bar_
    progress_bar_ = progress_bar(total=count)

    storage_filter = rosbag2_py.StorageFilter(topics=topics_filter)
    reader.set_filter(storage_filter)

    # main conversion loop --------------------------------------------------------------------
    # start the PNG encoding workers
    worker_threads: Sequence[threading.Thread] = []
    for _ in range(jobs):
        t = threading.Thread(target=img_worker, daemon=True)
        t.start()
        worker_threads.append(t)
    logger.info("Consuming rosbag...")

    # consume rosbag
    while reader.has_next():
        (topic, data, _) = reader.read_next()
        msg_type = get_message(rosbag_topics[topic])
        try:
            msg = deserialize_message(data, msg_type)
        except Exception as e:
            logger.error(
                f"Could not deserialize {msg_type} message from rosbag2\n\n"
                f"Original error: {e}"
            )
            continue
        writers[topic].write(msg)
    logger.info("Consumed rosbag.")

    logger.info("Flushing pending data...")
    imgq.join()
    logger.info("Flushed pending data.")

    # teardown actions ------------------------------------------------------------------------
    for writer in writers.values():
        writer.teardown()

    logger.warning(f"Finished converting {bag_path} -> {out_path} .")


if __name__ == "__main__":
    main()
