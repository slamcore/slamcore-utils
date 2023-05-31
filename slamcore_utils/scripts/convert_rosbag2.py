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
from geometry_msgs.msg import PoseStamped
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
            f"Failed to load ROS 2 converter plugin from {plugin_path}\n\nError: {e}\n\n"
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
    def __init__(self, directory):
        super().__init__(directory=directory)

        self.ofs_gt = (self.directory / "data.csv").open("w", newline="")
        self.csv_gt = csv.writer(self.ofs_gt, delimiter=",")
        self.csv_gt.writerow(
            [
                "#timestamp [ns]",
                "p_RS_R_x [m]",
                "p_RS_R_y [m]",
                "p_RS_R_z [m]",
                "q_RS_w []",
                "q_RS_x []",
                "q_RS_y []",
                "q_RS_z []",
            ]
        )

    def write(self, msg):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)
        p = msg.pose.position
        q = msg.pose.orientation
        self.csv_gt.writerow([ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z])

    def teardown(self):
        self.ofs_gt.close()


class OdometryAsPoseStampedWriter(DatasetSubdirWriter):
    def __init__(self, directory):
        self.pose_stamped_writer = PoseStampedWriter(directory=directory)

    def write(self, msg):
        proxy = PoseStamped()
        proxy.header = msg.header
        proxy.pose = msg.pose.pose
        self.pose_stamped_writer.write(proxy)

    def teardown(self):
        self.pose_stamped_writer.teardown()


# MeasurementType <-> Writers -----------------------------------------------------------------
measurement_type_to_writer_type: Mapping[MeasurementType, Type[DatasetSubdirWriter]] = {
    names_to_measurement_types["Cam"]: CameraWriter,
    names_to_measurement_types["Imu"]: ImuWriter,
    names_to_measurement_types["Infrared"]: CameraWriter,
    names_to_measurement_types["Odometry"]: OdometryWriter,
    names_to_measurement_types["GroundTruth"]: PoseStampedWriter,
    names_to_measurement_types["SLAMPose"]: PoseStampedWriter,
    names_to_measurement_types["SmoothPose"]: OdometryAsPoseStampedWriter,
}

measurement_to_msg_type: Mapping[MeasurementType, str] = {
    names_to_measurement_types["Infrared"]: "sensor_msgs/msg/Image",
    names_to_measurement_types["Cam"]: "sensor_msgs/msg/Image",
    names_to_measurement_types["Imu"]: "sensor_msgs/msg/Imu",
    names_to_measurement_types["Odometry"]: "nav_msgs/msg/Odometry",
    names_to_measurement_types["GroundTruth"]: "geometry_msgs/msg/PoseStamped",
    names_to_measurement_types["SLAMPose"]: "geometry_msgs/msg/PoseStamped",
    names_to_measurement_types["SmoothPose"]: "nav_msgs/msg/Odometry",
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
        measurement_to_msg_type[cp.measurement_type] = cp.msg_type

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
        for a_topic in cfg_topics:
            rosbag_topic = rosbag_topics.get(a_topic)
            if rosbag_topic is None:
                raise RuntimeError(
                    f'Cannot find topic "{a_topic}" in the given rosbag.'
                    f"Available topics are: {list(rosbag_topics.keys())}"
                )
            rosbag_msg_type: str = rosbag_topics[a_topic]
            if measurement_to_msg_type[measurement_type] != rosbag_msg_type:
                raise RuntimeError(
                    f"Topic type mismatch for topic {a_topic} - "
                    f'Expected: "{measurement_to_msg_type[measurement_type]}", '
                    f'Actual rosbag topic -> "{rosbag_msg_type}"'
                )

    # populate writers ------------------------------------------------------------------------
    # build a map of topic name -> writer
    # during bag playback, only consider topics in our config
    writers: Dict[str, DatasetSubdirWriter] = {}
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
        msg = deserialize_message(data, msg_type)
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
