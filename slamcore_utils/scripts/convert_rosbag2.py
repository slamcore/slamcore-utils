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
from dataclasses import dataclass, field
from types import MethodType

from PIL import Image

from slamcore_utils.arg_parser import ArgumentDefaultsHelpAndRawFormatter
from slamcore_utils.logging import logger as pkg_logger
from slamcore_utils.math import is_symmetric
from slamcore_utils.utils import inform_about_app_extras

try:
    from slamcore_utils.ros2 import (
        ConversionFormat,
        get_internal_plugins_dir,
        get_topic_names_to_message_counts,
        get_topic_names_to_types,
        init_rosbag2_reader_handle_exceptions,
        load_converter_plugins_from_multiple_files,
    )
except ImportError:
    inform_about_app_extras(["ros2"])

try:
    import rosbag2_py
except ModuleNotFoundError:
    pkg_logger.error(
        "rosbag2_py module wasn't found. To make sure the rosbag2_py module is in your path, "
        "source your ROS2 setup.{*sh} file"
    )
    sys.exit(1)


import argparse
import csv
import json
import os
import queue
import threading
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, Tuple, Type, Union

import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage as RosCompressedImage
from sensor_msgs.msg import Image as RosImage

from slamcore_utils import fs
from slamcore_utils.dataset_subdir_writer import DatasetSubdirWriter
from slamcore_utils.logging import verbosity_to_logging_lvl
from slamcore_utils.measurement_type import (
    MeasurementType,
    get_measurement_type,
    names_to_measurement_types,
    register_measurement_type,
)
from slamcore_utils.progress_bar import DummyProgressBar, progress_bar
from slamcore_utils.ros_utils import (
    DEFAULT_NUM_JOBS,
    DEFAULT_OVERWRITE_OUTPUT_OPT,
    add_parser_args,
)
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

JsonConts = Mapping[str, Any]
# autodetect storage
DEFAULT_STORAGE_TYPE = ""

# limit the queue size for reading/writing images ---------------------------------------------
# deserialization + putting to the queue is much faster than consuming elements from the Queue
# which would lead to unbounded mem usage.
imgq: "queue.Queue[Tuple[Union[RosImage,RosCompressedImage], Path, Callable]]" = queue.Queue(
    maxsize=4_000
)
mutex = threading.Lock()


# Writers -------------------------------------------------------------------------------------
class ImuWriter(DatasetSubdirWriter):
    def __init__(self, directory, *args, **kargs):
        super().__init__(directory=directory, *args, **kargs)

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
    def __init__(self, directory, *, imgq_put_timeout_s: int, **kargs):
        super().__init__(directory=directory, **kargs)
        self.data_dir = self.directory / "data"
        self.data_dir.mkdir(parents=False, exist_ok=False)

        self.ofs_camera = (self.directory / "data.csv").open("w", newline="")
        self.csv_camera = csv.writer(self.ofs_camera, delimiter=",")
        self.csv_camera.writerow(["timestamp [ns]", "filename"])
        self._imgq_put_timeout_s = imgq_put_timeout_s

    def register_ros_msg_type(self, msg_type: str) -> None:
        if msg_type == "sensor_msgs/msg/Image":
            self.write = self.write_img
        elif msg_type == "sensor_msgs/msg/CompressedImage":
            self.write = self.write_compressed_img
        else:
            NotImplementedError(f"Cannot handle message type - {type(msg_type)}")

    @staticmethod
    def _save_raw_img(msg: RosImage, filepath: Path):
        ba = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        Image.fromarray(ba, mode="L").save(filepath, "PNG")

    def write_img(self, msg: RosImage):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)

        if msg.encoding not in ("mono8", "8UC1"):
            raise RuntimeError(f"format {msg.encoding} not supported yet")
        if msg.step != msg.width:
            raise RuntimeError(
                f"Strides other than mono8 width not supported yet. {msg.step} != {msg.width}"
            )

        img_path = self.data_dir / f"{ts}.png"

        try:
            self.logger.debug(
                f"Adding a new image to the image processing queue, queue size: {imgq.qsize()}"
            )
            imgq.put(
                (msg, img_path, self._save_raw_img),
                block=True,
                timeout=self._imgq_put_timeout_s,
            )
        except queue.Full as e:
            raise RuntimeError(
                f"Reached timeout of {self._imgq_put_timeout_s}s trying to write an image to the "
                f"queue. Couldn't write image {img_path.name}. This indicates a bug."
            ) from e

        self.csv_camera.writerow([ts, f"{ts}.png"])

    @staticmethod
    def _save_compressed_img(msg: RosCompressedImage, filepath: Path):
        arr = np.frombuffer(msg.data, np.uint8)
        bytes_ = arr.tobytes()
        filepath.write_bytes(bytes_)

    def write_compressed_img(self, msg: RosCompressedImage):
        ts = int(msg.header.stamp.sec * 1e9) + int(msg.header.stamp.nanosec)

        # image_transport/image_republisher populates msg.format with a string of the form:
        #   "mono8; png compressed "
        format_list_ = msg.format.strip().split(";")
        if "jpeg" in format_list_ or "jpeg compressed" in format_list_:
            actual_format = "jpeg"
        elif "png" in format_list_ or "png compressed":
            actual_format = "png"
        else:
            raise RuntimeError(f"Format [{msg.format}] not supported yet")

        img_fname = f"{ts}.{actual_format}"
        img_path = self.data_dir / img_fname

        try:
            # self.logger.debug(
            #     f"Adding a new image to the image processing queue, queue size: {imgq.qsize()}"
            # )
            imgq.put(
                (msg, img_path, self._save_compressed_img),
                block=True,
                timeout=self._imgq_put_timeout_s,
            )
        except queue.Full as e:
            raise RuntimeError(
                f"Reached timeout of {self._imgq_put_timeout_s}s trying to write an image to "
                f"the queue. Couldn't write image {img_path.name}. This indicates a bug."
            ) from e

        self.csv_camera.writerow([ts, img_fname])

    def write(self, msg) -> None:
        pass

    def teardown(self) -> None:
        self.ofs_camera.close()


class OdometryWriter(DatasetSubdirWriter):
    def __init__(self, directory, *args, **kargs):
        super().__init__(directory=directory, *args, **kargs)

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
    def __init__(self, directory, *args, **kargs):
        super().__init__(directory=directory, *args, **kargs)
        self._cov_matrix_warnings = 0

    def register_ros_msg_type(self, msg_type: str) -> None:
        self._is_pose_with_cov = msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped"
        if msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped":
            self.write = self.write_pose_with_cov_stamped
        elif msg_type == "geometry_msgs/msg/PoseStamped":
            self.write = self.write_pose_stamped
        elif msg_type == "nav_msgs/msg/Odometry":
            self.write = self.write_odom
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
                ["cov_00", "cov_01", "cov_02", "cov_03", "cov_04", "cov_05",
                 "cov_10", "cov_11", "cov_12", "cov_13", "cov_14", "cov_15",
                 "cov_20", "cov_21", "cov_22", "cov_23", "cov_24", "cov_25",
                 "cov_30", "cov_31", "cov_32", "cov_33", "cov_34", "cov_35",
                 "cov_40", "cov_41", "cov_42", "cov_43", "cov_44", "cov_45",
                 "cov_50", "cov_51", "cov_52", "cov_53", "cov_54", "cov_55"]
                # fmt: on
            )

        self.csv_writer.writerow(cols)

    def _log_cov_matrix_warning(self, msg):
        """Wrapper function to only log this warning X times to avoid extra noise in stdout."""
        if self._cov_matrix_warnings >= 3:
            self.logger.error(
                f"Encountered more than {self._cov_matrix_warnings} bad covariance matrices."
                " Will stop printing warnings now."
            )

            def increment_warnings_cnt(self, msg) -> None:
                self._cov_matrix_warnings += 1

            self._log_cov_matrix_warning = MethodType(increment_warnings_cnt, self)
        else:
            self.logger.warning(msg)
        self._cov_matrix_warnings += 1

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

        cov2d = cov.reshape((6, 6))

        # checks on the covariance
        if cov2d[-1, -1] == -1:
            self._log_cov_matrix_warning(
                f"Covariance of current message is invalid (== -1)\nMessage:\n\n{msg}"
            )
        if not is_symmetric(cov2d):
            self._log_cov_matrix_warning(
                f"Covariance of current message is not symmetric\nMessage:\n\n{msg}"
            )

        self.csv_writer.writerow([ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z, *cov])

    def write_odom(self, msg: Odometry):
        proxy = PoseStamped()
        proxy.header = msg.header
        proxy.pose = msg.pose.pose
        self.write_pose_stamped(proxy)

    def write(self, msg) -> None:
        pass

    def teardown(self):
        if self._cov_matrix_warnings:
            self.logger.error(
                f"Encountered more than {self._cov_matrix_warnings} bad covariance matrices "
                f"during conversion for output directory {self.directory}. "
                "Please have a look at the input data to find out more."
            )

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
    "nav_msgs/msg/Odometry",
]


slamcore_cam_compatible_msgs = ["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"]

measurement_to_msg_type: Mapping[MeasurementType, Sequence[str]] = {
    names_to_measurement_types["Infrared"]: slamcore_cam_compatible_msgs,
    names_to_measurement_types["Cam"]: slamcore_cam_compatible_msgs,
    names_to_measurement_types["Imu"]: ["sensor_msgs/msg/Imu"],
    names_to_measurement_types["Odometry"]: ["nav_msgs/msg/Odometry"],
    names_to_measurement_types["GroundTruth"]: slamcore_pose_compatible_msgs,
    names_to_measurement_types["SLAMPose"]: slamcore_pose_compatible_msgs,
    names_to_measurement_types["SmoothPose"]: ["nav_msgs/msg/Odometry"],
}


def img_worker(logger):
    while True:
        item = imgq.get()
        # logger.debug(
        #     f"Got  a new image from the image processing queue, queue size: {imgq.qsize()}"
        # )
        msg = item[0]
        filepath = item[1]
        save_fn = item[2]
        save_fn(msg, filepath)
        progress_bar_.update(1)  # type: ignore
        imgq.task_done()


# conversion struct ---------------------------------------------------------------------------
@dataclass
class ConversionProps:
    topic_name: str
    writer: DatasetSubdirWriter
    measurement_type: MeasurementType
    sc_directory: Path


# map of topic name -> conversion properties
ConversionMap = Mapping[str, ConversionProps]


# ConversionPropsAssembler class --------------------------------------------------------------
class ConversionPropsAssembler:
    def __init__(
        self,
        sc_dataset_path: Path,
        rosbag_topics: Mapping[str, str],
        imgq_put_timeout_s: int,
        logger=pkg_logger,
    ):
        self._sc_dataset_path = sc_dataset_path
        self._rosbag_topics = rosbag_topics
        self._logger = logger
        self._imgq_put_timeout_s = imgq_put_timeout_s

    def assemble_from_rigid_format(self, json_conts: JsonConts) -> ConversionMap:
        conversion_map: ConversionMap = {}

        for key, val in json_conts.items():
            measurement_type = get_measurement_type(key, val)
            topic_name = json_conts[key]["topic"]

            # sanity checks -------------------------------------------------------------------
            rosbag_topic = self._rosbag_topics.get(topic_name)
            if rosbag_topic is None:
                raise RuntimeError(
                    f'Cannot find topic "{topic_name}" in the given rosbag.'
                    f" Available topics are: {list(self._rosbag_topics.keys())}"
                )

            self._check_msg_type(topic_name=topic_name, measurement_type=measurement_type)

            # populate properties -------------------------------------------------------------
            conversion_map[topic_name] = self._init_conversion_props(
                key=key, measurement_type=measurement_type, topic_name=topic_name
            )

        return conversion_map

    def assemble_from_flexible_format(self, json_conts: JsonConts) -> ConversionMap:
        conversion_map: ConversionMap = {}

        # For each one of the keys:
        # * If this key is mandatory, iterate over its topics and make sure that at least one
        #   is in the rosbag
        # * If the key is not mandatory iterate over its topics and if none exists in the
        #   rosbag, issue a warning
        for key, val in json_conts.items():
            measurement_type = get_measurement_type(key, val)
            is_required = val.get("required", True)

            # determine the topic name --------------------------------------------------------
            topic_name = val.get("topic", None)
            if topic_name is None:  # no "topic", look for "topics"
                found_topics = [
                    topic for topic in val["topics"] if topic in self._rosbag_topics
                ]
                if not found_topics:
                    msg = (
                        f"None of the potential topics were found in the given rosbag. "
                        f'Topic names I\'m looking for: {val["topics"]}. '
                        f"Topics in given rosbag: {self._rosbag_topics}"
                    )
                    if is_required:
                        raise RuntimeError(msg)
                    else:
                        self._logger.warning(msg)
                        continue

                topic_name = found_topics[0]
                if len(found_topics) != 1:
                    self._logger.warning(
                        f"Multiple candidate topic names match for {key} -> {found_topics}. "
                        f"Choosing {topic_name} ."
                    )
            else:
                is_in_rosbag = topic_name in self._rosbag_topics
                if not is_in_rosbag:
                    msg = (
                        f"Topic was not found in the given rosbag. "
                        f"Topic I'm looking for: {topic_name}. "
                        f"Topics in given rosbag: {self._rosbag_topics}"
                    )
                    if is_required:
                        raise RuntimeError(msg)
                    else:
                        self._logger.warning(msg)
                        continue

            self._check_msg_type(topic_name=topic_name, measurement_type=measurement_type)

            # populate properties -------------------------------------------------------------
            conversion_map[topic_name] = self._init_conversion_props(
                key=key, measurement_type=measurement_type, topic_name=topic_name
            )
        return conversion_map

    def _init_conversion_props(
        self, key, measurement_type: MeasurementType, topic_name: str
    ) -> ConversionProps:
        self._logger.info(f"Mapping {measurement_type.name:15} - {topic_name} -> {key}...")
        sc_dataset_subdir = self._sc_dataset_path / key

        extras = {}
        if measurement_type.is_camera:
            extras["imgq_put_timeout_s"] = self._imgq_put_timeout_s

        writer = measurement_type_to_writer_type[measurement_type](
            directory=sc_dataset_subdir, logger=self._logger, **extras
        )
        rosbag_msg_type: str = self._rosbag_topics[topic_name]
        writer.register_ros_msg_type(rosbag_msg_type)
        writer.prepare_write()

        # add to conversion map -----------------------------------------------------------
        return ConversionProps(
            topic_name=topic_name,
            writer=writer,
            sc_directory=sc_dataset_subdir,
            measurement_type=measurement_type,
        )

    def _check_msg_type(self, topic_name: str, measurement_type: MeasurementType):
        """Make sure that the topic found contains messages of the right type.

        Raise RuntimeError otherwise.
        """
        rosbag_msg_type: str = self._rosbag_topics[topic_name]
        if rosbag_msg_type not in measurement_to_msg_type[measurement_type]:
            raise RuntimeError(
                f"Topic type mismatch for topic {topic_name} - "
                f'Expected: "{measurement_to_msg_type[measurement_type]}", '
                f'Actual rosbag topic -> "{rosbag_msg_type}"'
            )


# argument parsing ----------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    """CLI argument parsing and sanity checks."""
    parser = argparse.ArgumentParser(
        description="Convert ROS2 Bags to the Slamcore Dataset Reader format.",
        formatter_class=ArgumentDefaultsHelpAndRawFormatter,
    )
    add_parser_args(parser)

    parser.add_argument(
        "-s",
        "--storage",
        dest="storage_type",
        default=DEFAULT_STORAGE_TYPE,
        required=False,
        help="Type of rosbag storage (sqlite3, mcap). Omit to deduce based on file extension.",
    )

    parser.add_argument(
        "--disable-progress-bar",
        dest="disable_progress_bar",
        help="Disable the progress bar altogether",
        action="store_true",
    )

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

    return parser.parse_args()


@dataclass(frozen=True)
class Config:
    """convert_rosbag2 Configuration Class."""

    bag_path: Path
    config_path: Path
    output_dir: Path
    converter_plugin_paths: Sequence[Path] = field(default_factory=list)
    jobs: int = DEFAULT_NUM_JOBS
    overwrite_output: bool = DEFAULT_OVERWRITE_OUTPUT_OPT
    storage_type: str = DEFAULT_STORAGE_TYPE
    use_progress_bar: bool = True
    verbosity_lvl: int = 0
    imgq_put_timeout_s: int = 20


def get_cli_assembled_config() -> Config:
    """Assemble the app Config instance by parsing CLI arguments."""
    cli_args = vars(parse_args())

    cfg = Config(
        bag_path=cli_args["bag"],
        config_path=cli_args["config"],
        converter_plugin_paths=cli_args["converter_plugins"],
        jobs=cli_args["jobs"],
        output_dir=cli_args["output"],
        overwrite_output=cli_args["overwrite"],
        storage_type=cli_args["storage_type"],
        use_progress_bar=not cli_args["disable_progress_bar"],
        verbosity_lvl=cli_args["verbosity"],
    )
    return cfg


# conversion function -------------------------------------------------------------------------
def convert_rosbag2(cfg: Config, logger=pkg_logger):
    if logger is pkg_logger:
        logger.setLevel(verbosity_to_logging_lvl(cfg.verbosity_lvl))

    converter_plugins = load_converter_plugins_from_multiple_files(cfg.converter_plugin_paths)
    out_path = cfg.output_dir

    # internal plugins
    # attempt to load plugins that are shipped with this tool.
    internal_plugins_dir = get_internal_plugins_dir()
    internal_converter_plugin_paths = [
        internal_plugins_dir / internal_plugin
        for internal_plugin in os.listdir(internal_plugins_dir)
        if internal_plugin.endswith(".py")
    ]
    internal_converter_plugins = load_converter_plugins_from_multiple_files(
        internal_converter_plugin_paths, raise_on_error=False
    )

    # config file
    with cfg.config_path.open() as f:
        # skip lines starting with comments `// `, e.g., in .jsonc files
        lines = [li for li in f.readlines() if not li.lstrip().startswith("// ")]
        json_conts = json.loads(" ".join(lines))

    # determine format ------------------------------------------------------------------------
    conversion_format = ConversionFormat.detect_format(json=json_conts)

    # output path already exists? generate a new one
    out_path = fs.get_ready_output_path(
        out_path, overwrite_path=cfg.overwrite_output, logger=logger
    )
    out_path.mkdir()

    # storage type autodetection --------------------------------------------------------------
    storage_type = cfg.storage_type
    if not storage_type:
        file_extension = cfg.bag_path.suffix

        if not file_extension:
            raise RuntimeError(
                "The provided bag file does not have an extension and the storage type has "
                "not explicitly specified in the command line. Cannot proceed."
            )
        elif file_extension == ".mcap":
            storage_type = "mcap"
        elif file_extension in [".sqlite3", ".db3"]:
            storage_type = "sqlite3"
        else:
            raise NotImplementedError(f"Unrecognised file extension {file_extension}.")

        logger.info(
            f"Determined storage type {storage_type} from file extension {file_extension}"
        )

    # print summary ---------------------------------------------------------------------------
    if converter_plugins:
        plugins_str = " | ".join(str(p) for p in converter_plugins)
    else:
        plugins_str = "None"
    announce_items = {
        "Input bag file": cfg.bag_path,
        "Storage": storage_type,
        "Output directory": out_path,
        f"{len(converter_plugins)} Converter plugins ": plugins_str,
        "Overwrite output directory": cfg.overwrite_output,
    }
    logger.warning(
        format_dict(
            header="Configuration",
            items=announce_items,
            prefix="\n\n",
            suffix="\n",
        )
    )

    # load user-specified plugins -------------------------------------------------------------
    for cp in converter_plugins:
        register_measurement_type(cp.measurement_type)
        measurement_type_to_writer_type[cp.measurement_type] = cp.writer_type  # type: ignore
        measurement_to_msg_type[cp.measurement_type] = [cp.msg_type]

    # load internal plugins -------------------------------------------------------------------
    logger.info(
        format_list(
            header="Internal converter plugins",
            items=[str(p) for p in internal_converter_plugins],
            prefix="\n\n",
            suffix="\n",
        )
    )
    for cp in internal_converter_plugins:
        register_measurement_type(cp.measurement_type)
        measurement_type_to_writer_type[cp.measurement_type] = cp.writer_type  # type: ignore
        measurement_to_msg_type[cp.measurement_type] = [cp.msg_type]

    # parse rosbag ----------------------------------------------------------------------------
    rosbag_info, reader = init_rosbag2_reader_handle_exceptions(
        bag_path=cfg.bag_path, storage=storage_type
    )
    message_counts = get_topic_names_to_message_counts(rosbag_info)
    rosbag_topics = get_topic_names_to_types(reader)

    # discard topics with 0 messages
    for topic_name in list(rosbag_topics.keys()):
        if message_counts[topic_name] == 0:
            logger.warning(
                f"rosbag contains 0 messages for topic {topic_name}. Discarding it."
            )
            rosbag_topics.pop(topic_name)

    (out_path / "metadata.txt").write_text(str(rosbag_info))
    logger.info(f"Rosbag metadata:\n\n{rosbag_info}")

    # assemble conversion config --------------------------------------------------------------
    conv_props_assembler = ConversionPropsAssembler(
        sc_dataset_path=out_path,
        rosbag_topics=rosbag_topics,
        imgq_put_timeout_s=cfg.imgq_put_timeout_s,
        logger=logger,
    )
    logger.info("Validating input config file and contents of rosbag, initializing writers...")
    if conversion_format[0] is ConversionFormat.RIGID:
        conversion_map = conv_props_assembler.assemble_from_rigid_format(json_conts=json_conts)
    elif ConversionFormat.FLEXIBLE:
        conversion_map = conv_props_assembler.assemble_from_flexible_format(
            json_conts=json_conts
        )
    else:
        raise NotImplementedError(f"Cannot process conversion format {conversion_format[0]}")

    # initialize progress bar -----------------------------------------------------------------
    image_msgs_count = 0
    for conversion_props in conversion_map.values():
        if conversion_props.measurement_type.is_camera:
            image_msgs_count += message_counts[conversion_props.topic_name]
    global progress_bar_
    if cfg.use_progress_bar:
        progress_bar_ = progress_bar(total=image_msgs_count)
    else:
        progress_bar_ = DummyProgressBar()

    # consume only the topics we're interested in ---------------------------------------------
    storage_filter = rosbag2_py.StorageFilter(topics=list(conversion_map.keys()))
    reader.set_filter(storage_filter)

    # main conversion loop --------------------------------------------------------------------
    # start the PNG encoding workers
    worker_threads: Sequence[threading.Thread] = []
    for _ in range(cfg.jobs):
        t = threading.Thread(target=img_worker, daemon=True, args=[logger])
        t.start()
        worker_threads.append(t)
    logger.warning("Consuming rosbag, this may take a while...")

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

        conversion_map[topic].writer.write(msg)

    logger.info("Consumed rosbag.")
    logger.info("Flushing pending data...")
    imgq.join()
    logger.info("Flushed pending data.")

    # teardown actions ------------------------------------------------------------------------
    for writer in [v.writer for v in conversion_map.values()]:
        writer.teardown()

    logger.warning(f"Finished converting {cfg.bag_path} -> {out_path} .")


# main ----------------------------------------------------------------------------------------
def main():
    cfg = get_cli_assembled_config()
    convert_rosbag2(cfg=cfg)


if __name__ == "__main__":
    main()
