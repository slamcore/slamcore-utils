# `slamcore_utils`

<a href="https://github.com/slamcore/slamcore_utils/actions" alt="CI">
<img src="https://github.com/slamcore/slamcore_utils/actions/workflows/ci.yml/badge.svg" /></a>

<a href="https://github.com/slamcore/slamcore_utils/blob/master/LICENSE.md" alt="LICENSE">
<img src="https://img.shields.io/github/license/slamcore/slamcore_utils.svg" /></a>
<a href="https://pypi.org/project/slamcore_utils/" alt="pypi">
<img src="https://img.shields.io/pypi/pyversions/slamcore_utils.svg" /></a>
<a href="https://github.com/slamcore/slamcore_utils/actions" alt="lint">
<img src="https://img.shields.io/badge/checks-mypy%2C%20pyright-brightgreen" /></a>
<a href="https://badge.fury.io/py/slamcore_utils">
<img src="https://badge.fury.io/py/slamcore-utils.svg" alt="PyPI version" height="18"></a>
<!-- <a href="https://pepy.tech/project/slamcore_utils"> -->
<!-- <img alt="Downloads" src="https://pepy.tech/badge/slamcore_utils"></a> -->
<a href="https://github.com/psf/black">
<img alt="Code style: black" src="https://img.shields.io/badge/code%20style-black-000000.svg"></a>

## Description

<!-- Change this when we add more scripts -->

This repo contains a collection of complementary scripts to the main Slamcore
SDK. It currently offers the following two main scripts:

- [slamcore-setup-dataset](#slamcore-setup-dataset)
- [slamcore-convert-rosbag2](#slamcore-convert-rosbag2)

## `slamcore-setup-dataset`

### Description

`slamcore-setup-dataset` can be used for installing a sample dataset for
offline testing and evaluation of [Slamcore][slamcore]'s Localization and
Mapping capabilities.

Currently the following types of datasets are supported:

- [EuRoC MAV Datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (use the `ASL Dataset Format` format)
- [OpenLORIS-Scene Datasets](https://lifelong-robotic-vision.github.io/dataset/scene)
- [TUM VI Datasets](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) (use the `Euroc / DSO` format)

### Usage

After [installation](#installation) the script should be available in your path.
Executing it will guide you through a list of questions in order to properly
setup a sample SLAM dataset.

Here is a sample execution of the said script to enable processing of the `TUM-VI`
`dataset-room4_1024_16`

![setup-dataset2](https://github.com/slamcore/slamcore_utils/raw/master/share/images/slamcore-setup-dataset2.gif)

Here's the same execution for the `OpenLORIS` `cafe1-1` dataset

![setup-dataset1](https://github.com/slamcore/slamcore_utils/raw/master/share/images/slamcore-setup-dataset1.gif)

And here's the execution guiding the user to the right download page, when
the datasets are not available locally yet.

![setup-dataset3](https://github.com/slamcore/slamcore_utils/raw/master/share/images/slamcore-setup-dataset3.gif)

## `slamcore-convert-rosbag2`

### Description

The `slamcore-convert-rosbag2` script allows you to convert datasets stored in
a [rosbag2](https://github.com/ros2/rosbag2)-compatible format to the Slamcore
dataset format. Given the path to the rosbag2 _file_ (the `.db3`, `.mcap`) file
and given mappings of ROS 2 topics to subdirectories in the Slamcore Dataset
format, it will go through the rosbag2 and convert the required streams to
generate the Slamcore dataset. By default, the `slamcore-convert-rosbag2` is
only aware of the following messages:

- `geometry_msgs/PoseStamped`
- `geometry_msgs/PoseWithCovarianceStamped`
- `gps_msgs/GPSFix`
- `nav_msgs/Odometry`
- `sensor_msgs/Camera`
- `sensor_msgs/Imu`

The user can also specify plugins for conversions of arbitrary messages - see
the [plugins](#rosbag-2-converter-plugins) section for more.

It's also worth noting that `slamcore-convert-rosbag2` does not require the
existence of the `metadata.yaml` file of the rosbag2 as it can read the metadata
from the rosbag2 database file itself.

### Usage - Convert a sample `rosbag2`

```
source /opt/ros/galactic/setup.bash
slamcore-convert-rosbag2 \
  -b tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3 \
  -o output \
  -c tests/test_data/trimmed_rosbag2.json
```

Notes:

- A test `rosbag2` is included in the test directory of this repo. It's in the
  [mcap](https://mcap.dev/guides/getting-started/ros-2) format. For Ubuntu 20.04
  and the galactic distribution, you will have to install the
  `ros-galactic-rosbag2-storage-mcap` package to process a `rosbag2` in this
  format.
- The following mappings are used:

  ```sh
  cat tests/test_data/trimmed_rosbag2.json
  ```

  ```json
  {
    "ir0": {
      "topic": "/slamcore/visible_0/image_raw"
    },
    "ir1": {
      "topic": "/slamcore/visible_1/image_raw"
    },
    "imu0": {
      "topic": "/irrelevant_imu"
    },
    "odometry0": {
      "topic": "/slamcore/odom"
    },
    "groundtruth0": {
      "topic": "/irrelevant_gts"
    }
  }
  ```

- The layout of the output directory is as follows:

  ```sh
  tree -L 2 output
  ```

  ```text
  output/
  ├── groundtruth0
  │   └── data.csv
  ├── imu0
  │   ├── acc.csv
  │   └── gyro.csv
  ├── ir0
  │   ├── data
  │   └── data.csv
  ├── ir1
  │   ├── data
  │   └── data.csv
  ├── metadata.txt
  └── odometry0
      └── data.csv
  ```

- You should expect output like the following during execution
  <details>
    <summary>Command execution output</summary>

  ```text
  13:25:24 | WARNING  -  Output path already exists. Overwriting it...
  13:25:24 | WARNING  -

  Configuration:
  ===============

    - Input bag file            : tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3
    - Storage                   : sqlite3
    - Output directory          : output
    - Converter plugins 0       : None
    - Overwrite output directory: True

  INFO | 1685528724.276752680 | rosbag2_storage | Opened database 'tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3' for READ_ONLY.
  INFO | 1685528724.360516990 | rosbag2_storage | Opened database 'tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3' for READ_ONLY.
  13:25:24 | WARNING  -  Finished converting tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3 -> output .
  ```

  </details>

  You can also increase the verbosity of the tool with `-v` (for `INFO`) or with
  `-vv` for (for `DEBUG` and `INFO`) messages .

  <details>
    <summary>Command execution output with -vv</summary>

  ```text
  13:26:01 | INFO     -  Determined storage type sqlite3 from file extension .db3
  13:26:01 | WARNING  -  Output path already exists. Overwriting it...
  13:26:01 | WARNING  -

  Configuration:
  ===============

    - Input bag file            : tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3
    - Storage                   : sqlite3
    - Output directory          : output
    - Converter plugins 0       : None
    - Overwrite output directory: True

  INFO | 1685528761.113749019 | rosbag2_storage | Opened database 'tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3' for READ_ONLY.
  INFO | 1685528761.195022167 | rosbag2_storage | Opened database 'tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3' for READ_ONLY.
  13:26:01 | DEBUG    -  Rosbag metadata:

  Files:             tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3
  Bag size:          36.4 MiB
  Storage id:        sqlite3
  Duration:          16.350s
  Start:             May 31 2023 10:55:05.54 (1685519705.54)
  End:               May 31 2023 10:55:21.404 (1685519721.404)
  Messages:          94576
  Topic information: Topic: /irrelevant_gts | Type: geometry_msgs/msg/PoseStamped | Count: 490 | Serialization Format: cdr
                    Topic: /irrelevant_imu | Type: sensor_msgs/msg/Imu | Count: 817 | Serialization Format: cdr
                    Topic: /slamcore/accel | Type: sensor_msgs/msg/Imu | Count: 34530 | Serialization Format: cdr
                    Topic: /slamcore/gyro | Type: sensor_msgs/msg/Imu | Count: 34530 | Serialization Format: cdr
                    Topic: /slamcore/metadata/distance_travelled | Type: std_msgs/msg/Float64 | Count: 3453 | Serialization Format: cdr
                    Topic: /slamcore/metadata/num_features | Type: std_msgs/msg/Int64 | Count: 3453 | Serialization Format: cdr
                    Topic: /slamcore/metadata/slam_event | Type: slamcore_msgs/msg/SLAMEvent | Count: 3 | Serialization Format: cdr
                    Topic: /slamcore/metadata/tracked_features | Type: std_msgs/msg/Int64 | Count: 6906 | Serialization Format: cdr
                    Topic: /slamcore/metadata/tracking_status | Type: slamcore_msgs/msg/TrackingStatus | Count: 3454 | Serialization Format: cdr
                    Topic: /slamcore/odom | Type: nav_msgs/msg/Odometry | Count: 3453 | Serialization Format: cdr
                    Topic: /slamcore/pose | Type: geometry_msgs/msg/PoseStamped | Count: 3453 | Serialization Format: cdr
                    Topic: /slamcore/visible_0/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 7 | Serialization Format: cdr
                    Topic: /slamcore/visible_0/image_raw | Type: sensor_msgs/msg/Image | Count: 11 | Serialization Format: cdr
                    Topic: /slamcore/visible_1/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 8 | Serialization Format: cdr
                    Topic: /slamcore/visible_1/image_raw | Type: sensor_msgs/msg/Image | Count: 8 | Serialization Format: cdr

  13:26:01 | INFO     -  Validating input config file and contents of rosbag...
  13:26:01 | DEBUG    -  Mapping Infrared        - /slamcore/visible_0/image_raw -> ir0...
  13:26:01 | DEBUG    -  Mapping Infrared        - /slamcore/visible_1/image_raw -> ir1...
  13:26:01 | DEBUG    -  Mapping Imu             - /irrelevant_imu -> imu0...
  13:26:01 | DEBUG    -  Mapping Odometry        - /slamcore/odom -> odometry0...
  13:26:01 | DEBUG    -  Mapping GroundTruth     - /irrelevant_gts -> groundtruth0...
  13:26:01 | INFO     -  Consuming rosbag...
  13:26:01 | INFO     -  Consumed rosbag.
  13:26:01 | INFO     -  Flushing pending data...
  13:26:01 | INFO     -  Flushed pending data.
  13:26:01 | WARNING  -  Finished converting tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3 -> output .
  ```

  </details>

### Rosbag 2 Converter Plugins

In addition to the standard message types specified in the tool's description,
the `slamcore-convert-rosbag2` tool allows the conversion of arbitrary topics
given that you teach it how to convert them and what type of messages they are.
You can specify this in a separate python module, and specify this module's
path during the `slamcore-convert-rosbag2` execution. For example, here's one
plugin to convert the `TrackingStatus` messages published by the [Slamcore ROS 2
wrapper](https://docs.slamcore.com/release_23.04/ros2-wrapper.html)

<details>
  <summary>Code in Rosbag2 Converter Plugin - plugin.py</summary>

```python
from slamcore_utils import DatasetSubdirWriter, MeasurementType
from slamcore_utils.ros2 import Ros2ConverterPlugin

class TrackingStatusWriter(DatasetSubdirWriter):
    def __init__(self, directory):
        super().__init__(directory=directory)

        self.ofs_tracking_status = (self.directory / "data.csv").open("w", newline="")
        self.csv_tracking_status = csv.writer(self.ofs_tracking_status, delimiter=",")
        self.csv_tracking_status.writerow(["tracking_status_val", "tracking_status_str"])

    def write(self, msg):
        if msg.data is TrackingStatus.NOT_INITIALISED:
            self.csv_tracking_status.writerow([msg.data, "NOT_INITIALISED"])
        elif msg.data is TrackingStatus.OK:
            self.csv_tracking_status.writerow([msg.data, "OK"])
        elif msg.data is TrackingStatus.LOST:
            self.csv_tracking_status.writerow([msg.data, "LOST"])
        else:
            logger.error(f"Unknown TrackingStatus {msg.data}")

    def teardown(self):
        self.ofs_tracking_status.close()

converter_plugins = [
    Ros2ConverterPlugin(
        writer_type=TrackingStatusWriter,
        measurement_type=MeasurementType(
            name="TrackingStatus", shortname="tracking_status", is_camera=False
        ),
        topic_type="slamcore_msgs/msg/TrackingStatus",
    ),
  ]
```

</details>

You can then specify the path to the plugin above during the tool execution and
specify the extra topics to convert in the provided JSON file

```json
  ...

  "tracking_status0": {
    "topic": "/slamcore/metadata/tracking_status"
  },

  ...
```

```sh
slamcore-convert-rosbag2 \
  -b tests/test_data/trimmed_rosbag2/trimmed_rosbag2_0.db3 \
  -o output -c ammended_trimmed_rosbag2.json
  -p /path/to/plugin.py
```

This will, in addition to the standard topics conversion also write the
tracking status data to the `output/tracking_status0/data.csv` file.

You can also have a look at the blackbox test of the plugins at
[slamcore_convert_rosbag2_with_plugin](tests/test_data/executables/slamcore_convert_rosbag2_with_plugin)
and at
[test_ros2_conversions.py](tests/test_ros2_conversions.py)

## Installation

Install it directly from PyPI:

```sh
pip3 install --user --upgrade slamcore_utils[tqdm]

# Or if you don't want tqdm's polished progress bars
pip3 install --user --upgrade slamcore_utils

# Always install the `ros2` extra if you intend to use the
# `slamcore-convert-rosbag2` executable
pip3 install --user --upgrade slamcore_utils[ros2]
```

<details>
  <summary>I don't want to have to install it</summary>

Make sure the project dependencies are installed:

```sh
pip3 install -r requirements.txt
```

Then adjust your `PYTHONPATH` variable and run accordingly:

```sh
git clone https://github.com/slamcore/slamcore_utils
cd slamcore_utils
export PYTHONPATH=$PYTHONPATH:$PWD
./slamcore_utils/scripts/setup_dataset.py
```

</details>

<details>
  <summary>I don't want to install any of your dependencies in my user's install directory</summary>

Consider using either [pipx](https://github.com/pypa/pipx) or
[poetry](https://github.com/python-poetry/poetry) to install this package and
its dependencies isolated in a virtual environment:

```sh
git clone https://github.com/slamcore/slamcore_utils
cd slamcore_utils
poetry install
poetry shell

# the executables should now be available in your $PATH
setup-dataset
```

</details>

## About Slamcore

Slamcore offers commercial-grade visual-inertial
simultaneous localisation and mapping (SLAM) software for real-time autonomous
navigation on robots and drones. Find out more at [slamcore.com][slamcore].

[slamcore]: https://www.slamcore.com/
