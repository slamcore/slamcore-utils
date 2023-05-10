# ``slamcore_utils``

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

* [slamcore-setup-dataset](#slamcore-setup-dataset)
* [slamcore-convert-rosbag2](#slamcore-convert-rosbag2)

## `slamcore-setup-dataset`

### Description

`slamcore-setup-dataset`  can be used for installing a sample dataset for
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
dataset format. Given the path to the rosbag2 *file* (the `.db3`, `.mcap`) file
and given mappings of ROS 2 topics to subdirectories in the Slamcore Dataset
format, it will go through the rosbag2 and convert the required streams to
generate the Slamcore dataset. By default, the `slamcore-convert-rosbag2` is
only aware of the following messages:

- `sensor_msgs/Imu`
- `sensor_msgs/Camera`
- `nav_msgs//Odometry`
- `geometry_msgs/PoseStamped`

The user can also specify plugins for conversions of arbitrary messages - see
the [plugins](#rosbag2-converter-plugins) section for more.

### Usage - Convert a sample `rosbag2`

```
source /opt/ros/galactic/setup.bash
slamcore-convert-rosbag2 -b tests/test_data/rosbag2_trimmed/rosbag2_2023_01_30-15_00_00_0.mcap -o output -c tests/test_data/test_rosbag2.json
```

Notes:

* A test `rosbag2` is included in the test directory of this repo. It's in the
  [mcap](https://mcap.dev/getting-started/ros-2.html) format. For Ubuntu 20.04
  and the galactic distribution, you will have to install the
  `ros-galactic-rosbag2-storage-mcap` package to process a `rosbag2` in this format.
* The following mappings are used:

  ```sh
  cat tests/test_data/test_rosbag2.json
  ```

  ```json
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

* The layout of the output directory is as follows:

  ```sh
  tree -L 2 output
  ```

  ```
  output
  ├── ir0
  │   ├── data
  │   └── data.csv
  ├── ir1
  │   ├── data
  │   └── data.csv
  ├── metadata.txt
  ├── slam_pose0
  │   └── data.csv
  └── smooth_pose0
      └── data.csv
  ```

* You should expect output like the following during execution
  <details>
    <summary>Command execution output</summary>

    ```
    16:32:16 | WARNING  -  Path already exists. Will choose another name...
    16:32:16 | WARNING     -

    Configuration:
    ===============

      - Input bag file            : tests/test_data/rosbag2_trimmed/rosbag2_2023_01_30-15_00_00_0.mcap
      - Storage                   : mcap
      - Output directory          : output
      - Converter plugins 0       : None
      - Overwrite output directory: False

    16:32:23 | WARNING  -  Finished converting tests/test_data/rosbag2_trimmed/rosbag2_2023_01_30-15_00_00_0.mcap -> output
    ```

  </details>

  You can also increase the verbosity of the tool with `-v` (for `INFO`) or with
  `-vv` for (for `DEBUG` and `INFO`) messages .

  <details>
    <summary>Command execution output with -vv</summary>

    ```
    16:36:39 | INFO     -  Determined storage type mcap from file extension .mcap
    16:36:39 | WARNING  -  Path already exists. Will choose another name...
    16:36:39 | WARNING  -

    Configuration:
    ===============

      - Input bag file            : tests/test_data/rosbag2_trimmed/rosbag2_2023_01_30-15_00_00_0.mcap
      - Storage                   : mcap
      - Output directory          : output
      - Converter plugins 0       : None
      - Overwrite output directory: False



    16:36:39 | DEBUG    -  Rosbag metadata:


    Files:             tests/test_data/rosbag2_trimmed/rosbag2_2023_01_30-15_00_00_0.mcap
    Bag size:          4.1 MiB
    Storage id:        mcap
    Duration:          9.95s
    Start:             Jan 15 2023 00:59:46.904 (1673737186.904)
    End:               Jan 15 2023 00:59:55.999 (1673737195.999)
    Messages:          3399
    Topic information: Topic: /gnss | Type: sensor_msgs/msg/NavSatFix | Count: 9 | Serialization Format: cdr
                      Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 561 | Serialization Format: cdr
                      Topic: /stereo/right/mono/image_raw | Type: sensor_msgs/msg/Image | Count: 71 | Serialization Format: cdr
                      Topic: /stereo/right/mono/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 71 | Serialization Format: cdr
                      Topic: /stereo/left/mono/image_raw | Type: sensor_msgs/msg/Image | Count: 71 | Serialization Format: cdr
                      Topic: /stereo/left/mono/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 71 | Serialization Format: cdr
                      Topic: /slamcore/pose | Type: geometry_msgs/msg/PoseStamped | Count: 545 | Serialization Format: cdr
                      Topic: /slamcore/odom | Type: nav_msgs/msg/Odometry | Count: 545 | Serialization Format: cdr
                      Topic: /gts | Type: geometry_msgs/msg/PoseStamped | Count: 545 | Serialization Format: cdr
                      Topic: /stereo/imu | Type: sensor_msgs/msg/Imu | Count: 910 | Serialization Format: cdr

    16:36:39 | INFO     -  Validating input config file and contents of rosbag...
    16:36:39 | DEBUG    -  Mapping Infrared        - /stereo/left/mono/image_raw -> ir0...
    16:36:39 | DEBUG    -  Mapping Infrared        - /stereo/right/mono/image_raw -> ir1...
    16:36:39 | DEBUG    -  Mapping Imu             - /stereo/imu -> imu0...
    16:36:39 | DEBUG    -  Mapping Odometry        - /odom -> odometry0...
    16:36:39 | DEBUG    -  Mapping GroundTruth     - /gts -> groundtruth0...
    16:36:39 | INFO     -  Consuming rosbag...
    16:36:43 | INFO     -  Consumed rosbag.
    16:36:43 | INFO     -  Flushing pending data...
    16:36:43 | INFO     -  Flushed pending data.
    16:36:43 | WARNING  -  Finished converting tests/test_data/rosbag2_trimmed/rosbag2_2023_01_30-15_00_00_0.mcap -> output_0
    ```

  </details>

### Rosbag 2 Converter Plugins

In addition to the standard message types specified in the tool's description,
the `slamcore-convert-rosbag2` tool allows the conversion of arbitrary topics
given that you teach it how to convert them and what type of messages they are.
You can specify this in a separate python module, and specify this module's
path during the `slamcore-convert-rosbag2` execution. For example, here's one
plugin to convert the `TrackingStatus` messages published by the [Slamcore ROS 2
wrapper](https://docs.slamcore.com/release_23.01/ros2-wrapper.html)

<details>
  <summary>Code in Rosbag2 Converter Plugin - plugin.py</summary>`

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
</details

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
slamcore-convert-rosbag2 -b tests/test_data/rosbag2_trimmed/rosbag2_2023_01_30-15_00_00_0.mcap -o output -c tests/test_data/test_rosbag2.json -p /path/to/plugin.py
```

This will, in addition to the standard topics conversion also write the
tracking status data to the `output/tracking_status0/data.csv` file.

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

`pip3 install -r requirements.txt`

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
