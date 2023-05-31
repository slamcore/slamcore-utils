# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## 0.1.6 (2022-05-31)

### Added

- New script: `slamcore-convert-rosbag2`
- New package extra - `ros2`

### Fixed

- Bug in test_utils - assertions on stdout / stderr were not working properly

## 0.1.4 (2022-04-07)

### Changed

- Support `python >= 3.6.8` instead of `>=3.7`

## 0.1.3 (2022-02-08)

### Changed

- Amend `capture_info.json` files that are to be embedded in the supported
  datasets. Make them reflect the structure of the dataset and sensors
  modality more accurately.

## 0.1.0 (2021-12-07)

- Add scripts for OpenLORIS dataset conversion (`convert_openrloris.py`) and
  interactive `setup-dataset` script
- First minor version 🎉
- Bootstrap project via
  [python_bootstrap](https://github.com/bergercookie/python_package_cookiecutter)

<!-- ### Added -->
<!-- ### Changed -->
<!-- ### Deprecated -->
<!-- ### Removed -->
<!-- ### Fixed -->
<!-- ### Security -->
