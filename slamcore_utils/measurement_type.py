__copyright__ = """

    SLAMcore Confidential
    ---------------------

    SLAMcore Limited
    All Rights Reserved.
    (C) Copyright 2020

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


import re
from functools import lru_cache
from typing import Dict, Optional, Sequence


class UnknownMeasurementTypeError(BaseException):
    """Exception raised when the measurement type cannot be found."""

    def __init__(self, name):
        digit_stripped_name = re.sub(r"\d+$", "", name)
        super().__init__(
            f"No MeasurementType found for {digit_stripped_name}. "
            "The following measurement types are defined: "
            f'{", ".join([m.shortname for m in measurement_types])}'
        )


class MeasurementType:
    __slots__ = ("name", "shortname", "is_camera")

    def __init__(self, *, name: str, shortname: str, is_camera: bool = False):
        self.name = name
        self.shortname = shortname
        self.is_camera = is_camera

    @staticmethod
    @lru_cache()
    def from_str(s: str) -> "MeasurementType":
        """Given a string, return the corresponding measurement type.

        .. raises:: Exception if the given string doesn't match any MeasurementType.

        Examples:
        ----------
        >>> MeasurementType.from_str("ir1").name
        'Infrared'
        >>> MeasurementType.from_str("ir100").name
        'Infrared'
        >>> MeasurementType.from_str("imu0").name
        'Imu'
        >>> MeasurementType.from_str("depth0").name
        'Depth'
        >>> MeasurementType.from_str("odometry7").name
        'Odometry'
        >>> MeasurementType.from_str("ir")
        Traceback (most recent call last):
        ...
        RuntimeError: Invalid string provided ...
        >>> MeasurementType.from_str("depthandsomerandomstring0")
        Traceback (most recent call last):
        ...
        slamcore_utils.measurement_type.UnknownMeasurementTypeError: ...

        >>> MeasurementType.from_str("new_sensor0")
        Traceback (most recent call last):
        ...
        slamcore_utils.measurement_type.UnknownMeasurementTypeError: ...

        >>> register_measurement_type(MeasurementType(name="NewSensor", shortname="new_sensor", is_camera=True))
        >>> MeasurementType.from_str("new_sensor0").name
        'NewSensor'
        """

        s = s.lower()

        match = re.match(r"([A-z]+)\d+", s)
        if match is None:
            raise RuntimeError(
                f'Invalid string provided -> "{s}". '
                'Valid string examples: "ir1", "odometry2", "imu0"'
            )

        groups = match.groups()
        if not groups or len(groups) > 1:
            raise RuntimeError(
                f'Invalid string provided -> "{s}". '
                'Valid string examples: "ir1", "odometry2", "imu0"'
            )

        try:
            return _vals_to_measurement_types[groups[0]]
        except KeyError as e:
            raise UnknownMeasurementTypeError(s) from e


measurement_types: Sequence[MeasurementType] = []
names_to_measurement_types: Dict[str, MeasurementType] = {}
_vals_to_measurement_types: Dict[str, MeasurementType] = {}


def get_measurement_type(key, val) -> MeasurementType:
    """
    Get the measurement type from a JSON element.

    See whether a "measurement" type has been explicitly specified, otherwise, try
    autodetecting the measurement type based on the key of the JSON key-value pair.
    """
    # see if there JSON value specifies a converter
    measurement_type_str: Optional[str] = val.get("measurement_type")
    if measurement_type_str is not None:
        measurement_type = MeasurementType.from_str(f"{measurement_type_str}0")
    else:
        # measurement type wasn't explicitly specified, try deducing from key.
        try:
            measurement_type = MeasurementType.from_str(key)
        except UnknownMeasurementTypeError as e:
            raise RuntimeError(
                f"MeasurementType wasn't specified and cannot be deduced from the key -> {key}"
            ) from e

    return measurement_type


def register_measurement_type(m: MeasurementType):
    """Register a new measurement type."""
    measurement_types.append(m)
    names_to_measurement_types[m.name] = m
    _vals_to_measurement_types[m.shortname] = m


_r = register_measurement_type

_r(MeasurementType(name="Infrared", shortname="ir", is_camera=True))
_r(MeasurementType(name="Depth", shortname="depth", is_camera=True))
_r(MeasurementType(name="Cam", shortname="cam", is_camera=True))
_r(MeasurementType(name="Imu", shortname="imu", is_camera=False))
_r(MeasurementType(name="Odometry", shortname="odometry", is_camera=False))
_r(MeasurementType(name="GroundTruth", shortname="groundtruth", is_camera=False))
_r(MeasurementType(name="SLAMPose", shortname="slam_pose", is_camera=False))
_r(MeasurementType(name="SmoothPose", shortname="smooth_pose", is_camera=False))

del _r
