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


import argparse
import os
from pathlib import Path
from typing import NoReturn

from slamcore_utils.arg_parser import add_bool_argument, existing_file

"""ROS Utilities - independent of ROS version."""

# default number of jobs to use for conversion
DEFAULT_NUM_JOBS = len(os.sched_getaffinity(0))
DEFAULT_OVERWRITE_OUTPUT_OPT = False


def _positive_int(val: str):
    def _raise() -> NoReturn:
        raise argparse.ArgumentTypeError(f"Value has to be a positive integer - got {val}")

    try:
        out = int(val)
    except ValueError:
        _raise()

    if out <= 0:
        _raise()

    return out


def add_parser_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-b", "--bag", type=existing_file, required=True, help="Path to input ROS bag file"
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="Path to output directory in Slamcore Dataset Format",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=existing_file,
        required=True,
        help="Path to JSON configuration file",
    )
    parser.add_argument(
        "-v", "--verbosity", action="count", help="Increase output verbosity", default=0
    )
    parser.add_argument(
        "-j",
        "--jobs",
        help="Number of parallel jobs to use during conversion",
        default=DEFAULT_NUM_JOBS,
        required=False,
        type=_positive_int,
    )

    add_bool_argument(
        parser=parser,
        arg_name="overwrite",
        default=DEFAULT_OVERWRITE_OUTPUT_OPT,
        true_help="Overwrite destination location",
    )
