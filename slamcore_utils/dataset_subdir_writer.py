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


import abc
from pathlib import Path
from typing import Type

from slamcore_utils.logging import logger as pkg_logger


class DatasetSubdirWriter(abc.ABC):
    """Writer class.

    Responsible for initializing, converting and writing messages to a subdirectory of a
    Slamcore output dataset directory.
    """

    def __init__(self, directory: Path, logger=pkg_logger):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=False)
        self.logger = logger

    def register_ros_msg_type(self, msg_type: Type) -> None:
        """
        In case the writer supports multiple ROS 2 msg types and changes its behavior based on
        the particular ROS 2 msg type being passed, the derived class can implement this
        function to become aware of the concrete message type that it will receive.
        """

    def prepare_write(self) -> None:
        """
        Initialization actions for the writer at hand.
        """

    @abc.abstractmethod
    def write(self, msg):
        pass

    def teardown(self) -> None:
        pass
