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


from typing import Optional, Type

from slamcore_utils.dataset_subdir_writer import DatasetSubdirWriter
from slamcore_utils.measurement_type import MeasurementType


class Ros2ConverterPlugin:
    def __init__(
        self,
        writer_type: Type[DatasetSubdirWriter],
        measurement_type: MeasurementType,
        msg_type: str,
    ):
        self.writer_type = writer_type
        self.measurement_type = measurement_type
        self.msg_type = msg_type

    def description_long(self) -> str:
        return (
            f"Converter of {self.msg_type:20} messages to "
            f"{self.measurement_type.name:10} subdir "
            f"using writer {self.writer_type.__name__}"
        )

    def description_short(self) -> str:
        return f"Msgs: {self.msg_type:20}, Writer {self.writer_type.__name__}"

    def __str__(self):
        return self.description_short()


class Ros2PluginInitializationFailureError(BaseException):
    """Exception raised when the plugin at hand could not be initailized correctly."""

    def __init__(self, plugin_name, msg: Optional[str] = None):
        final_msg = f"Error initializing the {plugin_name} plugin."
        if msg is not None:
            final_msg = f"{final_msg} | Original Error: {msg}"

        super().__init__(final_msg)
