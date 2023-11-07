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


try:
    from .conversion_format import ConversionFormat
    from .plugin_utils import (
        get_internal_plugins_dir,
        load_converter_plugins,
        load_converter_plugins_from_multiple_files,
    )
    from .ros2_converter_plugin import (
        Ros2ConverterPlugin,
        Ros2PluginInitializationFailureError,
    )
    from .ros2_utils import (
        get_topic_names_to_message_counts,
        get_topic_names_to_types,
        init_rosbag2_reader,
        init_rosbag2_reader_handle_exceptions,
    )

    __all__ = [
        "Ros2ConverterPlugin",
        "Ros2PluginInitializationFailureError",
        "get_topic_names_to_message_counts",
        "get_topic_names_to_types",
        "init_rosbag2_reader",
        "init_rosbag2_reader_handle_exceptions",
        "get_internal_plugins_dir",
        "load_converter_plugins",
        "load_converter_plugins_from_multiple_files",
        "ConversionFormat",
    ]
except ModuleNotFoundError as e:
    print(
        "ROS2-related imports are unsuccessful. "
        "Make sure you have sourced the ROS2 installation and re-try.\n\n"
        f"Original error:\n\n{e}"
    )
