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

import sys
from pathlib import Path
from typing import Dict, Mapping, Tuple

import rosbag2_py

from slamcore_utils.logging import logger

InfoAndReader = Tuple[rosbag2_py._storage.BagMetadata, rosbag2_py._reader.SequentialReader]


def init_rosbag2_reader_handle_exceptions(bag_path: Path, storage: str) -> InfoAndReader:
    try:
        return init_rosbag2_reader(bag_path=bag_path, storage=storage)
    except:  # noqa: E722
        logger.critical(
            "Couldn't initialize rosbag2 reader. Either bad rosbag or you might be missing some packages"
        )
        sys.exit(1)


def init_rosbag2_reader(bag_path: Path, storage: str) -> InfoAndReader:
    bag_str = str(bag_path)
    info = rosbag2_py.Info().read_metadata(bag_str, storage)

    if info.storage_identifier != storage:
        logger.critical(
            f"Mismatch between given storage id {storage} and the one in metadata {info.storage_identifier}"
        )
        sys.exit(1)

    # initialise reader
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_str, storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    return info, reader


def get_topic_names_to_types(reader) -> Dict[str, str]:
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def get_topic_names_to_message_counts(info) -> Mapping[str, int]:
    return {
        topic_message_count.topic_metadata.name: topic_message_count.message_count
        for topic_message_count in info.topics_with_message_count
    }
