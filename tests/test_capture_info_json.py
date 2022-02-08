import json
import operator
from functools import reduce
from pathlib import Path
from typing import Any, Sequence


def _get_from_dict(data: dict, keys_seq: Sequence) -> Any:
    """Access a specific item from a dictionary given a specific sequence of keys."""
    return reduce(operator.getitem, keys_seq, data)


def _check_mandatory_keys(p: Path):
    """
    Make sure that the given JSON file has valid values at the designated keys, i.e., their
    value is not "unknown".
    """
    props_to_check: Sequence[str] = (
        "CameraProperties",
        "EncoderOdometry",
        "EncoderOdometryProperties",
        "GPSProperties",
        "IMUProperties",
        "LIDARProperties",
        "PoseOdometryProperties",
        "PoseProperties",
    )

    for prop in props_to_check:
        path_parts_in_json = ("value0", "Propeties", prop, 0, "ReaderProperties", "value0")
        path_in_json = ".".join((str(part) for part in path_parts_in_json))
        keys_of_interest = ("Manufacturer", "Model", "SerialNumber")
        with p.open() as f:
            json_data = json.load(f)
            try:
                reader_props = _get_from_dict(data=json_data, keys_seq=path_parts_in_json)
            except (KeyError, IndexError):
                continue

            for reader_prop_idx, reader_prop in enumerate(reader_props):
                if reader_prop["key"] in keys_of_interest:
                    path_in_json = ".".join(str(i) for i in (path_in_json, reader_prop_idx))
                    curr_val = reader_prop["value"]["Value"]
                    if curr_val.lower() == "unknown":
                        raise AssertionError(
                            f'Invalid value "{curr_val}" for path in JSON "{path_in_json}"'
                            f'for JSON file "{p}"'
                        )


def _find_capture_info_json_files() -> Sequence[Path]:
    """Return a list of all the capture_info.json files."""
    root = Path(__file__).absolute().parent.parent
    capture_info_dir = root / "slamcore_utils" / "share" / "capture_infos"
    return list(capture_info_dir.iterdir())


def test_mandatory_keys_in_capture_info_files():
    capture_infos = _find_capture_info_json_files()
    for capture_info_p in capture_infos:
        _check_mandatory_keys(capture_info_p)
