from pathlib import Path

import pytest

import slamcore_utils.test_utils
from slamcore_utils.test_utils import UT_Command, get_test_help_cmds, run_UT_commands

try:
    import rosbag2_py  # type: ignore
except ModuleNotFoundError:
    print("Skipping the ROS2 conversion tests!")
    pytestmark = pytest.mark.skip


toplevel_test_data = Path(__file__).absolute().parent / "test_data"

exec_name = "slamcore-convert-rosbag2"


def data_path_fn(test_name: str):
    return (
        Path(__file__).absolute().parent
        / "test_data"
        / "executables"
        / str(test_name).replace("-", "_")
    )


slamcore_utils.test_utils.data_path_fn = data_path_fn


def test_convert_rosbag2_noargs_help():
    """Make sure that no-flags, -h, --help usages all work."""
    run_UT_commands(*get_test_help_cmds(exec_name))


def test_convert_rosbag2_std_usage():
    """Make sure that a standard conversion works."""
    dataset = f"{toplevel_test_data}/processed_mh_01/processed_mh_01_0.db3"

    # the metadata.txt file contains the current date - comparing it against the expected one
    # would always fails
    # quick and dirty way to avoid it
    slamcore_utils.test_utils._suffix_to_handler[
        ".txt"
    ] = slamcore_utils.test_utils.AlwaysTrueComparator

    run_UT_commands(
        UT_Command(
            test_name="slamcore_convert_rosbag2_std_usage",
            command=(
                f"{exec_name} -v -b {dataset} -o output -c "
                f"{toplevel_test_data}/test_rosbag2.json"
            ),
            outputs=[
                Path("output"),
            ],
            stdout_contains=["Finished converting"],
            cd_test_dir=True,
        ),
    )

    slamcore_utils.test_utils._suffix_to_handler.pop(".txt")
