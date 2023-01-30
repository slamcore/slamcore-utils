from pathlib import Path

from slamcore_utils.test_utils import UT_Command, get_test_help_cmds, run_UT_commands

exec_name = "slamcore-convert-openloris"

# top-level test data - not specific to this executable
toplevel_test_data = Path(__file__).absolute().parent / "test_data"


def test_convert_openloris_noargs_help():
    """Make sure that no-flags, -h, --help usages all work."""
    run_UT_commands(*get_test_help_cmds(exec_name))


def test_convert_openloris_std_usage():
    """Make sure that a standard conversion works."""
    dataset = f"{toplevel_test_data}/sample_dataset"

    run_UT_commands(
        UT_Command(
            test_name="slamcore_convert_openloris_std_usage",
            command=f"{exec_name} -i {dataset} -o output",
            outputs=[
                Path("output"),
            ],
            stdout_contains=["Converting OpenLORIS Dataset"],
            cd_test_dir=True,
        ),
    )


def test_convert_openloris_overwrite():
    """Make sure the --overwrite flag works."""
    dataset = f"{toplevel_test_data}/sample_dataset"

    run_UT_commands(
        UT_Command(
            test_name="slamcore_convert_openloris_overwrite",
            command=f"{exec_name} -i {dataset} -o output --overwrite",
            outputs=[
                Path("output"),
            ],
            stdout_contains=["Converting OpenLORIS Dataset", "Overwriting"],
            cd_test_dir=True,
        ),
    )


def test_convert_openloris_dont_overwrite():
    """Make sure the --no-overwrite flag works."""
    dataset = f"{toplevel_test_data}/sample_dataset"

    run_UT_commands(
        UT_Command(
            test_name="slamcore_convert_openloris_dont_overwrite",
            command=f"{exec_name} -i {dataset} -o output --no-overwrite",
            outputs=[
                Path("output_0"),
            ],
            stdout_contains=["Converting OpenLORIS Dataset", "Will choose another"],
            cd_test_dir=True,
        ),
    )
