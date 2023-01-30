import abc
import filecmp
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from subprocess import PIPE
from typing import Dict, List, Optional, Sequence, Type, Union

import numpy as np

from slamcore_utils.fs import safe_rmtree
from slamcore_utils.logging import logger
from slamcore_utils.string import get_rand_string


def data_path_fn(test_name: str):
    """
    Declare the data path for the tests at hand.

    Replace this with a function that returns the root tests location for your own module.
    """
    return (
        Path(__file__).absolute().parent.parent
        / "tests"
        / "test_data"
        / "executables"
        / str(test_name).replace("-", "_")
    )


class FileComparator(abc.ABC):
    @abc.abstractmethod
    def __call__(self, file: Path, expected_file: Path):
        raise


class CsvFileComparator(FileComparator):
    def __call__(self, file: Path, expected_file: Path):
        """Compare two csv files.

        Assert that their contents are close. We assume that these files can be loaded via
        np.loadtxt.
        """
        try:
            # load and try comparing as numeric arrays.
            # will fail if the values in the CSV are not exclusively numeric
            arr = np.loadtxt(fname=file, delimiter=",", skiprows=1)
            arr_expected = np.loadtxt(fname=expected_file, delimiter=",", skiprows=1)
            assert np.allclose(arr, arr_expected), (
                "Output file contents don't match the expected contents."
                f"Take a look at the following files: {file.absolute()} | {expected_file.absolute()}"
            )
        except ValueError:
            logger.warning(
                "Cannot currently compare CSVs as they have non-numeric values\n"
                f"\t- {file}\n"
                f"\t- {expected_file}\n"
            )


class GenericComparator(FileComparator):
    def __call__(self, file: Path, expected_file: Path):
        """
        Assert that the contents of the two given files are the same using `filecmp.cmp`.
        """
        assert filecmp.cmp(file, expected_file, shallow=False), (
            "Output file contents don't match the expected contents. "
            f"Take a look at the following files:{file.absolute()} {expected_file.absolute()}"
        )


class AlwaysTrueComparator(FileComparator):
    def __call__(self, file: Path, expected_file: Path):
        assert True


_suffix_to_handler: Dict[str, Type[FileComparator]] = {
    ".csv": CsvFileComparator,
    # # do not try to compare PDF / PNG files - too fragile
    ".pdf": AlwaysTrueComparator,
    ".png": AlwaysTrueComparator,
}


def compare_dirs(directory: Path, expected_directory: Path):
    """Iteratively compare two directory structures.

    Delegate the actual comparison to the corresponding handler for each file.
    E.g., for CSVs it will call the `compare_csvs` method on these files.
    """
    directory = directory.resolve()

    # make sure that they have the same number of files and the same names
    iterdir = list(directory.iterdir())
    expected_iterdir = list(expected_directory.iterdir())
    assert len(iterdir) == len(
        expected_iterdir
    ), "Mismatch between number of files in generated and expected directories, len(iterdir) != len(expected_iterdir)"

    for file in iterdir:

        # find the corresponding file in the expected directory
        path_full = directory / file
        expected_path_full = expected_directory / file.name

        if path_full.is_dir():
            assert (
                expected_path_full.is_dir()
            ), f"Expected directory was not generated as expected -> {expected_path_full}"

            # traverse this directory as well
            compare_dirs(path_full, expected_path_full)
        else:
            assert (
                expected_path_full.is_file()
            ), f"Expected file was not generated as expected -> {expected_path_full}"
            comparator = _suffix_to_handler.get(file.suffix, GenericComparator)()
            comparator(path_full, expected_path_full)


@dataclass
class UT_Command:
    # name of the test case at hand
    test_name: str
    # Either specify exec_path and args above OR command
    # Specifying the command will override the exec_path and args fields and vice-versa
    exec_path: Optional[Union[Path, str]] = None  # type: ignore
    args: List[str] = field(default_factory=list)
    command: Optional[str] = None
    # List of files and directories it outputs
    outputs: List[Union[str, Path]] = field(default_factory=list)  # type: ignore
    # Expected return code
    return_code: int = 0
    # Whether its stderr, stdout contains specific strings
    stdout_contains: Optional[List[str]] = None
    stderr_contains: Optional[List[str]] = None
    cd_test_dir: bool = False

    def __post_init__(self):
        # effectively always use self.exec_path and self.args instead of self.command.
        # however, we fill self.command anyway to potentially use it in debugging.
        if self.exec_path is not None and self.args:
            assert self.command is None, (
                "You have specified both the exec_path,args pair as well as the command string."
                " You have to specify only one of the above"
            )
            if not self.args:
                self.command = str(self.exec_path)
            else:
                self.command = f'{self.exec_path}  {" ".join(self.args)}'

            self.exec_path: Path = self.exec_path

        else:
            assert self.command is not None, (
                "You have specified both the exec_path,args pair as well as the command string."
                " You have to specify only one of the above"
            )
            exec_path_and_args = self.command.split(" ")
            self.exec_path = Path(exec_path_and_args[0])
            self.args = exec_path_and_args[1:]

        self.test_name = self.test_name.replace("-", "_")
        self.outputs: List[Path] = [Path(output) for output in self.outputs]

    def run(self):
        """Run the test case."""
        exec_path = self.exec_path
        args = self.args
        outputs = self.outputs
        expected_return_code = self.return_code
        stderr_contains = self.stderr_contains
        stdout_contains = self.stdout_contains
        cd_test_dir = self.cd_test_dir
        data_path = data_path_fn(test_name=self.test_name)
        # output paths that are relative must be relative with regards to the "test_data"
        # directory for the test case at hand.
        for i in range(len(outputs)):
            if not outputs[i].is_absolute():
                outputs[i] = data_path / outputs[i]

        # mark the files to be generated by the test case -------------------------------------
        to_remove: List[Path] = []
        to_restore: List[Path] = []
        to_restore_backup: List[Path] = []
        for output in outputs:
            if output.exists():
                to_restore.append(output)
                backup_dir = (
                    Path(tempfile.gettempdir()) / f"slamcore_utils_{get_rand_string(len=5)}"
                )
                shutil.copytree(output, backup_dir)
                to_restore_backup.append(Path(backup_dir))
            else:
                to_remove.append(output)

        s = "Will automatically remove the following files/directories\n\n"
        s += "\n".join(map(str, to_remove))
        s += "\n"
        s += "\nWill preserve the following files/directories\n\n"
        s += "\n".join(map(str, to_restore))
        s += "\n"
        logger.info(s)

        try:
            # change directory? ---------------------------------------------------------------
            if cd_test_dir:
                try:
                    oldpwd = Path(".").absolute()
                    os.chdir(data_path)
                except FileNotFoundError:
                    raise RuntimeError(
                        f'Could not `cd` into "{data_path}".\n'
                        " Please make sure that path is there or specify that you "
                        "don't need an output directory - see `cd_test_dir` flag"
                    )

            # run it --------------------------------------------------------------------------
            cmd = [exec_path, *args]
            proc = subprocess.run(cmd, stdout=PIPE, stderr=PIPE, check=False)
            stdout, stderr = proc.stdout, proc.stderr
            stdout = stdout.decode("utf-8")
            stderr = stderr.decode("utf-8")

            # assert stdout/err ---------------------------------------------------------------
            if stdout_contains is not None:
                for stdout_sample in stdout_contains:
                    assert stdout_sample in stdout, (
                        "Required stdout string not found, "
                        f"[{stdout}] should have contained [{stdout_sample}]"
                    )

            if stderr_contains is not None:
                for stderr_sample in stderr_contains:
                    assert stderr_sample in stderr, (
                        "Required stderr string not found, "
                        f"[{stderr}] should have contained [{stderr_sample}]"
                    )

            # assert error code ---------------------------------------------------------------
            if proc.returncode != expected_return_code:
                assert False, (
                    "Got an unexpected error code, "
                    f"{proc.returncode} instead of the expected {expected_return_code}\n\n"
                    f"stdout:\n\n{stdout}\n\nstderr:\n\n{stderr}"
                )

            # assert all the required files are actually there --------------------------------
            for output in outputs:
                expected_output = output.parent / f"expected_{output.name}"
                # make sure that the expected output files were generated
                assert (
                    output.exists()
                ), f"Output file/directory {output} was not generated as was expected"

                # if there's an expected output file then make sure that the output is
                # identical ot the expected output. For example, for a file named
                # `path/to/output/<filename>`, its optional expected output is
                # `/path/to/output/expected_<filename>`
                if expected_output.exists():
                    if output.is_file() and expected_output.is_file():
                        # find the appropriate function to compare these files with,
                        # otherwise, resort to the default handler
                        comparator = _suffix_to_handler.get(output.suffix, GenericComparator)()
                        comparator(output, expected_output)

                    elif output.is_dir() and expected_output.is_dir():
                        compare_dirs(output, expected_output)
                    else:
                        raise RuntimeError(
                            "Mismatched types of the expected and the generated output. One is a file, the other a directory"
                        )
                else:
                    logger.info(
                        f"No expected file/directory found to compare the generated output -> {output}"
                    )

        finally:
            # Cleanup created files -----------------------------------------------------------
            for output in outputs:
                try:
                    if output.is_file():
                        output.unlink()
                    else:
                        safe_rmtree(output)
                except:
                    pass

            # Restore files that were already there -------------------------------------------
            for output_i, output in enumerate(to_restore):
                try:
                    if output.is_file():
                        shutil.copy2(to_restore_backup[output_i], output)
                        to_restore_backup[output_i].unlink()
                    else:
                        shutil.copytree(to_restore_backup[output_i], output)
                        safe_rmtree(to_restore_backup[output_i])
                except:
                    pass

            # change back to starting dir -----------------------------------------------------
            if cd_test_dir:
                os.chdir(oldpwd)  # type: ignore


# Helper commands around UT_Command -----------------------------------------------------------
def get_test_noargs_cmds(exec_path) -> UT_Command:
    return UT_Command(
        test_name=f"{exec_path}_noargs",
        exec_path=exec_path,
        args=[],
        outputs=[],
        return_code=2,
    )


def get_test_help_cmds(exec_path) -> Sequence[UT_Command]:
    return (
        UT_Command(
            test_name=f"{exec_path}_help", exec_path=exec_path, args=["-h"], outputs=[]
        ),
        UT_Command(
            test_name=f"{exec_path}_help2", exec_path=exec_path, args=["--help"], outputs=[]
        ),
    )


def get_std_usage_cmds(exec_path) -> Sequence[UT_Command]:
    """Wrapper around get_test_noargs_cmds and get_test_help_cmds."""
    return (
        *get_test_help_cmds(exec_path),
        get_test_noargs_cmds(exec_path),
    )


def run_UT_commands(*commands: UT_Command):
    """Process all the UT commands."""
    for command in commands:
        command.run()
