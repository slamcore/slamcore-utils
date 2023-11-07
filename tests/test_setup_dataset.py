import os
import re
import shutil
import signal
import tempfile
import time
from pathlib import Path

import pexpect
import pytest

from .key_inputs import KeyInputs

# 7-bit C1 ANSI sequences
# Get rid of junk control codes when comparing strings
ansi_escape = re.compile(
    r"""
    \x1B  # ESC
    (?:   # 7-bit C1 Fe (except CSI)
        [@-Z\\-_]
    |     # or [ for CSI, followed by a control sequence
        \[
        [0-?]*  # Parameter bytes
        [ -/]*  # Intermediate bytes
        [@-~]   # Final byte
    )
""",
    re.VERBOSE,
)


@pytest.fixture()
def proc() -> pexpect.spawn:
    # poetry doesn't relay exit statuses right now -
    # https://github.com/python-poetry/poetry/pull/2904
    # Let's call the script directly
    executable = (
        Path(__file__).absolute().parent.parent
        / "slamcore_utils"
        / "scripts"
        / "setup_dataset.py"
    )
    return pexpect.spawn(str(executable))


def post_checks(proc: pexpect.spawn):
    time.sleep(0.3)
    assert proc.isalive() is False
    proc.close()
    assert proc.exitstatus is 0


def assert_in_answer(proc: pexpect.spawn, *expected_str: str):
    answer = proc.read_nonblocking(10000, timeout=3)
    assert isinstance(answer, bytes)
    answer = ansi_escape.sub("", answer.decode("utf-8"))
    for expected in expected_str:
        assert expected in answer


def expect(proc, *args, **kargs):
    return proc.expect(*args, **kargs, timeout=2)


class TestSetupDatasetExecutable:
    """Tests suite for the setup_dataset.py interactive script."""

    @pytest.mark.parametrize(
        ("expected_link", "pos"),
        [
            (
                "https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets",
                0,
            ),
            ("https://lifelong-robotic-vision.github.io/dataset/scene", 1),
            ("https://lifelong-robotic-vision.github.io/dataset/scene", 2),
            ("https://vision.in.tum.de/data/datasets/visual-inertial-dataset", 3),
            ("https://vision.in.tum.de/data/datasets/visual-inertial-dataset", 4),
        ],
    )
    def test_provide_instructions_if_not_downloaded(
        self, proc: pexpect.spawn, expected_link, pos
    ):
        """
        Test that if the dataset has not been downloaded the user is referred to the right
        link.
        """

        expect(proc, "which one of these datasets")
        proc.send(KeyInputs.DOWN * pos + KeyInputs.ENTER)
        expect(proc, "you already downloaded the")
        proc.send("n")

        expect(proc, re.escape(expected_link))

        post_checks(proc)

    def test_handle_ctrlc(self, proc: pexpect.spawn):
        """Make sure we don't crash on ctrl-c."""
        expect(proc, "which one of these datasets")
        proc.kill(signal.SIGINT)
        time.sleep(0.3)
        assert proc.isalive() is False

        proc.close()
        assert proc.exitstatus == 1

    def test_catch_invalid_euroc_dataset(self, proc: pexpect.spawn):
        """Catch invalid euroc datasets."""
        with tempfile.TemporaryDirectory() as tmpdir:
            expect(proc, "which one of these datasets")
            proc.send(KeyInputs.ENTER)
            expect(proc, "you already downloaded the")
            proc.send("y")
            expect(proc, "write the path to the downloaded dataset")
            proc.send(tmpdir + KeyInputs.ENTER)
            time.sleep(2.0)

            assert proc.isalive() is False

            expected_answer = "does not seem to be a valid EuRoC format"
            assert_in_answer(proc, expected_answer)
            proc.close()
            assert proc.exitstatus == 1

    def test_guess_mav0(self, proc: pexpect.spawn):
        """
        Make sure that if the user gives the parent directory, we successfully pick the
        <...>/mav0 instead.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            expect(proc, "which one of these datasets")
            proc.send(KeyInputs.ENTER)
            expect(proc, "you already downloaded the")
            proc.send("y")
            expect(proc, "write the path to the downloaded dataset")
            tmpdir_path = Path(tmpdir)
            dataset_path = tmpdir_path / "mav0"
            ir0 = dataset_path / "ir0"
            ir0.mkdir(parents=True, exist_ok=False)

            proc.send(tmpdir + KeyInputs.ENTER)
            time.sleep(2.0)

            assert proc.isalive() is False

            assert_in_answer(proc, "Dataset is now ready", "mav0")
            proc.close()
            assert proc.exitstatus == 0

    @pytest.mark.parametrize(
        ("fname", "errmsg"),
        [
            ("a_file", "should be a directory"),
            ("a_file.zip", "first uncompress the dataset"),
        ],
    )
    def test_catch_file_instead_of_dir(self, proc: pexpect.spawn, fname: str, errmsg: str):
        """Catch if given files instead of directories."""
        with tempfile.TemporaryDirectory() as tmpdir:
            expect(proc, "which one of these datasets")
            proc.send(KeyInputs.DOWN + KeyInputs.DOWN + KeyInputs.ENTER)
            expect(proc, "you already downloaded the")
            proc.send("y")
            expect(proc, "write the path to the downloaded dataset")
            tmpdir_path = Path(tmpdir)
            file = tmpdir_path / fname
            file.touch()
            proc.send(str(file))
            time.sleep(2.0)

            assert proc.isalive() is True

            assert_in_answer(proc, errmsg)
            proc.kill(signal.SIGTERM)
            proc.close()

    def test_embed_capture_info(self, proc: pexpect.spawn):
        """Test the conversion"""
        with tempfile.TemporaryDirectory() as tmpdir:
            expect(proc, "which one of these datasets")
            proc.send(KeyInputs.DOWN * 3 + KeyInputs.ENTER)
            expect(proc, "you already downloaded the")
            proc.send("y")
            expect(proc, "write the path to the downloaded dataset")
            tmpdir_path = Path(tmpdir)
            dataset_path = tmpdir_path
            ir0 = dataset_path / "ir0"
            ir0.mkdir(parents=True, exist_ok=False)

            proc.send(tmpdir + KeyInputs.ENTER)
            time.sleep(2.0)

            assert proc.isalive() is False

            assert_in_answer(proc, "Dataset is now ready")
            assert (dataset_path / "capture_info.json").exists()
            proc.close()
            assert proc.exitstatus == 0

    def test_embed_with_conversion(self):
        """Make sure converting and embedding a capture_info.json works."""
        executable = (
            Path(__file__).absolute().parent.parent
            / "slamcore_utils"
            / "scripts"
            / "setup_dataset.py"
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            proc = pexpect.spawn(str(executable))

            tmpdir_path = Path(tmpdir)
            dataset_path = Path(
                shutil.copytree(
                    src=Path(__file__).absolute().parent / "test_data" / "sample_dataset",
                    dst=tmpdir_path / "sample_dataset",
                )
            )

            expect(proc, "which one of these datasets")
            proc.send(KeyInputs.DOWN + KeyInputs.ENTER)
            expect(proc, "you already downloaded the")
            proc.send("y")
            expect(proc, "write the path to the downloaded dataset")
            proc.send(str(dataset_path) + KeyInputs.ENTER)
            expect(proc, "dataset requires conversion")
            proc.send("y")
            time.sleep(5.0)

            assert proc.isalive() is False

            converted_dataset = dataset_path.with_name("sample_dataset_converted")
            assert_in_answer(proc, "Dataset is now ready")
            assert (converted_dataset / "capture_info.json").exists()
            proc.close()
            assert proc.exitstatus == 0

    @pytest.mark.parametrize(
        ("overwrite", "converted_dataset_name"),
        [(True, "sample_dataset_converted"), (False, "sample_dataset_converted_0")],
    )
    def test_embed_with_conversion_overwrite(self, overwrite, converted_dataset_name):
        """Make sure converting and embedding a capture_info.json works."""
        executable = (
            Path(__file__).absolute().parent.parent
            / "slamcore_utils"
            / "scripts"
            / "setup_dataset.py"
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            proc = pexpect.spawn(str(executable))

            tmpdir_path = Path(tmpdir)
            dataset_path = Path(
                shutil.copytree(
                    src=Path(__file__).absolute().parent / "test_data" / "sample_dataset",
                    dst=tmpdir_path / "sample_dataset",
                )
            )
            dataset_path.with_name("sample_dataset_converted").mkdir()

            expect(proc, "which one of these datasets")
            proc.send(KeyInputs.DOWN + KeyInputs.ENTER)
            expect(proc, "you already downloaded the")
            proc.send("y")
            expect(proc, "write the path to the downloaded dataset")
            proc.send(str(dataset_path) + KeyInputs.ENTER)
            expect(proc, "overwrite it")
            proc.send("y" if overwrite is True else "n")
            time.sleep(5.0)

            assert proc.isalive() is False

            assert_in_answer(proc, "Dataset is now ready")
            converted_dataset = dataset_path.with_name(converted_dataset_name)
            assert (converted_dataset / "capture_info.json").exists()
            proc.close()
            assert proc.exitstatus == 0
