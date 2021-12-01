from pathlib import Path

import pytest

from slamcore_utils.fs import safe_rmtree


def test_safe_rmtree_dir(fs):
    # "fs" is the reference to the fake file system
    path = Path("/some/directory/")
    path.mkdir(exist_ok=False, parents=True)
    safe_rmtree(path)
    assert not path.exists() and "Path still exists!"


def test_safe_rmtree_current(fs):
    # "fs" is the reference to the fake file system
    # note: this uses pyfakefs - no interaction with the host filesystem.
    path = Path(".")
    with pytest.raises(RuntimeError):
        safe_rmtree(path)
