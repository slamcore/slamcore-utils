from pathlib import Path
from shutil import rmtree

from slamcore_utils.logging import logger

"""Filesystem-related utilities."""

def get_ready_output_path(path: Path, overwrite_path: bool = False, logger=logger) -> Path:
    """
    Get a new valid path to write to. Depending on the `overwrite_path` flag it will either:

    - Remove all the contents of the given path (using `safe_rmtree`) and return the given path
      as is.
    - Or, it will use the specified path as a base to find the next suitable filename that's
      not already occupied.
    """
    if path.exists():
        if overwrite_path:
            logger.warning("Output path already exists. Overwriting it...")
            safe_rmtree(path)
        else:
            logger.warning("Path already exists. Will choose another name...")
            path = find_suitable_output_name(path)

    return path


def find_suitable_output_name(path: Path) -> Path:
    """Return a valid, currently non-existent path.

    The current function will return the given path as is if it doesn't exist. Otherwise, it
    will append a suffix to it, such as "_0", and check whether it exists again. if it does it
    will continue appending the trailing digit until it finds a path that doesn't exist.
    """
    # if path doesn't exist, just return it as is.
    if not path.exists():
        return path

    # path exists, amend its suffix and check again
    i = 0
    path = path.parent / (str(path.name) + f"_{i}" + path.suffix)
    while path.exists():
        i += 1
        # Use the path until last "_" as the new name
        # this handles both output_name_0, output_name_1, ...
        # AND output_name_10, output_name_100, ...
        new_name = "_".join(str(path.name).split("_")[:-1])
        path = path.parent / (new_name + f"_{i}" + path.suffix)

    return path


def safe_rmtree(path: Path, not_exist_ok=False):
    """
    Wrapper around the shutil.rmtree command which throws if given a Path that you shouldn't be
    given.

    Use this function in cases where you have to rmtree paths provided by the user. In such
    cases, the user shouldn't provide a `.` as the directory to remove since that could
    potentially wipe out their home directory or important files that it's not meant to. In
    that case while the standard `rmtree` will happily remove all the contents of the given
    path, this function will throw instead.
    """
    if path.resolve() == Path(".").resolve():
        raise RuntimeError(
            "You've specified your current working directory as the directory to remove."
            " Such command will wipe your current directory, and this is not allowed!"
            " Please specify another directory by name."
        )

    try:
        rmtree(path)
    except FileNotFoundError:
        if not_exist_ok:
            pass
        else:
            raise
