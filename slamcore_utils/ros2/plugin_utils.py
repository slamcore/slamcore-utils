import operator
from functools import reduce
from pathlib import Path
from runpy import run_path
from typing import Sequence, cast

import pkg_resources

from slamcore_utils.logging import logger

from .ros2_converter_plugin import Ros2ConverterPlugin


def get_internal_plugins_dir() -> Path:
    """
    Get the path to the internal ROS 2 plugins.

    This depends on whether the user has installed this package and executed the installed
    script or whether they've directly executed the script via `python3 -m`
    """
    if __package__:
        return (
            Path(pkg_resources.resource_filename(__package__.split(".")[0], "ros2"))
            / "ros2_converter_plugins"
        )
    else:
        return Path(__file__).absolute().parent / "ros2_converter_plugins"


def load_converter_plugins_from_multiple_files(
    converter_plugin_paths: Sequence[Path],
    raise_on_error: bool = True,
) -> Sequence[Ros2ConverterPlugin]:
    if converter_plugin_paths:
        converter_plugins = cast(
            Sequence[Ros2ConverterPlugin],
            reduce(
                operator.concat,
                (
                    load_converter_plugins(plugin_path, raise_on_error)
                    for plugin_path in converter_plugin_paths
                ),
            ),
        )
    else:
        return []

    # Sanity check, each one of converter_plugins var is of the right type
    for cp in converter_plugins:
        if not isinstance(cp, Ros2ConverterPlugin):
            raise RuntimeError(
                "One of the specified converter plugins is not of type "
                "Ros2ConverterPlugin, cannot proceed"
            )

    return converter_plugins


def load_converter_plugins(
    plugin_path: Path, raise_on_error: bool
) -> Sequence[Ros2ConverterPlugin]:
    """Load all the ROS 2 Converter Plugins specified in the given plugin python module.

    In case of errors print the right error messages acconrdingly.
    """
    logger.debug(f"Loading ROS 2 converter plugin from {plugin_path} ...")
    try:
        ns_dict = run_path(str(plugin_path))
    except BaseException as e:
        e_str = (
            "Failed to load ROS 2 converter plugin from "
            f"{plugin_path.relative_to(plugin_path.parent)}\n\nError: {e}\n\n"
        )
        if raise_on_error is True:
            raise RuntimeError(f"{e_str}Exiting ...") from e
        else:
            logger.debug(e_str)
            return []

    # Sanity check, specified converter_plugins var
    if "converter_plugins" not in ns_dict:
        raise RuntimeError(
            f"No converter plugins were exported in specified plugin -> {plugin_path}\n",
            (
                'Make sure that you have initialized a variable named "converter_plugins" '
                "at the top-level of the said plugin."
            ),
        )

    converter_plugins = ns_dict["converter_plugins"]

    # Sanity check, converter_plugins var is of the right type
    if not isinstance(converter_plugins, Sequence):
        raise RuntimeError(
            f"Found the converter_plugins at the top-level of the plugin ({plugin_path}) "
            "but that variable is not of type Sequence. Its value is "
            f"{converter_plugins} and of type {type(converter_plugins)}"
        )

    return converter_plugins
