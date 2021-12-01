"""Helper methods to utilize to use with the argparse module for command line options parsing. """

from argparse import ArgumentParser, ArgumentTypeError
from pathlib import Path
from typing import Any, Dict

help_msgs: Dict[str, str] = {}
arg_defaults: Dict[str, Any] = {}


def existing_path(path: str) -> Path:
    p = Path(path).expanduser()
    if not p.exists():
        raise ArgumentTypeError(f"Path doesn't exist -> {path}")

    return p


def existing_file(path: str) -> Path:
    p = existing_path(path)
    if not p.is_file():
        raise ArgumentTypeError(f"Path is not a file -> {path}")

    return p


def existing_dir(path: str) -> Path:
    p = existing_path(path)
    if not p.is_dir():
        raise ArgumentTypeError(f"Path is not a directory -> {path}")

    return p


def non_existing_path(path: str) -> Path:
    p = Path(path).expanduser()
    if p.exists():
        raise ArgumentTypeError(f"Path already exists -> {path}")

    return p


def non_existing_file(path: str) -> Path:
    return non_existing_path(path)


def non_existing_dir(path: str) -> Path:
    return non_existing_path(path)


def add_bool_argument(
    parser: ArgumentParser, arg_name: str, default: bool = None, true_help: str = None
):
    """Add a boolean CLI argument to the given ArgumentParser object.

    The flag defaults to *False* if no other is specified

    Usage::


    >>> import argparse
    >>> parser = argparse.ArgumentParser(description="Example1")
    >>> add_bool_argument(parser, "flag1")
    >>> add_bool_argument(parser, "flag2", default=True)
    >>> add_bool_argument(parser, "flag3", default=False)
    >>> add_bool_argument(parser, "flag4", true_help="help4")
    >>> add_bool_argument(parser, "a-flag")
    >>> config = parser.parse_args(["--flag1"]) # basic
    >>> config.flag1
    True
    >>> config = parser.parse_args(["--no-flag1"])
    >>> config.flag1
    False

    >>> config = vars(parser.parse_args([])) # defaults
    >>> config["flag1"] == False
    True
    >>> config["flag2"] == True
    True
    >>> config["flag3"] == False
    True
    >>> config["flag4"] == False
    True
    >>> "help4" in parser.format_help()
    True

    >>> parser.parse_args(["--flag1", "--no-flag1"]) # exception
    Traceback (most recent call last):
    ...
    SystemExit: 2

    >>> config = vars(parser.parse_args(["--a_flag"])) # exception
    >>> config["a_flag"] == True
    True

    """

    # can't handle these cases for now - will throw on 'eval'
    arg_name = arg_name.replace("-", "_")

    def _format(s: str):
        return "{}{}".format(s, arg_name)

    group = parser.add_mutually_exclusive_group()

    true_help = true_help if default == False else f"{true_help} [default]"
    false_help = "" if default == True else "[default]"

    group.add_argument(_format("--"), dest=arg_name, action="store_true", help=true_help)
    group.add_argument(_format("--no-"), dest=arg_name, action="store_false", help=false_help)

    if default:
        eval("group.set_defaults({}=default)".format(arg_name))
