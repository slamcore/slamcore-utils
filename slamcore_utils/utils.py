import importlib
import json
import sys
import tempfile
from pathlib import Path
from typing import Any, Callable, Mapping, NoReturn, Sequence

import pkg_resources

from slamcore_utils.logging import logger

share_dir = Path(pkg_resources.resource_filename("slamcore_utils", "share"))  # type: ignore


def valid_path(s: str) -> Path:
    p = Path(s)
    if not p.is_file():
        raise FileNotFoundError(p)

    return p


def format_list(  # pylint: disable=R0913
    items: Sequence[str],
    header: str,
    indent=2,
    bullet_char="-",
    header_sep="=",
    prefix: str = "",
    suffix: str = "",
    list_start_sep="\n\n",
    list_end_sep="\n\n",
) -> str:
    """
    Format and return a string with the corresponding header and all the items each occupying a
    single line and with the specified indentation.
    """
    indentation = " " * indent
    header_indentation = indentation[:-2]
    s = f"{header}: "
    if not items:
        s += " None."
    else:
        s += f"\n{header_indentation}{len(s) * header_sep}"
        s += list_start_sep
        s += "\n".join(f"{indentation}{bullet_char} {item}" for item in items)
    s += list_end_sep
    return f"{prefix}{s}{suffix}"


def format_dict(items: Mapping[Any, Any], align_items: bool = True, **kargs) -> str:
    """
    Utility for formatting a dictionary - similar to print.pformat.

    Accepts mostly the same arguments as format_list.
    """

    items_: Sequence[str]
    if align_items:
        keys_length = max(len(str(key)) for key in items.keys())
        format_ = "{0: <%d}" % keys_length  # pylint: disable=C0209
        items_ = [f"{format_.format(k)}: {v}" for k, v in items.items()]
    else:
        items_ = [f"{k}: {v}" for k, v in items.items()]

    return format_list(items=items_, **kargs)  # type: ignore


def get_default_config(name: str) -> Mapping[str, Any]:
    """Get configuration for the module/executable at hand.

    Configuration should be stored in JSON.
    """
    path = (share_dir / name).with_suffix(".json")
    if not path.is_file():
        raise RuntimeError(f"Cannot find JSON configuration file for {name}")

    with path.open("rb") as f:
        return json.load(f)  # type: ignore


def inform_about_extra_deps(
    pkg_name: str, pkg_to_extra: Mapping[str, str], log_fn: Callable[[str], None] = print
):
    """
    Inform the user about the extra dependencies of this package in case they want to install
    them as well. This info message comes up only on the first run. It may show up again after
    the temp directory of the machine is cleared (e.g. on system restart).
    """

    def wrapper2(fn):
        def wrapper(*args, **kargs):
            marker_file = Path(tempfile.gettempdir()) / f"marker_{pkg_name}"
            # inform the user of the extra dependencies only once.
            if not marker_file.exists():
                extras_to_install = []
                for pkg, extra_key in pkg_to_extra.items():
                    try:
                        importlib.import_module(pkg)
                    except ModuleNotFoundError:
                        extras_to_install.append(extra_key)

                if extras_to_install:
                    log_fn(
                        "Consider installing the optional dependencies for a richer experience - "
                        f"pip install --user {pkg_name}[{','.join(extras_to_install)}]\n"
                    )

                marker_file.touch()

            return fn(*args, **kargs)

        return wrapper

    return wrapper2


def inform_about_app_extras(extras: Sequence[str]) -> NoReturn:
    """Inform the user about *required* package extras and exit."""
    exec_name = Path(sys.argv[0]).stem
    extras_str = ",".join(extras)
    logger.error(
        "\n\nYou have to install the"
        f' {extras_str} {"extra" if len(extras) == 1 else "extras"} for {exec_name} to'
        ' work.\nWith pip, you can do it with something like: "pip3 install'
        f' slamcore_utils[{extras_str}]"\nExiting.'
    )
    sys.exit(1)


def xor(*args) -> bool:
    """True if exactly one of the arguments of the iterable is True.

    >>> xor(0,1,0,)
    True
    >>> xor(1,2,3,)
    False
    >>> xor(False, False, False)
    False
    >>> xor("kalimera", "kalinuxta")
    False
    >>> xor("", "a", "")
    True
    >>> xor("", "", "")
    False
    """
    return sum([bool(i) for i in args]) == 1
