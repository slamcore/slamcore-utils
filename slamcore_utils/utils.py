import importlib
import json
import tempfile
from pathlib import Path
from typing import Any, Callable, Mapping

import pkg_resources

share_dir = Path(pkg_resources.resource_filename("slamcore_utils", "share"))  # type: ignore


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
