import logging
from enum import Enum, auto
from pathlib import Path

from slamcore_utils.progress_bar import progress_bar

class LoggingProfile(Enum):
    """Offers the available logging profiles that the slam_utils apps can run under.

    Add more if needed (e.g., log simultaneously to a file or to syslog) and adjust their
    behavior in the :py:func:`setup_pkg_logging` accordingly.
    """

    LVL_ONLY = auto()
    LVL_TIME = auto()
    LVL_FNAME = auto()
    LVL_FNAME_TIME = auto()
    LVL_FULLFNAME = auto()
    LVL_FNAME_LNO = auto()
    ALL_DEBUG = auto()
    MSG_ONLY = auto()


class TqdmLoggingHandler(logging.Handler):
    """Custom handler to also respect the progress bar during writing."""

    def __init__(self, level=logging.NOTSET):
        super().__init__(level)

    def emit(self, record):
        try:
            msg = self.format(record)
            progress_bar.write(msg)
            self.flush()
        except Exception:
            self.handleError(record)


# Set this variable accordingly whenever you want to run with a different
# logging profile
_LOGGING_PROFILE = LoggingProfile.LVL_TIME


def _setup_logging(name: str, format_: str, level: int):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    f = logging.Formatter(format_, datefmt="%H:%M:%S")
    ch = TqdmLoggingHandler()
    ch.setFormatter(f)
    logger.addHandler(ch)


def setup_pkg_logging(
    fname: str, profile: LoggingProfile = _LOGGING_PROFILE
) -> logging.Logger:
    """
    Setup a logger based on the filename of the current file.

    Usage::


    >>> __LOGGING_PROFILE = LoggingProfile.LVL_ONLY
    >>> l = setup_pkg_logging("file.py")
    >>> isinstance(l, logging.Logger)
    True
    >>> l.getEffectiveLevel() == logging.INFO
    True
    >>> l.name
    'file.py'

    >>> l2 = setup_pkg_logging("dir1/dir2/file.py")
    >>> isinstance(l, logging.Logger)
    True
    >>> l2.getEffectiveLevel() == logging.INFO
    True
    >>> l2.name
    'file.py'
    """

    format_: str = ""
    fname_short = str(Path(fname).name)
    use_full_name = False
    level = logging.INFO

    # Determine on the logging format
    if profile == LoggingProfile.LVL_ONLY:
        format_ = "%(levelname)-8s | %(message)s"
    elif profile == LoggingProfile.LVL_TIME:
        format_ = "%(asctime)s | %(levelname)-8s -  %(message)s"
    elif profile == LoggingProfile.LVL_FNAME:
        format_ = "%(levelname)-8s | %(name)s - %(message)s"
    elif profile == LoggingProfile.LVL_FNAME_TIME:
        format_ = "%(asctime)s | %(levelname)-8s | %(name)s - %(message)s"
    elif profile == LoggingProfile.LVL_FULLFNAME:
        format_ = "%(levelname)-8s | %(name)s - %(message)s"
        use_full_name = True
    elif profile == LoggingProfile.LVL_FNAME_LNO:
        format_ = "%(levelname)-8s | %(name)s:%(lineno)-3s - %(message)s"
    elif profile == LoggingProfile.ALL_DEBUG:
        format_ = "%(asctime)s | %(levelname)-8s | %(name)s:%(lineno)-3s - %(message)s"
        level = logging.DEBUG
        use_full_name = True
    elif profile == LoggingProfile.MSG_ONLY:
        format_ = "%(message)s"
    else:
        raise RuntimeError(f"Unhandled LoggingProfile given: {profile}")

    # Initialise the logger
    name = fname if use_full_name else fname_short
    logger = logging.getLogger(name)
    _setup_logging(name, format_=format_, level=level)

    return logger


_pkg_logger_name = Path(__file__).parent.name
logger = logging.getLogger(_pkg_logger_name)
setup_pkg_logging(_pkg_logger_name)
