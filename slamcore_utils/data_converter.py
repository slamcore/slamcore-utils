from abc import ABC, abstractmethod
from pathlib import Path


class DataConverter(ABC):
    """Base class for all the converters.

    Converter classes are responsible for converting a particular stream of data to the
    corresponding file/directory hierarchy for the SLAMcore Dataset Format. They implement the
    `_convert` method.
    """

    def __init__(self, *, out_path: Path, **kargs):
        """
        :param out_path: dataset top-level directory for the particular converter at hand to
                         fill.
        """
        self._out_path = out_path

        # if by this point there are still unused arguments raise an exception
        if kargs:
            raise RuntimeError(
                f"Unknown arguments passed to {self.__class__.__name__}: {kargs}"
            )

    def __call__(self):
        self._convert()

    @abstractmethod
    def _convert(self) -> None:
        raise NotImplementedError
