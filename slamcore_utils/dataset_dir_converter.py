from abc import abstractmethod
from pathlib import Path

from slamcore_utils.data_converter import DataConverter


class DatasetDirConverter(DataConverter):
    """Base class for converters of datasets that sit in a directory structure."""

    def __init__(self, *, in_path: Path, **kargs):
        """
        :param in_path: dataset top-level directory for the particular converter at hand to
                        pull all the required data from.

        See the DataConverter parent class for more arguments that this requires
        """
        self._in_path = in_path

        super().__init__(**kargs)

    @abstractmethod
    def _convert(self):
        return True
