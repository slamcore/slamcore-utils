from enum import Enum, auto
from typing import Any, Dict, Tuple

from slamcore_utils.logging import logger


class ConversionFormat(Enum):
    """Conversion formats supported.

    These define the way we parse the conversion configuration file.
    """

    RIGID = auto()
    FLEXIBLE = auto()

    @staticmethod
    def from_name(name: str) -> "ConversionFormat":
        return _names_to_conversion_formats[name]

    def __str__(self) -> str:
        return self.name.lower()

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}.{self.name}"

    @classmethod
    def detect_format(cls, json: Dict[str, Any]) -> Tuple["ConversionFormat", int]:
        """
        Return a tuple of (conversion_format, version_format) by parsing the json contents
        provided. Always resort to the rigid format in case of errors.

        Do remove the conversion meta section from the JSON contents at the end of this call.
        """
        conversion_meta_section = "conversion_meta"
        meta_section = json.pop(conversion_meta_section, None)

        if meta_section is None:
            logger.warning(
                f"No {conversion_meta_section} section. "
                f"Resorting to {ConversionFormat.RIGID} conversion format ..."
            )
            return (ConversionFormat.RIGID, 0)

        try:
            format_str = meta_section["format"]
            version_int = int(meta_section["version"])
        except Exception as e:
            raise RuntimeError(
                f"Invalid {conversion_meta_section} section. Cannot continue."
            ) from e

        try:
            format_and_version = cls.from_name(format_str), version_int
        except KeyError as e:
            raise RuntimeError(f"Unknown conversion format -> {format_str}.") from e

        logger.info(f"Determined conversion format/version: {format_and_version} .")
        return format_and_version


_conversion_formats_to_names = {cf: str(cf) for cf in ConversionFormat}
_names_to_conversion_formats = {v: k for k, v in _conversion_formats_to_names.items()}
