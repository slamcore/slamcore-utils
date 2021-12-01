"""Progress-bar related utilties

TL;DR: import `progress_bar`. This will resolve to tqdm.tqdm if available or _ProgressBar
       otherwise.
"""

import importlib
import sys
import time
from typing import Any, Iterable


class _ProgressBar:
    def __init__(self, iterable: Iterable[Any] = None, total=None, *args, **kargs):
        """
        Poor man's Tqdm.

        This offers a minimal API with __iter__, and update as the two alternatives for
        updating the progress of a task. For a richer experience, just install Tqdm itself

        :param iterable: Iterable to decorate with a progressbar.
                         Leave blank to manually manage the updates.

        :pram total: total number of elements to iterate on. I'll try to infer it from the
                     iterable if possible, e.g. with range(N)
        """
        self._iterable = iterable

        # find total number of elements
        if total is None and iterable is not None:
            try:
                total = len(iterable)  # type: ignore
            except (TypeError, AttributeError):
                total = None
        if total == float("inf"):
            # Infinite iterations, behave same as unknown
            total = None
        self._total = total
        self._cur = 0
        self._max_width = 40
        # used for unknown total
        self._symbols = ["|", "/", "-", "\\"]
        self._last_t = time.time()
        self._thresh_t = 0.3  # seconds
        # used when total is known
        self._symbol_filled = "="
        self._symbol_unfilled = "-"
        self._last_filled_symbol = ">"

        self.update = (
            self._update_known_total if self._total is not None else self._update_unknown_total
        )

    def _update_known_total(self, n=1, *args, **kargs):
        assert self._total is not None
        num_filled: int = round(self._cur * (self._max_width / float(self._total)))

        prog_bar = (
            f"[{self._symbol_filled * (num_filled - 1)}"
            f"{self._last_filled_symbol}{self._symbol_unfilled * (self._max_width - num_filled)}]"
            f" [{self._cur}/{self._total}]"
        )

        self.write(f"{prog_bar}", end="")
        self._cur += n

    def _update_unknown_total(self, n=1, *args, **kargs):
        if not time.time() - self._last_t >= self._thresh_t:
            return

        self._last_t = time.time()
        symbol = self._symbols[self._cur % 4]
        self.write(f" {symbol}", end="")
        self._cur += n

    def __iter__(self) -> Any:
        if self._iterable is None:
            raise RuntimeError(
                "Cannot iterate on progress bar if iterable has not been provided"
            )

        for val in self._iterable:
            yield val
            self.update()

    @classmethod
    def write(cls, s: str, end: str = "\n", *args, **kargs):
        """Print a message via the progress bar (without overlap with bars)."""
        sys.stdout.write("\033[K")  # Clear to the end of line
        sys.stdout.write(f"{s}{end}\r")
        sys.stdout.flush()


try:
    tqdm = importlib.import_module("tqdm")
    progress_bar = tqdm.tqdm  # type: ignore
except:
    progress_bar = _ProgressBar
