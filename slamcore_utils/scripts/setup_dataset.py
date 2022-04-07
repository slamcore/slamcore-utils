#!/usr/bin/env python3

import shutil
import sys
import traceback
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, Type

import questionary as q

from slamcore_utils import DataConverter, OpenLORISConverter, __version__
from slamcore_utils.fs import get_ready_output_path
from slamcore_utils.utils import get_default_config, inform_about_extra_deps, share_dir


# class to manage a sample dataset
class Dataset(Enum):
    Euroc = "EuRoC"
    OpenlorisMain = "OpenLORIS Main"
    OpenlorisMarket = "OpenLORIS Market"
    Tum512 = "TUM VI [512px]"
    Tum1024 = "TUM VI [1024px]"

    @property
    def dataset_link(self):
        return _dataset_links[self.name]

    @staticmethod
    def from_fullname(fullname: str):
        return _fullname_to_dataset[fullname]

    @property
    def manual_dl_remarks(self) -> Optional[str]:
        return _manual_dl_remarks.get(self.name)

    def requires_conversion(self) -> bool:
        return self in _dataset_to_converter

    @property
    def capture_info(self) -> Path:
        path = capture_info_dir / f"{self.name.lower()}_capture_info.json"
        if not path.is_file():
            raise RuntimeError(
                f"Programming error: capture_info.json file for {self.name} does not exist"
            )

        return path


# default configuration -----------------------------------------------------------------------
exec_name = Path(__file__).stem
def_config = get_default_config(exec_name)
_dataset_links: Dict[str, str] = def_config["dataset_links"]
_manual_dl_remarks: Dict[str, str] = def_config["manual_dl_remarks"]


_dataset_to_converter: Dict[Dataset, Type[DataConverter]] = {
    Dataset.OpenlorisMain: OpenLORISConverter,
    Dataset.OpenlorisMarket: OpenLORISConverter,
}

_fullname_to_dataset: Dict[str, Dataset] = {d.value: d for d in Dataset}

capture_info_dir = share_dir / "capture_infos"

# sanity check - check there's a capture_info for each dataset, beforehand.
_ = [d.capture_info for d in Dataset]


# helper methods ------------------------------------------------------------------------------
def user_error(msg: str):
    q.print(msg, style="bold fg:red")


def user_note(msg: str):
    q.print(msg, style="fg:grey")


def user_info(msg: str):
    q.print(msg)


def user_warning(msg: str):
    q.print(msg, style="fg:orange")


def user_success(msg: str):
    q.print(msg, style="bold green")


def catch_ctrlc_n_exit(fn):
    def wrapper(*args, **kargs):
        try:
            return fn(*args, **kargs)
        except KeyboardInterrupt:
            user_error("Exiting...")
            return 1

    return wrapper


# main ----------------------------------------------------------------------------------------
@catch_ctrlc_n_exit
@inform_about_extra_deps(
    pkg_name="slamcore_utils",
    pkg_to_extra={
        "tqdm": "tqdm",
    },
    log_fn=user_note,
)
def _main():
    # setup -----------------------------------------------------------------------------------
    user_note(
        f"{exec_name} - {__version__}\nSetup a sample dataset for offline SLAM evaluation\n"
    )

    # select dataset --------------------------------------------------------------------------
    dataset_fullname = q.select(
        "which one of these datasets do you want to set up?",
        choices=tuple(_fullname_to_dataset.keys()),
    ).unsafe_ask()

    # download dataset / find the dataset path ------------------------------------------------
    dataset: Dataset = Dataset.from_fullname(fullname=dataset_fullname)
    have_dl_dataset = q.confirm(
        f"Have you already downloaded the {dataset.value} dataset"
    ).unsafe_ask()

    if have_dl_dataset:

        class InputDirectoryValidator(q.Validator):
            def validate(self, document):
                p = Path(document.text.strip()).expanduser()
                if not p.is_dir():
                    if p.suffix in [".zip", ".7z", ".tar"]:
                        raise q.ValidationError(
                            message=f"You have to first uncompress the dataset, cannot handle {p}",
                            cursor_position=len(document.text),
                        )
                    else:
                        raise q.ValidationError(
                            message=f"Path to dataset should be a directory {p}",
                            cursor_position=len(document.text),
                        )

        dataset_path: Path = Path(
            q.path(
                "Please write the path to the downloaded dataset in your filesystem",
                validate=InputDirectoryValidator,
            ).unsafe_ask()
        ).expanduser()
    else:
        # point the user to the download page - let them download and umcompress it first. ----
        s = (
            "\nPlease download the dataset of your preference from the following"
            f" link:\n\t{dataset.dataset_link}\n\n"
        )
        if dataset.manual_dl_remarks:
            s = f"{s}{dataset.manual_dl_remarks}\n"
        s = f"{s}Remeber to decompress the dataset after download, if compressed\n"
        user_info(s)
        return 0

    # convert? --------------------------------------------------------------------------------
    if dataset.requires_conversion():
        user_info(
            f"The selected {dataset.value} dataset requires conversion to the SLAMcore format."
        )
        dataset_path_conv = Path(f"{dataset_path.name}_converted")

        # exists already? should I overwrite it? ----------------------------------------------
        if dataset_path_conv.exists():
            user_warning(
                f"Path already exists -> {dataset_path_conv}\n"
                "Should I overwrite it or find another suitable path?"
            )
            do_overwrite: bool = q.confirm("Overwrite", default=False).unsafe_ask()
            dataset_path_conv = get_ready_output_path(
                dataset_path_conv, overwrite_path=do_overwrite
            )
        else:
            user_info(
                f"The conversion process will create a new dataset directory, stored under {dataset_path_conv}."
            )
            do_proceed = q.confirm("Proceed").unsafe_ask()
            if not do_proceed:
                user_info("Exiting...")
                return 0

        # do convert --------------------------------------------------------------------------
        Converter = _dataset_to_converter[dataset]
        converter = Converter(in_path=dataset_path, out_path=dataset_path_conv)
        try:
            converter()
        except:
            user_note(traceback.format_exc())
            user_error(
                "Dataset conversion failed, are you sure you are passing the right path to the dataset?\n"
                "If so, contact support@slamcore.com and attach the above logs"
            )
            return 1

        dataset_path = dataset_path_conv
    else:
        # no conversion needed
        # sanity check - make sure that some known subdirectories are in the dataset directory
        # force to "mav0" if the user selected its parent
        valid_subdirs = ["cam0", "ir0", "depth0"]
        subdirs = {p.name for p in dataset_path.iterdir()}
        if not subdirs.intersection(valid_subdirs):
            if "mav0" in subdirs:
                mav0_path = dataset_path / "mav0"
                mav0_subdirs = {p.name for p in mav0_path.iterdir()}
                if not mav0_subdirs.intersection(valid_subdirs):
                    user_error(
                        "The specified path does not seem to be a valid EuRoC format. Please correct it and re-try"
                    )
                    return 1

                # choose mav0 instead. it seems to have the right format.
                dataset_path = mav0_path
                user_warning(f"Not a valid dataset, selecting instead {dataset_path}")
            else:
                user_error(
                    "The specified path does not seem to be a valid EuRoC format. Please correct it and re-try"
                )
                return 1

    # dataset is now downloaded and in the right format

    # embed capture_info.json -----------------------------------------------------------------
    dst_capture_info = dataset_path / "capture_info.json"
    shutil.copy(dataset.capture_info, dst_capture_info)
    user_info(f"Embedding capture_info.json file -> {dst_capture_info}")

    # exit ------------------------------------------------------------------------------------
    user_success(
        "Dataset is now ready. Try running it, for example with the SLAMcore visualiser:\n\n"
        f"\tslamcore_visualiser dataset -u {dataset_path}"
    )

    return 0


def main():
    return _main()


if __name__ == "__main__":
    sys.exit(main())
