#!/usr/bin/env python3

import argparse
from pathlib import Path

from slamcore_utils.arg_parser import add_bool_argument, existing_dir
from slamcore_utils.fs import get_ready_output_path
from slamcore_utils.logging import logger
from slamcore_utils.openloris_converter import OpenLORISConverter


def main():
    """Main."""

    # argument parsing ------------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        description="Convert an OpenLORIS dataset to the SLAMcore Dataset format"
    )
    parser.add_argument(
        "-i",
        "--input",
        required=True,
        type=existing_dir,
        help=("Path to the top-level OpenLORIS input directory to convert"),
    )
    parser.add_argument(
        "-o",
        "--output",
        default=Path("output"),
        type=Path,
        help=("Path to the top-level output directory"),
    )

    add_bool_argument(
        parser,
        arg_name="overwrite",
        true_help="Overwrite the output directory instead of creating a new one",
        default=False,
    )

    parser_args = vars(parser.parse_args())

    # basic input checks ----------------------------------------------------------------------
    input_dir: Path = parser_args["input"]
    output_dir = Path(parser_args["output"])
    overwrite_output = parser_args["overwrite"]

    # overwrite output?
    output_dir = get_ready_output_path(
        output_dir, overwrite_path=overwrite_output, logger=logger
    )

    # announce to user ------------------------------------------------------------------------
    s = (
        "\n\nConverting OpenLORIS Dataset to internal SLAMcore format\n"
        f"\t* Input directory:            {input_dir}\n"
        f"\t* Output directory:           {output_dir}\n"
        f"\t* Overwrite output directory: {overwrite_output}\n"
    )
    logger.info(s)

    # run conversion --------------------------------------------------------------------------
    converter = OpenLORISConverter(
        in_path=input_dir,
        out_path=output_dir,
        preserve_orig_timestamps=False,
    )
    converter()


if __name__ == "__main__":
    main()
