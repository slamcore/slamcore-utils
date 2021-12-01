import shutil
from pathlib import Path
from typing import Dict, List

import numpy as np

from slamcore_utils.dataset_dir_converter import DatasetDirConverter
from slamcore_utils.logging import logger
from slamcore_utils.progress_bar import progress_bar


class OpenLORISConverter(DatasetDirConverter):
    """Convert a dataset from the OpenLORIS format to the SLAMcore Dataset Format."""

    def __init__(
        self,
        *,
        preserve_orig_timestamps: bool = False,
        **kargs,
    ):
        self._preserve_orig_timestamps = preserve_orig_timestamps
        super().__init__(**kargs)

    def _convert(self) -> None:
        # create required structure
        self._out_path.mkdir(parents=True, exist_ok=False)

        # ground truth poses ------------------------------------------------------------------
        self.convert_poses()

        # IMU ---------------------------------------------------------------------------------
        self.convert_imu(
            self._in_path / "t265_gyroscope.txt",
            self._in_path / "t265_accelerometer.txt",
            self._out_path / "imu0",
        )

        # T265 images -------------------------------------------------------------------------
        self.convert_img("fisheye1.txt", "ir0")
        self.convert_img("fisheye2.txt", "ir1")

        # d435i images ------------------------------------------------------------------------
        self.convert_img(
            "color.txt",
            "cam0",
            replace_rate_mean=not self._preserve_orig_timestamps,
        )
        self.convert_img(
            "depth.txt",
            "depth0",
            replace_rate_mean=not self._preserve_orig_timestamps,
        )

    def convert_img(
        self,
        txt_fname: str,
        output_dirname: str,
        replace_rate_mean: bool = False,
    ):
        txt_data = np.loadtxt(
            self._in_path / txt_fname, delimiter=" ", skiprows=0, dtype=object
        )
        timestamps: np.ndarray = np.array(txt_data[:, 0], dtype=float)
        img_relpaths: List[str] = np.array(txt_data[:, 1], dtype=str).tolist()

        num_images = len(timestamps)
        logger.info(f"Converting {num_images} images for {txt_fname}...")
        interframe_count = np.ones(num_images - 1)
        if replace_rate_mean:
            dt_full = timestamps[-1] - timestamps[0]
            rate = dt_full / (num_images - 1)

        time_frames = []
        for i in progress_bar(range(num_images), leave=True):
            time_sec = timestamps[i]
            if replace_rate_mean:
                time_sec = timestamps[0] + np.sum(interframe_count[0:i]) * rate  # type: ignore

            fpath_rel = img_relpaths[i]
            time_ns = int(time_sec * 1e9)
            data_folder = self._out_path / output_dirname / "data"
            data_folder.mkdir(exist_ok=True, parents=True)
            fpath = self._in_path / fpath_rel
            fname = str(time_ns) + ".png"
            fpath_out = data_folder / fname
            shutil.copy(fpath, fpath_out)
            time_frames.append([str(time_ns), fname])

        with (self._out_path / output_dirname / "data.csv").open("w") as f:
            f.write("timestamp [ns],filename\n")
            for data in time_frames:
                f.write(",".join(data) + "\n")

    def convert_poses(self):
        logger.info("Converting ground truth poses...")
        in_data = np.loadtxt(self._in_path / "groundtruth.txt", delimiter=" ")
        time: np.ndarray = (in_data[:, 0] * 1e9).astype(int)

        pose0_dir = self._out_path / "pose0"
        pose0_dir.mkdir(exist_ok=True)
        data_csv = pose0_dir / "data.csv"

        out_header = "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_x [],q_RS_y [],q_RS_z [],q_RS_w []"
        with open(data_csv, "w") as file:
            file.write(f"{out_header}\n")
            for i, imu_sample in enumerate(in_data[:, 1:]):
                imu_sample_line = np.array2string(
                    imu_sample,
                    floatmode="fixed",
                    max_line_width=1000,
                    precision=20,
                    separator=",",
                    suppress_small=True,
                )[1:-1]
                print(
                    str(time[i]) + "," + imu_sample_line,
                    file=file,
                )

    def convert_imu(self, gyro_path: Path, acc_path: Path, out_dir: Path):
        logger.info("Converting IMU measurements...")
        csv_gyro = np.loadtxt(gyro_path, delimiter=" ")
        csv_acc = np.loadtxt(acc_path, delimiter=" ")
        t_start_sec = np.max([csv_gyro[0, 0], csv_acc[0, 0]])
        t_end_sec = np.min([csv_gyro[-1, 0], csv_acc[-1, 0]])

        time: List[int] = []
        dict_: Dict[str, List[float]] = {
            "w_x": [],
            "w_y": [],
            "w_z": [],
            "a_x": [],
            "a_y": [],
            "a_z": [],
        }

        i = 0
        while csv_gyro[i, 0] < t_start_sec:
            i += 1
        i_acc = 0
        while (i < csv_gyro.shape[0]) and (csv_gyro[i, 0] <= t_end_sec):
            t = csv_gyro[i, 0]
            w = csv_gyro[i, 1:4]
            while (i_acc + 1 < csv_acc.shape[0]) and (csv_acc[i_acc + 1, 0] < csv_gyro[i, 0]):
                i_acc += 1
            acc_prev = csv_acc[i_acc, 1:4]
            acc_next = csv_acc[i_acc + 1, 1:4]
            t_acc_prev = csv_acc[i_acc, 0]
            t_acc_next = csv_acc[i_acc + 1, 0]
            alpha = (t - t_acc_prev) / (t_acc_next - t_acc_prev)
            acc = (1 - alpha) * acc_prev + alpha * acc_next
            time.append(int(t * 1e9))

            dict_["w_x"].append(w[0])
            dict_["w_y"].append(w[1])
            dict_["w_z"].append(w[2])
            dict_["a_x"].append(acc[0])
            dict_["a_y"].append(acc[1])
            dict_["a_z"].append(acc[2])
            i += 1

        imu_data = np.empty(shape=(len(dict_["w_x"]), len(dict_)))
        imu_data[:, 0] = dict_["w_x"]
        imu_data[:, 1] = dict_["w_y"]
        imu_data[:, 2] = dict_["w_z"]
        imu_data[:, 3] = dict_["a_x"]
        imu_data[:, 4] = dict_["a_y"]
        imu_data[:, 5] = dict_["a_z"]

        out_dir.mkdir(exist_ok=False)
        out_file = out_dir / "data.csv"
        out_header = (
            "#timestamp [ns],"
            "w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
            "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
        )

        with open(out_file, "w") as file:
            file.write(f"{out_header}\n")
            for i, imu_sample in enumerate(imu_data):
                imu_sample_line = np.array2string(
                    imu_sample,
                    floatmode="fixed",
                    max_line_width=1000,
                    precision=20,
                    separator=",",
                    suppress_small=True,
                )[1:-1]
                print(
                    str(time[i]) + "," + imu_sample_line,
                    file=file,
                )
