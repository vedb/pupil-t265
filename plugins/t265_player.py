"""
(*)~---------------------------------------------------------------------------
Pupil - eye tracking platform
Copyright (C) 2012-2020 Pupil Labs

Distributed under the terms of the GNU
Lesser General Public License (LGPL v3.0).
See COPYING and COPYING.LESSER for license details.
---------------------------------------------------------------------------~(*)
"""
import os
import csv

import numpy as np

from pyglui import ui
from plugin import Plugin
from raw_data_exporter import Raw_Data_Exporter, _Base_Positions_Exporter
import player_methods as pm
from file_methods import load_pldata_file, load_object

import logging
logger = logging.getLogger(__name__)


class T265_Exporter(Raw_Data_Exporter):
    """ Exporter for data recorded from the RealSense T265. """

    icon_chr = chr(0xE873)
    icon_font = "pupil_icons"

    def __init__(self, g_pool):
        """ Constructor. """
        Plugin.__init__(self, g_pool)

        # TODO check if this blocks too long for long recordings
        g_pool.odometry_bisector = self.init_bisector(g_pool.rec_dir)

        self.extrinsics = self.load_extrinsics(g_pool.rec_dir)

    @classmethod
    def init_bisector(cls, rec_dir):
        """ Abuse Bisector class for odometry data. """
        odometry_data_file = load_pldata_file(rec_dir, "odometry")

        return pm.Bisector(
            odometry_data_file.data, odometry_data_file.timestamps)

    @classmethod
    def load_extrinsics(cls, directory, side="left"):
        """ Load extrinsics from user dir. """
        load_path = os.path.join(directory, f"t265_{side}.extrinsics")
        try:
            logger.info(f"Loaded t265_{side}.extrinsics")
            return load_object(load_path)
        except OSError:
            logger.warning("No extrinsics found. Use the T265 Calibration "
                           "plugin to calculate extrinsics.")

    @classmethod
    def export_extrinsics(cls, directory, t265_extrinsics, side="left"):
        """ Export extrinsics to export dir. """
        export_path = os.path.join(directory, f"t265_{side}_extrinsics.csv")

        translation = np.array(t265_extrinsics['translation']).flatten()
        T_dict = {f'T_{idx}': value for idx, value in enumerate(translation)}

        rotation = np.array(t265_extrinsics['rotation'])
        R_dict = {f'R_{row}_{col}': value
                  for (row, col), value in np.ndenumerate(rotation)}

        with open(export_path, "w", encoding="utf-8", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(("key", "value"))
            for row in T_dict.items():
                writer.writerow(row)
            for row in R_dict.items():
                writer.writerow(row)

        logger.info(f"Created 't265_{side}_extrinsics.csv' file.")

    def export_data(self, export_window, export_dir):
        """ Export data when notified. """
        odometry_exporter = OdometryExporter()
        odometry_exporter.csv_export_write(
                positions_bisector=self.g_pool.odometry_bisector,
                timestamps=self.g_pool.timestamps,
                export_window=export_window,
                export_dir=export_dir,
        )

        if self.extrinsics is not None:
            self.export_extrinsics(export_dir, self.extrinsics)

    def init_ui(self):
        """ Initialize plugin UI. """
        self.add_menu()
        self.menu.label = "T265 Exporter"
        self.menu.append(
            ui.Info_Text(
                "The export can be found in the exports folder, "
                "under the recording directory."))
        self.menu.append(
            ui.Info_Text(
                "Press the export button or type 'e' to start the export."))

    def deinit_ui(self):
        """ De-initialize plugin UI. """
        self.remove_menu()


class OdometryExporter(_Base_Positions_Exporter):
    """ Exporter backend. """

    @classmethod
    def csv_export_filename(cls):
        """ Filename of exported file. """
        return "odometry.csv"

    @classmethod
    def csv_export_labels(cls):
        """ CSV header. """
        return (
            "capture_timestamp",
            "realsense_timestamp",
            "world_index",
            "tracker_confidence",
            "position_x",
            "position_y",
            "position_z",
            "orientation_w",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "linear_velocity_x",
            "linear_velocity_y",
            "linear_velocity_z",
            "angular_velocity_x",
            "angular_velocity_y",
            "angular_velocity_z",
        )

    @classmethod
    def dict_export(cls, raw_value, world_index):
        """ One row of the CSV file. """
        return {
            "capture_timestamp": raw_value["timestamp"],
            "realsense_timestamp": raw_value["rs_timestamp"],
            "world_index": world_index,
            "tracker_confidence": raw_value["tracker_confidence"],
            "position_x": raw_value["position"][0],
            "position_y": raw_value["position"][1],
            "position_z": raw_value["position"][2],
            "orientation_w": raw_value["orientation"][0],
            "orientation_x": raw_value["orientation"][1],
            "orientation_y": raw_value["orientation"][2],
            "orientation_z": raw_value["orientation"][3],
            "linear_velocity_x": raw_value["linear_velocity"][0],
            "linear_velocity_y": raw_value["linear_velocity"][1],
            "linear_velocity_z": raw_value["linear_velocity"][2],
            "angular_velocity_x": raw_value["angular_velocity"][0],
            "angular_velocity_y": raw_value["angular_velocity"][1],
            "angular_velocity_z": raw_value["angular_velocity"][2],
        }
