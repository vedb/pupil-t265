"""
(*)~---------------------------------------------------------------------------
Pupil - eye tracking platform
Copyright (C) 2012-2019 Pupil Labs

Distributed under the terms of the GNU
Lesser General Public License (LGPL v3.0).
See COPYING and COPYING.LESSER for license details.
---------------------------------------------------------------------------~(*)
"""
import os
import multiprocessing as mp

import numpy as np
import cv2

import pyrealsense2 as rs

from pyglui import ui
from pyglui.pyfontstash import fontstash
from pyglui.ui import get_opensans_font_path
from pyglui.cygl.utils import draw_gl_texture
from glfw import *
from gl_utils import \
    basic_gl_setup, clear_gl_screen, make_coord_system_norm_based

from plugin import Plugin
from uvc import get_time_monotonic
from file_methods import PLData_Writer, load_object, save_object
from camera_intrinsics_estimation import \
    Camera_Intrinsics_Estimation, _gen_pattern_grid, _make_grid, on_resize

import logging
logger = logging.getLogger(__name__)


class CannotStartPipeline(RuntimeError):
    """ RealSense pipeline can't be started. """


class T265_Recorder(Plugin):
    """ Pupil Capture plugin for the Intel RealSense T265 tracking camera.

    This plugin will start a connected RealSense T265 tracking camera and
    fetch odometry data (position, orientation, velocities) from it. When
    recording, the odometry will be saved to `odometry.pldata`.

    Note that the recorded timestamps come from uvc.get_time_monotonic and will
    probably have varying latency wrt the actual timestamp of the odometry
    data. Because of this, the timestamp from the T265 device is also recorded
    to the field `rs_timestamp`.
    """

    uniqueness = "unique"
    icon_chr = chr(0xE061)
    icon_font = "pupil_icons"

    def __init__(self, g_pool):
        """ Constructor. """
        super().__init__(g_pool)

        self.writer = None
        self.max_latency_ms = 1.
        self.verbose = False

        self.menu = None
        self.buttons = {
            "start_stop_button": None}
        self.infos = {
            "sampling_rate": None,
            "confidence": None,
            "position": None,
            "orientation": None,
            "angular_velocity": None,
            "linear_velocity": None}

        self.g_pool.t265_extrinsics = self.load_extrinsics(
            self.g_pool.user_dir)

        self.odometry_queue = mp.Queue()
        self.video_queue = mp.Queue()

        self.pipeline = None

        self.t265_window = None
        self.t265_window_should_close = False
        self.last_video_frame = None

    @classmethod
    def get_serial_numbers(cls, suffix='T265'):
        """ Return serial numbers of connected devices.

        based on https://github.com/IntelRealSense/librealsense/issues/2332
        """
        serials = []
        context = rs.context()
        for d in context.devices:
            if suffix and not d.get_info(rs.camera_info.name).endswith(suffix):
                continue
            serial = d.get_info(rs.camera_info.serial_number)
            serials.append(serial)

        return serials

    @classmethod
    def start_pipeline(cls, callback=None, odometry=True, video=True):
        """ Start the RealSense pipeline. """
        try:
            serials = cls.get_serial_numbers()
        except RuntimeError as e:
            raise CannotStartPipeline(
                f"Could not determine connected T265 devices. Reason: {e}")

        # check number of connected devices
        if len(serials) == 0:
            raise CannotStartPipeline(
                "No T265 device connected.")
        elif len(serials) > 1:
            raise CannotStartPipeline(
                "Multiple T265 devices not yet supported.")

        try:
            pipeline = rs.pipeline()

            config = rs.config()
            # TODO not working with librealsense 2.32.1 but is necessary for
            #  simultaneously operating multiple devices:
            #  config.enable_device(serials[0])

            if odometry:
                config.enable_stream(rs.stream.pose)
            if video:
                config.enable_stream(rs.stream.fisheye, 1)
                config.enable_stream(rs.stream.fisheye, 2)

            if callback is None:
                pipeline.start(config)
            else:
                pipeline.start(config, callback)

        except RuntimeError as e:
            raise CannotStartPipeline(
                f"Could not start RealSense pipeline. Maybe another plugin "
                f"is using it. Reason: {e}")

        return pipeline

    def start_stop_callback(self):
        """ Callback for the start/stop button. """
        if self.pipeline is None:
            try:
                self.pipeline = self.start_pipeline(
                        callback=self.frame_callback,
                        odometry=self.odometry_queue is not None,
                        video=self.video_queue is not None)
                self.buttons["start_stop_button"].label = "Stop Pipeline"
                logger.info("RealSense pipeline started.")
            except CannotStartPipeline as e:
                logger.error(str(e))
        else:
            self.pipeline.stop()
            self.pipeline = None
            self.buttons["start_stop_button"].label = "Start Pipeline"
            logger.info("RealSense pipeline stopped.")

            # Close video stream video if open
            if self.t265_window is not None:
                self.t265_window_should_close = True

    def frame_callback(self, rs_frame):
        """ Callback for new RealSense frames. """
        if rs_frame.is_pose_frame() and self.odometry_queue is not None:
            odometry = self.get_odometry_data(rs_frame)
            self.odometry_queue.put(odometry)
        elif rs_frame.is_frameset() and self.video_queue is not None:
            video_data = self.get_video_data(rs_frame)
            self.video_queue.put(video_data)

    @classmethod
    def save_extrinsics(cls, directory, t265_extrinsics, side="left"):
        """ Save extrinsics to user dir. """
        save_path = os.path.join(directory, f"t265_{side}.extrinsics")
        save_object(t265_extrinsics, save_path)

        logger.info(f"Extrinsics for {side} T265 camera saved to {save_path}")

    @classmethod
    def load_extrinsics(cls, directory, side="left"):
        """ Load extrinsics from user dir. """
        load_path = os.path.join(directory, f"t265_{side}.extrinsics")
        try:
            extrinsics = load_object(load_path)
            logger.info(f"Loaded t265_{side}.extrinsics")
            return extrinsics
        except OSError:
            logger.warning("No extrinsics found. Use the T265 Calibration "
                           "plugin to calculate extrinsics.")

    @classmethod
    def get_odometry_data(cls, rs_frame):
        """ Get odometry data from RealSense pose frame. """
        t = rs_frame.get_timestamp() / 1e3
        t_pupil = get_time_monotonic()

        pose = rs_frame.as_pose_frame()
        c = pose.pose_data.tracker_confidence
        p = pose.pose_data.translation
        q = pose.pose_data.rotation
        v = pose.pose_data.velocity
        w = pose.pose_data.angular_velocity

        return {
            "topic": "odometry",
            "timestamp": t_pupil,
            "rs_timestamp": t,
            "tracker_confidence": c,
            "position": (p.x, p.y, p.z),
            "orientation": (q.w, q.x, q.y, q.z),
            "linear_velocity": (v.x, v.y, v.z),
            "angular_velocity": (w.x, w.y, w.z)
        }

    @classmethod
    def get_video_data(cls, rs_frame, side="both"):
        """ Extract video frame and timestamp from a RealSense frame. """
        t = rs_frame.get_timestamp() / 1e3
        t_pupil = get_time_monotonic()

        frameset = rs_frame.as_frameset()

        if side == "left":
            video_frame = np.asanyarray(
                frameset.get_fisheye_frame(1).as_video_frame().get_data())
        elif side == "right":
            video_frame = np.asanyarray(
                frameset.get_fisheye_frame(2).as_video_frame().get_data())
        elif side == "both":
            video_frame = np.hstack([
                np.asanyarray(
                    frameset.get_fisheye_frame(1).as_video_frame().get_data()),
                np.asanyarray(
                    frameset.get_fisheye_frame(2).as_video_frame().get_data())
            ])
        else:
            raise ValueError(f"Unsupported mode: {side}")

        return {
            "topic": "fisheye",
            "timestamp": t_pupil,
            "rs_timestamp": t,
            "frame": video_frame
        }

    @classmethod
    def get_info_str(cls, values, axes, unit=None):
        """ Get string with current values for display. """
        if unit is None:
            return ", ".join(
                f"{a}: {v: .2f}" for v, a in zip(values, axes))
        else:
            return ", ".join(
                f"{a}: {v: .2f} {unit}" for v, a in zip(values, axes))

    def show_infos(self, odometry_data):
        """ Show current RealSense data in the plugin menu. """
        odometry_dict = {
            k: np.array([d[k] for d in odometry_data])
            for k in odometry_data[0]}

        if len(odometry_data) > 1:
            f_odometry = np.mean(1 / np.diff(odometry_dict["rs_timestamp"]))
        else:
            f_odometry = 0.

        c = np.mean(odometry_dict["tracker_confidence"])
        p = np.mean(odometry_dict["position"], axis=0)
        q = np.mean(odometry_dict["orientation"], axis=0)
        v = np.mean(odometry_dict["linear_velocity"], axis=0)
        w = np.mean(odometry_dict["angular_velocity"], axis=0)

        if self.pipeline is not None:
            self.infos["sampling_rate"].text = \
                f"Odometry sampling rate: {f_odometry:.2f} Hz"
            self.infos["confidence"].text = \
                f"Tracker confidence: {c}"
            self.infos["position"].text = self.get_info_str(
                p, ("x", "y", "z"), "m")
            self.infos["orientation"].text = self.get_info_str(
                q, ("w", "x", "y", "z"))
            self.infos["linear_velocity"].text = self.get_info_str(
                v, ("x", "y", "z"), "m/s")
            self.infos["angular_velocity"].text = self.get_info_str(
                w, ("x", "y", "z"), "rad/s")
        else:
            for info in self.infos.values():
                info.text = "Waiting..."

    def recent_events(self, events):
        """ Main loop callback. """
        try:
            odometry_data = []
            video_data = []
            while not self.odometry_queue.empty():
                odometry_data.append(self.odometry_queue.get())
            while not self.video_queue.empty():
                video_data.append(self.video_queue.get())
                self.last_video_frame = video_data[-1]["frame"]

        except RuntimeError as e:
            logger.error(str(e))
            return

        # Show (and possibly record) collected odometry data
        if len(odometry_data) > 0:
            self.show_infos(odometry_data)
            if self.writer is not None:
                self.write(odometry_data)

        if self.t265_window_should_close:
            self.close_t265_window()

    def write(self, odometry_data):
        """ Write new odometry to the .pldata file """
        for d in odometry_data:
            try:
                self.writer.append(d)
            except AttributeError:
                pass

    def cleanup(self):
        """ Cleanup callback. """
        if self.pipeline is not None:
            self.start_stop_callback()

    def open_t265_window(self):
        """ Open a window to show the T265 video stream. """
        if self.pipeline is None:
            logger.error("Start pipeline to show T265 video stream")
            return

        if not self.t265_window:

            width, height = 1696, 800
            self.t265_window = glfwCreateWindow(
                width, height, "T265 Video Stream", monitor=None,
                share=glfwGetCurrentContext())

            glfwSetWindowPos(self.t265_window, 200, 31)

            # Register callbacks
            glfwSetFramebufferSizeCallback(self.t265_window, on_resize)
            glfwSetWindowCloseCallback(self.t265_window, self.on_t265_close)

            on_resize(
                self.t265_window, *glfwGetFramebufferSize(self.t265_window))

            # gl_state settings
            active_window = glfwGetCurrentContext()
            glfwMakeContextCurrent(self.t265_window)
            basic_gl_setup()
            glfwMakeContextCurrent(active_window)

    def gl_display(self):
        """ Display routines called by the world process after each frame. """
        if self.t265_window:
            self.gl_display_in_t265_window()

    def gl_display_in_t265_window(self):
        """ Show new frame in window. """
        active_window = glfwGetCurrentContext()
        glfwMakeContextCurrent(self.t265_window)

        clear_gl_screen()
        if self.last_video_frame is not None:
            make_coord_system_norm_based()
            draw_gl_texture(self.last_video_frame)

        glfwSwapBuffers(self.t265_window)
        glfwMakeContextCurrent(active_window)

    def on_t265_close(self, window=None):
        """ Callback when windows is closed. """
        self.t265_window_should_close = True

    def close_t265_window(self):
        """ Close T265 video stream window. """
        self.t265_window_should_close = False
        if self.t265_window:
            glfwDestroyWindow(self.t265_window)
            self.t265_window = None

    def on_notify(self, notification):
        """ Callback for notifications. """
        # Start or stop recording based on notification
        if notification["subject"] == "recording.started":
            if self.pipeline is None:
                logger.warning(
                    "Pipeline is not running. T265 Recorder will not record "
                    "any data.")
                return
            if self.writer is None:
                self.writer = PLData_Writer(
                    notification["rec_path"], "odometry")
        if notification["subject"] == "recording.stopped":
            if self.writer is not None:
                self.writer.close()
                if hasattr(self.g_pool, "t265_extrinsics"):
                    self.save_extrinsics(
                        notification["rec_path"], self.g_pool.t265_extrinsics)

    def add_info_menu(self, measure):
        """ Add growing menu with infos. """
        self.infos[measure] = ui.Info_Text("Waiting...")
        info_menu = ui.Growing_Menu(measure.replace("_", " ").capitalize())
        info_menu.append(self.infos[measure])
        self.menu.append(info_menu)

    def init_ui(self):
        """ Initialize plugin UI. """
        self.add_menu()
        self.menu.label = "T265 Recorder"

        self.buttons["start_stop_button"] = ui.Button(
            "Start Pipeline", self.start_stop_callback)
        self.menu.append(self.buttons["start_stop_button"])

        self.buttons["show_video_button"] = ui.Button(
            "Show T265 Video Stream", self.open_t265_window)
        self.menu.append(self.buttons["show_video_button"])

        self.infos["sampling_rate"] = ui.Info_Text("Waiting...")
        self.menu.append(self.infos["sampling_rate"])
        self.infos["confidence"] = ui.Info_Text("")
        self.menu.append(self.infos["confidence"])

        self.add_info_menu("position")
        self.add_info_menu("orientation")
        self.add_info_menu("linear_velocity")
        self.add_info_menu("angular_velocity")

    def deinit_ui(self):
        """ De-initialize plugin UI. """
        self.remove_menu()
        # TODO do we need to call cleanup() here?
        self.cleanup()


class T265_Calibration(Camera_Intrinsics_Estimation, T265_Recorder):
    """ Pupil Capture plugin for calibrating RealSense T265 with world camera.

    This plugin computes the extrinsics of the left camera of a connected
    RealSense T265 tracking camera wrt the world camera. Calibration results
    are stored in `t265_left.extrinsics` in the `capture_settings` folder.
    """

    uniqueness = "unique"
    icon_chr = chr(0xEC06)
    icon_font = "pupil_icons"

    def __init__(self, g_pool, fullscreen=False, monitor_idx=0):
        """ Constructor. """
        Plugin.__init__(self, g_pool)

        self.collect_new = False
        self.calculated = False
        self.obj_grid = _gen_pattern_grid((4, 11))
        self.img_points = []
        self.img_points_left = []
        self.img_points_right = []
        self.obj_points = []
        self.count = 10
        self.display_grid = _make_grid()

        self._window = None

        # initialize empty menu
        self.menu = None
        # TODO merge these
        self.buttons = {
            "start_stop_button": None}
        self.button = None
        self.clicks_to_close = 5
        self.window_should_close = False
        self.monitor_idx = monitor_idx
        self.fullscreen = fullscreen
        self.dist_mode = "Fisheye"
        self.show_undistortion = False

        self.glfont = fontstash.Context()
        self.glfont.add_font("opensans", get_opensans_font_path())
        self.glfont.set_size(32)
        self.glfont.set_color_float((0.2, 0.5, 0.9, 1.0))
        self.glfont.set_align_string(v_align="center")

        self.verbose = False

        self.odometry_queue = None
        self.video_queue = mp.Queue()
        self.pipeline = None

        self.t265_window = None
        self.t265_window_should_close = False
        self.last_video_frame = None

    def collect_new_points(self, world_frame, realsense_frame):
        """ Collect calibration points from all cameras. """
        img_left = realsense_frame[:, :realsense_frame.shape[1] // 2]
        img_right = realsense_frame[:, realsense_frame.shape[1] // 2:]

        status_world, grid_points_world = cv2.findCirclesGrid(
            world_frame, (4, 11), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
        status_left, grid_points_left = cv2.findCirclesGrid(
            img_left, (4, 11), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
        status_right, grid_points_right = cv2.findCirclesGrid(
            img_right, (4, 11), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)

        if status_world and status_left and status_right:
            self.img_points.append(grid_points_world)
            self.obj_points.append(self.obj_grid)
            self.img_points_left.append(grid_points_left)
            self.img_points_right.append(grid_points_right)
            self.collect_new = False
            self.count -= 1
            self.button.status_text = f"{self.count:d} to go"

    def advance(self, _):
        """ Prepare to grad the next calibration frames. """
        if self.pipeline is None:
            logger.error("Pipeline must be started for calibration.")
            return

        if self.count == 10:
            self.img_points_left = []
            self.img_points_right = []

        super().advance(_)

    @classmethod
    def calculate_intrinsics(cls, img_shape, img_points, obj_points, count=10,
                             dist_mode="Fisheye"):
        """ Calculate intrinsic parameters for one camera. """
        if dist_mode == "Fisheye":
            calibration_flags = (
                cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                + cv2.fisheye.CALIB_CHECK_COND
                + cv2.fisheye.CALIB_FIX_SKEW)

            max_iter = 30
            eps = 1e-6

            camera_matrix = np.zeros((3, 3))
            dist_coefs = np.zeros((4, 1))
            rvecs = [
                np.zeros((1, 1, 3), dtype=np.float64) for i in range(count)]
            tvecs = [
                np.zeros((1, 1, 3), dtype=np.float64) for i in range(count)]

            objPoints = [x.reshape(1, -1, 3) for x in obj_points]
            imgPoints = img_points

            rms, _, _, _, _ = cv2.fisheye.calibrate(
                objPoints, imgPoints, img_shape, camera_matrix, dist_coefs,
                rvecs, tvecs, calibration_flags,
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                 max_iter, eps))

        else:
            raise ValueError(
                f"Unkown distortion model: {dist_mode}")

        logger.info(f"Calibrated Camera, RMS:{rms:.6f}")

        return camera_matrix, dist_coefs

    def calculate(self):
        """ Calculate pose of left T265 camera wrt world camera. """
        try:
            cam_mtx_world, dist_coefs_world = self.calculate_intrinsics(
                self.g_pool.capture.frame_size, self.img_points,
                self.obj_points, self.count, self.dist_mode)
            cam_mtx_left, dist_coefs_left = self.calculate_intrinsics(
                (800, 848), self.img_points_left, self.obj_points, self.count)
            cam_mtx_right, dist_coefs_right = self.calculate_intrinsics(
                (800, 848), self.img_points_right, self.obj_points, self.count)

        except cv2.error as e:
            logger.warning(
                f"Camera calibration failed. Reason: {e}")
            logger.warning(
                "Please try again with a better coverage of the cameras FOV!")
            return


        obj_points = np.array(
            [x.reshape(1, -1, 3) for x in self.obj_points], dtype=np.float64)
        img_points = np.array(
            [x.reshape(1, -1, 2) for x in self.img_points], dtype=np.float64)
        img_points_left = np.array(
            [x.reshape(1, -1, 2) for x in self.img_points_left], 
            dtype=np.float64)
        img_points_right = np.array(
            [x.reshape(1, -1, 2) for x in self.img_points_right], 
            dtype=np.float64)

        R = np.zeros((1, 1, 3), dtype=np.float64)
        T = np.zeros((1, 1, 3), dtype=np.float64)

        try:
            rms, _, _, _, _, R_right_left, T_right_left = \
                cv2.fisheye.stereoCalibrate(
                    obj_points, img_points_right, img_points_left,
                    cam_mtx_right, dist_coefs_right,
                    cam_mtx_left, dist_coefs_left,
                    (0, 0), R, T, cv2.CALIB_FIX_INTRINSIC)

            # orientation inaccuracy is deviation from identity matrix
            inacc = np.linalg.norm(R_right_left - np.eye(3))
            logger.info(f"Calibrated right/left pair, RMS:{rms:.6f}, "
                        f"orientation inaccuracy:{inacc:.6f}")

        except cv2.error as e:
            logger.warning(
                f"Right/left stereo calibration failed. Reason: {e}")

        try:
            rms, _, _, _, _, R_world_left, T_world_left = \
                cv2.fisheye.stereoCalibrate(
                    obj_points, img_points, img_points_left,
                    cam_mtx_world, dist_coefs_world,
                    cam_mtx_left, dist_coefs_left,
                    (0, 0), R, T, cv2.CALIB_FIX_INTRINSIC)

            # correct difference in coordinate systems
            R_realsense_pupil = np.array(
                [[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])
            R_world_left = R_realsense_pupil @ R_world_left
            T_world_left = R_realsense_pupil @ T_world_left

        except cv2.error as e:
            logger.warning(
                f"World/left stereo calibration failed. Reason: {e}")

        else:
            # TODO save world camera resolution and side?
            # TODO align pupil and realsense coordinate systems
            self.g_pool.t265_extrinsics = {
                "translation": T_world_left.tolist(),
                "rotation": R_world_left.tolist()}

            self.save_extrinsics(
                self.g_pool.user_dir, self.g_pool.t265_extrinsics)

    def recent_events(self, events):
        """ Main loop callback. """
        frame = events.get("frame")
        if not frame:
            return

        try:
            # get the latest video frame from the queue
            t265_video_frame = None
            while not self.video_queue.empty():
                t265_video_frame = self.video_queue.get()["frame"]
                self.last_video_frame = t265_video_frame

        except RuntimeError as e:
            logger.error(str(e))
            return

        # Perform calibration if requested
        if self.collect_new and t265_video_frame is not None:
            self.collect_new_points(frame.img, t265_video_frame)

        if self.count <= 0 and not self.calculated:
            self.count = 10
            self.calculate()
            self.button.status_text = ""

        if self.window_should_close:
            self.close_window()

        if self.t265_window_should_close:
            self.close_t265_window()

        if self.show_undistortion:
            logger.warning("Undistortion not yet implemented.")

    def gl_display(self):
        """ Display routines called by the world process after each frame. """
        super().gl_display()
        if self.t265_window:
            self.gl_display_in_t265_window()

    def on_notify(self, notification):
        """ Callback for notifications. """

    def init_ui(self):
        """ Initialize plugin UI. """
        self.add_menu()
        self.menu.label = "T265 Calibration"

        self.buttons["start_stop_button"] = ui.Button(
            "Start Pipeline", self.start_stop_callback)
        self.menu.append(self.buttons["start_stop_button"])

        self.buttons["show_video_button"] = ui.Button(
            "Show T265 Video Stream", self.open_t265_window)
        self.menu.append(self.buttons["show_video_button"])

        def get_monitors_idx_list():
            monitors = [glfwGetMonitorName(m) for m in glfwGetMonitors()]
            return range(len(monitors)), monitors

        if self.monitor_idx not in get_monitors_idx_list()[0]:
            logger.warning(
                "Monitor at index %s no longer availalbe using default" %
                self.monitor_idx)
            self.monitor_idx = 0

        self.menu.append(
            ui.Info_Text(
                "Estimate relative pose of world camera wrt left T265 camera. "
                "Using an 11x9 asymmetrical circle grid. "
                "Click 'i' to capture a pattern."))

        self.menu.append(ui.Button("show Pattern", self.open_window))
        self.menu.append(
            ui.Selector(
                "monitor_idx", self, selection_getter=get_monitors_idx_list,
                label="Monitor",))

        self.menu.append(ui.Switch("fullscreen", self, label="Use Fullscreen"))

        self.button = ui.Thumb(
            "collect_new", self, setter=self.advance, label="I", hotkey="i")
        self.button.on_color[:] = (0.3, 0.2, 1.0, 0.9)
        self.g_pool.quickbar.insert(0, self.button)

    def deinit_ui(self):
        """ De-initialize plugin UI. """
        self.cleanup()
        super().deinit_ui()

    def get_init_dict(self):
        """ Return persistent values. """
        return {"monitor_idx": self.monitor_idx}
