import os
import csv
import json

import numpy as np
import pandas as pd
import xarray as xr

from scipy.signal import fftconvolve
from quaternion import as_quat_array, as_float_array, from_rotation_matrix

try:
    from fast_histogram import histogram2d
except ImportError:
    from numpy import histogram2d


# ----- IO METHODS ----- #
def load_info(folder):
    """ Load data from info.player.json file as dict.

    Parameters
    ----------
    folder: str
        Path to the recording folder.

    Returns
    -------
    dict
        Loaded recording info.
    """
    with open(os.path.join(folder, "info.player.json")) as f:
        info = json.load(f)

    return info


def load_user_info(folder, info, filename="user_info.csv"):
    """ Load data from user info file as dict.

    Parameters
    ----------
    folder: str
        Path to the recording folder.

    info: dict
        Previously loaded recording info.

    filename: str, default 'user_info.csv'
        Filename of the user info file.

    Returns
    -------
    dict
        Loaded user info.
    """
    if not os.path.exists(os.path.join(folder, filename)):
        raise FileNotFoundError(
            "File {} not found in folder {}".format(filename, folder)
        )

    with open(os.path.join(folder, filename)) as f:
        reader = csv.reader(f)
        user_info = {rows[0]: rows[1] for rows in reader if rows[0] != "key"}

    for k, v in user_info.items():
        if k.endswith(("start", "end")) or "time" in k:
            user_info[k] = pd.to_timedelta(v) + pd.to_datetime(
                info["start_time_system_s"], unit="s"
            )

    return user_info


def load_gaze(folder, info):
    """ Load exported gaze positions as xarray.Dataset.

    Parameters
    ----------
    folder: str
        Path to the export folder.

    info: dict
        Previously loaded recording info.

    Returns
    -------
    xarray.Dataset
        Loaded gaze positions.
    """
    df = pd.read_csv(
        os.path.join(folder, "gaze_positions.csv"), index_col="gaze_timestamp"
    )

    df.index = pd.to_datetime(
        df.index - info["start_time_synced_s"] + info["start_time_system_s"], unit="s"
    )

    coords = {
        "time": pd.to_datetime(df.index.values, unit="s"),
        "cartesian_axis": ["x", "y", "z"],
        "pixel_axis": ["x", "y"],
    }

    # fix sign flip issue in 3d gaze point z coordinate
    df.loc[:, "gaze_point_3d_z"] = np.abs(df.loc[:, "gaze_point_3d_z"])

    data_vars = {
        "gaze_point": (
            ["time", "cartesian_axis"],
            df.loc[:, "gaze_point_3d_x":"gaze_point_3d_z"],
        ),
        "gaze_norm_pos": (
            ["time", "quaternion_axis"],
            df.loc[:, "norm_pos_x":"norm_pos_y"],
        ),
        "gaze_confidence": ("time", df["confidence"]),
    }

    ds = xr.Dataset(data_vars, coords)

    # remove duplicate timestamps
    _, idx = np.unique(ds.time, return_index=True)
    ds = ds.isel(time=idx)

    return ds


def load_odometry(folder, info):
    """ Load exported odometry as xarray.Dataset.

    Parameters
    ----------
    folder: str
        Path to the export folder.

    info: dict
        Previously loaded recording info.

    Returns
    -------
    xarray.Dataset
        Loaded odometry.
    """
    df = pd.read_csv(
        os.path.join(folder, "odometry.csv"), index_col="realsense_timestamp"
    )

    df.index = pd.to_datetime(df.index, unit="s")

    coords = {
        "time": pd.to_datetime(df.index.values, unit="s"),
        "cartesian_axis": ["x", "y", "z"],
        "quaternion_axis": ["w", "x", "y", "z"],
    }

    data_vars = {
        "position": (["time", "cartesian_axis"], df.loc[:, "position_x":"position_z"]),
        "orientation": (
            ["time", "quaternion_axis"],
            df.loc[:, "orientation_w":"orientation_z"],
        ),
        "linear_velocity": (
            ["time", "cartesian_axis"],
            df.loc[:, "linear_velocity_x":"linear_velocity_z"],
        ),
        "angular_velocity": (
            ["time", "cartesian_axis"],
            df.loc[:, "angular_velocity_x":"angular_velocity_z"],
        ),
        "tracker_confidence": ("time", df["tracker_confidence"]),
        "capture_timestamp": (
            "time",
            pd.to_datetime(
                df["capture_timestamp"]
                - info["start_time_synced_s"]
                + info["start_time_system_s"],
                unit="s",
            ),
        ),
    }

    return xr.Dataset(data_vars, coords)


def load_extrinsics(folder, filename="t265_left_extrinsics.csv"):
    """ Load camera extrinsics.

    Parameters
    ----------
    folder: str
        Path to the export folder.

    filename: str, default 't265_left_extrinsic.csv'
        Filename of the extrinsics file.

    Returns
    -------
    t: numpy.ndarray, shape (3,)
        Translation between the two cameras.

    q: numpy.ndarray, shape (4,)
        Rotation between the two cameras in quaternions.
    """
    with open(os.path.join(folder, filename)) as f:
        reader = csv.reader(f)
        t = np.array([float(row[1]) for row in reader if row[0].startswith("T")])
        f.seek(0)
        R = np.array([float(row[1]) for row in reader if row[0].startswith("R")])

    q = as_float_array(from_rotation_matrix(R.reshape(3, 3)))

    return t, q


# ----- GAZE PRE-PROCESSING ----- #
def smooth(data, axis=0, window_len=10):
    """ Smooth data with a boxcar filer.

    Parameters
    ----------
    data: xarray.DataArray
        Input data.

    axis: int, default 0
        The axis of the data representing the time dimension.

    window_len: int, default 10
        Length of the boxcar filter window.

    Returns
    -------
    xarray.DataArray
        Smoothed data.
    """
    # TODO dim
    data = data.copy()
    data.values = fftconvolve(
        data.values, np.ones((window_len, 1)) / window_len, mode="same", axes=axis
    )

    return data


def clean(data, *rules):
    """ Replace samples with nan according to specified rules.

    Parameters
    ----------
    data: xarray.DataArray
        Input data. First axis represents sample dimension.

    *rules: array_like, bool dtype
        Boolean indices of samples to keep. Must be one-dimensional and have
        the same length as the first axis of the input array.

    Returns
    -------
    xarray.DataArray
        Cleaned data.
    """
    data = data.copy()
    idx = np.ones(data.sizes["time"], dtype=bool)
    for r in rules:
        idx *= r
    data.values[~idx] = np.nan

    return data


# ----- SPATIAL TRANSFORMS ----- #
r_pupil_analysis = as_float_array(
    from_rotation_matrix(
        np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]).T
    )
)


def stack_rotations(*rotations):
    """ Chain multiple rotation arrays expressed in quaternions

    Parameters
    ----------
    *rotations: array_like, shape (..., 4, ...)
        Quaternion arrays. All axes except for singleton axes must match.

    Returns
    -------
    array_like, shape (..., 4, ...)
        The resulting chained rotation.
    """
    from functools import reduce
    from operator import mul

    # TODO return as_float_array(np.prod(as_quat_array(r) for r in rotations))
    return as_float_array(reduce(mul, (as_quat_array(r) for r in rotations), 1))


def rotate_vectors(v, q, axis=-1, inverse=False):
    """ Rotate an array of vectors by an array of quaternions.

    Parameters
    ----------
    v: array_like
        The array of vectors to be rotated.

    q: array_like
        Array of quaternions.

    axis: int, default -1
        The axis of the `v` array representing the coordinates of the
        vectors. Must have length 3.

    inverse: bool, default False
        If True, rotate by the inverse of q.

    Returns
    -------
    array_like
        The array of rotated vectors.
    """
    # TODO DataArray input/output
    if q.shape[axis] != 4:
        raise ValueError(
            "Expected axis {} of q to have length 4, got {}".format(axis, q.shape[axis])
        )

    if v.shape[axis] != 3:
        raise ValueError(
            "Expected axis {} of v to have length 3, got {}".format(axis, v.shape[axis])
        )

    if inverse:
        q = np.swapaxes(
            as_float_array(1.0 / as_quat_array(np.swapaxes(q, axis, -1))), axis, -1
        )

    # compute rotation
    r = np.take(q, (1, 2, 3), axis=axis)
    s = np.take(q, (0,), axis=axis)
    m = np.linalg.norm(q, axis=axis, keepdims=True)
    rxv = np.cross(r, v, axis=axis)
    vr = v + 2 * np.cross(r, s * v + rxv, axis=axis) / m

    return vr


def shortest_arc_rotation(v1, v2, dim="cartesian_axis", new_dim="quaternion_axis"):
    """ Estimate the shortest-arc rotation between two arrays of vectors.

    Parameters
    ----------
    v1: xarray.DataArray, shape (..., 3, ...)
        The first array of vectors.

    v2: array_like, shape (..., 3, ...)
        The second array of vectors.

    dim: str, default 'cartesian_axis'
        Dimension representing the coordinates of the vectors.
        Must be of length 3.

    Returns
    -------
    xarray.DataArray, shape (..., 4, ...)
        The quaternion representation of the shortest-arc rotation.
    """
    axis = v1.dims.index(dim)
    sn1 = np.sum(v1 ** 2, axis=axis, keepdims=True)
    sn2 = np.sum(v2 ** 2, axis=axis, keepdims=True)
    d12 = np.sum(v1 * v2, axis=axis, keepdims=True)
    c12 = np.cross(v1, v2, axis=axis)
    q = np.concatenate((np.sqrt(sn1 * sn2) + d12, c12), axis=axis)
    q /= np.linalg.norm(q, axis=axis, keepdims=True)

    coords = dict(v1.drop(dim).coords)
    dims = list(v1.dims)
    dims.remove(dim)
    dims.insert(axis, new_dim)
    coords[new_dim] = ["w", "x", "y", "z"]

    return xr.DataArray(q, coords, dims)


def cartesian_to_spherical(data, dim="cartesian_axis", new_dim="spherical_axis"):
    """ Transform cartesian to spherical coordinates in three dimensions.

    The spherical coordinate system is defined according to ISO 80000-2.

    Parameters
    ----------
    data: xarray.DataArray
        Input array.

    dim: str, default 'cartesian_axis'
        Dimension of input array representing x, y and z in cartesian
        coordinates. Must be of length 3.

    Returns
    -------
    xarray.DataArray
        Output array.
    """
    if data.sizes[dim] != 3:
        raise ValueError(
            "Expected length of dimension {} to be 3, got {} instead.".format(
                dim, data.data.sizes[dim]
            )
        )

    data = data.copy()
    arr = data.values
    axis = data.dims.index(dim)

    r = np.linalg.norm(arr, axis=axis)
    theta = np.arccos(np.take(arr, 2, axis=axis) / r)
    phi = np.arctan2(np.take(arr, 1, axis=axis), np.take(arr, 0, axis=axis))
    data.values = np.stack((r, theta, phi), axis=axis)

    data = data.rename({dim: new_dim})
    data[new_dim] = ["r", "theta", "phi"]

    return data


def center_spherical_coordinates(data):
    """ Transform coordinates such that azimuth is measured from x-axis origin.

    Parameters
    ----------
    data: xarray.DataArray
        Input data.

    Returns
    -------
    xarray.DataArray
        Transformed data.
    """
    # TODO dim
    data = data.copy()
    data.values[:, 1] = data.values[:, 1] - np.pi / 2

    return data


def radians_to_degrees(data):
    """ Transform spherical coordinates from radians to degrees.

    Parameters
    ----------
    data: xarray.DataArray
        Input data.

    Returns
    -------
    xarray.DataArray
        Transformed data.
    """
    # TODO dim
    data = data.copy()
    data.values[:, 1:] = data.values[:, 1:] / np.pi * 180

    return data
