[![DOI](https://img.shields.io/badge/DOI-10.1145%2F3379156.3391365-blue)](https://doi.org/10.1145/3379156.3391365)

# pupil-t265

**pupil-t265** provides plugins for the Pupil Core mobile eye tracking
 software for calibrating, recording and exporting tracking data from the
 Intel RealSense T265 tracking camera. This way, the T265 can be used for
 simultaneous head tracking when rigidly mounted with respect to the Pupil Core
 world camera. 
 
## Citation

If you use **pupil-t265** in your research, please cite our paper 
[Positional head-eye tracking outside the lab: an open-source solution](https://dl.acm.org/doi/abs/10.1145/3379156.3391365):

    @inproceedings{10.1145/3379156.3391365,
        author = {Hausamann, Peter and Sinnott, Christian and MacNeilage, Paul R.},
        title = {Positional Head-Eye Tracking Outside the Lab: An Open-Source Solution},
        year = {2020},
        isbn = {9781450371346},
        publisher = {Association for Computing Machinery},
        address = {New York, NY, USA},
        url = {https://doi.org/10.1145/3379156.3391365},
        doi = {10.1145/3379156.3391365},
        booktitle = {Symposium on Eye Tracking Research and Applications},
        articleno = {14},
        numpages = {5},
        keywords = {gaze estimation, locomotion, open source, Eye tracking, head tracking, 
                    mobile, simultaneous localization and mapping},
        location = {Stuttgart, Germany},
        series = {ETRA ’20}
    }


## Installation

The plugins require a recent version of the Pupil Core software
 (recommended `1.21`). Copy `t265_capture.py ` to 
 `pupil_capture_settings/plugins` and `t265_player.py` to 
 `pupil_player_settings/plugins`.

## Usage

### Capture

Use the *T265 Calibration* plugin to calculate the camera extrinsics, i.e.
 the position and orientation of the left T265 camera with respect to the world 
 camera.
 
The *T265 Recorder* will display live position, orientation and velocities
 of the T265 device. During a recording, these will also be saved to disk.

### Player

Use the *T265 Exporter* plugin to export the recorded data to csv. The
 export contains two files: `odometry.csv` and `t265_left_extrinsics.csv`.

## Example analysis

The `examples` folder contains two recorded and exported datasets along with
 analysis code as part of a short paper submission to the  12th ACM Symposium 
 on Eye Tracking Research and Applications (ETRA 2020).
 
To run the example code, clone or download the repository code first. The
 recommended way of installing the dependencies is through Anaconda/Miniconda 
 by creating a dedicated conda environment:
 
    conda env create -f examples/example_env.yml
    
In your base environment, install `nb_conda`:

    conda install nb_conda
    
Afterwards, start a Jupyter Notebook server from your base environment, open
 the `etra2020.ipynb` Jupyter Notebook, go to *Kernel > Change kernel*
 and select *Python [conda env:pupil-t265]*.
 
Alternatively, you can install the dependencies via `pip`:

    pip install xarray scipy numpy-quaternion numba fast-histogram matplotlib colorcet
    
## T265 mount

The repository contains `.stl` files for 3D printing a mount for the T265
 that can be attached to the Pupil Core world camera in the `mount` folder. 
 `t265_housing.stl` is a housing for the tracker and `t265_clip.stl` is a
 clip that is printed twice and glued to the housing in order to attach it to
 the world camera (see `mount_example.jpg`).
