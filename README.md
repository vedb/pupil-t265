# pupil-t265

**pupil-t265** provides plugins for the Pupil Core mobile eye tracking
 software for recording, calibrating and exporting tracking data from the Intel
 RealSense T265 tracking camera. This way, the T265 can be used for
 simultaneous head tracking when rigidly mounted wrt to the Pupil Core
 world camera. 

## Installation

Copy `t265_capture.py` to `pupil_capture_settings/plugins` and 
`t265_player.py` to `pupil_player_settings/plugins`.

## Usage

### Capture

Use the *T265 Calibration* plugin to calculate the camera extrinsics, i.e.
 the position and orientation of the left T265 camera wrt to the world camera.
 
The *T265 Recorder* will display live position, orientation and velocities
 of the T265 device. During a recording, these will also be saved to disk.

### Player

Use the *T265 Exporter* plugin to export the recorded data to csv. The
 export contains two files: `odometry.csv` and `t265_left_extrinsics.csv`.
