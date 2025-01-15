## Vision-based Navigation

This code is a part of the practical course ["Vision-based Navigation" (IN2106)](https://cvg.cit.tum.de/teaching/ss2023/visnav_ss2023) taught at the Technical University of Munich.

The authors of the initial version are Vladyslav Usenko, Nikolaus Demmel, David Schubert and Zhakshylyk Nurlanov.

I implemented the missing parts for visual-SLAM pipeline during the practical course:
* 3D geometry and camera models
* Nonlinear optimisation and camera calibration
* Feature detectors and descriptors, Feature matching, RANSAC 
* Structure from Motion (SfM)
* Visual odometry and localisation

### Stereo Camera Calibration
[calibration.webm](https://github.com/user-attachments/assets/92df9ce2-09d5-4e7b-952f-ff97fd626aeb)

### Structure from Motion
[sfm.webm](https://github.com/user-attachments/assets/9ac28cb2-07fe-446a-9028-11db5a6a4e11)

### Odometry
[odometry.webm](https://github.com/user-attachments/assets/85a57ee3-c60b-4055-a5ca-bfb6a80802ff)

### How to run

#### Calibration
* ./calibration --dataset-path ../data/euroc_calib/
it generates opt_calib.json

#### SfM
it uses opt_calib.json calibration file
* ./sfm --dataset-path ../data/euroc_V1 --voc-path ../data/ORBvoc.cereal
using opt_calib.json

#### Visual Odometry
run download_dataset.sh to download dataset

it uses opt_calib.json calibration file
* ./odometry --dataset-path ../data/V1_01_easy/mav0/

### License

The code for this practical course is provided under a BSD 3-clause license. See the LICENSE file for details.

Parts of the code (`include/tracks.h`, `include/union_find.h`) are adapted from OpenMVG and distributed under an MPL 2.0 licence.

Parts of the code (`include/local_parameterization_se3.hpp`, `src/test_ceres_se3.cpp`) are adapted from Sophus and distributed under an MIT license.

Note also the different licenses of thirdparty submodules.


You can find [setup instructions here.](wiki/Setup.md)
