![Kalibr](https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/images/kalibr_small.png)

[![ROS1 Ubuntu 20.04](https://github.com/ethz-asl/kalibr/actions/workflows/docker_2004_build.yaml/badge.svg)](https://github.com/ethz-asl/kalibr/actions/workflows/docker_2004_build.yaml)
[![ROS1 Ubuntu 18.04](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1804_build.yaml/badge.svg)](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1804_build.yaml)
[![ROS1 Ubuntu 16.04](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1604_build.yaml/badge.svg)](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1604_build.yaml)

## Introduction
Kalibr is a toolbox that solves the following calibration problems:

1. **Multi-Camera Calibration**: Intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view with support for a wide range of [camera models](https://github.com/ethz-asl/kalibr/wiki/supported-models).
1. **Visual-Inertial Calibration (CAM-IMU)**: Spatial and temporal calibration of an IMU w.r.t a camera-system along with IMU intrinsic parameters
1. **Multi-Inertial Calibration (IMU-IMU)**: Spatial and temporal calibration of an IMU w.r.t a base inertial sensor along with IMU intrinsic parameters (requires 1-aiding camera sensor).
1. **Rolling Shutter Camera Calibration**: Full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras.

To install follow the [install wiki page](https://github.com/ethz-asl/kalibr/wiki/installation) instructions for which you can either use Docker or install from source in a ROS workspace.
Please find more information on the [wiki pages](https://github.com/ethz-asl/kalibr/wiki) of this repository.
For questions or comments, please open an issue on Github.


## News / Events

* **Nov 24, 2022** - Some new visualization of trajectory and IMU rate for the generated report along with fixed support for exporting poses to file (see PR [#578](https://github.com/ethz-asl/kalibr/pull/578),[#581](https://github.com/ethz-asl/kalibr/pull/581),[#582](https://github.com/ethz-asl/kalibr/pull/582))
* **May 3, 2022** - Support for Ubuntu 20.04 along with Docker scripts have been merged into master via PR [#515](https://github.com/ethz-asl/kalibr/pull/515). A large portion was upgrading to Python 3. A special thanks to all the contributors that made this possible. Additionally, contributed fixes for the different validation and visualization scripts have been merged.
* **Febuary 3, 2020** - Initial Ubuntu 18.04 support has been merged via PR [#241](https://github.com/ethz-asl/kalibr/pull/241). Additionally, support for inputting an initial guess for focal length can be provided from the cmd-line on failure to initialize them.
* **August 15, 2018** - Double sphere camera models have been contributed to the repository via PR [#210](https://github.com/ethz-asl/kalibr/pull/210). If you are interested you can refer to the [paper](https://arxiv.org/abs/1807.08957) for a nice overview of the models in the repository.
* **August 25, 2016** - Rolling shutter camera calibration support was added as a feature via PR [#65](https://github.com/ethz-asl/kalibr/pull/65). The [paper](https://www.cv-foundation.org/openaccess/content_cvpr_2013/papers/Oth_Rolling_Shutter_Camera_2013_CVPR_paper.pdf) provides details for those interested.
* **May 18, 2016** - Support for multiple IMU-to-IMU spacial and IMU intrinsic calibration was released.
* **June 18, 2014** - Initial public release of the repository.


## Authors
* Paul Furgale
* Hannes Sommer
* Jérôme Maye
* Jörn Rehder
* Thomas Schneider ([email](thomas.schneider@voliro.com))
* Luc Oth


## References
The calibration approaches used in Kalibr are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)

## Acknowledgments
This work is supported in part by the European Union's Seventh Framework Programme (FP7/2007-2013) under grants #269916 (V-Charge), and #610603 (EUROPA2).

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
