![Kalibr](https://raw.githubusercontent.com/wiki/schneith/Kalibr-test/images/kalibr_small.png)

##Introduction

Kalibr is a collection of tools developed at the Autonomous Systems Lab at ETH Zurich is a collection of tools for calibrating multi-camera systems as well as sensor units comprised of one or multiple cameras and an IMU.
Similarly to the well established camera calibration toolbox by Jean-Yves Bouguet [citation], it allows for intrinsic and extrinsic calibration of camera systems. Kalibr extends this functionality by supporting both N-camera systems as well as different projection and distortion models, which makes it  well suited for both pinhole and unified projection cameras cameras [cite Christopher Mei's projection model here that Paul is presumably using?] as well as for heterogeneous systems, where both types of cameras are present.  The ROS camera calibration toolbox [citation] introduced a heuristic for selecting a set of dissimilar views of the calibration target  from a stream of images to reduce redundancy in the calibration data and improve results. Kalibr takes this approach a step further by incorporating a more theoretically grounded, information theoretical measure to [cite Jerome here] for novel view selection.

In contrast to other EKF-based calibration methods [cite Jonathan Kelly, Mourikis here], the camera/IMU calibration tool employs a continuous-time batch formulation of the estimation problem, which allows for a simultaneous estimation of both the rigid transformation between the cameras and the IMU as well as the temporal offset between these sensors. In previous research [cite Paul here], this method has proven to accurately estimate time offsets of a fraction of the sampling time and emphasized the importance of accounting for exposure periods when timestaming camera images.

In addition to it core functionality, the toolbox provides convenience tools for focusing cameras and verifying calibration results.


Kalibr is a toolbox that solves the following calibration problems:

1. **Multiple camera calibration**: 
    intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view
1. **Camera-IMU calibration**:
    spatial and temporal calibration of an IMU w.r.t a camera-system

**Please find more information on the [wiki pages](https://github.com/ethz-asl/kalibr/wiki) of this repository.**

**For questions or comments, please use the [user forum](https://groups.google.com/forum/#!forum/kalibr-users).**

##Authors
* Paul Furgale ([email](paul.furgale@mavt.ethz.ch))
* Jérôme Maye ([email](jerome.maye@mavt.ethz.ch))
* Jörn Rehder ([email](joern.rehder@mavt.ethz.ch))
* Thomas Schneider ([email](schneith@ethz.ch))

## References
The calibration approaches used in Kalibr are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)

## Acknowledgments
This work is supported in part by the European Community's Seventh Framework Programme (FP7/2007-2013) under grant #269916 (V-Charge).

##License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
