Jeewon Kim, KAIST URL

You can generate "left.yaml", "right.yaml", and "realsense_stereo_imu_config.yaml" for VINS-Fusion using two cameras and one IMU, into the folder you give as a parameter when you run the kalibr_calibrate_imu_camera package.

Note that you should store the bag file for calibration in the folder you desired to use as config folder.

How to run

You need to add "--vin [folder path to store yaml files for vins fusion]" when you run kalibr_calibrate_imu_camera

Example)

$ cd kalibr_workspace
$ source devel/setup.bash
$ rosrun kalibr kalibr_calibrate_imu_camera --target '/home/jeewonkim/kalibr_workspace/cam_imu_cfg/apriltag.yaml' --bag '/home/jeewonkim/kalibr_workspace/config/calib2.bag' --cam '/home/jeewonkim/kalibr_workspace/cam_imu_cfg/trial3-camchain.yaml' --imu '/home/jeewonkim/kalibr_workspace/cam_imu_cfg/imu.yaml' --vins '/home/jeewonkim/catkin_ws/src/VINS-Fusion/config/realsense_d435i' 

