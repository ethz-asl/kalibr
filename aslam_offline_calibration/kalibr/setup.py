## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['kalibr_errorterms',
              'kalibr_common',
              'kalibr_camera_calibration',
              'kalibr_imu_camera_calibration'],
    package_dir={'':'python'},
    scripts=['python/kalibr_bagcreater',
             'python/kalibr_bagextractor',
             'python/kalibr_calibrate_cameras',
             'python/kalibr_calibrate_imu_camera',
             'python/kalibr_camera_focus',
             'python/kalibr_camera_validator',
             'python/kalibr_create_target_pdf',
             'python/kalibr_aslam_config']
)

setup(**setup_args)