# KISS-ICP ROS 2 Wrapper with GPS-IMU ESKF Estimator

### How to build

You should not need any extra dependency, just clone and build:

```sh
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev

git clone git@gitlab.com:iu-vail/iu-racing/ai-driver/vehicle-autonomy-stack.git
colcon build
source install/setup.bash
```

### How to run

##### LIO(ICP, IMU-Preintegration, iSAM) + INS(ESKF)

```sh
ros2 launch kiss_icp odometry.launch.py
```

##### CONFIG
located in ros/config/params.yaml
I assume the config file is very eary to understand.