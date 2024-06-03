# KISS-ICP ROS 2 Wrapper with GPS-IMU ESKF Estimator

### How to build

You should not need any extra dependency, just clone and build:

```sh
git clone https://github.com/PRBonn/kiss-icp
colcon build
source install/setup.bash
```

### How to run

# LIO(ICP, IMU-Preintegration, iSAM) + INS(ESKF)

```sh
ros2 launch kiss_icp odometry_lio_ins.launch.py
```

# LIO

```sh
ros2 launch kiss_icp odometry_lio.launch.py
```

# LO(ICP)

```sh
ros2 launch kiss_icp odometry_lo.launch.py
```
