<div align="center">
    <h1>KISS-ICP</h1>
    <p>with ESKF GPS+IMU Estimator</p>
    <a href="https://github.com/PRBonn/kiss-icp/releases"><img src="https://img.shields.io/github/v/release/PRBonn/kiss-icp?label=version" /></a>
    <a href="https://github.com/PRBonn/kiss-icp/blob/main/LICENSE"><img src="https://img.shields.io/github/license/PRBonn/kiss-icp" /></a>
    <a href="https://github.com/PRBonn/kiss-icp/blob/main/"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <!-- <a href="https://github.com/PRBonn/kiss-icp/blob/main/"><img src="https://img.shields.io/badge/Windows-0078D6?st&logo=windows&logoColor=white" /></a> -->
    <!-- <a href="https://github.com/PRBonn/kiss-icp/blob/main/"><img src="https://img.shields.io/badge/mac%20os-000000?&logo=apple&logoColor=white" /></a> -->
    <br />
    <br />
    <a href=https://user-images.githubusercontent.com/21349875/219626075-d67e9165-31a2-4a1b-8c26-9f04e7d195ec.mp4>Demo</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/PRBonn/kiss-icp/blob/main/README.md#Install">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/PRBonn/kiss-icp/blob/main/ros">ROS 2</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf>Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/PRBonn/kiss-icp/issues>Contact Us</a>
  <br />
  <br />

[KISS-ICP](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf) is a LiDAR Odometry pipeline that **just works** on most of the cases without tunning any parameter.

  <p align="center">
    <a href="https://user-images.githubusercontent.com/21349875/219626075-d67e9165-31a2-4a1b-8c26-9f04e7d195ec.mp4"><img alt="KISS-ICP Demo" src="https://user-images.githubusercontent.com/21349875/211829074-474bec08-0129-4e34-85e7-62265e44a7de.png"></a>
  </p>
</div>

<hr />

## <span style="color:Tomato">ROS support</span>
<details>
<summary>ROS 2</summary>

```sh
cd ~/ros2_ws/src/ && git clone https://github.com/PRBonn/kiss-icp && cd ~/ros2_ws/ && colcon build --packages-select kiss_icp
```

For more detailed instructions on the ROS wrapper, please visit this [ROS README](ros/README.md)
</details>

## Python Support
<details>
<summary>This should print the following help message:</summary>

![out](https://user-images.githubusercontent.com/21349875/193282970-25a400aa-ebcd-487a-b839-faa04eeca5b9.png)

For advanced instructions on the Python package please see [Python README](python/README.md)
</details>

## Citation

If you use this library for any academic work, please cite our original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf).

```bibtex
@article{vizzo2023ral,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {IEEE Robotics and Automation Letters (RA-L)},
  pages     = {1029--1036},
  doi       = {10.1109/LRA.2023.3236571},
  volume    = {8},
  number    = {2},
  year      = {2023},
  codeurl   = {https://github.com/PRBonn/kiss-icp},
}
```