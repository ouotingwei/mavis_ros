# mavis_ros

A ROS 1 wrapper for [OpenMAVIS](https://github.com/MAVIS-SLAM/OpenMAVIS), enabling dataset evaluation via rosbag playback.

OpenMAVIS is an open-source implementation of [MAVIS](https://arxiv.org/abs/2309.08142) — an optimization-based Visual-Inertial SLAM system for multiple partially-overlapped camera configurations, presented at ICRA 2024.

---

## What This Repo Adds

The original OpenMAVIS runs on offline datasets in EuRoC format. This wrapper extends it with:

- **ROS 1 (Noetic) compatibility** — run the system directly from `.bag` files
- **`rosbag`-based dataset pipeline** — subscribe to image and IMU topics in real time
- **Image enhancement preprocessing** — adaptive Gamma correction + CLAHE for low-light sequences

---

## System Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 20.04 |
| ROS | Noetic |
| OpenCV | ≥ 4.2 |
| Eigen | ≥ 3.3 |
| C++ | 17 |

---

## Installation

```bash
# 1. Create workspace
mkdir -p ~/mavis_ros/src && cd ~/mavis_ros/src

# 2. Clone this repo
git clone https://github.com/<YOUR_USERNAME>/mavis_ros.git

# 3. Initialize submodules (required — includes g2o, DBoW2)
cd mavis_ros
git submodule update --init --recursive

# 4. Build
cd ~/mavis_ros
catkin_make -DCMAKE_BUILD_TYPE=Release
```

---

## Running with Hilti Challange Dataset

```bash
# Terminal 1 — launch the SLAM node
source devel/setup.bash
roslaunch mavis_ros mavis.launch

# Terminal 2 — play your bag
rosbag play your_dataset.bag
```

Edit `launch/mavis.launch` to set your vocabulary path, config file, and topic names.

---


## Tested Datasets

| Dataset | Sequence | Notes |
|---|---|---|
| EuRoC MAV | MH_01 ~ MH_05, V1, V2 | Standard benchmark |
| Hilti 2022 | site1, site2 | Multi-cam fisheye |

---

## Image Enhancement

For dark or low-contrast sequences, an optional preprocessing step applies:

1. **Adaptive Gamma correction** — based on mean image brightness
2. **CLAHE** — local contrast enhancement per tile

> ⚠️ This modifies pixel values. Not recommended if using direct-method SLAM backends.

---

## Acknowledgements

This project builds on:

- [OpenMAVIS](https://github.com/MAVIS-SLAM/OpenMAVIS) — MAVIS-SLAM Team
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) — Campos et al.

Please cite the original paper if you use this in academic work:

```bibtex
@inproceedings{wang2024mavis,
  title     = {MAVIS: Multi-Camera Augmented Visual-Inertial SLAM using SE2(3) Based Exact IMU Pre-integration},
  author    = {Wang, Yifu and Ng, Yonhon and Sa, Inkyu and Parra, Alvaro and
               Rodriguez-Opazo, Cristian and Lin, Taojun and Li, Hongdong},
  booktitle = {2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages     = {1694--1700},
  year      = {2024}
}
```

---

## License

This wrapper follows the license of OpenMAVIS. See [LICENSE](./LICENSE) for details.