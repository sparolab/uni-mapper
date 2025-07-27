# Uni-Mapper

> 
> [!warning]
> This `uni-mapper` branch is the ICRA workshop version of **Uni-Mapper**. As it is based on ROS1 and has not been maintained for nearly two years, the codebase is currently unorganized and incomplete. When time permits, I plan to continue related research and eventually integrate this module into the `open-lmm` project.

<!-- ## Uni-Mapper -->

[<img src="https://img.youtube.com/vi/SK0TU9Vy3Is/maxresdefault.jpg" alt="Uni-Mapper Youtube Video" width="100%">](https://www.youtube.com/watch?v=SK0TU9Vy3Is)

This repository is the official code of the paper:
> **Uni-Mapper: Unified Mapping Framework for Multi-modal LiDARs in Complex and Dynamic Environments**
>
> [Gilhwan Kang](https://scholar.google.com/citations?user=F6dY8DoAAAAJ&hl=ko), [Hogyun Kim](https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko), [Byunghee Choi](https://scholar.google.com/citations?user=JCJAwgIAAAAJ&hl=ko), [Seokhwan Jeong](https://scholar.google.com/citations?user=ZAO6skQAAAAJ&hl=ko), [Young-Sik Shin](https://scholar.google.com/citations?user=gGfBRawAAAAJ&hl=ko)&ast;, and [Younggun Cho](https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko&oi=ao)&ast;. <br>
> &ast; Corresponding Authors. <br> 
>
> *Accepted in Transaction on Intelligent Vehicles* <br>

## Updates
- `25.05.31` : Accepted in Transaction on Intelligent Vehicles
- `24.10.09` : Resubmitted to T-IV
- `24.05.08` : Accepted in ICRAW on Future of Construction (3rd prize)


## Introduction
**Uni-mapper** is a map-merging framework for multi-modal LiDARs in complex and dynamic environments.
Our approach consists of three core components: dynamic object removal, dynamic-aware scene description, and multiple map alignment. A voxel-wise free space hash map is built to remove dynamic objects by combining sequential free spaces. This is integrated with a stable triangle descriptor (STD) to form DynaSTD, which preserves static points and is effective across multi-modal LiDARs. DynaSTD is used for pose graph optimizations in intra-session and inter-map loop closures, with a centralized anchor-node approach to reduce intra-session drift errors.



## Table of Contents
> **Setup**
> - Prerequisites
> - Installation
> - Datasets
>
> **How to Use**
> - Merging multiple maps
> 
> **Citation**
> 
> **Acknowledgments**

## Setup
### Prerequisites
- docker
- nvidia-docker
- docker compose


### Installation
```
# 0. make your own workspace
mkdir -p ws_UniMapper/src
cd ws_UniMapper/src

# 1. Clone repository
git clone -b workshop https://github.com/sparolab/uni-mapper.git

# 2. Set uni-mapper via dockerfile
cd uni-mapper
bash run_docker.sh $(YOUR_DATASET_PATH)

# 3. Catkin_make in docker container (via VsCode-Docker, Terminal...)
cd /root/workspace
catkin_make

# 4. Run UniMapper
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
source devel/setup.bash
roslaunch uni_mapper run.launch
```

### Datasets
```
root
├─ dataset_root
│  ├─ map1
│  │  ├─ Scans
│  │  │  ├─ 000000.pcd
│  │  │  ├─ 000001.pcd
│  │  │  └─ ...
│  │  └─ optimized_poses.txt (kitti format)
│  ├─ map2
│  │  ├─ Scans
│  │  └─ optimized_poses.txt (kitti format)
│  └─ ...
└─ ...
```

- All `Scans` and `optimized_poses.txt` is same with [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM). Refer ths repository.
- Because `UniMapper` utilize `STD` as a base loop descriptor, you have to use accumulated scans (accumulating 10 frames is enough)
- Test data will be uploaded soon.

## How to use
- TBD



## Citation
```
@article{kang2025uni,
  title={Uni-Mapper: Unified Mapping Framework for Multi-modal LiDARs in Complex and Dynamic Environments},
  author={Kang, Gilhwan and Kim, Hogyun and Choi, Byunghee and Jeong, Seokhwan and Shin, Young-Sik and Cho, Younggun},
  journal={IEEE Transactions on Intelligent Vehicles},
  year={2025},
  publisher={IEEE}
}
```

## Acknowledgments
We would like to express our sincere gratitude to **[Prof. Giseop Kim](https://sites.google.com/view/aprl-dgist/people)** for developing **[LT-Mapper](https://github.com/gisbi-kim/lt-mapper)**, which served as the foundation for the **Uni-Mapper** project, and to **[HKU MARS Lab](https://github.com/hku-mars)** for their contribution of **[STD](https://github.com/hku-mars/STD)**, a state-of-the-art LiDAR descriptor.


