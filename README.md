# OpenLMM

> [!note]
> This branch is a generalized version of **Uni-Mapper**. As the main author has graduated and follow-up work based on this paper is ongoing, the official code release is limited. Instead, we decided to provide **OpenLMM**, a more generalized version of **Uni-Mapper** that includes multi-map alignment and dynamic object removal modules.


<details>
<summary><b>📌 Uni-Mapper details</b></summary>

## Uni-mapper
[<img src="https://img.youtube.com/vi/SK0TU9Vy3Is/maxresdefault.jpg" alt="Uni-Mapper Youtube Video" width="100%">](https://www.youtube.com/watch?v=SK0TU9Vy3Is)






This repository is the official code of the paper:

> **Uni-Mapper: Unified Mapping Framework for Multi-modal LiDARs in Complex and Dynamic Environments**
>
> [Gilhwan Kang](https://scholar.google.com/citations?user=F6dY8DoAAAAJ&hl=ko), [Hogyun Kim](https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko), [Byunghee Choi](https://scholar.google.com/citations?user=JCJAwgIAAAAJ&hl=ko), [Seokhwan Jeong](https://scholar.google.com/citations?user=ZAO6skQAAAAJ&hl=ko), [Young-Sik Shin](https://scholar.google.com/citations?user=gGfBRawAAAAJ&hl=ko)&ast;, and [Younggun Cho](https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko&oi=ao)&ast;. <br>
> &ast; Corresponding Authors. <br> 
>
> *Accepted in Transaction on Intelligent Vehicles* <br>


## Introduction
**Uni-mapper** is a map-merging framework for multi-modal LiDARs in complex and dynamic environments.
Our approach consists of three core components: dynamic object removal, dynamic-aware scene description, and multiple map alignment. A voxel-wise free space hash map is built to remove dynamic objects by combining sequential free spaces. This is integrated with a stable triangle descriptor (STD) to form DynaSTD, which preserves static points and is effective across multi-modal LiDARs. DynaSTD is used for pose graph optimizations in intra-session and inter-map loop closures, with a centralized anchor-node approach to reduce intra-session drift errors.



## Table of Contents
> **__Note__** The code will be released after the paper is accepted.
>
> **Setup**
> - Installation
> - Datasets
>
> **Example**
> - Merging multiple maps


## How to use
- TBD


## Updates
- `25.05.31` : Accepted in Transaction on Intelligent Vehicles
- `24.10.09` : Resubmitted to T-IV
- `24.05.08` : Accepted in ICRAW on Future of Construction (3rd prize)



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
</details>

## Intro
**OpenLMM** is a modularized **L**iDAR **M**ap **M**erging and **L**ong-term **M**ap **M**anagement framework, including loop closure detection, robust optimization, and dynamic removal modules for long-term LiDAR mapping.



<!-- <p align="center">
  <img width='100%' src="https://gist.github.com/user-attachments/assets/73bade41-570d-447b-9831-6474bc97bbd3.mp4">
</p> -->

[Click here for a demo!](https://gist.github.com/user-attachments/assets/73bade41-570d-447b-9831-6474bc97bbd3)

<!-- <video width="100%" autoplay loop muted playsinline>
  <source src="https://gist.github.com/user-attachments/assets/73bade41-570d-447b-9831-6474bc97bbd3.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video> -->







**Features**

- **Data Loader**
  - Supports `pcd`, `bin`, and custom formats for scan data
  - Supports `KITTI`, `TUM`, and custom formats for pose data

- **Loop Detector**
  - Dynamic loading of place recognition (PR) modules
  - Supports k-d tree–based vector search modules (`Scan Context`, `Solid`)

- **Backend Optimizer**
  - Supports `iSAM2` based optimization

- **Dynamic Remover**
  - Dynamic loading of dynamic object removal (DOR) modules
  - Supports online removal modules (`HMM-MOS`, `DUFOMap`)
  - Supports offline removal module (`ERASOR`)
  

## Setting
### System requirenments
- [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [GTSAM 4.2a9](https://github.com/borglab/gtsam/releases/tag/4.2a9)


### Local build
```
# 1. make your own workspace
mkdir -p ws_OpenLMM/src
cd ws_OpenLMM/src

# 2. Clone repository
git clone https://github.com/hwan0806/open-lmm

# 3. colcon build
cd ..
colcon build --symlink-install

# 4. Run OpenLMM
source install/setup.bash
ros2 run open_lmm_ros open_lmm_rosnode
```

## Docker Build
### System requirenments
- docker
- docker-compose


### Docker setting
```
# 1. make your own workspace in local system
mkdir -p ws_OpenLMM/src
cd ws_OpenLMM/src

# 2. Clone repository
git clone https://github.com/hwan0806/open-lmm

# 3. Set open-lmm via dockerfile
cd open-lmm
bash run_docker.sh $(YOUR_LOCAL_DATASET_ROOT_PATH)

# 4. colcon build in docker container (via VsCode container extension, Terminal...)
cd /root/workspace
colcon build --symlink-install

# 5. Run OpenLMM
source install/setup.bash
ros2 run open_lmm_ros open_lmm_rosnode
```

## How to use?
### Example dataset
Sample dataset collected in our campus (**[Google Drive link](https://drive.google.com/drive/folders/1MiwAkoHn0tzPc5O6FQhFEykhTY4JBqxU?usp=sharing)**)

### Data format
```
root
├─ dataset_root
│  ├─ agent1
│  │  ├─ Scans
│  │  │  ├─ 000000.pcd
│  │  │  ├─ 000001.pcd
│  │  │  └─ ...
│  │  └─ poses.txt
│  ├─ agent2
│  │  ├─ Scans
│  │  └─ poses.txt
│  └─ ...
└─ ...
```
- **Refer to the `config/core/data_loader/file_based.json` file for detailed usage.**
- You can modify the `Scans` directory name, as well as the file name and extension of `poses.txt`.
- Supports `pcd` and `bin(KITTI)` file formats for scan data.
- Supports both `KITTI` and `TUM` pose formats.
- You can also define a `custom` type for both scan format and pose format.
- Since **Uni-Mapper** utilizes **STD** as the base loop descriptor, I also use accumulated scans as keyframes (by accumulating 10 frames of Fast-LIO2 output points).



# TODO
- [ ] Add visualization support
- [ ] Refactor the centralized `shared_data` structure
- [ ] Replace `PCL` types with `Eigen` vectors
- [ ] Add hash map-based LiDAR descriptors


## Acknowledgments
While working on the **Uni-Mapper** project, I (Gilhwan Kang) found it particularly challenging to tackle dynamic object removal, LiDAR-based place recognition, and map alignment all at once. Originally started as a personal study rather than as a novel research contribution, this project is intended to serve as a practical tool to help other researchers easily test or adapt their own custom datasets and algorithms when working on similar problems.

This project stands on the shoulders of giants—it is built upon a foundation of outstanding prior research and open-source contributions. The following acknowledgments are our gratitude for those invaluable works.
- **[LT-Mapper](https://github.com/gisbi-kim/lt-mapper)** – used as the baseline framework of **Uni-Mapper**
- **[GLIM](https://github.com/koide3/glim)** – referenced for dynamic loading modules and configuration structure
- **[KISS-ICP](https://github.com/PRBonn/kiss-icp)** – served as a baseline for the overall codebase structure and modern CMake setup
- **[KISS-MATCHER](https://github.com/MIT-SPARK/KISS-Matcher)** – provided essential map-to-map alignment functionalities
- **[ScanContext](https://github.com/DanMcGann/scan_context)** – used as the base DB structure for KD-tree-based scan retrieval
- **[SOLID](https://github.com/sparolab/solid)** – contributed insights into light-weight LiDAR descriptor
- **[Dynamic Benchmark](https://github.com/KTH-RPL/DynamicMap_Benchmark)** – offered C++-refactored versions of dynamic object removal algorithms such as **[ERASOR](https://github.com/LimHyungTae/ERASOR)** and **[DUFOMap](https://github.com/Kin-Zhang/dufomap)**
- **[HmmMOS](https://github.com/vb44/HMM-MOS)** – referred to as a SOTA online dynamic object removal approach

## Contact
Maintained by Gilhwan Kang and please contact the author via gilhwan@hyundai.com