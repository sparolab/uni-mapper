# Uni-mapper
[**Paper**]() | [**Website**](https://sites.google.com/view/uni-mapper/home) | [**Video**](https://www.youtube.com/watch?v=MjOGxNkyeOM)


This repository is the official code of the paper:

> **Uni-Mapper: Unified Mapping Framework for Multi-modal LiDARs in Complex and Dynamic Environments**
>
> [Gilhwan Kang](https://scholar.google.com/citations?user=F6dY8DoAAAAJ&hl=ko), [Hogyun Kim](https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko), [Byunghee Choi](), [Seokhwan Jeong](), [Young-Sik Shin](https://scholar.google.com/citations?user=gGfBRawAAAAJ&hl=ko)&ast;, and [Younggun Cho](https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko&oi=ao)&ast;. <br>
> &ast; Corresponding Authors. <br> 
>
> *under review for journal paper* <br>
> (Best Research Award 3rd prize for *ICRA* workshop on Future of Construction, 2024.)

![main](./fig/main.jpg)

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