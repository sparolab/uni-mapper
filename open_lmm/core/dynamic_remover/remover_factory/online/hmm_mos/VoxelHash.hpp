#pragma once

#ifndef HMMMOS_VOXELHASH_H_
#define HMMMOS_VOXELHASH_H_

#include "eigen3/Eigen/Dense"

using Voxel = Eigen::Vector3i;

/**
 * @brief Voxel hash for uniquely storing entries described by (x,y,z) integers.
 *        Voxel hash map implementation from KISS-ICP (https://github.com/PRBonn/kiss-icp). 
 */
struct VoxelHash
{
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *> (voxel.data());
        return ((1 << 30) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

#endif // HMMMOS_VOXELHASH_H