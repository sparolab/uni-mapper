#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>

struct EIGEN_ALIGN16 PointXYZILI
{
    PCL_ADD_POINT4D;

    float intensity;
    uint16_t label;
    int ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    inline PointXYZILI()
    {
      x = y = z = intensity = 0.0f;
      label = 0;
      ring = 0;
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILI,
	(float,x,x)
	(float,y,y)
	(float,z,z)
	(float,intensity,intensity)
	(uint16_t,label,label)
    (int,ring,ring)
)

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

namespace otd::common {

constexpr double G_m_s2 = 9.81;  // Gravity const in GuangDong/China

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::array<S, 3> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::array<S, 9> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

}  // namespace otd::common
#endif