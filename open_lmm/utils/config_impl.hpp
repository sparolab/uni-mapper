#pragma once

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <open_lmm/utils/logging.hpp>
#include <nlohmann/json.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "config.hpp"
#include "convert_to_string.hpp"

namespace open_lmm {

namespace {

// IO traits
template <typename T>
struct traits {
  using InType = T;
  using OutType = T;

  static std::optional<OutType> convert(const InType& in) { return in; }

  static InType invert(const OutType& value) { return value; }
};

// General Eigen IO
template <typename T, int N, int M>
struct traits<Eigen::Matrix<T, N, M>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Matrix<T, N, M>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != N * M) {
      return std::nullopt;
    }
    return Eigen::Map<const OutType>(in.data());
  }

  static std::vector<double> invert(const OutType& value) { return std::vector<double>(value.data(), value.data() + N * M); }
};

// Eigen Quaternion IO
template <typename T>
struct traits<Eigen::Quaternion<T>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Quaternion<T>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 4) {
      return std::nullopt;
    }

    return OutType(in.data()).normalized();
  }

  static std::vector<double> invert(const OutType& value) { return std::vector<double>{value.x(), value.y(), value.z(), value.w()}; }
};

// Eigen Isometry
template <typename T>
struct traits<Eigen::Transform<T, 3, Eigen::Isometry>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Transform<T, 3, Eigen::Isometry>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 7) {
      return std::nullopt;
    }

    OutType se3 = OutType::Identity();
    se3.translation() = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(in.data());
    se3.linear() = Eigen::Quaternion<T>(in.data() + 3).normalized().toRotationMatrix();
    return se3;
  }

  static std::vector<double> invert(const OutType& value) {
    Eigen::Matrix<T, 3, 1> t = value.translation();
    Eigen::Quaternion<T> q(value.linear());
    return std::vector<double>{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
  }
};

template <>
struct traits<std::vector<Eigen::Isometry3d>> {
  using InType = std::vector<double>;
  using OutType = std::vector<Eigen::Isometry3d>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() % 7) {
      return std::nullopt;
    }

    OutType poses(in.size() / 7);
    for (unsigned int i = 0; i < poses.size(); i++) {
      poses[i].setIdentity();
      poses[i].translation() << in[i * 7], in[i * 7 + 1], in[i * 7 + 2];
      poses[i].linear() = Eigen::Quaterniond(in[i * 7 + 6], in[i * 7 + 3], in[i * 7 + 4], in[i * 7 + 5]).normalized().toRotationMatrix();
    }

    return poses;
  }

  static std::vector<double> invert(const OutType& value) {
    std::vector<double> values(value.size() * 7);
    for (unsigned int i = 0; i < value.size(); i++) {
      Eigen::Vector3d t = value[i].translation();
      Eigen::Quaterniond q(value[i].linear());

      values[i * 7] = t.x();
      values[i * 7 + 1] = t.y();
      values[i * 7 + 2] = t.z();
      values[i * 7 + 3] = q.x();
      values[i * 7 + 4] = q.y();
      values[i * 7 + 5] = q.z();
      values[i * 7 + 6] = q.w();
    }

    return values;
  }
};

}  // namespace

template <typename T>
T Config::param_cast(const std::string& module_name, const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  auto module = json.find(module_name);
  if (module == json.end()) {
    throw std::runtime_error("required config parameter " + module_name +
                             "/" + param_name + " not found");
  }

  auto parameter = module->find(param_name);
  if (parameter == module->end()) {
    throw std::runtime_error("required config parameter " + module_name +
                             "/" + param_name + " not found");
  }

  try {
    auto converted = traits<T>::convert(
        parameter->get<typename traits<T>::InType>());
    if (!converted) {
      throw std::runtime_error("invalid value shape");
    }
    return *converted;
  } catch (const std::exception& e) {
    throw std::runtime_error("invalid config parameter " + config_path + ":" +
                             module_name + "/" + param_name + ": " + e.what());
  }
}

#define DEFINE_CONFIG_IO_SPECIALIZATION(TYPE)                                  \
  template TYPE Config::param_cast(const std::string&, const std::string&) const;

}  // namespace open_lmm
