
#pragma once
#include <Eigen/Core>
#include <vector>
namespace kiss_icp {
using IteratorPoints = std::vector<Eigen::Vector3d>::const_iterator;
using IteratorStamps = std::vector<double>::const_iterator;

struct PointCloudIterator {
    PointCloudIterator(const IteratorPoints &pts, const IteratorStamps &stamps)
        : pts_(pts), stamps_(stamps) {}

    const std::tuple<Eigen::Vector3d, double> &operator*() const {
        return std::make_tuple(*pts_, *stamps_);
    }

    PointCloudIterator &operator++() {
        ++pts_;
        ++stamps_;
        return *this;
    }

    IteratorPoints pts_;
    IteratorStamps stamps_;
};

struct StampedPointCloud {
    StampedPointCloud(const std::vector<Eigen::Vector3d> &points, const std::vector<double> &stamps)
        : points_(points), stamps_(stamps) {}

    PointCloudIterator begin() { return {points_.cbegin(), stamps_.cbegin()}; }
    PointCloudIterator end() { return {points_.cend(), stamps_.cend()}; }

    const std::vector<Eigen::Vector3d> &points_;
    const std::vector<double> &stamps_;
};
}  // namespace kiss_icp
