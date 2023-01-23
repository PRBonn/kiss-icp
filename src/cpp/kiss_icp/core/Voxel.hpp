// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <Eigen/Core>
#include <array>
#include <vector>

namespace kiss_icp {

struct Voxel {
    Voxel(int32_t x, int32_t y, int32_t z) : ijk({x, y, z}) {}
    Voxel(const Eigen::Vector3d &point, double voxel_size)
        : ijk({static_cast<int32_t>(point.x() / voxel_size),
               static_cast<int32_t>(point.y() / voxel_size),
               static_cast<int32_t>(point.z() / voxel_size)}) {}
    inline bool operator==(const Voxel &vox) const {
        return ijk[0] == vox.ijk[0] && ijk[1] == vox.ijk[1] && ijk[2] == vox.ijk[2];
    }

    std::array<int32_t, 3> ijk;
};
}  // namespace kiss_icp

// Specialization of std::hash for our custom type Voxel
namespace std {

template <>
struct hash<kiss_icp::Voxel> {
    std::size_t operator()(const kiss_icp::Voxel &vox) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(vox.ijk.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
}  // namespace std
