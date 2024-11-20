#include <kiss_icp/pipeline/KissICP.hpp>

int main() {
    struct kiss_icp::pipeline::KISSConfig config;
    config.deskew = true;
    config.initial_threshold = 2.0;
    config.min_range = 8.0;
    config.max_range = 100.0;
    config.max_points_per_voxel = 20;
    config.min_motion_th = 0.1;
    config.voxel_size = 1.0;
    kiss_icp::pipeline::KissICP icp_pipeline{config};
    // TODO: use pipeline here
}
