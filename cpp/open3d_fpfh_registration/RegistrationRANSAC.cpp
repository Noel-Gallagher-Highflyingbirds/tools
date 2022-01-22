// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"

using namespace open3d;
using namespace std;
std::tuple<std::shared_ptr<geometry::PointCloud>,
           std::shared_ptr<pipelines::registration::Feature>>
PreprocessPointCloud(const char *file_name, float voxel_size = 2.0) {
    //从文件读取点云
    auto pcd = open3d::io::CreatePointCloudFromFile(file_name);
    //降采样
    auto pcd_down = pcd->VoxelDownSample(voxel_size);
    std::cout << "read " << pcd->points_.size() << " points from " << file_name
              << std::endl;
    std::cout << "voxel_size=" << voxel_size << ", after downsample "
              << pcd_down->points_.size() << "points left" << endl;
    //计算法向量
    pcd_down->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2, 30));
    
    /*
    https://github.com/isl-org/Open3D/blob/master/cpp/open3d/geometry/PointCloud.h#L239
    void OrientNormalsToAlignWithDirection(
    const Eigen::Vector3d& orientation_reference =
        Eigen::Vector3d(0.0, 0.0, 1.0));
    https://github.com/isl-org/Open3D/blob/7c62640441e3da18bcbe146723ed83ff544b2fbb/cpp/open3d/geometry/EstimateNormals.cpp#L338
    */
    //指定法向量方向
    pcd_down->OrientNormalsToAlignWithDirection();
    //计算fpfh特征
    auto pcd_fpfh = pipelines::registration::ComputeFPFHFeature(
            *pcd_down,
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));
    return std::make_tuple(pcd_down, pcd_fpfh);
}


void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > RegistrationRANSAC source_pcd target_pcd  voxel_size [--method=feature_matching] [--mutual_filter] [--visualize]");
    // clang-format on
    utility::LogInfo("");
}

int main(int argc, char *argv[]) {
    using namespace open3d;

    clock_t start_time = clock();

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 4 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    bool visualize = false;
    if (utility::ProgramOptionExists(argc, argv, "--visualize")) {
        visualize = true;
    }
    //visualize = true;
    utility::optional<unsigned int> seed_ = utility::nullopt;
    if (utility::ProgramOptionExists(argc, argv, "--fix_seed")){
        seed_ = 123456;
    }

    bool mutual_filter = false;
    //if (utility::ProgramOptionExists(argc, argv, "--mutual_filter")) {
    //    mutual_filter = true;
    //}
    mutual_filter = true;

    // Prepare input
    std::shared_ptr<geometry::PointCloud> source, target;
    std::shared_ptr<pipelines::registration::Feature> source_fpfh, target_fpfh;
    //set voxel_size
    float voxel_size = std::atof(argv[3]);
    std::cout << voxel_size << std::endl;
    float distance_threshold = voxel_size * 1.5;

    std::tie(source, source_fpfh) = PreprocessPointCloud(argv[1], voxel_size);
    std::tie(target, target_fpfh) = PreprocessPointCloud(argv[2], voxel_size);

    pipelines::registration::RegistrationResult registration_result;
    clock_t pre_proccess_time = clock();
    std::cout << "pre_proccess costs: " << (double)(pre_proccess_time - start_time) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;


    // Prepare checkers
    std::vector<std::reference_wrapper<
            const pipelines::registration::CorrespondenceChecker>>
            correspondence_checker;
    auto correspondence_checker_edge_length =
            pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
                    0.9);
    auto correspondence_checker_distance =
            pipelines::registration::CorrespondenceCheckerBasedOnDistance(
                    distance_threshold);

    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);


    registration_result = pipelines::registration::
        RegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_fpfh, *target_fpfh,
            mutual_filter, distance_threshold,
            pipelines::registration::
            TransformationEstimationPointToPoint(false),
            3, correspondence_checker,
            pipelines::registration::RANSACConvergenceCriteria(1000000, 0.999),seed_);
    cout << endl
            << "fpfh matrix:" << endl
            << registration_result.transformation_ << endl;
    cout << "inlier(correspondence_set size):"
            << registration_result.correspondence_set_.size() << endl;
    clock_t ransac_time = clock();
    std::cout << "ransac costs: " << (double)(ransac_time - pre_proccess_time) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;

    clock_t total_time = clock();
    std::cout << "total time: " << (double)(total_time - start_time) / (double)CLOCKS_PER_SEC << " s" << std::endl;


    std::vector<std::pair<int, int>> correspondences_ransac;
    for (int m = 0; m < registration_result.correspondence_set_.size();
            ++m) {
        std::pair<int, int> pair_(0, 0);
        pair_.first = registration_result.correspondence_set_[m][0];
        pair_.second = registration_result.correspondence_set_[m][1];
        correspondences_ransac.push_back(pair_);
    }
    // std::shared_ptr<open3d::geometry::LineSet>
    auto ransac_lineset =
            geometry::LineSet::CreateFromPointCloudCorrespondences(
                    *source, *target, correspondences_ransac);

    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
            new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> source_ptr(
            new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ptr(
            new geometry::PointCloud);
    *source_transformed_ptr = *source;
    *target_ptr = *target;
    *source_ptr = *source;
    target->PaintUniformColor({1, 0, 0});                  //红
    source->PaintUniformColor({0, 1, 0});                  //绿
    source_transformed_ptr->PaintUniformColor({0, 0, 1});  //蓝
    source_transformed_ptr->Transform(registration_result.transformation_);
    
    if (visualize) {
        visualization::DrawGeometries(
                {target, source, source_transformed_ptr, ransac_lineset},
                "Registration result", 960, 900, 960, 100);
    }

    return 0;
}
