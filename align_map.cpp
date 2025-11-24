#include "utility.hpp"
#include "Pose.hpp"
#include "MapInfo.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

const char* config_file = "align.yaml";
double thre_z_min = -0.2;
double thre_z_max = 2.0;
std::string base_map_name = "Maps/22F";
std::string aligned_map_name = "Maps/22F-ext";
std::string final_map_name = "Maps/22F-final";
Eigen::Vector3f align_to_base_t = Eigen::Vector3f::Zero();
Eigen::Vector3f align_to_base_rpy_deg = Eigen::Vector3f::Zero();

void printParams() {
    std::cout << "==================== " << "MapEdit's params" <<  " ===================="<< std::endl;
    std::cout << "base_map_name: " << base_map_name << std::endl;
    std::cout << "aligned_map_name: " << aligned_map_name << std::endl;
    std::cout << "final_map_name: " << final_map_name << std::endl;
    std::cout << "align_to_base_t: [" << align_to_base_t.transpose() << "]" << std::endl;
    std::cout << "align_to_base_rpy_deg: [" << align_to_base_rpy_deg.transpose() << "]" << std::endl;
    std::cout << "thre_z_min: " << thre_z_min << std::endl;
    std::cout << "thre_z_max: " << thre_z_max << std::endl;
    std::cout << "==================== " << "MapEdit's params" <<  " ===================="<< std::endl;
}

int main(int argc, char **argv)
{
    // =================== 1.读取配置参数 ===================
    try {
        YAML::Node cfg = YAML::LoadFile(config_file);
        base_map_name = cfg["base_map_dir"].as<std::string>("Maps/22F");
        aligned_map_name = cfg["align_map_dir"].as<std::string>("Maps/22F-ext");
        final_map_name = cfg["final_map_dir"].as<std::string>("Maps/22F-final");
        thre_z_min = cfg["thre_z_min"].as<double>(-0.2);
        thre_z_max = cfg["thre_z_max"].as<double>(2.0);
        std::vector<double> t_vec = cfg["align_to_base_t"].as<std::vector<double>>();
        if (t_vec.size() == 3) {
            align_to_base_t = Eigen::Vector3f(t_vec[0], t_vec[1], t_vec[2]);
        } else {
            std::cerr << "align_to_base_t size is not 3!" << std::endl;
            return -1;
        }
        std::vector<double> rpy_vec = cfg["align_to_base_rpy_deg"].as<std::vector<double>>();
        if (rpy_vec.size() == 3) {
            align_to_base_rpy_deg = Eigen::Vector3f(rpy_vec[0] * M_PI / 180.0, rpy_vec[1] * M_PI / 180.0, rpy_vec[2] * M_PI / 180.0);
        } else {
            std::cerr << "align_to_base_rpy_deg size is not 3!" << std::endl;
            return -1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading " << config_file << e.what() << std::endl;
        return -1;
    }
    printParams();

    // NOTICE: Convert from ROS coordinate to LeGO-LOAM coordinate
    Eigen::Vector3f align_to_base_t_lego;
    align_to_base_t_lego.x() = align_to_base_t.y();
    align_to_base_t_lego.y() = align_to_base_t.z();
    align_to_base_t_lego.z() = align_to_base_t.x();
    Eigen::Vector3f align_to_base_rpy_lego;
    align_to_base_rpy_lego.x() = align_to_base_rpy_deg.y();
    align_to_base_rpy_lego.y() = align_to_base_rpy_deg.z();
    align_to_base_rpy_lego.z() = align_to_base_rpy_deg.x();

    // =================== 2.加载点云并初步对齐 ===================
    PointCloudPtr base_map(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(base_map_name + "/edited/CornerMap_active.pcd", base_map);
    // convertToROSCoordinate<PointType>(base_map);
    // savePCDFile<PointType>(base_map_name + "/GlobalMap_ros.pcd", base_map); // 调试

    PointCloudPtr aligned_map(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(aligned_map_name + "/CornerMap.pcd", aligned_map);
    // convertToROSCoordinate<PointType>(aligned_map);
    // savePCDFile<PointType>(aligned_map_name + "/GlobalMap_ros.pcd", aligned_map); // 调试

    auto align_to_base_R_lego = (Eigen::AngleAxisf(align_to_base_rpy_lego[2], Eigen::Vector3f::UnitZ()) *
                                 Eigen::AngleAxisf(align_to_base_rpy_lego[1], Eigen::Vector3f::UnitY()) *
                                 Eigen::AngleAxisf(align_to_base_rpy_lego[0], Eigen::Vector3f::UnitX())).toRotationMatrix();

    PointCloudPtr aligned_map_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    aligned_map_transformed->resize(aligned_map->points.size());
    applyTransform<PointType>(aligned_map, aligned_map_transformed, align_to_base_R_lego, align_to_base_t_lego);
    // savePCDFile<PointType>(aligned_map_name + "/GlobalMap_transformed.pcd", aligned_map_transformed);

    // =================== 3.NDT精细对齐 ===================
    PointCloudPtr final_cloud(new pcl::PointCloud<PointType>);
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setInputSource(aligned_map_transformed);
    ndt.setInputTarget(base_map);
    ndt.align(*final_cloud);
    auto finalTransformation = ndt.getFinalTransformation(); // NOTICE: 这是我们最终需要的变换矩阵
    std::cout << "Final finalTransformation matrix:\n" << finalTransformation << std::endl;
    Eigen::Matrix3f R_ndt = finalTransformation.block<3, 3>(0, 0);
    Eigen::Vector3f t_ndt = finalTransformation.block<3, 1>(0, 3);
    // 将初步对齐的变换和NDT的变换结合，得到最终的变换
    Eigen::Matrix3f R_final = R_ndt * align_to_base_R_lego;
    Eigen::Vector3f t_final = R_ndt * align_to_base_t_lego + t_ndt;

    // =================== 4.合并pose.txt和trajectory.pcd ===================
    std::map<int, Pose> base_poses;
    loadPosesFromFile(base_map_name + "/pose.txt", base_poses);
    std::map<int, Pose> aligned_poses;
    loadPosesFromFile(aligned_map_name + "/pose.txt", aligned_poses);

    int base_traj_max_idx = -1;

    // 加载trajectory_active.pcd才能知道哪些位姿没有被删除
    PointCloudPtr base_trajectory(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(base_map_name + "/edited/trajectory_active.pcd", base_trajectory);
    for (size_t i = 0; i < base_trajectory->points.size(); ++i) {
        auto& point = base_trajectory->points[i];
        int idx = static_cast<int>(point.intensity);
        if (idx > base_traj_max_idx) {
            base_traj_max_idx = idx;
        }
        base_poses.at(idx).active = true;
        // final_poses.emplace_back(base_poses.at(idx));
        // std::cout << "Base Traj Point " << i << ": " << point.x << " " << point.y << " " << point.z << " " << point.intensity << std::endl;
    }
    std::cout << "Base Traj Max Idx: " << base_traj_max_idx << std::endl;

    // 加载准备合并的地图的trajectory.pcd，修改它的位姿序号从base_map最大值开始递增
    PointCloudPtr aligned_trajectory(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(aligned_map_name + "/trajectory.pcd", aligned_trajectory);
    for (size_t i = 0; i < aligned_trajectory->points.size(); ++i) {
        auto &point = aligned_trajectory->points[i];
        int idx = static_cast<int>(point.intensity);
        point.intensity = idx + base_traj_max_idx + 1;
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f pt_transformed = R_final * pt + t_final;
        point.x = pt_transformed.x();
        point.y = pt_transformed.y();
        point.z = pt_transformed.z();
        auto &pose = aligned_poses.at(idx);
        pose.index = idx + base_traj_max_idx + 1;
        pose.position = align_to_base_R_lego * pose.position + align_to_base_t_lego;
        Eigen::Matrix3f R_pose = (Eigen::AngleAxisf(pose.rpy[2], Eigen::Vector3f::UnitZ()) *
                                  Eigen::AngleAxisf(pose.rpy[1], Eigen::Vector3f::UnitY()) *
                                  Eigen::AngleAxisf(pose.rpy[0], Eigen::Vector3f::UnitX()))
                                     .toRotationMatrix();
        R_pose = R_final * R_pose;
        auto zyx = R_pose.eulerAngles(2, 1, 0);
        pose.rpy.x() = zyx[2];
        pose.rpy.y() = zyx[1];
        pose.rpy.z() = zyx[0];
        // final_poses.emplace_back(aligned_poses.at(idx));
        // std::cout << "Aligned Traj Point " << i << ": " << point.x << " " << point.y << " " << point.z << " " << point.intensity << std::endl;
    }
    std::vector<Pose> final_poses;
    for (const auto& kv : base_poses) {
        if (kv.second.active) {
            final_poses.emplace_back(kv.second);
        }
    }
    for (auto& kv : aligned_poses) {
        final_poses.emplace_back(kv.second);
    }
    savePosesToFile(final_map_name + "/pose.txt", final_poses); // 1.保存融合后的pose.txt

    PointCloudPtr merged_trajectory(new pcl::PointCloud<pcl::PointXYZI>);
    *merged_trajectory = *base_trajectory + *aligned_trajectory;
    savePCDFile<PointType>(final_map_name + "/trajectory.pcd", merged_trajectory); // 2.保存融合后的trajectory.pcd

    // =================== 5.根据得到的变换合并地图 ===================
    // 加载CornerMap.pcd、SurfMap.pcd，应用R_final和t_final变换后与base_map的地图点云合并，最后合并成GlobalMap.pcd
    // trajectory.pcd中的点也需要应用同样的变换，并且修改点的intensity值以避免与base_map的轨迹点冲突
    // pose.txt中的位姿也需要相应修改
    PointCloudPtr base_corner_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(base_map_name + "/edited/CornerMap_active.pcd", base_corner_map);
    PointCloudPtr aligned_corner_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/CornerMap.pcd", aligned_corner_map);
    for (size_t i = 0; i < aligned_corner_map->points.size(); ++i) {
        auto& point = aligned_corner_map->points[i];
        int idx = static_cast<int>(point.intensity);
        point.intensity = idx + base_traj_max_idx + 1;
    }
    PointCloudPtr final_corner_map(new pcl::PointCloud<PointType>);
    applyTransform<PointType>(aligned_corner_map, R_final, t_final);
    *final_corner_map = *base_corner_map + *aligned_corner_map;
    savePCDFile<PointType>(final_map_name + "/CornerMap.pcd", final_corner_map);

    PointCloudPtr base_surf_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(base_map_name + "/edited/SurfMap_active.pcd", base_surf_map);
    PointCloudPtr aligned_surf_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/SurfMap.pcd", aligned_surf_map);
    for (size_t i = 0; i < aligned_surf_map->points.size(); ++i) {
        auto& point = aligned_surf_map->points[i];
        int idx = static_cast<int>(point.intensity);
        point.intensity = idx + base_traj_max_idx + 1;
    }
    PointCloudPtr final_surf_map(new pcl::PointCloud<PointType>);
    applyTransform<PointType>(aligned_surf_map, R_final, t_final);
    *final_surf_map = *base_surf_map + *aligned_surf_map;
    savePCDFile<PointType>(final_map_name + "/SurfMap.pcd", final_surf_map);

    PointCloudPtr final_global_map(new pcl::PointCloud<PointType>);
    *final_global_map = *final_corner_map + *final_surf_map;
    savePCDFile<PointType>(final_map_name + "/GlobalMap.pcd", final_global_map);
    getBoundary(final_global_map).dumpConfig(final_map_name + "/map.ini");

}