#include "utility.hpp"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Dense>

int main()
{
    std::string base_map_name = "Maps/22F";
    std::string aligned_map_name = "Maps/22F-ext";
    std::string final_map_name = "Maps/22F-final";
    Eigen::Vector3f align_to_base_t(1.4694, -0.65825, 0.05324);
    Eigen::Vector3f align_to_base_rpy(0.57 * M_PI / 180.0, 3.43 * M_PI / 180.0, 79.5 * M_PI / 180.0); // Convert degrees to radians

    // NOTICE: DO NOT Try to understand this conversion, it is just to convert from ROS coordinate to LeGO-LOAM coordinate
    Eigen::Vector3f align_to_base_t_lego;
    align_to_base_t_lego.x() = align_to_base_t.y();
    align_to_base_t_lego.y() = align_to_base_t.z();
    align_to_base_t_lego.z() = align_to_base_t.x();
    Eigen::Vector3f align_to_base_rpy_lego;
    align_to_base_rpy_lego.x() = align_to_base_rpy.y();
    align_to_base_rpy_lego.y() = align_to_base_rpy.z();
    align_to_base_rpy_lego.z() = align_to_base_rpy.x();

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
    savePCDFile<PointType>(aligned_map_name + "/GlobalMap_transformed.pcd", aligned_map_transformed);

    PointCloudPtr final_cloud(new pcl::PointCloud<PointType>);
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setInputSource(aligned_map_transformed);
    ndt.setInputTarget(base_map);
    ndt.align(*final_cloud);
    auto finalTransformation = ndt.getFinalTransformation(); // NOTICE: 这是我们最终需要的变换矩阵
    std::cout << "Final finalTransformation matrix:\n" << finalTransformation << std::endl;
    // 从finalTransformation中提取旋转矩阵R_final和位移向量t_final
    Eigen::Matrix3f R_ndt = finalTransformation.block<3, 3>(0, 0);
    Eigen::Vector3f t_ndt = finalTransformation.block<3, 1>(0, 3);
    Eigen::Matrix3f R_final = R_ndt * align_to_base_R_lego;
    Eigen::Vector3f t_final = R_ndt * align_to_base_t_lego + t_ndt;

    /**
     * 加载CornerMap.pcd、SurfMap.pcd，应用R_final和t_final变换后与base_map的地图点云合并，最后合并成GlobalMap.pcd
     */
    PointCloudPtr base_corner_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(base_map_name + "/edited/CornerMap_active.pcd", base_corner_map);
    PointCloudPtr aligned_corner_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/CornerMap.pcd", aligned_corner_map);
    PointCloudPtr final_corner_map(new pcl::PointCloud<PointType>);
    applyTransform<PointType>(aligned_corner_map, R_final, t_final);
    *final_corner_map = *base_corner_map + *aligned_corner_map;
    savePCDFile<PointType>(final_map_name + "/CornerMap.pcd", final_corner_map);

    PointCloudPtr base_surf_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(base_map_name + "/edited/SurfMap_active.pcd", base_surf_map);
    PointCloudPtr aligned_surf_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/SurfMap.pcd", aligned_surf_map);
    PointCloudPtr final_surf_map(new pcl::PointCloud<PointType>);
    applyTransform<PointType>(aligned_surf_map, R_final, t_final);
    *final_surf_map = *base_surf_map + *aligned_surf_map;
    savePCDFile<PointType>(final_map_name + "/SurfMap.pcd", final_surf_map);

}