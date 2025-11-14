#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Dense>

template <typename T>
bool loadPCDFile(const std::string& filename, typename pcl::PointCloud<T>::Ptr& cloud)
{
    if (pcl::io::loadPCDFile<T>(filename, *cloud) == -1) {
        std::cerr << "Couldn't read file " << filename << std::endl;
        return false;
    }
    std::cout << "Loaded " << cloud->points.size() << " data points from " << filename << std::endl;
    return true;
}

template <typename T>
void convertToROSCoordinate(typename pcl::PointCloud<T>::Ptr& cloud)
{
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto& point = cloud->points[i];
        double x = point.x;
        double y = point.y;
        double z = point.z;
        point.y = x;
        point.z = y;
        point.x = z;
    }
}

template <typename T>
void convertToLeGOLOAMCoordinate(typename pcl::PointCloud<T>::Ptr& cloud)
{
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto& point = cloud->points[i];
        double x = point.x;
        double y = point.y;
        double z = point.z;
        point.z = x;
        point.x = y;
        point.y = z;
    }
}

int main()
{
    using PointType = pcl::PointXYZI;
    using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
    Eigen::Vector3f align_to_base_t(1.4694, -0.65825, 0.05324);
    Eigen::Quaternionf align_to_base_q(0.768279, -0.0164673, 0.0277229, 0.6393025);
    align_to_base_q.normalize();
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    Eigen::Vector3f euler_angles = align_to_base_q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order
    std::cout << "Euler Angles (Yaw, Pitch, Roll): " << euler_angles.transpose() << std::endl;
    std::string global_map_file = "global_map.pcd";
    PointCloudPtr global_map(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(global_map_file, global_map);
    convertToROSCoordinate<PointType>(global_map);
    if (pcl::io::savePCDFileASCII("global_map_swapped.pcd", *global_map) == -1) {
        std::cerr << "Failed to save swapped point cloud to global_map_swapped.pcd" << std::endl;
        return -1;
    }
    std::string aligned_map_file = "aligned_map.pcd";
    PointCloudPtr aligned_map(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(aligned_map_file, aligned_map);
    convertToROSCoordinate<PointType>(aligned_map);

    if (pcl::io::savePCDFileASCII("aligned_map_swapped.pcd", *aligned_map) == -1) {
        std::cerr << "Failed to save swapped point cloud to aligned_map_swapped.pcd" << std::endl;
        return -1;
    }
    auto R = align_to_base_q.toRotationMatrix();
    PointCloudPtr aligned_map_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    aligned_map_transformed->resize(aligned_map->points.size());
    for (size_t i = 0; i < aligned_map->points.size(); ++i) {
        const auto& point = aligned_map->points[i];
        Eigen::Vector3f pt(point.x, point.y, point.z);
        // Apply rotation and translation
        Eigen::Vector3f pt_transformed = R * pt + align_to_base_t;
        // Assign transformed point
        auto& new_point = aligned_map_transformed->points[i];
        new_point.x = pt_transformed.x();
        new_point.y = pt_transformed.y();
        new_point.z = pt_transformed.z();
        new_point.intensity = point.intensity;
    }

    // Save the transformed point cloud
    std::string transformed_map_file = "aligned_map_transformed.pcd";
    // pcl::IterativeClosestPoint<PointType, PointType> icp;
    // icp.setInputSource(aligned_map_transformed);
    // icp.setInputTarget(global_map);
    // PointCloudPtr final_cloud(new pcl::PointCloud<PointType>);
    // icp.align(*final_cloud);
    // std::cout << "ICP has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    // Eigen::Matrix4f transformation = icp.getFinalTransformation();
    // std::cout << "Final transformation matrix:\n" << transformation << std::endl;d.pcd";
    if (pcl::io::savePCDFileASCII(transformed_map_file, *aligned_map_transformed) == -1) {
        std::cerr << "Failed to save transformed point cloud to " << transformed_map_file << std::endl;
        return -1;
    }
    std::cout << "Saved transformed point cloud to " << transformed_map_file << std::endl;

    PointCloudPtr final_cloud(new pcl::PointCloud<PointType>);
    // pcl::IterativeClosestPoint<PointType, PointType> icp;
    // icp.setInputSource(aligned_map_transformed);
    // icp.setInputTarget(global_map);

    // icp.align(*final_cloud);
    // std::cout << "ICP has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    // Eigen::Matrix4f transformation = icp.getFinalTransformation();
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setInputSource(aligned_map_transformed);
    ndt.setInputTarget(global_map);
    ndt.align(*final_cloud);
    auto transformation = ndt.getFinalTransformation();
    std::cout << "Final transformation matrix:\n" << transformation << std::endl;

    // Extract rotation matrix (top-left 3x3 submatrix)
    Eigen::Matrix3f rotation_matrix = transformation.block<3, 3>(0, 0);

    // Convert rotation matrix to Euler angles (ZYX order: Yaw, Pitch, Roll)
    Eigen::Vector3f euler_angles2 = rotation_matrix.eulerAngles(2, 1, 0); // ZYX order
    float yaw = euler_angles2[0];                                         // Z-axis rotation
    float pitch = euler_angles2[1];                                       // Y-axis rotation
    float roll = euler_angles2[2];                                        // X-axis rotation
    std::cout << "Yaw, Pitch, Roll " << yaw << " " << pitch << " " << roll << std::endl;
    if (pcl::io::savePCDFileASCII("final_aligned_map.pcd", *final_cloud) == -1) {
        std::cerr << "Failed to save final aligned point cloud to final_aligned_map.pcd" << std::endl;
        return -1;
    }
}