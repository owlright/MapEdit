#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using PointType = pcl::PointXYZI;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;

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
bool savePCDFile(const std::string& filename, typename pcl::PointCloud<T>::Ptr& cloud)
{
    try {
        if (pcl::io::savePCDFileASCII(filename, *cloud) == -1) {
            std::cerr << "Failed to save point cloud to " << filename << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << filename << " " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "Unknown error occurred while saving point cloud to " << filename << std::endl;
        return false;
    }

    std::cout << "Saved " << cloud->points.size() << " data points to " << filename << std::endl;
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

template <typename T>
void applyTransform(const typename pcl::PointCloud<T>::Ptr& cloudIn, typename pcl::PointCloud<T>::Ptr& cloudOut,
                    const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
{
    cloudOut->resize(cloudIn->points.size());
    for (size_t i = 0; i < cloudIn->points.size(); ++i) {
        const auto& point = cloudIn->points[i];
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f pt_transformed = R * pt + t;
        auto& new_point = cloudOut->points[i];
        new_point.x = pt_transformed.x();
        new_point.y = pt_transformed.y();
        new_point.z = pt_transformed.z();
        new_point.intensity = point.intensity;
    }
}

template <typename T>
void applyTransform(typename pcl::PointCloud<T>::Ptr& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
{
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto& point = cloud->points[i];
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f pt_transformed = R * pt + t;
        point.x = pt_transformed.x();
        point.y = pt_transformed.y();
        point.z = pt_transformed.z();
    }
}