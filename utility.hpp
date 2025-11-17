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
    if (pcl::io::savePCDFileASCII(filename, *cloud) == -1) {
        std::cerr << "Failed to save point cloud to " << filename << std::endl;
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