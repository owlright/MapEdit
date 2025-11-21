#include "utility.hpp"
#include <iostream>
struct MapInfo // NOTICE: 按照ROS坐标系定义
{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    int width;
    int height;
    float map_resolution = 0.05;

    bool dumpConfig(const std::string& filename)
    {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return false;
        }
        ofs << "[Info]" << std::endl;
        ofs << "resolution=" << map_resolution << std::endl;
        ofs << "origin_x=" << x_min << std::endl;
        ofs << "origin_y=" << y_min << std::endl;
        ofs << "origin_z=0.0" << std::endl;
        ofs << "width=" << width << std::endl;
        ofs << "height=" << height << std::endl;
        ofs.close();
        return true;
    }
    float lego_x_min() { return y_min; }
    float lego_x_max() { return y_max; }
    float lego_y_min() { return z_min; }
    float lego_y_max() { return z_max; }
    float lego_z_min() { return x_min; }
    float lego_z_max() { return x_max; }
};

inline std::ostream& operator<<(std::ostream& os, const MapInfo& m)
{
    os << "MapInfo{"
       << " x_min: " << m.x_min
       << ", x_max: " << m.x_max
       << ", y_min: " << m.y_min
       << ", y_max: " << m.y_max
       << ", z_min: " << m.z_min
       << ", z_max: " << m.z_max
       << ", width: " << m.width
       << ", height: " << m.height
       << ", map_resolution: " << m.map_resolution
       << " }";
    return os;
}

MapInfo getBoundary(PointCloudPtr cloud, const float thre_z_min = -0.2, const float thre_z_max = 2.0)
{
    MapInfo map_info;
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    // lego-loam建立的地图 not the same with ros
    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX())); // fix frame trans,later
    // transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY())); // fix frame trans,first
    // pcl::transformPointCloud(*cloud, *cloud, transform);
    // for (const auto& point : cloud->points) {
    //     if (point.z < thre_z_min || thre_z_max > 2.0) continue; // filter ground and ceiling points
    //     if (point.x < x_min) x_min = point.x;
    //     if (point.x > x_max) x_max = point.x;
    //     if (point.y< y_min) y_min = point.y;
    //     if (point.y > y_max) y_max = point.y;
    // }
    for (const auto& point : cloud->points) { // HACK: swap x,y,z to get ros coordinate
        if (point.y < thre_z_min || point.y > thre_z_max) continue; // filter ground and ceiling points
        if (point.z < x_min) x_min = point.z;
        if (point.z > x_max) x_max = point.z;
        if (point.x < y_min) y_min = point.x;
        if (point.x > y_max) y_max = point.x;
        if (point.y < z_min) z_min = point.y;
        if (point.y > z_max) z_max = point.y;
    }

    map_info.x_min = x_min;
    map_info.x_max = x_max;
    map_info.y_min = y_min;
    map_info.y_max = y_max;
    map_info.z_min = z_min;
    map_info.z_max = z_max;
    map_info.width = static_cast<int>((x_max - x_min) / map_info.map_resolution) + 1;
    map_info.height = static_cast<int>((y_max - y_min) / map_info.map_resolution) + 1;
    // transform = Eigen::Affine3f::Identity();
    // transform.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));
    // transform.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()));
    // pcl::transformPointCloud(*cloud, *cloud, transform); // transform back
    return map_info;
}