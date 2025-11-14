#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " input.pcd" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file, *cloud) == -1) {
        std::cerr << "Couldn't read file " << input_file << std::endl;
        return -1;
    }

    // 按intensity分组
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> frames;
    for (const auto& pt : cloud->points) {
        int frame_id = static_cast<int>(pt.intensity);
        if (frames.find(frame_id) == frames.end()) {
            frames[frame_id] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }
        frames[frame_id]->points.push_back(pt);
    }

    // 保存每一帧
    for (const auto& kv : frames) {
        std::string out_name = "frame_" + std::to_string(kv.first) + ".pcd";
        kv.second->width = kv.second->points.size();
        kv.second->height = 1;
        kv.second->is_dense = true;
        pcl::io::savePCDFileBinary(out_name, *kv.second);
        std::cout << "Saved " << out_name << " with " << kv.second->points.size() << " points." << std::endl;
    }

    return 0;
}