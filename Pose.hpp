#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>

struct Pose
{
    // NOTICE: position和rpy是LeGO-LOAM的顺序
    Eigen::Vector3f position;
    Eigen::Vector3f rpy;
    int index;
    double timestamp;
    bool active = false;

    // Overload << operator for Pose
    friend std::ostream& operator<<(std::ostream& os, const Pose& pose) {
        os << "Index: " << pose.index << ", "
           << "Active: " << (pose.active ? "True" : "False") << ", "
           << "Position: [" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << "], "
           << "RPY: [" << pose.rpy.x() << ", " << pose.rpy.y() << ", " << pose.rpy.z() << "], "
           << "Timestamp: " << pose.timestamp;
        return os;
    }
};

bool loadPosesFromFile(const std::string& filename, std::map<int, Pose>& poses)
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Failed to open pose file: " << filename << std::endl;
        return false;
    }
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        Pose pose;
        iss >> pose.position.x() >> pose.position.y() >> pose.position.z();
        iss >> pose.index >> pose.rpy.x() >> pose.rpy.y() >> pose.rpy.z() >> pose.timestamp;
        pose.active = false;
        poses[pose.index] = pose;
    }
    infile.close();
    return true;
}

bool savePosesToFile(const std::string& filename, const std::map<int, Pose>& poses)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Failed to open pose file for writing: " << filename << std::endl;
        return false;
    }
    for (const auto& pair : poses) {
        const Pose& pose = pair.second;
        outfile << pose.position.x() << " " << pose.position.y() << " " << pose.position.z() << " "
                << pose.index << " "
                << pose.rpy.x() << " " << pose.rpy.y() << " " << pose.rpy.z() << " "
                << pose.timestamp << std::endl;
    }
    outfile.close();
    return true;
}

bool savePosesToFile(const std::string& filename, const std::vector<Pose>& poses)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Failed to open pose file for writing: " << filename << std::endl;
        return false;
    }
    for (const auto& pose : poses) {
        outfile << pose.position.x() << " " << pose.position.y() << " " << pose.position.z() << " "
                << pose.index << " "
                << pose.rpy.x() << " " << pose.rpy.y() << " " << pose.rpy.z() << " "
                << pose.timestamp << std::endl;
    }
    outfile.close();
    return true;
}