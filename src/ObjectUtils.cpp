#include <ros/ros.h>
#include "assignment_2/ObjectUtils.h"
#include <map>
#include <string>


namespace ObjectUtils {

ObjectType getObjectType(const std::string& type) {
    if (type == ObjectTypes::CUBE) {
        return CUBE;
    } else if (type == ObjectTypes::HEXAGON) {
        return HEXAGON;
    } else if (type == ObjectTypes::TRIANGLE) {
        return TRIANGLE;
    } else {
        return UNKNOWN;
    }
}

std::map<std::string, std::string> objectLinkMap = {
    {"1", "Hexagon::Hexagon_link"},
    {"2", "Hexagon_2::Hexagon_2_link"},
    {"3", "Hexagon_3::Hexagon_3_link"},
    {"4", "cube::cube_link"},
    {"5", "cube_5::cube_5_link"},
    {"6", "cube_6::cube_6_link"},
    {"7", "Triangle::Triangle_link"},
    {"8", "triangle_8::Triangle_8_link"},
    {"9", "triangle_9::Triangle_9_link"},
};

std::map<std::string, std::string> objectNameMap = {
    {"1", "Hexagon"},
    {"2", "Hexagon_2"},
    {"3", "Hexagon_3"},
    {"4", "cube"},
    {"5", "cube_5"},
    {"6", "cube_6"},
    {"7", "Triangle"},
    {"8", "Triangle_8"},
    {"9", "Triangle_9"},
};

std::string getLinkName(const std::string& objectID) {
    auto it = objectLinkMap.find(objectID);
    if (it != objectLinkMap.end()) {
        return it->second;
    } else {
        ROS_WARN("No LINK found for object ID %s", objectID.c_str());
        return "";
    }
}

std::string getName(const std::string& objectID) {
    auto it = objectNameMap.find(objectID);
    if (it != objectNameMap.end()) {
        return it->second;
    } else {
        ROS_WARN("No NAME found for object ID %s", objectID.c_str());
        return "";
    }
}

} // namespace ObjectUtils