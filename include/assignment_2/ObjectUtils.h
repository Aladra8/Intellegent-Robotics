#ifndef OBJECT_UTILS_H
#define OBJECT_UTILS_H

#include <ros/ros.h>
#include <map>
#include <string>

namespace ObjectUtils {

enum ObjectType {
    CUBE,
    HEXAGON,
    TRIANGLE,
    UNKNOWN
};

namespace ObjectTypes {
    const std::string CUBE = "cube";
    const std::string HEXAGON = "hexagon";
    const std::string TRIANGLE = "triangle";
}  // namespace ObjectTypes

ObjectType getObjectType(const std::string& type);

extern std::map<std::string, std::string> objectLinkMap;
extern std::map<std::string, std::string> objectNameMap;

std::string getLinkName(const std::string& objectID);
std::string getName(const std::string& objectID);

} // namespace ObjectUtils

#endif // OBJECT_UTILS_H

