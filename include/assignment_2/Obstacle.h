#ifndef COLLISIONOBJECT_H
#define COLLISIONOBJECT_H

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>



class Obstacle {
private: 
    moveit_msgs::CollisionObject collisionObject_;
public: 
    Obstacle(const std::string tag_id, const std::string frame_id, const geometry_msgs::Pose pose);
    ~Obstacle() {}

    moveit_msgs::CollisionObject getCollisionObject() const {
        return collisionObject_;
    }

    moveit_msgs::CollisionObject createCollisionObject(const std::string id, const std::string frame_id, const geometry_msgs::Pose pose, const shape_msgs::SolidPrimitive primitive);
    shape_msgs::SolidPrimitive createBox(double x, double y, double z);
    shape_msgs::SolidPrimitive createHexagonalPrism();
    shape_msgs::SolidPrimitive createCube();
    shape_msgs::SolidPrimitive createTriangularPrism();
    shape_msgs::SolidPrimitive createTable();
};

#endif // COLLISIONOBJECT_H
