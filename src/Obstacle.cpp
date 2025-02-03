#include "assignment_2/Obstacle.h"
#include <ros/ros.h>


// Constants for the dimensions of each shape
const double HEXAGONAL_PRISM_HEIGHT = 0.2;
const double HEXAGONAL_PRISM_RADIUS = 0.05;

const double CUBE_LENGTH = 0.05;

const double TRIANGULAR_PRISM_BASE = 0.05;
const double TRIANGULAR_PRISM_HEIGHT = 0.035;
const double TRIANGULAR_PRISM_LENGTH = 0.07;

const double TABLE_LENGTH = 0.9;
const double TABLE_HEIGHT = 0.9;

Obstacle::Obstacle(const std::string tag_id, const std::string frame_id, const geometry_msgs::Pose pose) {
    shape_msgs::SolidPrimitive primitive;
    int tag_id_num = std::stoi(tag_id);
    geometry_msgs::Pose adjusted_pose = pose;

    if (tag_id_num <= 3) {
        primitive = createHexagonalPrism();
        adjusted_pose.position.z -= HEXAGONAL_PRISM_HEIGHT / 2;
    } else if (tag_id_num > 3 && tag_id_num <= 6) {
        primitive = createCube();
        adjusted_pose.position.z -= CUBE_LENGTH / 2;
    } else if (tag_id_num > 6 && tag_id_num <= 9) {
        primitive = createTriangularPrism();
        adjusted_pose.position.z -= TRIANGULAR_PRISM_HEIGHT / 2;
    } else {
        ROS_INFO("(TABLE) Tag ID: %s", tag_id.c_str());
        primitive = createTable();
        adjusted_pose.position.z -= TABLE_HEIGHT / 2;
        adjusted_pose.position.x += TABLE_LENGTH / 2;
        adjusted_pose.position.y -= TABLE_LENGTH / 2;
    }

    collisionObject_ = createCollisionObject(tag_id, frame_id, adjusted_pose, primitive);
    }


moveit_msgs::CollisionObject Obstacle::createCollisionObject(const std::string id, const std::string frame_id, const geometry_msgs::Pose pose, const shape_msgs::SolidPrimitive primitive) {
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = frame_id;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    return collision_object;
}

shape_msgs::SolidPrimitive Obstacle::createBox(double x, double y, double z) {
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = x;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = y;
    primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = z;
    return primitive;
}

shape_msgs::SolidPrimitive Obstacle::createHexagonalPrism() {
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = HEXAGONAL_PRISM_HEIGHT;
    primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = HEXAGONAL_PRISM_RADIUS;
    return primitive;
}


shape_msgs::SolidPrimitive Obstacle::createCube() {
    return createBox(CUBE_LENGTH, CUBE_LENGTH, CUBE_LENGTH);
}

shape_msgs::SolidPrimitive Obstacle::createTriangularPrism() {
    // Approximate a triangular prism as a box for simplicity
    return createBox(TRIANGULAR_PRISM_BASE, TRIANGULAR_PRISM_LENGTH, TRIANGULAR_PRISM_HEIGHT);
}

shape_msgs::SolidPrimitive Obstacle::createTable() {
    return createBox(TABLE_LENGTH, TABLE_LENGTH, TABLE_HEIGHT);
}
