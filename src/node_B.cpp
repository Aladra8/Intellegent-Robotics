#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <actionlib/server/simple_action_server.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>

#include "assignment_2/ApriltagDetection.h"
#include "assignment_2/Obstacle.h"
#include <assignment_2/detectedApriltags.h>
#include "assignment_2/ObjectUtils.h"

#include <assignment_2/RemoveCollisionObjectAction.h>
#include <assignment_2/RemoveAllCollisionObjectsAction.h>
#include <assignment_2/AddCollisionObjectsAction.h>

#include <mutex>
#include <map>
#include <unordered_set>

std::mutex planning_scene_mutex;

/// Node_B class handles Apriltag detections and manages collision objects
class Node_B
{
private:
    ros::NodeHandle nh_; ///< ROS NodeHandle for managing publishers and subscribers
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    ApriltagDetection apriltagDetection_; ///< Handles Apriltag detections

    std::map<std::string, geometry_msgs::PoseStamped> detectedTags_;                                          ///< Detected tags
    actionlib::SimpleActionServer<assignment_2::RemoveCollisionObjectAction> remove_collision_object_server_; ///< Action server for removing collision objects
    actionlib::SimpleActionServer<assignment_2::RemoveAllCollisionObjectsAction> remove_all_collision_objects_server_; ///< Action server for removing all collision objects
    actionlib::SimpleActionServer<assignment_2::AddCollisionObjectsAction> add_collision_objects_server_; ///< Action server for removing collision objects

public:
    /// Constructor initializes the node and sets up the action server
    Node_B()
        : nh_(),
        apriltagDetection_(nh_),
        remove_collision_object_server_(nh_, "remove_collision_object", boost::bind(&Node_B::removeCollisionObjectCB, this, _1), false),
        remove_all_collision_objects_server_(nh_, "remove_all_collision_objects", boost::bind(&Node_B::removeAllCollisionObjectsCB, this, _1), false),
        add_collision_objects_server_(nh_, "add_collision_objects", boost::bind(&Node_B::addCollisionObjectsCB, this, _1), false)
    {
        ROS_INFO("Node B initialized.");
        remove_collision_object_server_.start();
        remove_all_collision_objects_server_.start();
        add_collision_objects_server_.start();
        ROS_INFO("Action servers started.");
    }

    /// Main spin loop for processing Apriltags and managing collision objects
    void spin()
    {
        ros::Rate rate(10); // 2 Hz
        while (ros::ok())
        {
            ros::spinOnce();
            detectedTags_ = apriltagDetection_.getDetectedTags();
            apriltagDetection_.publishTags();
            rate.sleep();
        }
    }

    /// Callback for removing collision objects from the planning scene
    void removeCollisionObjectCB(const assignment_2::RemoveCollisionObjectGoalConstPtr &goal)
    {
        ROS_INFO("Remove Collision Object goal received for ID: %s", goal->objectID.c_str());
        std::lock_guard<std::mutex> lock(planning_scene_mutex); // Lock the mutex

        // Remove object from the planning scene and mark it as removed
        planning_scene_interface_.removeCollisionObjects({goal->objectID});
        ROS_INFO("Collision object '%s' removed.", goal->objectID.c_str());
        remove_collision_object_server_.setSucceeded();
    }

    void removeAllCollisionObjectsCB(const assignment_2::RemoveAllCollisionObjectsGoalConstPtr &goal) 
    {
        std::lock_guard<std::mutex> lock(planning_scene_mutex); // Lock the mutex
        std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();

        if (!object_ids.empty()) {
            planning_scene_interface_.removeCollisionObjects(object_ids);
            ROS_INFO("Removed %zu collision objects.", object_ids.size());
        } else {
            ROS_WARN("No collision objects found to remove.");
        }
        remove_all_collision_objects_server_.setSucceeded();
        ROS_INFO("All collision objects removed.");
    }

    void addCollisionObjectsCB(const assignment_2::AddCollisionObjectsGoalConstPtr &goal) 
    {
        ROS_INFO("Update collision objects"); 
        std::lock_guard<std::mutex> lock(planning_scene_mutex); // Lock the mutex
        std::vector<moveit_msgs::CollisionObject> collisionObjects;

        std::map<std::string, geometry_msgs::PoseStamped> detectedTagsCopy = detectedTags_;
        for (const auto &tag : detectedTagsCopy)
        {
            const std::string &id_str = tag.first;
            const geometry_msgs::PoseStamped &pose_stamped = tag.second;

            if (id_str == goal->objectBeingManipulatedID) continue;
            
            Obstacle obstacle(id_str, pose_stamped.header.frame_id, pose_stamped.pose);
            moveit_msgs::CollisionObject collisionObject = obstacle.getCollisionObject();

            if (!collisionObject.id.empty())
            {
                collisionObjects.push_back(collisionObject);
            }
        }

        if (!collisionObjects.empty())
        {
            ROS_INFO("Adding %lu collision objects to the planning scene.", collisionObjects.size());
            planning_scene_interface_.addCollisionObjects(collisionObjects);
        }
    }
    

    /// Destructor for cleaning up resources
    ~Node_B()
    {
        ROS_INFO("Shutting down Node B.");
    }
};

/// Main function initializes the node and starts the processing loop
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_B");
    Node_B node;
    node.spin();
    return 0;
}
