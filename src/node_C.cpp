// #include <ros/ros.h>

// #include <actionlib/server/simple_action_server.h>
// #include <actionlib/client/simple_action_client.h>
// #include <assignment_2/MoveToDefaultPositionAction.h>
// #include <assignment_2/RemoveCollisionObjectAction.h>
// #include <assignment_2/StartPickingSequenceAction.h>
// #include <assignment_2/StartPlacingSequenceAction.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <gazebo_ros_link_attacher/Attach.h>
// #include "assignment_2/ObjectUtils.h"


// // Default joint positions for resetting the arm
// static const double ARM_JOINTS_DEFAULT[] = {0.07, 0.34, -3.13, 1.31, 1.58, 0.0, 0.0};


// // Gripper configuration
// static const double OPEN_GRIPPER_MAX_SIZE = 0.045;
// static const double GRIPPER_LENGTH = 0.23;
// static const double OFFSET = 0.1;

// // Object-specific dimensions
// static const double CUBE_SIDE = 0.05;

// //static const double TRIANGLE_LENGTH = 0.0325;
// static const double TRIANGLE_WIDTH = 0.0325;
// static const double TRIANGLE_HEIGHT = 0.021664;

// static const double HEXAGON_HEIGHT = 0.105364;
// static const double HEXAGON_DIAMETER = 0.0325;

// class ManipulateObjectAction {
// protected:
//     ros::NodeHandle nh_;
//     actionlib::SimpleActionServer<assignment_2::MoveToDefaultPositionAction> move_to_default_as_;
//     actionlib::SimpleActionServer<assignment_2::StartPickingSequenceAction> start_picking_sequence_as_;
//     actionlib::SimpleActionServer<assignment_2::StartPlacingSequenceAction> start_placing_sequence_as_;

//     moveit::planning_interface::MoveGroupInterface arm_group_;
//     moveit::planning_interface::MoveGroupInterface gripper_group_;
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
//     actionlib::SimpleActionClient<assignment_2::RemoveCollisionObjectAction> remove_collision_object_client_;

//     ros::ServiceClient attach_client_;
//     ros::ServiceClient detach_client_;

// public:
//     ManipulateObjectAction()
//         : move_to_default_as_(nh_, "move_to_default_position", boost::bind(&ManipulateObjectAction::moveToDefaultCB, this, _1), false),
//           start_picking_sequence_as_(nh_, "start_picking_sequence", boost::bind(&ManipulateObjectAction::startPickingSequenceCB, this, _1), false),
//           start_placing_sequence_as_(nh_, "start_placing_sequence", boost::bind(&ManipulateObjectAction::startPlacingSequenceCB, this, _1), false),
//           arm_group_("arm"),
//           gripper_group_("gripper"),
//           remove_collision_object_client_("remove_collision_object", true)
//     {
//         // Initialize ROS service clients for attaching/detaching objects
//         attach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
//         detach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

//         // Configure MoveIt! parameters for the arm
//         arm_group_.setPlanningTime(10.0);
//         arm_group_.setPoseReferenceFrame("base_link");
//         arm_group_.setMaxVelocityScalingFactor(0.8);

//         // Configure MoveIt! parameters for the gripper
//         gripper_group_.setPlanningTime(10.0);
//         gripper_group_.setMaxVelocityScalingFactor(0.8);

//         // Start the action servers
//         move_to_default_as_.start();
//         start_picking_sequence_as_.start();
//         start_placing_sequence_as_.start();
//         ROS_INFO("Node C action servers started.");
//     }

//     /// Callback for moving the arm to the default position
//     void moveToDefaultCB(const assignment_2::MoveToDefaultPositionGoalConstPtr &goal)
//     {
//         ROS_INFO("Moving to default arm position...");

//         std::vector<double> joint_values(std::begin(ARM_JOINTS_DEFAULT), std::end(ARM_JOINTS_DEFAULT));
//         arm_group_.setJointValueTarget(joint_values);

//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         if (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
//         {
//             arm_group_.move();
//             ROS_INFO("Arm moved to default position.");
//             move_to_default_as_.setSucceeded();
//         }
//         else
//         {
//             ROS_ERROR("Failed to move arm to default position.");
//             move_to_default_as_.setAborted();
//         }
//     }

//     /// Callback for starting the picking sequence
//     void startPickingSequenceCB(const assignment_2::StartPickingSequenceGoalConstPtr &goal)
//     {
//         ROS_INFO("Starting picking sequence for object ID: %s", goal->objectID.c_str());
        
//         if (!removeObjectFromPlanningScene(goal->objectID)) {
//             ROS_ERROR("Failed to remove object from planning scene.");
//             start_picking_sequence_as_.setAborted();
//             return;
//         }
        
//         if (!moveArmAbovePickPose(goal->pose, goal->objectType))
//         {
//             ROS_ERROR("Failed to move arm above pick position.");
//             start_picking_sequence_as_.setAborted();
//             return;
//         }


//         if (!moveArmToPick(goal->pose, goal->objectType))
//         {
//             ROS_ERROR("Failed to move arm to pick position.");
//             start_picking_sequence_as_.setAborted();
//             return;
//         }

//         if (!attachObject(goal->objectID))
//         {
//             ROS_ERROR("Failed to attach object to gripper.");
//             start_picking_sequence_as_.setAborted();
//             return;
//         }

//         if (!controlGripper(false, goal->objectType))
//         {
//             ROS_ERROR("Failed to close gripper.");
//             start_picking_sequence_as_.setAborted();
//             return;
//         }

//         if (!moveArmUp(goal->pose, goal->objectType))
//         {
//             ROS_ERROR("Failed to lift arm after placing.");
//             start_picking_sequence_as_.setAborted();
//             return;
//         }

//         ROS_INFO("Picking sequence completed successfully.");
//         assignment_2::StartPickingSequenceResult result;
//         result.success = true;
//         start_picking_sequence_as_.setSucceeded(result);
//     }

//     /// Callback for starting the placing sequence
//     void startPlacingSequenceCB(const assignment_2::StartPlacingSequenceGoalConstPtr &goal)
//     {
//         ROS_INFO("Starting placing sequence for object ID: %s", goal->objectID.c_str());

//         if (!moveArmToPlace(goal->pose, goal->objectType))
//         {
//             ROS_ERROR("Failed to move arm to place position.");
//             start_placing_sequence_as_.setAborted();
//             return;
//         }


//         if (!controlGripper(true, goal->objectType))
//         {
//             ROS_ERROR("Failed to open gripper during placing.");
//             start_placing_sequence_as_.setAborted();
//             return;
//         }

//         if (!detachObject(goal->objectID))
//         {
//             ROS_ERROR("Failed to detach object from gripper.");
//             start_placing_sequence_as_.setAborted();
//             return;
//         }
//         ros::Duration(0.2).sleep();

//         if (!removeObjectFromPlanningScene("10")) {
//             ROS_ERROR("Failed to remove object from planning scene.");
//             start_picking_sequence_as_.setAborted();
//             return;
//         }

//         if (!moveToDefaultPosition())
//         {
//             ROS_ERROR("Failed to return to default position after placing.");
//             start_placing_sequence_as_.setAborted();
//             return;
//         }

//         ROS_INFO("Placing sequence completed successfully.");
//         assignment_2::StartPlacingSequenceResult result;
//         result.success = true;
//         start_placing_sequence_as_.setSucceeded(result);
//     }

//     //Helper Functions

//     bool removeObjectFromPlanningScene(const std::string objectID)
//     {
//         assignment_2::RemoveCollisionObjectGoal remove_goal;
//         remove_goal.objectID = objectID;
//         remove_collision_object_client_.sendGoal(remove_goal);
//         return remove_collision_object_client_.waitForResult(ros::Duration(5.0));
//     }


//     bool moveArmToPick(const geometry_msgs::PoseStamped pose, const std::string objectType)
//     {
//         geometry_msgs::PoseStamped target_pose = pose;
//         tf2::Quaternion orientation;
//         target_pose.pose.orientation = tf2::toMsg(orientation);

//         if (objectType == "cube") {
//             orientation.setRPY(0, M_PI / 2, 0);
//             target_pose.pose.position.z += GRIPPER_LENGTH - CUBE_SIDE / 3;
//         } else if (objectType == "triangle") {
//             orientation.setRPY(0, M_PI / 2, M_PI / 2); 
//             target_pose.pose.position.z += GRIPPER_LENGTH;
//         } else if (objectType == "hexagon") {
//             orientation.setRPY(0, M_PI / 2, 0);
//             target_pose.pose.position.z += GRIPPER_LENGTH - HEXAGON_HEIGHT / 3;
//         }

//         arm_group_.setPoseTarget(target_pose);
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
//     }

//     bool moveArmAbovePickPose (const geometry_msgs::PoseStamped pose, const std::string objectType) {
//         geometry_msgs::PoseStamped target_pose = pose;
//         tf2::Quaternion orientation;
//         if (objectType == "triangle") {
//             orientation.setRPY(0, M_PI / 2, M_PI / 2); 
//         } else {
//             orientation.setRPY(0, M_PI / 2, 0);
//         }

//         target_pose.pose.orientation = tf2::toMsg(orientation);
//         target_pose.pose.position.z += GRIPPER_LENGTH + OFFSET;
//         arm_group_.setPoseTarget(target_pose);
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
//     }

//     bool moveArmUp(const geometry_msgs::PoseStamped pose, const std::string objectType)
//     {
//         geometry_msgs::PoseStamped adjusted_pose = pose;
//         tf2::Quaternion orientation;
//         if (objectType == "triangle") {
//             orientation.setRPY(0, M_PI / 2, M_PI / 2); 
//         } else {
//             orientation.setRPY(0, M_PI / 2, 0);
//         }
//         adjusted_pose.pose.orientation = tf2::toMsg(orientation);
//         adjusted_pose.pose.position.z += GRIPPER_LENGTH + OFFSET;

//         arm_group_.setPoseTarget(adjusted_pose);
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
//     }

//     bool moveArmToPlace(const geometry_msgs::PoseStamped pose, const std::string objectType)
//     {
//         geometry_msgs::PoseStamped adjusted_pose = pose;
//         tf2::Quaternion orientation;
//         orientation.setRPY(0, M_PI / 2, 0);

//         adjusted_pose.pose.orientation = tf2::toMsg(orientation);
//         adjusted_pose.pose.position.z += GRIPPER_LENGTH;
//         arm_group_.setPoseTarget(adjusted_pose);
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
//     }

//     bool controlGripper(bool is_open, const std::string objectType)
//     {
//         std::vector<double> gripper_values;
//         if (is_open) {
//             ROS_INFO("Opening gripper");
//             gripper_values = {OPEN_GRIPPER_MAX_SIZE, OPEN_GRIPPER_MAX_SIZE};
//         } else {
//             ROS_INFO("Closing gripper");
        
//             if (objectType == "cube") {
//                 gripper_values = {CUBE_SIDE/2, CUBE_SIDE/2};
//             } else if (objectType == "triangle") {
//                 gripper_values = {TRIANGLE_WIDTH/2, TRIANGLE_WIDTH/2};
//             } else if (objectType == "hexagon") {
//                 gripper_values = {HEXAGON_DIAMETER/2, HEXAGON_DIAMETER/2};
//             }
//         }
    
//         gripper_group_.setJointValueTarget(gripper_values);
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         return gripper_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && gripper_group_.move();
//     }

//     bool attachObject(const std::string objectID)
//     {
//         gazebo_ros_link_attacher::Attach srv;
//         srv.request.model_name_1 = ObjectUtils::getName(objectID);
//         srv.request.link_name_1 = ObjectUtils::getLinkName(objectID);
//         srv.request.model_name_2 = "tiago";
//         srv.request.link_name_2 = "arm_7_link";
//         return attach_client_.call(srv);
//     }

//     bool detachObject(const std::string objectID)
//     {
//         ROS_INFO("Detaching object with ID: %s", objectID.c_str());
//         gazebo_ros_link_attacher::Attach srv;
//         srv.request.model_name_1 = ObjectUtils::getName(objectID);
//         srv.request.link_name_1 = ObjectUtils::getLinkName(objectID);
//         srv.request.model_name_2 = "tiago";
//         srv.request.link_name_2 = "arm_7_link";
//         return detach_client_.call(srv); 
//     }

//     bool moveToDefaultPosition()
//     {
//         std::vector<double> joint_values(std::begin(ARM_JOINTS_DEFAULT), std::end(ARM_JOINTS_DEFAULT));
//         arm_group_.setJointValueTarget(joint_values);
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "node_C");
//     ros::AsyncSpinner spinner(1); // Use 1 thread
//     spinner.start();

//     ManipulateObjectAction manipulateObjectAction;

//     ros::waitForShutdown();

//     return 0;
// }

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2/MoveToDefaultPositionAction.h>
#include <assignment_2/RemoveCollisionObjectAction.h>
#include <assignment_2/StartPickingSequenceAction.h>
#include <assignment_2/StartPlacingSequenceAction.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include "assignment_2/ObjectUtils.h"


#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

std::string detected_color;

// Default joint positions for resetting the arm
static const double ARM_JOINTS_DEFAULT[] = {0.07, 0.34, -3.13, 1.31, 1.58, 0.0, 0.0};


// Gripper configuration
static const double OPEN_GRIPPER_MAX_SIZE = 0.045;
static const double GRIPPER_LENGTH = 0.23;
static const double OFFSET = 0.1;

// Object-specific dimensions
static const double CUBE_SIDE = 0.05;

//static const double TRIANGLE_LENGTH = 0.0325;
static const double TRIANGLE_WIDTH = 0.0325;
static const double TRIANGLE_HEIGHT = 0.021664;

static const double HEXAGON_HEIGHT = 0.105364;
static const double HEXAGON_DIAMETER = 0.0325;

class ManipulateObjectAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment_2::MoveToDefaultPositionAction> move_to_default_as_;
    actionlib::SimpleActionServer<assignment_2::StartPickingSequenceAction> start_picking_sequence_as_;
    actionlib::SimpleActionServer<assignment_2::StartPlacingSequenceAction> start_placing_sequence_as_;

    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    actionlib::SimpleActionClient<assignment_2::RemoveCollisionObjectAction> remove_collision_object_client_;

    ros::ServiceClient attach_client_;
    ros::ServiceClient detach_client_;

public:
    ManipulateObjectAction()
        : move_to_default_as_(nh_, "move_to_default_position", boost::bind(&ManipulateObjectAction::moveToDefaultCB, this, _1), false),
          start_picking_sequence_as_(nh_, "start_picking_sequence", boost::bind(&ManipulateObjectAction::startPickingSequenceCB, this, _1), false),
          start_placing_sequence_as_(nh_, "start_placing_sequence", boost::bind(&ManipulateObjectAction::startPlacingSequenceCB, this, _1), false),
          arm_group_("arm"),
          gripper_group_("gripper"),
          remove_collision_object_client_("remove_collision_object", true)
    {
        // Initialize ROS service clients for attaching/detaching objects
        attach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        detach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

        // Configure MoveIt! parameters for the arm
        arm_group_.setPlanningTime(10.0);
        arm_group_.setPoseReferenceFrame("base_link");
        arm_group_.setMaxVelocityScalingFactor(0.8);

        // Configure MoveIt! parameters for the gripper
        gripper_group_.setPlanningTime(10.0);
        gripper_group_.setMaxVelocityScalingFactor(0.8);

        // Start the action servers
        move_to_default_as_.start();
        start_picking_sequence_as_.start();
        start_placing_sequence_as_.start();
        ROS_INFO("Node C action servers started.");
    }

    /// Callback for moving the arm to the default position
    void moveToDefaultCB(const assignment_2::MoveToDefaultPositionGoalConstPtr &goal)
    {
        ROS_INFO("Moving to default arm position...");

        std::vector<double> joint_values(std::begin(ARM_JOINTS_DEFAULT), std::end(ARM_JOINTS_DEFAULT));
        arm_group_.setJointValueTarget(joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            arm_group_.move();
            ROS_INFO("Arm moved to default position.");
            move_to_default_as_.setSucceeded();
        }
        else
        {
            ROS_ERROR("Failed to move arm to default position.");
            move_to_default_as_.setAborted();
        }
    }

    /// Callback for starting the picking sequence
    void startPickingSequenceCB(const assignment_2::StartPickingSequenceGoalConstPtr &goal)
    {
        ROS_INFO("Starting picking sequence for object ID: %s", goal->objectID.c_str());
        
        if (!removeObjectFromPlanningScene(goal->objectID)) {
            ROS_ERROR("Failed to remove object from planning scene.");
            start_picking_sequence_as_.setAborted();
            return;
        }
        
        if (!moveArmAbovePickPose(goal->pose, goal->objectType))
        {
            ROS_ERROR("Failed to move arm above pick position.");
            start_picking_sequence_as_.setAborted();
            return;
        }


        if (!moveArmToPick(goal->pose, goal->objectType))
        {
            ROS_ERROR("Failed to move arm to pick position.");
            start_picking_sequence_as_.setAborted();
            return;
        }

        if (!attachObject(goal->objectID))
        {
            ROS_ERROR("Failed to attach object to gripper.");
            start_picking_sequence_as_.setAborted();
            return;
        }

        if (!controlGripper(false, goal->objectType))
        {
            ROS_ERROR("Failed to close gripper.");
            start_picking_sequence_as_.setAborted();
            return;
        }

        if (!moveArmUp(goal->pose, goal->objectType))
        {
            ROS_ERROR("Failed to lift arm after placing.");
            start_picking_sequence_as_.setAborted();
            return;
        }

        ROS_INFO("Picking sequence completed successfully.");
        assignment_2::StartPickingSequenceResult result;
        result.success = true;
        start_picking_sequence_as_.setSucceeded(result);
    }

    /// Callback for starting the placing sequence
    void startPlacingSequenceCB(const assignment_2::StartPlacingSequenceGoalConstPtr &goal)
    {
        ROS_INFO("Starting placing sequence for object ID: %s", goal->objectID.c_str());

        if (!moveArmToPlace(goal->pose, goal->objectType))
        {
            ROS_ERROR("Failed to move arm to place position.");
            start_placing_sequence_as_.setAborted();
            return;
        }


        if (!controlGripper(true, goal->objectType))
        {
            ROS_ERROR("Failed to open gripper during placing.");
            start_placing_sequence_as_.setAborted();
            return;
        }

        if (!detachObject(goal->objectID))
        {
            ROS_ERROR("Failed to detach object from gripper.");
            start_placing_sequence_as_.setAborted();
            return;
        }
        ros::Duration(0.2).sleep();

        if (!removeObjectFromPlanningScene("10")) {
            ROS_ERROR("Failed to remove object from planning scene.");
            start_picking_sequence_as_.setAborted();
            return;
        }

        if (!moveToDefaultPosition())
        {
            ROS_ERROR("Failed to return to default position after placing.");
            start_placing_sequence_as_.setAborted();
            return;
        }

        ROS_INFO("Placing sequence completed successfully.");
        assignment_2::StartPlacingSequenceResult result;
        result.success = true;
        start_placing_sequence_as_.setSucceeded(result);
    }

    //Helper Functions

    bool removeObjectFromPlanningScene(const std::string objectID)
    {
        assignment_2::RemoveCollisionObjectGoal remove_goal;
        remove_goal.objectID = objectID;
        remove_collision_object_client_.sendGoal(remove_goal);
        return remove_collision_object_client_.waitForResult(ros::Duration(5.0));
    }


    bool moveArmToPick(const geometry_msgs::PoseStamped pose, const std::string objectType)
    {
        geometry_msgs::PoseStamped target_pose = pose;
        tf2::Quaternion orientation;
        target_pose.pose.orientation = tf2::toMsg(orientation);

        if (objectType == "cube") {
            orientation.setRPY(0, M_PI / 2, 0);
            target_pose.pose.position.z += GRIPPER_LENGTH - CUBE_SIDE / 3;
        } else if (objectType == "triangle") {
            orientation.setRPY(0, M_PI / 2, M_PI / 2); 
            target_pose.pose.position.z += GRIPPER_LENGTH;
        } else if (objectType == "hexagon") {
            orientation.setRPY(0, M_PI / 2, 0);
            target_pose.pose.position.z += GRIPPER_LENGTH - HEXAGON_HEIGHT / 3;
        }

        arm_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
    }

    bool moveArmAbovePickPose (const geometry_msgs::PoseStamped pose, const std::string objectType) {
        geometry_msgs::PoseStamped target_pose = pose;
        tf2::Quaternion orientation;
        if (objectType == "triangle") {
            orientation.setRPY(0, M_PI / 2, M_PI / 2); 
        } else {
            orientation.setRPY(0, M_PI / 2, 0);
        }

        target_pose.pose.orientation = tf2::toMsg(orientation);
        target_pose.pose.position.z += GRIPPER_LENGTH + OFFSET;
        arm_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
    }

    bool moveArmUp(const geometry_msgs::PoseStamped pose, const std::string objectType)
    {
        geometry_msgs::PoseStamped adjusted_pose = pose;
        tf2::Quaternion orientation;
        if (objectType == "triangle") {
            orientation.setRPY(0, M_PI / 2, M_PI / 2); 
        } else {
            orientation.setRPY(0, M_PI / 2, 0);
        }
        adjusted_pose.pose.orientation = tf2::toMsg(orientation);
        adjusted_pose.pose.position.z += GRIPPER_LENGTH + OFFSET;

        arm_group_.setPoseTarget(adjusted_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
    }

    bool moveArmToPlace(const geometry_msgs::PoseStamped pose, const std::string objectType)
    {
        geometry_msgs::PoseStamped adjusted_pose = pose;
        tf2::Quaternion orientation;
        orientation.setRPY(0, M_PI / 2, 0);

        adjusted_pose.pose.orientation = tf2::toMsg(orientation);
        adjusted_pose.pose.position.z += GRIPPER_LENGTH;
        arm_group_.setPoseTarget(adjusted_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
    }

    bool controlGripper(bool is_open, const std::string objectType)
    {
        std::vector<double> gripper_values;
        if (is_open) {
            ROS_INFO("Opening gripper");
            gripper_values = {OPEN_GRIPPER_MAX_SIZE, OPEN_GRIPPER_MAX_SIZE};
        } else {
            ROS_INFO("Closing gripper");
        
            if (objectType == "cube") {
                gripper_values = {CUBE_SIDE/2, CUBE_SIDE/2};
            } else if (objectType == "triangle") {
                gripper_values = {TRIANGLE_WIDTH/2, TRIANGLE_WIDTH/2};
            } else if (objectType == "hexagon") {
                gripper_values = {HEXAGON_DIAMETER/2, HEXAGON_DIAMETER/2};
            }
        }
    
        gripper_group_.setJointValueTarget(gripper_values);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return gripper_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && gripper_group_.move();
    }

    bool attachObject(const std::string objectID)
    {
        gazebo_ros_link_attacher::Attach srv;
        srv.request.model_name_1 = ObjectUtils::getName(objectID);
        srv.request.link_name_1 = ObjectUtils::getLinkName(objectID);
        srv.request.model_name_2 = "tiago";
        srv.request.link_name_2 = "arm_7_link";
        return attach_client_.call(srv);
    }

    bool detachObject(const std::string objectID)
    {
        ROS_INFO("Detaching object with ID: %s", objectID.c_str());
        gazebo_ros_link_attacher::Attach srv;
        srv.request.model_name_1 = ObjectUtils::getName(objectID);
        srv.request.link_name_1 = ObjectUtils::getLinkName(objectID);
        srv.request.model_name_2 = "tiago";
        srv.request.link_name_2 = "arm_7_link";
        return detach_client_.call(srv); 
    }

    bool moveToDefaultPosition()
    {
        std::vector<double> joint_values(std::begin(ARM_JOINTS_DEFAULT), std::end(ARM_JOINTS_DEFAULT));
        arm_group_.setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_group_.move();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_C");
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    ManipulateObjectAction manipulateObjectAction;

    ros::waitForShutdown();

    return 0;
}
