#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include <assignment_2/detectedApriltags.h>

#include <assignment_2/MoveToDefaultPositionAction.h>
#include <assignment_2/StartPickingSequenceAction.h>
#include <assignment_2/StartPlacingSequenceAction.h>
#include <assignment_2/RemoveAllCollisionObjectsAction.h>
#include <assignment_2/AddCollisionObjectsAction.h>

#include <map>
#include <string>

// WAYPOINTS
// Waypoint definitions
const static double poseW1_x = 8.0000, poseW1_y = 0.8000, orientationW1_z = 0, orientationW1_w = 1;
const static double poseW2_x = 8.7000, poseW2_y = 0.0000, orientationW2_z = -0.7095, orientationW2_w = 0.7046;
const static double poseW3_x = 8.8000, poseW3_y = -2.900, orientationW3_z = 1.0000, orientationW3_w = 0.0000;


// Waypoint 5 (placement table) coordinates
const static double poseW5_x = 8.7000, poseW5_y = -2.00, orientationW5_z = 1.0000, orientationW5_w = 0.0000; 

std::map<std::string, geometry_msgs::PoseStamped> allDetectedTags;

void detectedTagsCallback(const assignment_2::detectedApriltags::ConstPtr& msg) {
    //ROS_INFO("Received detected tags message");
    std::map<std::string, geometry_msgs::PoseStamped> latestMsg;
    for (int i = 0; i < msg->keys.size(); ++i) {
        const std::string id = msg->keys[i].data;
        const geometry_msgs::PoseStamped &pose = msg->poses.at(i);

        // Always update to the most recent pose
        latestMsg[id] = pose;
    }
    allDetectedTags = latestMsg;

    // ROS_INFO("Total detected tags: %zu", allDetectedTags.size());
}

/// @brief Send a goal to the move_base action server
/// @param goal 
bool moveTo(ros::NodeHandle &nh, const geometry_msgs::PoseStamped &goal)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    if (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Timeout waiting for move_base action server.");
        return false;
    }

    move_base_msgs::MoveBaseGoal goalMsg;
    goalMsg.target_pose = goal;
    ROS_INFO("Sending goal to move_base...");
    ac.sendGoal(goalMsg);
    if (!ac.waitForResult(ros::Duration(30.0)))
    {
        ROS_ERROR("Timeout waiting for move_base to reach goal.");
        return false;
    }

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Reached goal successfully.");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to reach goal: %s", ac.getState().toString().c_str());
        return false;
    }
}

void liftTorso(ros::NodeHandle& nh) {
    ros::Publisher torso_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 10);
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back("torso_lift_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.35); // Adjust as needed
    point.time_from_start = ros::Duration(1.0);
    traj.points.push_back(point);

    ROS_INFO("Lifting torso");
    for (int i = 0; i < 10; ++i) {
        torso_pub.publish(traj);
        ros::Duration(0.1).sleep(); // Small delay between publishes
    }

    // Give some time for the torso to move
    ros::Duration(2.0).sleep();
}

void defaultPosClientCall() {
    actionlib::SimpleActionClient<assignment_2::MoveToDefaultPositionAction> defaultPosClient("move_to_default_position", true);
    defaultPosClient.waitForServer();
    assignment_2::MoveToDefaultPositionGoal goal;
    defaultPosClient.sendGoal(goal);
    defaultPosClient.waitForResult();
}

geometry_msgs::PoseStamped createPoseStamped(double x, double y, double orientation_z, double orientation_w) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = orientation_z;
    pose.pose.orientation.w = orientation_w;
    return pose;
}

void tiltHeadDown(ros::NodeHandle& nh, float tiltAngle) {
    ros::Publisher head_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);;
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back("head_1_joint");
    traj.joint_names.push_back("head_2_joint");

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.0);  // Pan angle (head_1_joint)
    point.positions.push_back(tiltAngle); // Tilt angle (head_2_joint), adjust as needed
    point.time_from_start = ros::Duration(1.0);
    traj.points.push_back(point);

    for (int i = 0; i < 10; ++i) {
        head_pub.publish(traj);
        ros::Duration(0.1).sleep(); // Small delay between publishes
    }

    // Give some time for the head to move
    ros::Duration(3.0).sleep();
}


std::pair<float, float> retrieveLineCoeffs(ros::NodeHandle &nh) {
    ros::ServiceClient lineClient = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    tiago_iaslab_simulation::Coeffs coeffsSrv;
    coeffsSrv.request.ready = true;
    if (!lineClient.call(coeffsSrv))
    {
        ROS_ERROR("Failed to call /straight_line_srv");
    }
    return std::make_pair(coeffsSrv.response.coeffs.at(0), coeffsSrv.response.coeffs.at(1));
}

// Function to compute the placement pose
geometry_msgs::PoseStamped computePlacementPose(
    const double& m, 
    const double& q, 
    const double& x, 
    //tf2_ros::Buffer &tfBuffer, 
    const geometry_msgs::PoseStamped& tableTagPose)
{
    geometry_msgs::PoseStamped placingPose;
    placingPose.header.frame_id = "base_link";
    placingPose.header.stamp = ros::Time::now();
    placingPose.pose.position.x = tableTagPose.pose.position.x + x;
    placingPose.pose.position.y = tableTagPose.pose.position.y - (m * x + q); //Subtracted because robot is turned around
    placingPose.pose.position.z = 0.8; //FIX
    
    //startPickingGoal.pose = allDetectedTags.at(objectID);
    return placingPose;
}


bool addCollisionObjectsClientCall(
    actionlib::SimpleActionClient<assignment_2::AddCollisionObjectsAction>& addCollisionObjectsClient,
    std::string objectID)
{
    assignment_2::AddCollisionObjectsGoal addCollisionObjectsGoal;
    addCollisionObjectsGoal.goal = true;
    addCollisionObjectsGoal.objectBeingManipulatedID = objectID;
    addCollisionObjectsClient.sendGoal(addCollisionObjectsGoal);
    bool addedCollisionObjects = addCollisionObjectsClient.waitForResult();

    return addedCollisionObjects;
}

bool removeAllCollisionObjectsClientCall(
    actionlib::SimpleActionClient<assignment_2::RemoveAllCollisionObjectsAction>& removeAllCollisionObjectsClient
)
{
    assignment_2::RemoveAllCollisionObjectsGoal removeAllCollisionObjectsGoal;
    removeAllCollisionObjectsGoal.goal = true;
    removeAllCollisionObjectsClient.sendGoal(removeAllCollisionObjectsGoal);

    return removeAllCollisionObjectsClient.waitForResult();
}

assignment_2::StartPickingSequenceResultConstPtr startPickingSequenceClientCall(
    actionlib::SimpleActionClient<assignment_2::StartPickingSequenceAction>& startPickingSequenceClient, 
    const std::string objectID, 
    const std::string objectType, 
    const geometry_msgs::PoseStamped objectPose) 
{
    assignment_2::StartPickingSequenceGoal startPickingGoal;
    startPickingGoal.objectID = objectID;
    startPickingGoal.objectType = objectType;
    startPickingGoal.pose = objectPose;

    startPickingSequenceClient.sendGoal(startPickingGoal);  // Send goal to node_C
    startPickingSequenceClient.waitForResult();
    return startPickingSequenceClient.getResult();
}

assignment_2::StartPlacingSequenceResultConstPtr startPlacingSequenceClientCall(
    actionlib::SimpleActionClient<assignment_2::StartPlacingSequenceAction>& startPlacingSequenceClient, 
    const std::string objectID, 
    const std::string objectType,
    const geometry_msgs::PoseStamped computedPose) 
{
    assignment_2::StartPlacingSequenceGoal placeGoal;
    placeGoal.objectID = objectID;
    placeGoal.objectType = objectType;
    placeGoal.pose = computedPose;

    startPlacingSequenceClient.sendGoal(placeGoal);
    startPlacingSequenceClient.waitForResult();

    return startPlacingSequenceClient.getResult();
}

int main(int argc, char** argv) {
    ROS_INFO("Starting node A");
    ros::init(argc, argv, "node_A");
    ros::NodeHandle nh;

    // Subscribe to detected tags
    ros::Subscriber sub = nh.subscribe("detected_tags", 10, detectedTagsCallback);
    
    // Action clients for various tasks
    actionlib::SimpleActionClient<assignment_2::StartPickingSequenceAction> startPickingSequenceClient("start_picking_sequence", true);
    actionlib::SimpleActionClient<assignment_2::StartPlacingSequenceAction> startPlacingSequenceClient("start_placing_sequence", true);
    actionlib::SimpleActionClient<assignment_2::AddCollisionObjectsAction> addCollisionObjectsClient("add_collision_objects", true);
    actionlib::SimpleActionClient<assignment_2::RemoveAllCollisionObjectsAction> removeAllCollisionObjectsClient("remove_all_collision_objects", true);

    ROS_INFO("Waiting for action servers...");
    startPickingSequenceClient.waitForServer();
    startPlacingSequenceClient.waitForServer();
    addCollisionObjectsClient.waitForServer();
    removeAllCollisionObjectsClient.waitForServer();
    ROS_INFO("All Action servers ready.");


    // TF listener and buffer for transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Define waypoints
    geometry_msgs::PoseStamped poseW1 = createPoseStamped(poseW1_x, poseW1_y, orientationW1_z, orientationW1_w);
    geometry_msgs::PoseStamped poseW2 = createPoseStamped(poseW2_x, poseW2_y, orientationW2_z, orientationW2_w);
    geometry_msgs::PoseStamped poseW3 = createPoseStamped(poseW3_x, poseW3_y, orientationW3_z, orientationW3_w);
    geometry_msgs::PoseStamped poseW5 = createPoseStamped(poseW5_x, poseW5_y, orientationW5_z, orientationW5_w);

     // Navigate through Waypoints
    ROS_INFO("Navigating to Waypoint 1...");
    moveTo(nh, poseW1);
    ROS_INFO("Reached Waypoint 1. Performing torso lift and arm raise...");

    liftTorso(nh); // Adjust height at Waypoint 1
    defaultPosClientCall();  // Raise arm at Waypoint 1

    // Navigate to Waypoint 2
    ROS_INFO("Navigating to Waypoint 2...");
    moveTo(nh, poseW2);

    // Retrieve line coefficients from /straight_line_srv
    std::pair<float, float> lineCoeffs = retrieveLineCoeffs(nh);
    float m = lineCoeffs.first; 
    float q = lineCoeffs.second;
    ROS_INFO("Received line coefficients: m = %f, q = %f", m, q);

    float x_eq; 
    if (m >= 1) {
        x_eq = 0.10 / m;
    } else {
        x_eq = 0.10;
    }
    int i = 0;
    while (ros::ok && i < 3) {
        ROS_INFO("Navigating to Waypoint 3 (DOCKING POSITION)...");
        moveTo(nh, poseW3);

        std::string objectID, objectType;
        if (i == 0 ) {objectID = "5"; objectType = "cube";}
        if (i == 1 ) {objectID = "7"; objectType = "triangle";}
        if (i == 2 ) {objectID = "3"; objectType = "hexagon";}

        ROS_INFO("Tilting head down for AprilTag detection...");
        tiltHeadDown(nh, -0.8);

        allDetectedTags.clear(); 
        while (allDetectedTags.find(objectID) == allDetectedTags.end() && ros::ok()) {
            ros::spinOnce();
            ROS_INFO("Waiting for tags to be detected");
        }

        ROS_INFO("Calling on node B to add collision objects to the planning scene");
        if (!addCollisionObjectsClientCall(addCollisionObjectsClient, "")) {
            ROS_ERROR("Failed to add objects to planning scene.");
            return 1;
        }

        geometry_msgs::PoseStamped objectPose = allDetectedTags.at(objectID);
        ROS_INFO("Calling on node C to Start Picking sequence");
        if (!startPickingSequenceClientCall(startPickingSequenceClient, objectID, objectType, objectPose)->success) {
            ROS_ERROR("Failed to start picking sequence");
            return 1;
        }

        ROS_INFO("Calling on node B to remove all Collision Objects from the Planning Scene");
        if (!removeAllCollisionObjectsClientCall(removeAllCollisionObjectsClient))
        {
            ROS_ERROR("Failed to remove objects from planning scene.");
            return 1;
        }

        ROS_INFO("Navigating in front of Placement Table...");
        moveTo(nh, poseW5);

        tiltHeadDown(nh, -0.9);

        allDetectedTags.clear();
        ROS_INFO("Wait for tag 10 (Placement Table) to be detected and for pose to be stable");
        while (allDetectedTags.find("10") == allDetectedTags.end() && ros::ok()) {
            ros::spinOnce();
        }

        geometry_msgs::PoseStamped tableTagPose = allDetectedTags.at("10");
        ROS_INFO("Stored Apriltag 10 with pose (x: %.2f, y: %.2f, z: %.2f)", tableTagPose.pose.position.x, tableTagPose.pose.position.y, tableTagPose.pose.position.z);

        ROS_INFO("X_eq = %.2f", x_eq);
        geometry_msgs::PoseStamped computedPose = computePlacementPose(m, q, x_eq, tableTagPose);
        ROS_INFO("Computed placement pose: (x: %.2f, y: %.2f, z: %.2f)", computedPose.pose.position.x, computedPose.pose.position.y, computedPose.pose.position.z);


        ROS_INFO("Calling on node B to add collision objects to the planning scene");
        if (!addCollisionObjectsClientCall(addCollisionObjectsClient, objectID)) {
            ROS_ERROR("Failed to add objects to planning scene.");
            return 1;
        }

        ROS_INFO("Calling on node C to start placing sequence");
        if (!startPlacingSequenceClientCall(startPlacingSequenceClient, objectID, objectType, computedPose)->success){
            ROS_ERROR("Failed to place object");
        }

        ROS_INFO("Calling on node B to remove all Collision Objects from the Planning Scene");
        if (!removeAllCollisionObjectsClientCall(removeAllCollisionObjectsClient))
        {
            ROS_ERROR("Failed to remove objects from planning scene.");
            return 1;
        }

    if (m >= 1) {
        x_eq += 0.10 / m;
    } else {
        x_eq += 0.10;
    }
        ++i;
    }

    return 0;
}
