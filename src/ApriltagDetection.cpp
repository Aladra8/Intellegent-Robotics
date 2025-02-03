#include <ros/ros.h>
#include "assignment_2/ApriltagDetection.h"



void ApriltagDetection::detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) 
{
    ROS_INFO("In detection Callback.");
    detectedTags_.clear();
    
    std::string target_frame = "base_link";
    std::string source_frame = msg->header.frame_id;

    // If tags were detected, attempt to transform their poses into the map frame.
    if (!msg->detections.empty()) {

        // Wait until the transform from source_frame to target_frame is available.
        while (ros::ok() && !tfBuffer_.canTransform(target_frame, source_frame, ros::Time(0))) {
            ros::Duration(0.5).sleep();
            ROS_INFO("Waiting for transform from %s to %s...", source_frame.c_str(), target_frame.c_str());
        }

        if (!ros::ok()) {
            ROS_WARN("ROS shutdown detected. Exiting loop in detectionCallback.");
            return;
        }
        
        // Retrieve the transform.
        geometry_msgs::TransformStamped transformed = tfBuffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        
        //Transform available
        geometry_msgs::PoseStamped pos_in;
        geometry_msgs::PoseStamped pos_out;

        // Process each detected AprilTag.
        for(int i = 0; i < msg->detections.size(); ++i)
        {
            std::string id_str = std::to_string(msg->detections.at(i).id[0]);

            pos_in.header.frame_id = msg->detections.at(i).pose.header.frame_id;
            pos_in.pose = msg->detections.at(i).pose.pose.pose; 
        
            tf2::doTransform(pos_in, pos_out, transformed);

            ROS_INFO_STREAM("Detected Apriltag with ID: " << id_str);
            ROS_INFO_STREAM("Original pose:\n" << pos_in);
            ROS_INFO_STREAM("Transformed pose:\n" << pos_out);

            detectedTags_[id_str] = pos_out;
        
        }

    } else { // If No tags were detected
        ROS_INFO("No tags detected.");
    }
}


void ApriltagDetection::publishTags() {
    auto detectedTags = getDetectedTags();
    ROS_INFO("Detected %zu tags", detectedTags_.size());

    assignment_2::detectedApriltags msg;
    if (detectedTags_.size() > 0) {
        ROS_INFO("Found tags");
        for (const auto& tag : detectedTags_) {
            std_msgs::String tag_msg;
            tag_msg.data = tag.first;
            geometry_msgs::PoseStamped pose = tag.second;

            msg.keys.push_back(tag_msg);
            msg.poses.push_back(pose);
        }
    } else {
        ROS_INFO("No tags found");
    }

    tag_pub.publish(msg);
}

