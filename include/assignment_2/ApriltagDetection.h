#ifndef APRILTAG_DETECTION_H
#define APRILTAG_DETECTION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <assignment_2/detectedApriltags.h>



class ApriltagDetection {
private:
    ros::NodeHandle& nh_;

    ros::Subscriber apriltagDetectionSub_;
    ros::Publisher tag_pub;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    std::map<std::string, geometry_msgs::PoseStamped> detectedTags_;

public:
    ApriltagDetection(ros::NodeHandle& nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {
        apriltagDetectionSub_ = nh_.subscribe("/tag_detections", 1, &ApriltagDetection::detectionCallback, this);
        tag_pub = nh_.advertise<assignment_2::detectedApriltags>("detected_tags", 10);
    }

    void detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);
    void publishTags();

    std::map<std::string, geometry_msgs::PoseStamped> getDetectedTags() {
        return detectedTags_;
    }
};

#endif // APRILTAGDETECTION_H