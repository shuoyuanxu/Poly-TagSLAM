#ifndef PUBLISHING_UTILS_H
#define PUBLISHING_UTILS_H

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <gtsam/geometry/Pose2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For TF2 quaternion conversion functions
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <random>
#include <algorithm>

namespace aprilslam {
    void visualizeLoopClosure(ros::Publisher& lc_pub, const gtsam::Pose2& currentPose, const gtsam::Pose2& keyframePose, int currentPoseIndex, const std::string& frame_id);
    void publishMapToOdomTF(tf2_ros::TransformBroadcaster& tf_broadcaster, 
                            const gtsam::Values& result, int latest_index, 
                            const gtsam::Pose2& poseSE2, 
                            const std::string& map_frame, const std::string& odom_frame, const std::string& base_link_frame);
    void publishRefinedOdom(ros::Publisher& odom_pub,
                        const gtsam::Values& Estimates_visulisation,
                        int index_of_pose,
                        const std::string& odom_frame,      
                        const std::string& base_link_frame,
                        std::ofstream& refined_odom_csv,
                        const ros::Time& stamp);
    void publishLandmarks(ros::Publisher& landmark_pub, const std::map<int, gtsam::Point2>& landmarks, const std::string& frame_id);
    void publishPath(ros::Publisher& path_pub, const gtsam::Values& result, int max_index, const std::string& frame_id);
    void saveLandmarksToCSV(const std::map<int, gtsam::Point2>& landmarks, const std::string& filename);
    std::map<int, gtsam::Point2> loadLandmarksFromCSV(const std::string& filename);
    void processDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr& cam_msg, 
        const Eigen::Vector3d& xyTrans_cam_baselink, 
        std::vector<int>& Ids, 
        std::vector<Eigen::Vector2d>& tagPoss);
    std::pair<std::vector<int>, std::vector<Eigen::Vector2d>> getCamDetections(
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& mCam_msg,
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& rCam_msg,
        const apriltag_ros::AprilTagDetectionArray::ConstPtr& lCam_msg,
        const Eigen::Vector3d& xyTrans_lcam_baselink,
        const Eigen::Vector3d& xyTrans_rcam_baselink,
        const Eigen::Vector3d& xyTrans_mcam_baselink);
    std::vector<Eigen::Vector3d> initParticles(int Ninit);
    std::vector<Eigen::Vector3d> particleFilter(const std::vector<int>& Id,
        const std::vector<Eigen::Vector2d>& tagPos,
        const std::map<int, gtsam::Point2>& savedLandmarks,
        std::vector<Eigen::Vector3d>& x_P,
        int N,
        double rngVar,
        double brngVar);
    std::vector<Eigen::Vector3d> initParticlesFromFirstTag(
        const std::vector<int>& Id,
        const std::vector<Eigen::Vector2d>& tagPos,
        const std::map<int, gtsam::Point2>& savedLandmarks,
        int Ninit);
    double wrapToPi(double angle);
    gtsam::Pose2 relPoseFG(const gtsam::Pose2& lastPoseSE2, const gtsam::Pose2& PoseSE2);
    // Assuming detection messages include id and pose information
    struct Detection {
    int Id;
    geometry_msgs::Pose Pose;
    };

    struct CameraMessage {
        std::vector<Detection> Detections;
    };
}

#endif

