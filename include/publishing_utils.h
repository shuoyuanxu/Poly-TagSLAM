#ifndef PUBLISHING_UTILS_H
#define PUBLISHING_UTILS_H

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <gtsam/geometry/Pose3.h>  
#include <gtsam/geometry/Point3.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    struct CameraInfo {
        std::string name;
        std::string topic;
        std::string frame_id;
        gtsam::Pose3 transform;  
    };
    
    // Loop closure visualization 
    void visualizeLoopClosure(ros::Publisher& lc_pub, 
                            const gtsam::Pose3& currentPose, 
                            const gtsam::Pose3& keyframePose, 
                            int currentPoseIndex, 
                            const std::string& frame_id);
    
    // TF publishing 
    void publishMapToOdomTF(tf2_ros::TransformBroadcaster& tf_broadcaster, 
                            const gtsam::Values& result, 
                            int latest_index, 
                            const gtsam::Pose3& poseSE3,
                            const std::string& map_frame, 
                            const std::string& odom_frame, 
                            const std::string& base_link_frame);
    
    // Refined odometry publishing 
    void publishRefinedOdom(ros::Publisher& odom_pub,
                            const gtsam::Values& Estimates_visulisation,
                            int index_of_pose,
                            const std::string& odom_frame,      
                            const std::string& base_link_frame,
                            std::ofstream& refined_odom_csv,
                            const ros::Time& stamp);
    
    // Landmark publishing 
    void publishLandmarks(ros::Publisher& landmark_pub, 
                         const std::map<int, gtsam::Point3>& landmarks,
                         const std::string& frame_id);
    
    // Path publishing 
    void publishPath(ros::Publisher& path_pub, 
                    const gtsam::Values& result, 
                    int max_index, 
                    const std::string& frame_id);
    
    // CSV I/O for landmarks 
    void saveLandmarksToCSV(const std::map<int, gtsam::Point3>& landmarks,
                           const std::string& filename);
    
    std::map<int, gtsam::Point3> loadLandmarksFromCSV(const std::string& filename);
    
    // Detection processing 
    void processDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr& cam_msg, 
                        const gtsam::Pose3& pose_cam_baselink,
                        std::vector<int>& Ids, 
                        std::vector<Eigen::Vector3d>& tagPoss,
                        bool is_3d_mode); 
    
    // Get camera detections 
    std::pair<std::vector<int>, std::vector<Eigen::Vector3d>> getCamDetections(
        const std::vector<CameraInfo>& camera_infos,
        const std::map<std::string, apriltag_ros::AprilTagDetectionArray::ConstPtr>& camera_detections,
        bool is_3d_mode);

    std::vector<Eigen::Matrix<double, 6, 1>> initParticles(int Ninit);
    
    std::vector<Eigen::Matrix<double, 6, 1>> particleFilter(
        const std::vector<int>& Id,
        const std::vector<Eigen::Vector3d>& tagPos,
        const std::map<int, gtsam::Point3>& savedLandmarks,
        std::vector<Eigen::Matrix<double, 6, 1>>& x_P,
        int N,
        double rngVar,
        double brngVar);
    
    std::vector<Eigen::Matrix<double, 6, 1>> initParticlesFromFirstTag(
        const std::vector<int>& Id,
        const std::vector<Eigen::Vector3d>& tagPos,
        const std::map<int, gtsam::Point3>& savedLandmarks,
        int Ninit);
    
    // Utility functions
    double wrapToPi(double angle);
    
    // Relative pose computation 
    gtsam::Pose3 relPoseFG(const gtsam::Pose3& lastPoseSE3, const gtsam::Pose3& PoseSE3);
}

#endif