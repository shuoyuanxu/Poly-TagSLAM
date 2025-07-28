#ifndef APRIL_SLAM_H
#define APRIL_SLAM_H
#include "publishing_utils.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <gtsam/geometry/Pose2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For TF2 quaternion conversion functions
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <cxxabi.h>
#include <boost/pointer_cast.hpp>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <cmath>
#include <gtsam/nonlinear/Marginals.h>
#include <tf2_ros/transform_broadcaster.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <XmlRpcValue.h>


namespace aprilslam {

class aprilslamcpp {
public:
    explicit aprilslamcpp(ros::NodeHandle node_handle); // Constructor
    ~aprilslamcpp(); // Deconstructor
    void initializeGTSAM(); // Method to initialize GTSAM components
    gtsam::Pose2 translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg); // Removed redundant class scope
    void ISAM2Optimise(); //ISAM optimiser
    gtsam::Values SAMOptimise(); //SAM optimiser
    bool movementExceedsThreshold(const gtsam::Pose2& poseSE2);
    void initializeFirstPose(const gtsam::Pose2& poseSE2, gtsam::Pose2& pose0);
    gtsam::Pose2 predictNextPose(const gtsam::Pose2& poseSE2);
    void updateOdometryPose(const gtsam::Pose2& poseSE2);
    void generate2bePublished();
    std::set<gtsam::Symbol> updateGraphWithLandmarks(std::set<gtsam::Symbol> detectedLandmarksCurrentPos, const std::pair<std::vector<int>, std::vector<Eigen::Vector2d>>& detections);
    void addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg);
    void checkLoopClosure(const std::set<gtsam::Symbol>& detectedLandmarks);
    bool shouldAddKeyframe(const gtsam::Pose2& lastPose, const gtsam::Pose2& currentPose, std::set<gtsam::Symbol> oldlandmarks, std::set<gtsam::Symbol> detectedLandmarksCurrentPos);
    void cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg, const std::string& camera_name);
    void mCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void rCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void lCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void pfInitCallback(const ros::TimerEvent& event);
    void pruneGraphByPoseCount(int maxPoses);
    void smoothTrajectory(int window_size); 
    double computePoseDelta(const gtsam::Pose2& oldPose, const gtsam::Pose2& newPose);
private:
    ros::Timer check_data_timer_;  // Declare the timer here
    ros::Publisher path_pub_;
    ros::Publisher odom_traj_pub_;
    ros::Publisher landmark_pub_;
    ros::Publisher lc_pub_;
    nav_msgs::Path path;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    // camera info
    std::vector<aprilslam::CameraInfo> camera_infos_;
    std::map<std::string, apriltag_ros::AprilTagDetectionArray::ConstPtr> camera_detections_;
    XmlRpc::XmlRpcValue camera_list;
    std::vector<ros::Subscriber> camera_subscribers_;
    std::set<std::string> received_camera_names_;

    ros::Subscriber mCam_subscriber;
    ros::Subscriber rCam_subscriber;
    ros::Subscriber lCam_subscriber;
    apriltag_ros::AprilTagDetectionArray::ConstPtr mCam_msg;
    apriltag_ros::AprilTagDetectionArray::ConstPtr rCam_msg;
    apriltag_ros::AprilTagDetectionArray::ConstPtr lCam_msg;
    // cmd_vel subscriber
    ros::Subscriber cmd_vel_sub_;
    double linear_x_velocity_;  // Stores the latest controller input linear.x value
    // ros sub and put info
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    // gtsam variables
    std::map<gtsam::Symbol, std::map<gtsam::Symbol, std::tuple<double, double>>> poseToLandmarkMeasurementsMap;  //storing X-L pair
    std::map<gtsam::Key, gtsam::Point2> historicLandmarks;     // Maintain a persistent storage for historic landmarks
    gtsam::Values landmarkEstimates;  // for unwhitten error computing 
    gtsam::NonlinearFactorGraph keyframeGraph_;  // Keyframe graph: All keyframes and associated landmarks
    gtsam::Values keyframeEstimates_;            // Estimates for keyframes
    gtsam::Values Estimates_visulisation;
    gtsam::Pose2 Key_previous_pos;
    gtsam::Symbol previousKeyframeSymbol;
    gtsam::ISAM2 isam_;
    gtsam::Pose2 lastPoseSE2_;
    gtsam::Pose2 lastPoseSE2_vis;
    gtsam::Pose2 lastPose_;
    
    // initialisation variable
    gtsam::Pose2 pose0;
    int N_particles;
    bool usePFinitialise;
    double PFWaitTime;
    ros::Timer pf_init_timer_; // timer
    bool pfInitialized_ = false;
    bool pfInitInProgress_ = false;
    double rngVar_;
    double brngVar_;
    double pfInitStartTime_;
    std::vector<Eigen::Vector3d> x_P_pf_;
    std::map<int, gtsam::Point2> savedLandmarks;

    std::vector<std::string> possibleIds_; // Predefined tags in the environment
    std::map<int, gtsam::Symbol> tagToNodeIDMap_; // Map from tag IDs to node IDs
    int index_of_pose;
    bool batchOptimisation_;
    // Noise Models
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr brNoise;
    gtsam::noiseModel::Diagonal::shared_ptr pointNoise;
    gtsam::noiseModel::Diagonal::shared_ptr loopClosureNoise;
    double add2graph_threshold;
    std::string map_frame_id;
    gtsam::Pose2 initial_pose;
    // std::string ud_frame;
    std::string odom_trajectory_frame;
    std::string robot_frame;
    std::string odom_frame;
    std::string pathtosavelandmarkcsv;
    std::string pathtoloadlandmarkcsv;
    double maxfactors; // Allowed total number of factors in the graph before pruning
    bool useprunebysize;
    // For loop closure 
    bool useloopclosure;
    double historyKeyframeSearchRadius;
    int historyKeyframeSearchNum;
    int requiredReobservedLandmarks;

    double stationary_position_threshold;
    double stationary_rotation_threshold;
    bool savetaglocation;
    bool usepriortagtable;
    std::map<gtsam::Symbol, std::set<gtsam::Symbol>> poseToLandmarks; // Maps pose index to a set of detected landmark IDs, e.g. X1: L1,L2,L3.
    // For keyframe
    double distanceThreshold;
    double rotationThreshold;
    std::vector<double> xyTrans_lcam_baselink;
    std::vector<double> xyTrans_rcam_baselink;
    std::vector<double> xyTrans_mcam_baselink;
    std::string lCam_topic;
    std::string rCam_topic;
    std::string mCam_topic;
    std::string cmd_topic;
    Eigen::Vector3d lcam_baselink_transform;
    Eigen::Vector3d rcam_baselink_transform;
    Eigen::Vector3d mcam_baselink_transform;
    std::set<gtsam::Symbol> detectedLandmarksHistoric;

    // Symbol of the pose that currently has a prior
    gtsam::FastMap<gtsam::Symbol, bool> priorAddedToPose;

    // Flag to ensure SAMOptimise is called at the end 
    bool mCam_data_received_, rCam_data_received_, lCam_data_received_;
    bool optimizationExecuted_;
    double accumulated_time_;  // Accumulated time since last valid data

    // Optimisation type: true for ISAM2, false for SAM
    bool useisam2;

    // Use keyframe or not
    bool usekeyframe;

    std::ofstream refined_odom_csv;
    std::ofstream raw_odom_csv;

    // Some Heuristic things
    double jumpTranslationThreshold;
    double jumpRotationThreshold;
    double jumpCombinedThreshold;
    int smoothingStartIndex_;
    int smoothingwindow;
    int outlierRemovalStartIndex_;
    gtsam::Symbol previousframeSymbol;
    gtsam::Pose2 lastPose_for_jump;
    bool useoutlierremoval;
    bool usetrajsmoothing;
    };
} 

#endif // aprilslamcpp
