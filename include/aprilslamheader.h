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
#include <gtsam/geometry/Pose3.h>  // Changed from Pose2
#include <gtsam/geometry/Point3.h> // Added Point3
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    struct PoleConstraint {
    int tag_id_1;
    int tag_id_2;
    double distance;  // Known distance between tags
    };

    explicit aprilslamcpp(ros::NodeHandle node_handle); // Constructor
    ~aprilslamcpp(); // Destructor
    void initializeGTSAM(); // Method to initialize GTSAM components
    
    gtsam::Pose3 translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg);
    void ISAM2Optimise(); //ISAM optimiser
    gtsam::Values SAMOptimise(); //SAM optimiser
    bool movementExceedsThreshold(const gtsam::Pose3& poseSE3);
    void initializeFirstPose(const gtsam::Pose3& poseSE3, gtsam::Pose3& pose0);
    gtsam::Pose3 predictNextPose(const gtsam::Pose3& poseSE3);
    void updateOdometryPose(const gtsam::Pose3& poseSE3);
    void generate2bePublished();
    
    std::set<gtsam::Symbol> updateGraphWithLandmarks(
        std::set<gtsam::Symbol> detectedLandmarksCurrentPos, 
        const std::pair<std::vector<int>, std::vector<Eigen::Vector3d>>& detections);
    
    void addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg);
    void checkLoopClosure(const std::set<gtsam::Symbol>& detectedLandmarks);
    
    bool shouldAddKeyframe(const gtsam::Pose3& lastPose, const gtsam::Pose3& currentPose, 
                          std::set<gtsam::Symbol> oldlandmarks, 
                          std::set<gtsam::Symbol> detectedLandmarksCurrentPos);
    
    void cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg, 
                       const std::string& camera_name);
    void mCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void rCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void lCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void pfInitCallback(const ros::TimerEvent& event);
    void pruneGraphByPoseCount(int maxPoses);
    void smoothTrajectory(int window_size); 
    
    double computePoseDelta(const gtsam::Pose3& oldPose, const gtsam::Pose3& newPose);
    
    bool getStaticTransform(const std::string& target_frame,
                           const std::string& source_frame,
                           tf2::Transform& out_tf);
                           
private:
    ros::Timer check_data_timer_;
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
    double linear_x_velocity_;
    
    // ros sub and pub info
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    
    // Measurements (bearing, elevation, range)
    std::map<gtsam::Symbol, std::map<gtsam::Symbol, std::tuple<double, double, double>>> 
        poseToLandmarkMeasurementsMap;  // X-L pair: bearing, elevation, range
    
    // Historic landmarks
    std::map<gtsam::Key, gtsam::Point3> historicLandmarks;
    
    gtsam::Values landmarkEstimates;  // for unwhitened error computing 
    gtsam::NonlinearFactorGraph keyframeGraph_;
    gtsam::Values keyframeEstimates_;
    gtsam::Values Estimates_visulisation;
    
    gtsam::Pose3 Key_previous_pos;
    gtsam::Symbol previousKeyframeSymbol;
    gtsam::ISAM2 isam_;
    gtsam::Pose3 lastPoseSE3_;      
    gtsam::Pose3 lastPoseSE3_vis;
    gtsam::Pose3 lastPose_;
    
    // initialisation variable
    gtsam::Pose3 pose0;
    int N_particles;
    bool usePFinitialise;
    double PFWaitTime;
    ros::Timer pf_init_timer_;
    bool pfInitialized_ = false;
    bool pfInitInProgress_ = false;
    double rngVar_;
    double brngVar_;
    double pfInitStartTime_;
    
    // Particle filter poses (x, y, z, roll, pitch, yaw)
    std::vector<Eigen::Matrix<double, 6, 1>> x_P_pf_;  
    // Saved landmarks
    std::map<int, gtsam::Point3> savedLandmarks;

    std::vector<std::string> possibleIds_;
    std::map<int, gtsam::Symbol> tagToNodeIDMap_;
    int index_of_pose;
    bool batchOptimisation_;
    
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;     
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;        
    gtsam::noiseModel::Diagonal::shared_ptr brNoise;          
    gtsam::noiseModel::Diagonal::shared_ptr pointNoise;       
    gtsam::noiseModel::Diagonal::shared_ptr loopClosureNoise; 
    
    double add2graph_threshold;
    std::string map_frame_id;
    gtsam::Pose3 initial_pose;  // Changed from Pose2
    std::string odom_trajectory_frame;
    std::string robot_frame;
    std::string odom_frame;
    std::string pathtosavelandmarkcsv;
    std::string pathtoloadlandmarkcsv;
    double maxfactors;
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
    std::map<gtsam::Symbol, std::set<gtsam::Symbol>> poseToLandmarks;
    
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
    
    // Full 6DOF transforms
    Eigen::Matrix<double, 6, 1> lcam_baselink_transform;  // (x,y,z,roll,pitch,yaw)
    Eigen::Matrix<double, 6, 1> rcam_baselink_transform;
    Eigen::Matrix<double, 6, 1> mcam_baselink_transform;
    
    std::set<gtsam::Symbol> detectedLandmarksHistoric;
    gtsam::FastMap<gtsam::Symbol, bool> priorAddedToPose;

    // Flags
    bool mCam_data_received_, rCam_data_received_, lCam_data_received_;
    bool optimizationExecuted_;
    double accumulated_time_;
    bool useisam2;
    bool usekeyframe;

    std::ofstream refined_odom_csv;
    std::ofstream raw_odom_csv;

    // Heuristic things
    double jumpTranslationThreshold;
    double jumpRotationThreshold;
    double jumpCombinedThreshold;
    int smoothingStartIndex_;
    int smoothingwindow;
    int outlierRemovalStartIndex_;
    gtsam::Symbol previousframeSymbol;
    gtsam::Pose3 lastPose_for_jump;
    bool useoutlierremoval;
    bool usetrajsmoothing;
};

} 

#endif // aprilslamcpp