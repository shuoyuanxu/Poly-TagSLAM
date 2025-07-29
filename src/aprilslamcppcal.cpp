#include "aprilslamheader.h"
#include "publishing_utils.h"

namespace aprilslam {
// Constructor
aprilslamcpp::aprilslamcpp(ros::NodeHandle node_handle)
    : nh_(node_handle), tf_listener_(tf_buffer_) { 
    
    // Read topics and corresponding frame
    std::string odom_topic, trajectory_topic;
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("trajectory_topic", trajectory_topic);
    nh_.getParam("map_frame_id", map_frame_id);
    nh_.getParam("robot_frame", robot_frame);

    // Read batch optimization flag
    nh_.getParam("batch_optimisation", batchOptimisation_);

    // Read noise models
    std::vector<double> odometry_noise, prior_noise, bearing_range_noise, point_noise;
    nh_.getParam("noise_models/odometry", odometry_noise);
    nh_.getParam("noise_models/prior", prior_noise);
    nh_.getParam("noise_models/bearing_range", bearing_range_noise);
    nh_.getParam("noise_models/point", point_noise);

    // Read error threshold for a landmark to be added to the graph
    nh_.getParam("add2graph_threshold", add2graph_threshold);    

    // Stationary conditions
    nh_.getParam("stationary_position_threshold", stationary_position_threshold);
    nh_.getParam("stationary_rotation_threshold", stationary_rotation_threshold);

    // Read calibration and localization settings
    std::string package_path = ros::package::getPath("aprilslamcpp");
    std::string save_path, load_path;
    nh_.getParam("pathtosavelandmarkcsv", save_path);
    nh_.getParam("pathtoloadlandmarkcsv", load_path);

    // Construct the full paths
    pathtosavelandmarkcsv = package_path + "/" + save_path;
    pathtoloadlandmarkcsv = package_path + "/" + load_path;
    nh_.getParam("savetaglocation", savetaglocation);
    nh_.getParam("usepriortagtable", usepriortagtable);

    // Load camera topics
    if (nh_.getParam("camera_config/cameras", camera_list) && camera_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < camera_list.size(); ++i) {
            if (camera_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

            std::string name = static_cast<std::string>(camera_list[i]["name"]);
            std::string topic = static_cast<std::string>(camera_list[i]["topic"]);
            std::string frame_id = static_cast<std::string>(camera_list[i]["frame"]);

            camera_infos_.emplace_back(CameraInfo{name, topic, frame_id, Eigen::Vector3d::Zero()});
        }
    } else {
        ROS_WARN("Failed to load camera_config/cameras or invalid format.");
    }

    // Wait for static transforms using frame_id
    for (auto& cam : camera_infos_) {
        tf2::Transform tf;
        const int max_attempts = 20;
        const ros::Duration retry_interval(0.5);
        bool success = false;

        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            if (getStaticTransform(robot_frame, cam.frame_id, tf)) {
                double x = tf.getOrigin().x();
                double y = tf.getOrigin().y();
                double yaw = tf.getRotation().getAngle();
                tf2::Vector3 axis = tf.getRotation().getAxis();
                if (axis.z() < 0) yaw = -yaw;

                cam.transform = Eigen::Vector3d(x, y, yaw);
                ROS_INFO("Static TF loaded for camera [%s] (%s): (%.2f, %.2f, %.2f rad)",
                        cam.name.c_str(), cam.frame_id.c_str(), x, y, yaw);
                success = true;
                break;
            } else {
                ROS_WARN("Waiting for static TF from %s to %s... (attempt %d)",
                        robot_frame.c_str(), cam.frame_id.c_str(), attempt + 1);
                retry_interval.sleep();
            }
        }

        if (!success) {
            ROS_ERROR("Failed to get static transform for camera %s (%s). Shutting down.",
                    cam.name.c_str(), cam.frame_id.c_str());
            ros::shutdown();
            return;
        }
    }


    // Initialize noise models
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << odometry_noise[0], odometry_noise[1], odometry_noise[2]).finished());
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << prior_noise[0], prior_noise[1], prior_noise[2]).finished());
    brNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << bearing_range_noise[0], bearing_range_noise[1]).finished());
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << point_noise[0], point_noise[1]).finished());

    // Total number of IDs
    int total_tags;
    nh_.getParam("total_tags", total_tags);
    
    // Predefined tags to search for in the environment
    for (int j = 0; j < total_tags; ++j) {
        possibleIds_.push_back("tag_" + std::to_string(j));
    }

    // Bag stop flag
    double inactivity_threshold;
    nh_.getParam("inactivity_threshold", inactivity_threshold);

    // Initialize GTSAM components
    initializeGTSAM();
    // Index to keep track of the sequential pose.
    index_of_pose = 1;
    // Initialize the factor graphs
    keyframeGraph_ = gtsam::NonlinearFactorGraph();

    // Initialize camera subscribers
    for (const auto& cam : camera_infos_) {
        ros::Subscriber sub = nh_.subscribe<apriltag_ros::AprilTagDetectionArray>(
            cam.topic, 1,
            boost::bind(&aprilslamcpp::cameraCallback, this, _1, cam.name)
        );
        camera_subscribers_.push_back(sub);
    }

    // Subscriptions and Publications
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
    landmark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
    path.header.frame_id = map_frame_id; 

    // Timer to periodically check if valid data has been received by any camera
    check_data_timer_ = nh_.createTimer(ros::Duration(2.0), [this, inactivity_threshold](const ros::TimerEvent&) {
        if (!received_camera_names_.empty()) {
            accumulated_time_ = 0.0;
            received_camera_names_.clear();  // Reset for next cycle
        } else {
            accumulated_time_ += 2.0;
            ROS_WARN("No new valid data received from any camera. Accumulated time: %.1f seconds", accumulated_time_);

            if (accumulated_time_ >= inactivity_threshold) {
                ROS_ERROR("No valid data from any camera for %.1f seconds. Shutting down.", inactivity_threshold);
                this->~aprilslamcpp();  // Trigger the destructor
            }
        }
    });
}

// Destructor implementation
aprilslamcpp::~aprilslamcpp() {
    ROS_INFO("Node is shutting down. Executing SAMOptimise().");

    std::map<int, gtsam::Point2> landmarks_unoptimised;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point2 point = keyframeEstimates_.at<gtsam::Point2>(key);  // Access the Point2 value
            landmarks_unoptimised[gtsam::Symbol(key).index()] = point;
        }
    }

    if (savetaglocation) {
        saveLandmarksToCSV(landmarks_unoptimised, pathtoloadlandmarkcsv);
    }
    
    gtsam::Values result = SAMOptimise();
    keyframeEstimates_ = result;
    
    // Extract landmark estimates from the result
    std::map<int, gtsam::Point2> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point2 point = keyframeEstimates_.at<gtsam::Point2>(key);  // Access the Point2 value
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }

    // Publish the pose and landmarks
    aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);
    aprilslam::publishPath(path_pub_, keyframeEstimates_, index_of_pose, map_frame_id);

    // Save the landmarks into a CSV file if required
    if (savetaglocation) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
    optimizationExecuted_ = true;
    ROS_INFO("SAMOptimise() executed successfully.");
}

// Callback function for Cam topic
void aprilslamcpp::cameraCallback(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg,
    const std::string& camera_name) {    
    if (!msg->detections.empty()) {
        camera_detections_[camera_name] = msg;
        received_camera_names_.insert(camera_name);
    } else {
        camera_detections_.erase(camera_name);
    }
}

bool aprilslamcpp::getStaticTransform(const std::string& target_frame,
                                      const std::string& source_frame,
                                      tf2::Transform& out_tf) {
    try {
        geometry_msgs::TransformStamped transform_stamped =
            tf_buffer_.lookupTransform(target_frame, source_frame,
                                       ros::Time(0), ros::Duration(2.0));
        tf2::fromMsg(transform_stamped.transform, out_tf);
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Could not get static transform from %s to %s: %s",
                 source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

// Initialization of GTSAM components
void aprilslamcpp::initializeGTSAM() { 
    // Initialize graph parameters and stores them in isam_.
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(parameters);
}

gtsam::Pose2 aprilslamcpp::translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    tf2::Quaternion tfQuat(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
    return gtsam::Pose2(x, y, yaw);
}

gtsam::Values aprilslamcpp::SAMOptimise() {    
    // Perform batch optimization using Levenberg-Marquardt optimizer
    gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    gtsam::Values result = batchOptimizer.optimize();
    return result;
}

// Check if movement exceeds the stationary thresholds
bool aprilslam::aprilslamcpp::movementExceedsThreshold(const gtsam::Pose2& poseSE2) {
    double position_change = std::hypot(poseSE2.x() - lastPoseSE2_.x(), poseSE2.y() - lastPoseSE2_.y());
    double rotation_change = std::abs(wrapToPi(poseSE2.theta() - lastPoseSE2_.theta()));
    return position_change >= stationary_position_threshold || rotation_change >= stationary_rotation_threshold;
}

// Handle initialization of the first pose
void aprilslam::aprilslamcpp::initializeFirstPose(const gtsam::Pose2& poseSE2, gtsam::Pose2& pose0) {
    lastPoseSE2_ = poseSE2;
    lastPoseSE2_vis = poseSE2;
    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), pose0, priorNoise));
    keyframeEstimates_.insert(gtsam::Symbol('X', 1), pose0);
    Estimates_visulisation.insert(gtsam::Symbol('X', 1), pose0);
    lastPose_ = pose0; // Keep track of the last pose for odolandmarkKeymetry calculation
    // Load calibrated landmarks as priors if available
    if (usepriortagtable) {
    std::map<int, gtsam::Point2> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
        for (const auto& landmark : savedLandmarks) {
            gtsam::Symbol landmarkKey('L', landmark.first);
            keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, landmark.second, pointNoise));
            keyframeEstimates_.insert(landmarkKey, landmark.second);
            landmarkEstimates.insert(landmarkKey, landmark.second);
        }
    }
    Key_previous_pos = pose0;
    previousKeyframeSymbol = gtsam::Symbol('X', 1);
}

// Predict the next pose based on odometry
gtsam::Pose2 aprilslam::aprilslamcpp::predictNextPose(const gtsam::Pose2& poseSE2) {
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
    // gtsam::Pose2 adjustedOdometry = odometryDirection(odometry, linear_x_velocity_);
    return lastPose_.compose(odometry);
}

// Update the graph with landmarks detections
std::set<gtsam::Symbol> aprilslam::aprilslamcpp::updateGraphWithLandmarks(
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos, 
    const std::pair<std::vector<int>, std::vector<Eigen::Vector2d>>& detections) {

    // Access the elements of the std::pair   
    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector2d>& tagPos = detections.second;

    if (!Id.empty()) {
        for (size_t n = 0; n < Id.size(); ++n) {
            int tag_number = Id[n];        
            Eigen::Vector2d landSE2 = tagPos[n];

            // Compute prior location of the landmark using the current robot pose
            double theta = lastPose_.theta();
            Eigen::Rotation2Dd rotation(theta);  // Create a 2D rotation matrix
            Eigen::Vector2d rotatedPosition = rotation * landSE2;  // Rotate the position into the robot's frame
            gtsam::Point2 priorLand(rotatedPosition.x() + lastPose_.x(), rotatedPosition.y() + lastPose_.y());

            // Compute bearing and range
            double bearing = std::atan2(landSE2(1), landSE2(0));
            double range = std::sqrt(landSE2(0) * landSE2(0) + landSE2(1) * landSE2(1));

            // Construct the landmark key
            gtsam::Symbol landmarkKey('L', tag_number);  

            // Check if the landmark has been observed before
            if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                    // Existing landmark
                    gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                        gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                    );
                    gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                    // Threshold for ||projection - measurement||
                    if (fabs(error[0]) < add2graph_threshold) keyframeGraph_.add(factor);
            } 
            else {
                // If the current landmark was not detected in the calibration run 
                // Or it's on calibration mode
                if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                // New landmark detected
                detectedLandmarksHistoric.insert(landmarkKey);
                // Check if the key already exists in keyframeEstimates_ before inserting
                if (keyframeEstimates_.exists(landmarkKey)) {
                } else {
                    keyframeEstimates_.insert(landmarkKey, priorLand); // Simple initial estimate
                }

                // Check if the key already exists in landmarkEstimates before inserting
                if (landmarkEstimates.exists(landmarkKey)) {
                } else {
                    landmarkEstimates.insert(landmarkKey, priorLand);
                }

                // Add a prior for the landmark position to help with initial estimation.
                keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(
                    landmarkKey, priorLand, pointNoise)
                );
                }
                // Add a bearing-range observation for this landmark to the graph
                gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, gtsam::Rot2::fromAngle(bearing), range, brNoise
                );
                keyframeGraph_.add(factor);
            }
            // Store the bearing and range measurements in the map
            poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = std::make_tuple(bearing, range); 
        }
    }
    return detectedLandmarksCurrentPos;
}

void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Store the initial pose for relative calculations
    pose0 = gtsam::Pose2(0.0, 0.0, 0.0); // Prior at origin

    // Check if the movement exceeds the thresholds
    if (!movementExceedsThreshold(poseSE2)) return;

    index_of_pose += 1; // Increment the pose index for each new odometry message
    if (index_of_pose == 2) initializeFirstPose(poseSE2, pose0);

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 predictedPose = predictNextPose(poseSE2);

    // Determine if this pose should be a keyframe
    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);

    // Add odometry factor
    keyframeEstimates_.insert(gtsam::Symbol('X', index_of_pose), predictedPose);
    if (previousKeyframeSymbol) {
        gtsam::Pose2 relativePose = Key_previous_pos.between(predictedPose);
        keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
    }
        
    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), predictedPose);

    std::set<gtsam::Symbol> detectedLandmarksCurrentPos;
    
    // Iterate through all landmark detected IDs
    auto detections = getCamDetections(camera_infos_, camera_detections_);
    if (!detections.first.empty()) {
        detectedLandmarksCurrentPos = updateGraphWithLandmarks(detectedLandmarksCurrentPos, detections);
    }
    
    lastPoseSE2_ = poseSE2;
    Key_previous_pos = predictedPose;

    // Visulisation
    previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);
     // Extract landmark estimates from the result
    std::map<int, gtsam::Point2> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point2 point = keyframeEstimates_.at<gtsam::Point2>(key);  // Access the Point2 value
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }
    // Publish the pose and landmarks
    aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);
    aprilslam::publishPath(path_pub_, keyframeEstimates_, index_of_pose, map_frame_id);

    // Save the landmarks into a CSV file if required
    if (savetaglocation) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
}
}

int main(int argc, char **argv) {
    // Initialize the ROS system and specify the name of the node
    ros::init(argc, argv, "april_slam_cpp");

    // Create a handle to this process' node
    ros::NodeHandle nh;

    // Create an instance of the aprilslamcpp class, passing in the node handle
    aprilslam::aprilslamcpp slamNode(nh);

    // ROS enters a loop, pumping callbacks. Internally, it will call all the callbacks waiting to be called at that point in time.
    ros::spin();

    return 0;
}