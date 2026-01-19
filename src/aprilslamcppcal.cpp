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

    // ============ Read SLAM mode ============
    std::string slam_mode;
    nh_.param<std::string>("slam_mode", slam_mode, "2D");
    is_3d_mode_ = (slam_mode == "3D" || slam_mode == "3d");
    ROS_INFO("SLAM Mode: %s", is_3d_mode_ ? "3D" : "2D");

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
    if (nh_.getParam("camera_config/cameras", camera_list) && 
        camera_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < camera_list.size(); ++i) {
            if (camera_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

            std::string name = static_cast<std::string>(camera_list[i]["name"]);
            std::string topic = static_cast<std::string>(camera_list[i]["topic"]);
            std::string frame_id = static_cast<std::string>(camera_list[i]["frame"]);

            // Initialize transform as identity (will be populated from TF)
            gtsam::Pose3 transform = gtsam::Pose3();
            camera_infos_.emplace_back(CameraInfo{name, topic, frame_id, transform});
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
                
                tf2::Vector3 trans = tf.getOrigin();
                tf2::Quaternion rot = tf.getRotation();

                if (is_3d_mode_) {
                    // Full 6DOF transform for 3D mode
                    gtsam::Point3 translation(trans.x(), trans.y(), trans.z());
                    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(rot.w(), rot.x(), rot.y(), rot.z());
                    cam.transform = gtsam::Pose3(rotation, translation);
                    
                    ROS_INFO("TF loaded for [%s] (%s): xyz(%.2f, %.2f, %.2f), rpy(%.2f, %.2f, %.2f rad)",
                            cam.name.c_str(), cam.frame_id.c_str(), 
                            trans.x(), trans.y(), trans.z(),
                            rotation.roll(), rotation.pitch(), rotation.yaw());
                } else {
                    // 2D mode: Project camera z-axis to xy plane to get yaw
                    // Convert to Eigen
                    Eigen::Quaterniond tf_rot(rot.w(), rot.x(), rot.y(), rot.z());
                    Eigen::Matrix3d R = tf_rot.toRotationMatrix();
                    
                    Eigen::Vector3d z_axis_robot = R.col(2);  // Camera z-axis in robot frame
                    z_axis_robot.z() = 0.0;                   // Project to xy plane
                    z_axis_robot.normalize();                 // Normalize
                    double yaw = std::atan2(z_axis_robot.y(), z_axis_robot.x());
                    
                    // Final transform (planar)
                    gtsam::Point3 translation(trans.x(), trans.y(), 0.0);
                    gtsam::Rot3 rotation = gtsam::Rot3::Ypr(yaw, 0.0, 0.0);
                    cam.transform = gtsam::Pose3(rotation, translation);
                    
                    ROS_INFO("TF loaded for [%s] (%s) [2D]: xy(%.2f, %.2f), yaw(%.2f rad)", 
                            cam.name.c_str(), cam.frame_id.c_str(), 
                            trans.x(), trans.y(), yaw);
                }
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

    // ============ Initialize noise models ============
    if (is_3d_mode_) {
        // 3D mode: Full 6DOF noise models
        if (odometry_noise.size() != 6 || prior_noise.size() != 6 || 
            bearing_range_noise.size() != 3 || point_noise.size() != 3) {
            ROS_ERROR("3D mode requires: odometry(6), prior(6), bearing_range(3), point(3)");
            ros::shutdown();
            return;
        }

        odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << odometry_noise[0], odometry_noise[1], odometry_noise[2],
                                 odometry_noise[3], odometry_noise[4], odometry_noise[5]).finished());
        priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << prior_noise[0], prior_noise[1], prior_noise[2],
                                 prior_noise[3], prior_noise[4], prior_noise[5]).finished());
        brNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << bearing_range_noise[0], bearing_range_noise[1], 
                                 bearing_range_noise[2]).finished());
        pointNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << point_noise[0], point_noise[1], point_noise[2]).finished());

        ROS_INFO("3D noise models initialized");
    } else {
        // 2D mode: Use TIGHT constraints on z, roll, pitch to enforce planarity
        if (odometry_noise.size() < 3 || prior_noise.size() < 3 || 
            bearing_range_noise.size() < 2 || point_noise.size() < 2) {
            ROS_ERROR("2D mode requires: odometry(3), prior(3), bearing_range(2), point(2)");
            ros::shutdown();
            return;
        }

        const double TIGHT = 1e-9;  // Very tight constraint for unused dimensions
        
        odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << odometry_noise[0], odometry_noise[1], TIGHT,
                                 TIGHT, TIGHT, odometry_noise[2]).finished());
        priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << prior_noise[0], prior_noise[1], TIGHT,
                                 TIGHT, TIGHT, prior_noise[2]).finished());
        brNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << bearing_range_noise[0], TIGHT, bearing_range_noise[1]).finished());
        pointNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << point_noise[0], point_noise[1], TIGHT).finished());

        ROS_INFO("2D noise models initialized (z/roll/pitch constrained)");
    }

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
    // Index to keep track of the sequential pose
    index_of_pose = 1;
    // Initialize the factor graphs
    keyframeGraph_ = gtsam::NonlinearFactorGraph();

    // Initialize camera subscribers
    for (const auto& cam : camera_infos_) {
        ros::Subscriber sub = nh_.subscribe<apriltag_ros::AprilTagDetectionArray>(
            cam.topic, 1, boost::bind(&aprilslamcpp::cameraCallback, this, _1, cam.name));
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
    
    ROS_INFO("AprilSLAM initialized in %s mode", is_3d_mode_ ? "3D" : "2D");
}

// Destructor implementation
aprilslamcpp::~aprilslamcpp() {
    ROS_INFO("Node is shutting down. Executing SAMOptimise().");

    // Extract unoptimized landmarks before optimization
    std::map<int, gtsam::Point3> landmarks_unoptimised;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point3 point = keyframeEstimates_.at<gtsam::Point3>(key);  // Access the Point3 value
            landmarks_unoptimised[gtsam::Symbol(key).index()] = point;
        }
    }

    // Save unoptimized landmarks if required
    if (savetaglocation) {
        saveLandmarksToCSV(landmarks_unoptimised, pathtoloadlandmarkcsv);
    }
    
    // Perform batch optimization
    gtsam::Values result = SAMOptimise();
    keyframeEstimates_ = result;
    
    // Extract landmark estimates from the result
    std::map<int, gtsam::Point3> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point3 point = keyframeEstimates_.at<gtsam::Point3>(key);  // Access the Point3 value
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }

    ROS_INFO("Optimization complete: %zu landmarks", landmarks.size());

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

// Callback function for Camera topic
void aprilslamcpp::cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg,
                                   const std::string& camera_name) {    
    if (!msg->detections.empty()) {
        camera_detections_[camera_name] = msg;
        received_camera_names_.insert(camera_name);
    } else {
        camera_detections_.erase(camera_name);
    }
}

// Get static transform between two frames
bool aprilslamcpp::getStaticTransform(const std::string& target_frame,
                                      const std::string& source_frame,
                                      tf2::Transform& out_tf) {
    try {
        geometry_msgs::TransformStamped transform_stamped =
            tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2.0));
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
    // Initialize graph parameters and store them in isam_
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(parameters);
}

// ============ Helper: Make pose planar (z=0, roll=0, pitch=0) for 2D mode ============
gtsam::Pose3 aprilslamcpp::makePlanar(const gtsam::Pose3& pose) {
    if (is_3d_mode_) return pose;  // In 3D mode, return as-is
    
    // In 2D mode, constrain to xy plane with only yaw rotation
    double x = pose.x();
    double y = pose.y();
    double yaw = pose.rotation().yaw();
    
    return gtsam::Pose3(gtsam::Rot3::Ypr(yaw, 0.0, 0.0), gtsam::Point3(x, y, 0.0));
}

// Make point planar (z=0) for 2D mode
gtsam::Point3 aprilslamcpp::makePlanar(const gtsam::Point3& point) {
    if (is_3d_mode_) return point;  // In 3D mode, return as-is
    return gtsam::Point3(point.x(), point.y(), 0.0);
}

// Translate odometry message to GTSAM Pose3
gtsam::Pose3 aprilslamcpp::translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    if (is_3d_mode_) {
        // Full 6DOF pose from quaternion
        gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
        gtsam::Point3 translation(x, y, z);
        return gtsam::Pose3(rotation, translation);
    } else {
        // 2D: Extract yaw only, constrain z=0, roll=0, pitch=0
        tf2::Quaternion tfQuat(qx, qy, qz, qw);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
        
        return gtsam::Pose3(gtsam::Rot3::Ypr(yaw, 0.0, 0.0), gtsam::Point3(x, y, 0.0));
    }
}

// Batch optimization using Levenberg-Marquardt
gtsam::Values aprilslamcpp::SAMOptimise() {    
    ROS_INFO("Starting batch optimization: %zu factors, %zu variables", 
             keyframeGraph_.size(), keyframeEstimates_.size());
    
    // Perform batch optimization using Levenberg-Marquardt optimizer
    gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    gtsam::Values result = batchOptimizer.optimize();
    
    ROS_INFO("Optimization complete. Final error: %.6f", keyframeGraph_.error(result));
    return result;
}

// Check if movement exceeds the stationary thresholds
bool aprilslam::aprilslamcpp::movementExceedsThreshold(const gtsam::Pose3& poseSE3) {
    gtsam::Point3 currentPos = poseSE3.translation();
    gtsam::Point3 lastPos = lastPoseSE3_.translation();
    
    // Calculate position change
    double position_change;
    if (is_3d_mode_) {
        // 3D: Use full 3D distance
        position_change = (currentPos - lastPos).norm();
    } else {
        // 2D: Use only xy distance
        position_change = std::hypot(currentPos.x() - lastPos.x(), currentPos.y() - lastPos.y());
    }
    
    // Calculate rotation change using axis-angle representation
    gtsam::Rot3 rotationDelta = lastPoseSE3_.rotation().between(poseSE3.rotation());
    double rotation_change = rotationDelta.axisAngle().second;  // Get angle magnitude
    
    return position_change >= stationary_position_threshold || 
           rotation_change >= stationary_rotation_threshold;
}

// Handle initialization of the first pose
void aprilslam::aprilslamcpp::initializeFirstPose(const gtsam::Pose3& poseSE3, gtsam::Pose3& pose0) {
    lastPoseSE3_ = makePlanar(poseSE3);
    lastPoseSE3_vis = lastPoseSE3_;
    
    // Add prior factor at origin for the first pose
    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('X', 1), pose0, priorNoise));
    keyframeEstimates_.insert(gtsam::Symbol('X', 1), pose0);
    Estimates_visulisation.insert(gtsam::Symbol('X', 1), pose0);
    lastPose_ = pose0;  // Keep track of the last pose for odometry calculation
    
    // Load calibrated landmarks as priors if available
    if (usepriortagtable) {
        ROS_INFO("Loading landmarks from: %s", pathtoloadlandmarkcsv.c_str());
        std::map<int, gtsam::Point3> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
        ROS_INFO("Loaded %zu landmarks as priors", savedLandmarks.size());
        
        for (const auto& landmark : savedLandmarks) {
            gtsam::Symbol landmarkKey('L', landmark.first);
            gtsam::Point3 pt = makePlanar(landmark.second);  // Ensure z=0 in 2D mode
            
            // Add prior factor for each pre-calibrated landmark
            keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point3>(landmarkKey, pt, pointNoise));
            keyframeEstimates_.insert(landmarkKey, pt);
            landmarkEstimates.insert(landmarkKey, pt);
        }
    }
    
    Key_previous_pos = pose0;
    previousKeyframeSymbol = gtsam::Symbol('X', 1);
    ROS_INFO("First pose initialized at origin");
}

// Predict the next pose based on odometry
gtsam::Pose3 aprilslam::aprilslamcpp::predictNextPose(const gtsam::Pose3& poseSE3) {
    // Compute relative odometry between last and current pose
    gtsam::Pose3 odometry = lastPoseSE3_.between(poseSE3);
    // Compose with last optimized pose to get predicted pose
    gtsam::Pose3 predicted = lastPose_.compose(odometry);
    return makePlanar(predicted);  // Ensure planarity in 2D mode
}

// Update the graph with landmark detections
std::set<gtsam::Symbol> aprilslam::aprilslamcpp::updateGraphWithLandmarks(
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos, 
    const std::pair<std::vector<int>, std::vector<Eigen::Vector3d>>& detections) {

    // Access the elements of the std::pair   
    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector3d>& tagPos = detections.second;

    if (!Id.empty()) {
        for (size_t n = 0; n < Id.size(); ++n) {
            int tag_number = Id[n];        
            Eigen::Vector3d landSE3 = tagPos[n];  // Landmark position in robot frame (ORIGINAL 3D from camera!)

            // ============ CRITICAL: Compute prior location of the landmark in world frame ============
            // Transform landmark from robot frame to world frame using current robot pose
            gtsam::Point3 landmarkInRobotFrame(landSE3(0), landSE3(1), landSE3(2));
            gtsam::Point3 priorLand = lastPose_.transformFrom(landmarkInRobotFrame);
            
            // In 2D mode, constrain initial estimate to z=0
            priorLand = makePlanar(priorLand);

            // ============ Compute bearing and range ============
            // CRITICAL FOR 2D MODE: Must project bearing to xy plane!
            // Using 3D bearing in 2D mode causes landmarks to be pulled toward robot trajectory
            // because the 3D bearing direction conflicts with the z=0 constraint on landmarks.
            gtsam::Unit3 bearing;
            double range;
            
            if (is_3d_mode_) {
                // 3D: Use full 3D bearing and range
                Eigen::Vector3d landSE3_normalized = landSE3.normalized();
                bearing = gtsam::Unit3(landSE3_normalized);
                range = landSE3.norm();
            } else {
                // 2D: Project bearing to xy plane, ignore elevation!
                // Only use xy components for bearing direction
                Eigen::Vector3d bearing_2d(landSE3(0), landSE3(1), 0.0);
                bearing_2d.normalize();
                bearing = gtsam::Unit3(bearing_2d);  // Horizontal bearing only
                // Use xy distance for range in 2D mode
                range = std::sqrt(landSE3(0) * landSE3(0) + landSE3(1) * landSE3(1));
            }
            
            // Construct the landmark key
            gtsam::Symbol landmarkKey('L', tag_number);  

            // Check if the landmark has been observed before
            if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                // Existing landmark - add bearing-range measurement with error checking
                gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, bearing, range, brNoise);
                
                gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);
                
                // Threshold for ||projection - measurement||
                if (error.norm() < add2graph_threshold) {
                    keyframeGraph_.add(factor);
                }
            } else {
                // New landmark detected
                // If the current landmark was not detected in the calibration run 
                // Or it's on calibration mode
                if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                    detectedLandmarksHistoric.insert(landmarkKey);
                    
                    // Check if the key already exists in keyframeEstimates_ before inserting
                    if (!keyframeEstimates_.exists(landmarkKey)) {
                        keyframeEstimates_.insert(landmarkKey, priorLand);  // Simple initial estimate
                    }

                    // Check if the key already exists in landmarkEstimates before inserting
                    if (!landmarkEstimates.exists(landmarkKey)) {
                        landmarkEstimates.insert(landmarkKey, priorLand);
                    }

                    // Add a prior for the landmark position to help with initial estimation
                    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point3>(landmarkKey, priorLand, pointNoise));
                    
                    ROS_INFO("New landmark L%d at (%.2f, %.2f, %.2f)", 
                            tag_number, priorLand.x(), priorLand.y(), priorLand.z());
                }
                
                // Add a bearing-range observation for this landmark to the graph
                gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, bearing, range, brNoise);
                keyframeGraph_.add(factor);
            }
            
            // Store the bearing and range measurements in the map (for visualization)
            if (is_3d_mode_) {
                double bearing_angle = std::atan2(landSE3(1), landSE3(0));
                double elevation = std::atan2(landSE3(2), std::sqrt(landSE3(0)*landSE3(0) + landSE3(1)*landSE3(1)));
                poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = 
                    std::make_tuple(bearing_angle, elevation, range);
            } else {
                double bearing_angle = std::atan2(landSE3(1), landSE3(0));
                poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = 
                    std::make_tuple(bearing_angle, 0.0, range);  // elevation = 0 in 2D
            }
        }
    }
    return detectedLandmarksCurrentPos;
}

// Main odometry callback - adds odometry factors and processes landmarks
void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    // Convert the incoming odometry message to Pose3 format
    gtsam::Pose3 poseSE3 = translateOdomMsg(msg);  // Already planar in 2D mode
    
    // Store the initial pose for relative calculations
    pose0 = gtsam::Pose3();  // Prior at origin

    // Check if the movement exceeds the thresholds
    if (!movementExceedsThreshold(poseSE3)) return;

    index_of_pose += 1;  // Increment the pose index for each new odometry message
    
    if (index_of_pose == 2) {
        initializeFirstPose(poseSE3, pose0);
    }

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose3 predictedPose = predictNextPose(poseSE3);

    // Determine if this pose should be a keyframe
    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);

    // Add odometry factor (between factor)
    keyframeEstimates_.insert(currentKeyframeSymbol, predictedPose);
    
    if (previousKeyframeSymbol) {
        gtsam::Pose3 relativePose = Key_previous_pos.between(predictedPose);
        keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
    }
        
    // Update the last pose and initial estimates for the next iteration
    lastPose_ = predictedPose;
    landmarkEstimates.insert(currentKeyframeSymbol, predictedPose);

    std::set<gtsam::Symbol> detectedLandmarksCurrentPos;
    
    // Iterate through all landmark detected IDs
    // CRITICAL: Pass is_3d_mode_ so processDetections uses correct coordinate transformation
    auto detections = getCamDetections(camera_infos_, camera_detections_, is_3d_mode_);
    if (!detections.first.empty()) {
        detectedLandmarksCurrentPos = updateGraphWithLandmarks(detectedLandmarksCurrentPos, detections);
    }
    
    lastPoseSE3_ = poseSE3;
    Key_previous_pos = predictedPose;

    // Visualization
    previousKeyframeSymbol = currentKeyframeSymbol;
    
    // Extract landmark estimates from the result
    std::map<int, gtsam::Point3> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;  // Get the key
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point3 point = keyframeEstimates_.at<gtsam::Point3>(key);  // Access the Point3 value
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }
    
    // Publish the pose and landmarks
    aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);
    aprilslam::publishPath(path_pub_, keyframeEstimates_, index_of_pose, map_frame_id);

    // Save the landmarks into a CSV file if required (periodically to avoid overhead)
    if (savetaglocation && index_of_pose % 10 == 0) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
    
    // Periodic status update
    if (index_of_pose % 50 == 0) {
        ROS_INFO("Processed %d poses, tracking %zu landmarks", index_of_pose, landmarks.size());
    }
}
}

int main(int argc, char **argv) {
    // Initialize the ROS system and specify the name of the node
    ros::init(argc, argv, "april_slam_unified");

    // Create a handle to this process' node
    ros::NodeHandle nh;

    // Create an instance of the aprilslamcpp class, passing in the node handle
    aprilslam::aprilslamcpp slamNode(nh);

    // ROS enters a loop, pumping callbacks. Internally, it will call all the callbacks waiting to be called at that point in time.
    ros::spin();

    return 0;
}