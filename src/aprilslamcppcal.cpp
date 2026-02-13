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

    // ============ Noise models are now 6DOF/3DOF ============
    std::vector<double> odometry_noise, prior_noise, bearing_range_noise, point_noise;
    nh_.getParam("noise_models/odometry", odometry_noise);      // Expects 6 values
    nh_.getParam("noise_models/prior", prior_noise);            // Expects 6 values
    nh_.getParam("noise_models/bearing_range", bearing_range_noise);  // Expects 3 values
    nh_.getParam("noise_models/point", point_noise);            // Expects 3 values

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
    
    // Read geo-constraints between landmarks
    nh_.getParam("use_pole_constraints", use_pole_constraints_);

    if (use_pole_constraints_) {
        std::string pole_constraints_file;
        nh_.getParam("pole_constraints_file", pole_constraints_file);
        pole_constraints_path_ = package_path + "/" + pole_constraints_file;
        
        // Load the constraints from CSV
        pole_constraints_ = loadPoleConstraints(pole_constraints_path_);
        ROS_INFO("Loaded %zu pole constraints from: %s", 
                pole_constraints_.size(), pole_constraints_path_.c_str());
        
        // Load pole constraint noise model
        double pole_constraint_sigma;
        nh_.getParam("pole_constraint_noise", pole_constraint_sigma);
        poleConstraintNoise = gtsam::noiseModel::Isotropic::Sigma(1, pole_constraint_sigma);
        
        ROS_INFO("Pole constraint noise: %.4f m", pole_constraint_sigma);
        
        // Print some statistics
        if (!pole_constraints_.empty()) {
            double min_dist = pole_constraints_[0].distance;
            double max_dist = pole_constraints_[0].distance;
            double sum_dist = 0.0;
            
            for (const auto& pc : pole_constraints_) {
                min_dist = std::min(min_dist, pc.distance);
                max_dist = std::max(max_dist, pc.distance);
                sum_dist += pc.distance;
            }
            
            ROS_INFO("Pole constraint distances: min=%.4f, max=%.4f, avg=%.4f m",
                    min_dist, max_dist, sum_dist / pole_constraints_.size());
        }
    } else {
        ROS_INFO("Pole constraints disabled");
    }

    // ============ Camera info with full Pose3 transforms ============
    if (nh_.getParam("camera_config/cameras", camera_list) && 
        camera_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < camera_list.size(); ++i) {
            if (camera_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

            std::string name = static_cast<std::string>(camera_list[i]["name"]);
            std::string topic = static_cast<std::string>(camera_list[i]["topic"]);
            std::string frame_id = static_cast<std::string>(camera_list[i]["frame"]);

            gtsam::Pose3 transform = gtsam::Pose3();  // Initialize as identity

            camera_infos_.emplace_back(CameraInfo{name, topic, frame_id, transform});
        }
    } else {
        ROS_WARN("Failed to load camera_config/cameras or invalid format.");
    }

    // ============ Wait for static transforms and convert to Pose3 ============
    for (auto& cam : camera_infos_) {
        tf2::Transform tf;
        const int max_attempts = 20;
        const ros::Duration retry_interval(0.5);
        bool success = false;

        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            if (getStaticTransform(robot_frame, cam.frame_id, tf)) {
                
                tf2::Vector3 trans = tf.getOrigin();
                tf2::Quaternion rot = tf.getRotation();

                // Convert to GTSAM Pose3 (full 6DOF transform)
                gtsam::Point3 translation(trans.x(), trans.y(), trans.z());
                gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(rot.w(), rot.x(), rot.y(), rot.z());
                cam.transform = gtsam::Pose3(rotation, translation);
                
                ROS_INFO("TF loaded for [%s] (%s): xyz(%.2f, %.2f, %.2f), rpy(%.2f, %.2f, %.2f)",
                        cam.name.c_str(), cam.frame_id.c_str(), 
                        trans.x(), trans.y(), trans.z(),
                        rotation.roll(), rotation.pitch(), rotation.yaw());
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
    // Validate noise model sizes
    if (odometry_noise.size() != 6) {
        ROS_ERROR("Odometry noise must have 6 values (x,y,z,roll,pitch,yaw). Got %zu", odometry_noise.size());
        ros::shutdown();
        return;
    }
    if (prior_noise.size() != 6) {
        ROS_ERROR("Prior noise must have 6 values (x,y,z,roll,pitch,yaw). Got %zu", prior_noise.size());
        ros::shutdown();
        return;
    }
    if (bearing_range_noise.size() != 3) {
        ROS_ERROR("Bearing-range noise must have 3 values for BearingRangeFactor<Pose3,Point3>. Got %zu", 
                  bearing_range_noise.size());
        ros::shutdown();
        return;
    }
    if (point_noise.size() != 3) {
        ROS_ERROR("Point noise must have 3 values (x,y,z). Got %zu", point_noise.size());
        ros::shutdown();
        return;
    }

    // Odometry noise: 6 DOF (x, y, z, roll, pitch, yaw)
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << odometry_noise[0], odometry_noise[1], odometry_noise[2],
                             odometry_noise[3], odometry_noise[4], odometry_noise[5]).finished());
    
    // Prior noise: 6 DOF (x, y, z, roll, pitch, yaw)
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << prior_noise[0], prior_noise[1], prior_noise[2],
                             prior_noise[3], prior_noise[4], prior_noise[5]).finished());
    
    // Bearing-Range noise: 3 DOF for BearingRangeFactor<Pose3, Point3>
    // Dimension 1-2: Unit3 bearing on tangent space (like azimuth and elevation)
    // Dimension 3: Range
    brNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << bearing_range_noise[0], bearing_range_noise[1], 
                             bearing_range_noise[2]).finished());
    
    // Point noise: 3 DOF (x, y, z)
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << point_noise[0], point_noise[1], point_noise[2]).finished());

    ROS_INFO("Noise models initialized: Odometry(6D), Prior(6D), BearingRange(3D), Point(3D)");

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

    // Survey landmark settings
    nh_.getParam("use_survey_landmarks", use_survey_landmarks_);

    if (use_survey_landmarks_) {
        nh_.getParam("path_to_survey_landmarks", survey_landmarks_path_);
        survey_landmarks_path_ = package_path + "/" + survey_landmarks_path_;
        
        nh_.getParam("survey_landmark_noise", survey_noise_sigma);
        surveyLandmarkNoise = gtsam::noiseModel::Isotropic::Sigma(3, survey_noise_sigma);
        
        ROS_INFO("Survey landmark noise: %.4f m", survey_noise_sigma);
    }

    // Initialize GTSAM components
    initializeGTSAM();
    // Index to keep track of the sequential pose
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
            received_camera_names_.clear();
        } else {
            accumulated_time_ += 2.0;
            ROS_WARN("No new valid data received from any camera. Accumulated time: %.1f seconds", accumulated_time_);

            if (accumulated_time_ >= inactivity_threshold) {
                ROS_ERROR("No valid data from any camera for %.1f seconds. Shutting down.", inactivity_threshold);
                this->~aprilslamcpp();
            }
        }
    });
    
    ROS_INFO("3D AprilSLAM calibration node initialized successfully.");
}

// ============ Destructor ============
aprilslamcpp::~aprilslamcpp() {
    ROS_INFO("Node is shutting down. Executing SAMOptimise().");

    // Extract unoptimized landmarks (Point3)
    std::map<int, gtsam::Point3> landmarks_unoptimised;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point3 point = keyframeEstimates_.at<gtsam::Point3>(key);
            landmarks_unoptimised[gtsam::Symbol(key).index()] = point;
        }
    }

    if (savetaglocation) {
        ROS_INFO("Saving unoptimized landmarks...");
        saveLandmarksToCSV(landmarks_unoptimised, pathtoloadlandmarkcsv);
    }
    
    // Add pole constraints if enabled
    if (use_pole_constraints_) {
        ROS_INFO("Adding pole constraints before final optimization...");
        addPoleConstraintFactors();
        ROS_INFO("Added %zu/%zu pole constraints to the graph", 
                 pole_constraints_added_.size(), pole_constraints_.size());
    }

    // Perform final optimization
    ROS_INFO("Running final batch optimization...");
    gtsam::Values result = SAMOptimise();
    keyframeEstimates_ = result;
    
    // Extract optimized landmark estimates (Point3)
    std::map<int, gtsam::Point3> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point3 point = keyframeEstimates_.at<gtsam::Point3>(key);
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }

    ROS_INFO("Optimized %zu landmarks", landmarks.size());

    // Publish the pose and landmarks
    aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);
    aprilslam::publishPath(path_pub_, keyframeEstimates_, index_of_pose, map_frame_id);

    // Save the optimized landmarks to CSV
    if (savetaglocation) {
        ROS_INFO("Saving optimized landmarks to: %s", pathtosavelandmarkcsv.c_str());
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
    
    optimizationExecuted_ = true;
    ROS_INFO("SAMOptimise() executed successfully. Shutdown complete.");
}

// =============== loading geo-constraint from a given csv with header: ===============
// constraint_id	tag_id_1	tag_id_2	distance
std::vector<aprilslamcpp::PoleConstraint> aprilslamcpp::loadPoleConstraints(const std::string& filepath) {
    
    std::vector<PoleConstraint> constraints;
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        ROS_ERROR("Failed to open pole constraints file: %s", filepath.c_str());
        return constraints;
    }
    
    std::string line;
    // Skip header line
    std::getline(file, line);
    
    int line_number = 1;
    while (std::getline(file, line)) {
        line_number++;
        
        // Skip empty lines
        if (line.empty()) continue;
        
        std::stringstream ss(line);
        std::string token;
        PoleConstraint pc;
        
        try {
            // Parse: constraint_id,tag_id_1,tag_id_2,distance
            std::getline(ss, token, ',');
            pc.constraint_id = std::stoi(token);
            
            std::getline(ss, token, ',');
            pc.tag_id_1 = std::stoi(token);
            
            std::getline(ss, token, ',');
            pc.tag_id_2 = std::stoi(token);
            
            std::getline(ss, token, ',');
            pc.distance = std::stod(token);
            
            constraints.push_back(pc);
            
        } catch (const std::exception& e) {
            ROS_WARN("Failed to parse line %d in pole constraints file: %s", 
                     line_number, e.what());
            continue;
        }
    }
    
    file.close();
    ROS_INFO("Successfully parsed %zu pole constraints from CSV", constraints.size());
    return constraints;
}

// =============== loading total station survey landmarks from a given csv: ===============
// Headear: "Point Name","X (East)","Y (North)","Z (Elevation)"
std::map<int, gtsam::Point3> aprilslamcpp::loadSurveyLandmarks(
    const std::string& filepath, 
    const std::string& prefix) {
    
    std::map<int, gtsam::Point3> landmarks;
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        ROS_ERROR("Failed to open survey landmarks file: %s", filepath.c_str());
        return landmarks;
    }
    
    std::string line;
    // Skip header line
    if (!std::getline(file, line)) {
        ROS_ERROR("Empty file or cannot read header");
        return landmarks;
    }
    
    ROS_INFO("Header: %s", line.c_str());
    
    int line_number = 1;
    while (std::getline(file, line)) {
        line_number++;
        
        // Remove any trailing whitespace/carriage returns
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        if (line.empty()) continue;
        
        // Split by commas (CSV format)
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string token;
        
        while (std::getline(ss, token, ',')) {  // <-- Changed from '\t' to ','
            // Remove quotes and trim whitespace
            token.erase(std::remove(token.begin(), token.end(), '\"'), token.end());  // <-- Remove quotes
            token.erase(0, token.find_first_not_of(" \t\r\n"));
            token.erase(token.find_last_not_of(" \t\r\n") + 1);
            tokens.push_back(token);
        }
        
        if (tokens.size() < 4) {
            ROS_WARN("Line %d has only %zu tokens (expected 4)", line_number, tokens.size());
            continue;
        }
        
        std::string point_name = tokens[0];
        
        try {
            // Check prefix
            if (point_name.length() < prefix.length() || 
                point_name.substr(0, prefix.length()) != prefix) {
                ROS_DEBUG("Skipping '%s' (doesn't match prefix '%s')", 
                         point_name.c_str(), prefix.c_str());
                continue;
            }
            
            // Extract tag number
            std::string number_str = point_name.substr(prefix.length());
            int tag_id = std::stoi(number_str);
            
            // Parse coordinates
            double x = std::stod(tokens[1]);
            double y = std::stod(tokens[2]);
            double z = std::stod(tokens[3]);
            
            gtsam::Point3 point(x, y, z);
            landmarks[tag_id] = point;
            
            ROS_INFO("Loaded survey landmark L%d (%s): (%.3f, %.3f, %.3f)", 
                     tag_id, point_name.c_str(), x, y, z);
            
        } catch (const std::exception& e) {
            ROS_WARN("Failed to parse line %d: %s. Tokens: [%s, %s, %s, %s]", 
                     line_number, e.what(), 
                     tokens.size() > 0 ? tokens[0].c_str() : "",
                     tokens.size() > 1 ? tokens[1].c_str() : "",
                     tokens.size() > 2 ? tokens[2].c_str() : "",
                     tokens.size() > 3 ? tokens[3].c_str() : "");
            continue;
        }
    }
    
    file.close();
    ROS_INFO("Successfully loaded %zu survey landmarks with prefix '%s'", 
             landmarks.size(), prefix.c_str());
    return landmarks;
}

// ============ Add pole constraint factors to the graph ============
// For mapping, run it once before optimisation
void aprilslam::aprilslamcpp::addPoleConstraintFactors() {
    if (!use_pole_constraints_) return;
    
    int constraints_added_this_call = 0;
    
    for (const auto& pc : pole_constraints_) {
        // Check if we've already added this constraint
        if (pole_constraints_added_.find(pc.constraint_id) != pole_constraints_added_.end()) {
            continue;
        }
        
        gtsam::Symbol L1('L', pc.tag_id_1);
        gtsam::Symbol L2('L', pc.tag_id_2);
        
        // Only add constraint if both landmarks exist in the estimates
        if (keyframeEstimates_.exists(L1) && keyframeEstimates_.exists(L2)) {
            
            // Add RangeFactor to constrain the distance between the two landmarks
            gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> rangeFactor(
                L1, L2, pc.distance, poleConstraintNoise
            );
            
            keyframeGraph_.add(rangeFactor);
            pole_constraints_added_.insert(pc.constraint_id);
            constraints_added_this_call++;
            
            ROS_DEBUG("Added pole constraint %d: L%d <-> L%d (distance: %.4f m)",
                     pc.constraint_id, pc.tag_id_1, pc.tag_id_2, pc.distance);
        }
    }
    
    if (constraints_added_this_call > 0) {
        ROS_INFO("Added %d new pole constraints (total: %zu/%zu)",
                 constraints_added_this_call, 
                 pole_constraints_added_.size(),
                 pole_constraints_.size());
    }
}

// Callback function for camera topics
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
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(parameters);
}

// ============ Convert odometry message to Pose3 ============
gtsam::Pose3 aprilslamcpp::translateOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
    gtsam::Point3 translation(x, y, z);
    
    return gtsam::Pose3(rotation, translation);
}

gtsam::Values aprilslamcpp::SAMOptimise() {    
    ROS_INFO("Performing batch optimization with %zu factors and %zu variables...",
             keyframeGraph_.size(), keyframeEstimates_.size());
    
    gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    gtsam::Values result = batchOptimizer.optimize();
    
    ROS_INFO("Optimization converged. Final error: %.6f", keyframeGraph_.error(result));
    return result;
}

// ============ Check movement threshold ============
bool aprilslam::aprilslamcpp::movementExceedsThreshold(const gtsam::Pose3& poseSE3) {
    gtsam::Point3 currentPos = poseSE3.translation();
    gtsam::Point3 lastPos = lastPoseSE3_.translation();
    
    double position_change = (currentPos - lastPos).norm();
    
    // Calculate rotation change using axis-angle representation
    gtsam::Rot3 rotationDelta = lastPoseSE3_.rotation().between(poseSE3.rotation());
    double rotation_change = rotationDelta.axisAngle().second;  // Angle magnitude
    
    return position_change >= stationary_position_threshold || 
           rotation_change >= stationary_rotation_threshold;
}

// ============ Initialize first pose ============
void aprilslam::aprilslamcpp::initializeFirstPose(const gtsam::Pose3& poseSE3, gtsam::Pose3& pose0) {
    lastPoseSE3_ = poseSE3;
    lastPoseSE3_vis = poseSE3;
    
    // Add prior factor for the first pose
    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('X', 1), pose0, priorNoise));
    keyframeEstimates_.insert(gtsam::Symbol('X', 1), pose0);
    Estimates_visulisation.insert(gtsam::Symbol('X', 1), pose0);
    lastPose_ = pose0;
    
    // Load survey-grade landmarks FIRST (highest priority)
    if (use_survey_landmarks_) {
        ROS_INFO("Loading survey-grade landmarks from: %s", survey_landmarks_path_.c_str());
        std::string prefix;
        nh_.getParam("survey_landmark_prefix", prefix);
        
        std::map<int, gtsam::Point3> surveyLandmarks = loadSurveyLandmarks(survey_landmarks_path_, prefix);
        ROS_INFO("Loaded %zu survey-grade landmarks", surveyLandmarks.size());
        
        for (const auto& landmark : surveyLandmarks) {
            gtsam::Symbol landmarkKey('L', landmark.first);
            
            // Add VERY tight prior for survey landmarks
            keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point3>(landmarkKey, landmark.second, surveyLandmarkNoise));
            keyframeEstimates_.insert(landmarkKey, landmark.second);
            landmarkEstimates.insert(landmarkKey, landmark.second);
            
            // Mark as historically detected so we add bearing-range factors
            detectedLandmarksHistoric.insert(landmarkKey);
            
            ROS_INFO("Added survey prior for L%d at (%.3f, %.3f, %.3f)", 
                     landmark.first, landmark.second.x(), 
                     landmark.second.y(), landmark.second.z());
        }
    }
    
    // Load calibrated landmarks as priors if available
    if (usepriortagtable) {
        ROS_INFO("Loading prior landmark table from: %s", pathtoloadlandmarkcsv.c_str());
        std::map<int, gtsam::Point3> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
        ROS_INFO("Loaded %zu prior landmarks", savedLandmarks.size());
        
        for (const auto& landmark : savedLandmarks) {
            gtsam::Symbol landmarkKey('L', landmark.first);
            keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point3>(landmarkKey, landmark.second, pointNoise));
            keyframeEstimates_.insert(landmarkKey, landmark.second);
            landmarkEstimates.insert(landmarkKey, landmark.second);
            
            ROS_DEBUG("Added prior for landmark L%d at (%.2f, %.2f, %.2f)", 
                     landmark.first, landmark.second.x(), landmark.second.y(), landmark.second.z());
        }
    }
    
    Key_previous_pos = pose0;
    previousKeyframeSymbol = gtsam::Symbol('X', 1);
    
    ROS_INFO("First pose initialized at origin");
}

// ============ Predict next pose (can be improved with dynamics model) ============
gtsam::Pose3 aprilslam::aprilslamcpp::predictNextPose(const gtsam::Pose3& poseSE3) {
    gtsam::Pose3 odometry = lastPoseSE3_.between(poseSE3);
    return lastPose_.compose(odometry);
}

// ============ Update graph with landmarks ============
std::set<gtsam::Symbol> aprilslam::aprilslamcpp::updateGraphWithLandmarks(
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos, 
    const std::pair<std::vector<int>, std::vector<Eigen::Vector3d>>& detections) {

    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector3d>& tagPos = detections.second;

    if (!Id.empty()) {
        for (size_t n = 0; n < Id.size(); ++n) {
            int tag_number = Id[n];        
            Eigen::Vector3d landSE3 = tagPos[n];

            // Compute prior location of the landmark using current robot pose
            gtsam::Point3 landmarkInRobotFrame(landSE3(0), landSE3(1), landSE3(2));
            gtsam::Point3 priorLand = lastPose_.transformFrom(landmarkInRobotFrame);

            // Compute range for 3D measurement
            double range = landSE3.norm();
            
            // Construct the landmark key
            gtsam::Symbol landmarkKey('L', tag_number);  

            // Check if the landmark has been observed before
            if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                // Existing landmark - add bearing-range factor
                Eigen::Vector3d landSE3_normalized = landSE3.normalized();
                gtsam::Unit3 bearing(landSE3_normalized);

                gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3> factor(
                    gtsam::Symbol('X', index_of_pose), 
                    landmarkKey, 
                    bearing,  // Bearing as unit vector
                    range,                   // Range
                    brNoise                  // 3D noise model
                );
                
                gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                // Only add if error is within threshold
                if (error.norm() < add2graph_threshold) {
                    keyframeGraph_.add(factor);
                    ROS_DEBUG("Added factor for existing landmark L%d (error: %.3f)", tag_number, error.norm());
                }
            } 
            else {
                // New landmark detected
                if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                    detectedLandmarksHistoric.insert(landmarkKey);
                    
                    // Add initial estimate if not exists
                    if (!keyframeEstimates_.exists(landmarkKey)) {
                        keyframeEstimates_.insert(landmarkKey, priorLand);
                    }

                    if (!landmarkEstimates.exists(landmarkKey)) {
                        landmarkEstimates.insert(landmarkKey, priorLand);
                    }

                    // Add a prior for the landmark position
                    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point3>(
                        landmarkKey, priorLand, pointNoise)
                    );
                    
                    ROS_INFO("New landmark L%d detected at (%.2f, %.2f, %.2f)", 
                            tag_number, priorLand.x(), priorLand.y(), priorLand.z());
                }
                
                // Add bearing-range observation
                Eigen::Vector3d landSE3_normalized = landSE3.normalized();
                gtsam::Unit3 bearing(landSE3_normalized);

                gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3> factor(
                    gtsam::Symbol('X', index_of_pose), 
                    landmarkKey,
                    bearing, 
                    range, 
                    brNoise
                );
                keyframeGraph_.add(factor);
            }
            
            // Store measurements for later use (bearing, elevation, range)
            double bearing = std::atan2(landSE3(1), landSE3(0));
            double elevation = std::atan2(landSE3(2), std::sqrt(landSE3(0)*landSE3(0) + landSE3(1)*landSE3(1)));
            poseToLandmarkMeasurementsMap[gtsam::Symbol('X', index_of_pose)][landmarkKey] = 
                std::make_tuple(bearing, elevation, range);
        }
    }
    return detectedLandmarksCurrentPos;
}

// ============ Main odometry callback ============
void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    // Convert odometry message to Pose3
    gtsam::Pose3 poseSE3 = translateOdomMsg(msg);

    // Store the initial pose at origin
    pose0 = gtsam::Pose3(); // Identity pose

    // Check if movement exceeds thresholds
    if (!movementExceedsThreshold(poseSE3)) return;

    index_of_pose += 1;
    
    // Initialize first pose
    if (index_of_pose == 2) {
        initializeFirstPose(poseSE3, pose0);
    }

    // Predict the next pose based on odometry
    gtsam::Pose3 predictedPose = predictNextPose(poseSE3);

    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);

    // Add odometry factor
    keyframeEstimates_.insert(gtsam::Symbol('X', index_of_pose), predictedPose);
    
    if (previousKeyframeSymbol) {
        gtsam::Pose3 relativePose = Key_previous_pos.between(predictedPose);
        keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
    }
        
    // Update the last pose
    lastPose_ = predictedPose;
    landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), predictedPose);

    std::set<gtsam::Symbol> detectedLandmarksCurrentPos;
    
    // Process landmark detections
    auto detections = getCamDetections(camera_infos_, camera_detections_);
    if (!detections.first.empty()) {
        ROS_DEBUG("Processing %zu landmark detections at pose %d", detections.first.size(), index_of_pose);
        detectedLandmarksCurrentPos = updateGraphWithLandmarks(detectedLandmarksCurrentPos, detections);
    }
    
    lastPoseSE3_ = poseSE3;
    Key_previous_pos = predictedPose;
    previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);
    
    // Extract landmark estimates for visualization
    std::map<int, gtsam::Point3> landmarks;
    for (const auto& key_value : keyframeEstimates_) {
        gtsam::Key key = key_value.key;
        if (gtsam::Symbol(key).chr() == 'L') {
            gtsam::Point3 point = keyframeEstimates_.at<gtsam::Point3>(key);
            landmarks[gtsam::Symbol(key).index()] = point;
        }
    }
    
    // Publish visualization
    aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);
    aprilslam::publishPath(path_pub_, keyframeEstimates_, index_of_pose, map_frame_id);

    // Periodically save landmarks
    if (savetaglocation && index_of_pose % 10 == 0) {
        saveLandmarksToCSV(landmarks, pathtosavelandmarkcsv);
    }
    
    if (index_of_pose % 50 == 0) {
        ROS_INFO("Processed %d poses, tracking %zu landmarks", index_of_pose, landmarks.size());
    }
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "april_slam_cpp_3d_calibration");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting 3D AprilSLAM Calibration Node");
    
    aprilslam::aprilslamcpp slamNode(nh);
    
    ros::spin();
    
    return 0;
}