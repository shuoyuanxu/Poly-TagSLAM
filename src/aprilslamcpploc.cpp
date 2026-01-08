#include "aprilslamheader.h"
#include "publishing_utils.h"

namespace aprilslam {

gtsam::Pose3 odometryDirection(const gtsam::Pose3& odometry, double cmd_vel_linear_x){
    if (cmd_vel_linear_x == 0.0) {
        return odometry;
    }
    // For 3D, we only adjust the x translation based on direction
    gtsam::Point3 translation = odometry.translation();
    double dx = translation.x();
    
    if (cmd_vel_linear_x < 0) {
        dx = -std::abs(dx);  // Moving backward
    } else {
        dx = std::abs(dx);   // Moving forward
    }
    
    gtsam::Pose3 adjustedOdometry(odometry.rotation(), gtsam::Point3(dx, translation.y(), translation.z()));
    return adjustedOdometry;
}

// Constructor
aprilslamcpp::aprilslamcpp(ros::NodeHandle node_handle)
    : nh_(node_handle), tf_listener_(tf_buffer_){ 
    
    // Read topics and corresponding frame
    std::string odom_topic, trajectory_topic;
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("odom_frame", odom_frame);
    nh_.getParam("trajectory_topic", trajectory_topic);
    nh_.getParam("map_frame_id", map_frame_id);
    nh_.getParam("robot_frame", robot_frame);

    // Read batch optimization flag
    nh_.getParam("batch_optimisation", batchOptimisation_);

    std::vector<double> odometry_noise, prior_noise, bearing_range_noise, point_noise, loop_ClosureNoise;
    nh_.getParam("noise_models/odometry", odometry_noise);  
    nh_.getParam("noise_models/prior", prior_noise);         
    nh_.getParam("noise_models/bearing_range", bearing_range_noise);  
    nh_.getParam("noise_models/point", point_noise);         
    nh_.getParam("noise_models/loopClosureNoise", loop_ClosureNoise);  

    // Read error threshold for a landmark to be added to the graph
    nh_.getParam("add2graph_threshold", add2graph_threshold);

    // Read Prune conditions
    nh_.getParam("maxfactors", maxfactors);
    nh_.getParam("useprunebysize", useprunebysize);

    // Read initialisation conditions
    nh_.getParam("N_particles", N_particles);
    nh_.getParam("usePFinitialise", usePFinitialise);
    nh_.getParam("PFWaitTime", PFWaitTime);
    nh_.getParam("rngVar", rngVar_);
    nh_.getParam("brngVar", brngVar_);
    pfInitStartTime_ = 0.0;

    // Read loop closure parameters
    nh_.getParam("useloopclosure", useloopclosure);
    nh_.getParam("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
    nh_.getParam("historyKeyframeSearchNum", historyKeyframeSearchNum);
    nh_.getParam("requiredReobservedLandmarks", requiredReobservedLandmarks);

    // Keyframe parameters
    nh_.getParam("distanceThreshold", distanceThreshold);
    nh_.getParam("rotationThreshold", rotationThreshold);
    nh_.getParam("usekeyframe", usekeyframe);
    gtsam::Values landmarkEstimates;

    // Stationary conditions
    nh_.getParam("stationary_position_threshold", stationary_position_threshold);
    nh_.getParam("stationary_rotation_threshold", stationary_rotation_threshold);

    // Read calibration and localisation settings
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

            gtsam::Pose3 transform = gtsam::Pose3();  // Initialize as identity

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

                // Convert to GTSAM Pose3 - Full 6DOF transform for 3D mode
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
 
    // Load outlier removal conditions
    nh_.getParam("useoutlierremoval", useoutlierremoval); 
    nh_.getParam("jumpCombinedThreshold", jumpCombinedThreshold); 
    nh_.getParam("outlierRemovalStartIndex_", outlierRemovalStartIndex_);

    // Load trajectory smoothing conditions
    nh_.getParam("usetrajsmoothing", usetrajsmoothing); 
    nh_.getParam("smoothingwindow", smoothingwindow); 
    nh_.getParam("smoothingStartIndex_", smoothingStartIndex_);

    // Save localisation result
    refined_odom_csv.open("/home/shuoyuan/catkin_slam_ws/src/aprilslamcpp/refined_odometry.csv", std::ios::out);
    raw_odom_csv.open("/home/shuoyuan/catkin_slam_ws/src/aprilslamcpp/raw_odometry.csv", std::ios::out);
    refined_odom_csv << "time,x,y,z,roll,pitch,yaw\n";
    raw_odom_csv << "time,x,y,z,roll,pitch,yaw\n";

    // Load saveLandmarks
    savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);

    // Initialize noise models for 3D SLAM
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << odometry_noise[0], odometry_noise[1], odometry_noise[2],
                              odometry_noise[3], odometry_noise[4], odometry_noise[5]).finished());
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << prior_noise[0], prior_noise[1], prior_noise[2],
                              prior_noise[3], prior_noise[4], prior_noise[5]).finished());
    brNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << bearing_range_noise[0], bearing_range_noise[1], bearing_range_noise[2]).finished());
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << point_noise[0], point_noise[1], point_noise[2]).finished());
    loopClosureNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << loop_ClosureNoise[0], loop_ClosureNoise[1], loop_ClosureNoise[2],
                              loop_ClosureNoise[3], loop_ClosureNoise[4], loop_ClosureNoise[5]).finished());

    // Optimiser selection
    nh_.getParam("useisam2", useisam2);

    // Total number of IDs
    int total_tags;
    nh_.getParam("total_tags", total_tags);
    for (int j = 0; j < total_tags; ++j) {
        possibleIds_.push_back("tag_" + std::to_string(j));
    }
    
    ROS_INFO("Parameters loaded.");

    // Initialize GTSAM components
    initializeGTSAM();
    index_of_pose = 1;
    previousframeSymbol = index_of_pose;
    keyframeGraph_ = gtsam::NonlinearFactorGraph();

    // Initialize camera subscribers
    for (const auto& cam : camera_infos_) {
        ros::Subscriber sub = nh_.subscribe<apriltag_ros::AprilTagDetectionArray>(
            cam.topic, 1,
            boost::bind(&aprilslamcpp::cameraCallback, this, _1, cam.name)
        );
        camera_subscribers_.push_back(sub);
    }
    
    // Initialise pose0 using particle filter
    if (usePFinitialise) {
        pf_init_timer_ = nh_.createTimer(ros::Duration(0.5), &aprilslamcpp::pfInitCallback, this);
    } else {
        pose0 = gtsam::Pose3();  // Identity pose
    }
    
    // Subscriptions and Publications
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
    lc_pub_ = nh_.advertise<visualization_msgs::Marker>("loop_closure_markers", 1);
    landmark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
    path.header.frame_id = map_frame_id; 
    odom_traj_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom_tag", 1, true);
    
    ROS_INFO("AprilSLAM 3D initialized with full features (loop closure, PF init, etc.)");
}

double aprilslamcpp::computePoseDelta(const gtsam::Pose3& oldPose, const gtsam::Pose3& newPose){
    gtsam::Point3 oldPos = oldPose.translation();
    gtsam::Point3 newPos = newPose.translation();
    
    double dx = newPos.x() - oldPos.x();
    double dy = newPos.y() - oldPos.y();
    double dz = newPos.z() - oldPos.z();

    // Get the old heading (yaw)
    double oldYaw = oldPose.rotation().yaw();

    // Project the (dx,dy) onto the axis perpendicular to oldYaw
    double lat = dx * (-std::sin(oldYaw)) + dy * std::cos(oldYaw);

    // Also consider z displacement
    double totalLateral = std::sqrt(lat * lat + dz * dz);

    return std::fabs(totalLateral);
}

void aprilslamcpp::pfInitCallback(const ros::TimerEvent& event) {
    ROS_INFO("PF Running");
    
    if (pfInitialized_) {
        pf_init_timer_.stop();
        return;
    }

    // FIXED: Pass 'true' for 3D mode
    auto detections = getCamDetections(camera_infos_, camera_detections_, true);
    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector3d>& tagPos = detections.second;  
        
    std::vector<int> validIds;
    std::vector<Eigen::Vector3d> validTagPos;  
    
    for (size_t i = 0; i < Id.size(); ++i) {
        if (savedLandmarks.find(Id[i]) != savedLandmarks.end()) {
            validIds.push_back(Id[i]);
            validTagPos.push_back(tagPos[i]);
        } else {
            ROS_WARN("Skipping unknown tag ID: %d", Id[i]);
        }
    }
    
    if (validIds.empty()) {
        ROS_INFO("No tags detected, waiting for detections...");
        return;
    }

    ROS_INFO("Number of tags observed: %zu", validIds.size());

    double currentTime = ros::Time::now().toSec();

    if (!pfInitInProgress_) {
        pfInitInProgress_ = true;
        pfInitStartTime_ = currentTime;

        x_P_pf_ = initParticlesFromFirstTag(validIds, validTagPos, savedLandmarks, PFWaitTime);

        ROS_INFO("PF initialization started.");
    }

    double elapsed = currentTime - pfInitStartTime_;

    if (elapsed < PFWaitTime) {
        x_P_pf_ = particleFilter(validIds, validTagPos, savedLandmarks, x_P_pf_, PFWaitTime, rngVar_, brngVar_);
    } else {
        x_P_pf_ = particleFilter(validIds, validTagPos, savedLandmarks, x_P_pf_, PFWaitTime, rngVar_, brngVar_);

        Eigen::Matrix<double, 6, 1> sum_states = Eigen::Matrix<double, 6, 1>::Zero();
        for (const auto& particle : x_P_pf_) {
            sum_states += particle;
        }
        Eigen::Matrix<double, 6, 1> x_est_pf = sum_states / static_cast<double>(PFWaitTime);

        ROS_INFO("PF initialization result: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f", 
                 x_est_pf(0), x_est_pf(1), x_est_pf(2), x_est_pf(3), x_est_pf(4), x_est_pf(5));
        ROS_INFO("PLEASE DONT MOVE THE ROBOT!!!");

        std::string userInput;
        ROS_INFO("Are you satisfied with the PF initialization result? (yes/no): ");
        std::cin >> userInput;

        if (userInput == "yes") {
            // Create Pose3 from particle filter result
            gtsam::Rot3 init_rotation = gtsam::Rot3::RzRyRx(x_est_pf(3), x_est_pf(4), x_est_pf(5));
            gtsam::Point3 init_translation(x_est_pf(0), x_est_pf(1), x_est_pf(2));
            pose0 = gtsam::Pose3(init_rotation, init_translation);
            
            pfInitialized_ = true;
            pfInitInProgress_ = false;
            pf_init_timer_.stop();
            x_P_pf_.clear();

            ROS_INFO("PF initialization finalized successfully.");
        } else {
            pfInitInProgress_ = false;
            ROS_WARN("PF initialization rejected. Restarting initialization process.");
            x_P_pf_.clear();
        }
    }
}

void aprilslamcpp::cameraCallback(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg,
    const std::string& camera_name) {
    
    if (!msg->detections.empty()) {
        camera_detections_[camera_name] = msg;
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

void aprilslamcpp::smoothTrajectory(int window_size) {
    if (Estimates_visulisation.empty()) {
        return;
    }

    std::vector<std::pair<gtsam::Symbol, gtsam::Pose3>> xPoses;
    xPoses.reserve(Estimates_visulisation.size());

    for (const auto& key_value : Estimates_visulisation) {
        gtsam::Symbol key(key_value.key);
        if (key.chr() == 'X') {
            gtsam::Pose3 pose = key_value.value.cast<gtsam::Pose3>();
            xPoses.emplace_back(key, pose);
        }
    }

    if (xPoses.size() < static_cast<size_t>(window_size)) {
        return;
    }

    std::sort(xPoses.begin(), xPoses.end(),
              [](const auto& a, const auto& b) {
                  return a.first.index() < b.first.index();
              });

    // Sum up positions from the last window_size poses
    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    for (size_t i = xPoses.size() - window_size; i < xPoses.size(); ++i) {
        gtsam::Point3 trans = xPoses[i].second.translation();
        sumX += trans.x();
        sumY += trans.y();
        sumZ += trans.z();
    }

    double avgX = sumX / window_size;
    double avgY = sumY / window_size;
    double avgZ = sumZ / window_size;

    // Keep rotation as-is
    gtsam::Pose3 lastPose = xPoses.back().second;
    gtsam::Rot3 keepRotation = lastPose.rotation();

    gtsam::Pose3 smoothedPose(keepRotation, gtsam::Point3(avgX, avgY, avgZ));

    gtsam::Symbol lastKey = xPoses.back().first;
    if (Estimates_visulisation.exists(lastKey)) {
        Estimates_visulisation.update(lastKey, smoothedPose);
    }
}

void aprilslamcpp::initializeGTSAM() { 
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(parameters);
}

aprilslamcpp::~aprilslamcpp() {
    ROS_INFO("Shutting down aprilslamcpp.");
}

bool aprilslamcpp::shouldAddKeyframe(
    const gtsam::Pose3& lastPose, 
    const gtsam::Pose3& currentPose, 
    std::set<gtsam::Symbol> oldlandmarks, 
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos) {
    
    // Calculate 3D distance
    double distance = (currentPose.translation() - lastPose.translation()).norm();
    
    // Check for new landmarks
    for (const auto& landmark : detectedLandmarksCurrentPos) {
        if (oldlandmarks.find(landmark) == oldlandmarks.end()) {
            return true;
        }
    }
    
    // Calculate rotation difference
    gtsam::Rot3 deltaRotation = lastPose.rotation().between(currentPose.rotation());
    double angleDifference = deltaRotation.axisAngle().second;  // Get angle

    if (distance > distanceThreshold || angleDifference > rotationThreshold) {
        return true;
    }
    return false;
}

void aprilslamcpp::pruneGraphByPoseCount(int maxPoses) {
    std::set<gtsam::Key> poseKeys;
    for (const auto& factor : keyframeGraph_) {
        for (const auto& key : factor->keys()) {
            gtsam::Symbol symbol(key);
            if (symbol.chr() == 'X') {
                poseKeys.insert(key);
            }
        }
    }

    if (poseKeys.size() <= maxPoses) {
        return;
    }

    std::vector<gtsam::Key> sortedPoseKeys(poseKeys.begin(), poseKeys.end());
    std::sort(sortedPoseKeys.begin(), sortedPoseKeys.end(), [](gtsam::Key a, gtsam::Key b) {
        return gtsam::Symbol(a).index() < gtsam::Symbol(b).index();
    });

    std::set<gtsam::Key> keysToRemove(sortedPoseKeys.begin(), sortedPoseKeys.begin() + (poseKeys.size() - maxPoses));

    gtsam::NonlinearFactorGraph newGraph;
    for (const auto& factor : keyframeGraph_) {
        bool keepFactor = true;
        for (const auto& key : factor->keys()) {
            if (keysToRemove.count(key) > 0) {
                keepFactor = false;
                break;
            }
        }
        if (keepFactor) {
            newGraph.add(factor);
        }
    }

    gtsam::Values newEstimates;
    for (const auto& key_value : keyframeEstimates_) {
        if (keysToRemove.count(key_value.key) == 0) {
            newEstimates.insert(key_value.key, key_value.value);
        }
    }

    keyframeGraph_ = newGraph;
    keyframeEstimates_ = newEstimates;

    gtsam::Key oldestPoseKey = *std::min_element(sortedPoseKeys.begin() + (poseKeys.size() - maxPoses), sortedPoseKeys.end(), [](gtsam::Key a, gtsam::Key b) {
        return gtsam::Symbol(a).index() < gtsam::Symbol(b).index();
    });

    gtsam::Symbol oldestPoseSymbol(oldestPoseKey);

    if (!priorAddedToPose[oldestPoseSymbol]) {
        gtsam::Pose3 oldestPoseEstimate = keyframeEstimates_.at<gtsam::Pose3>(oldestPoseKey);
        keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose3>(
            oldestPoseKey, oldestPoseEstimate, priorNoise));
        priorAddedToPose[oldestPoseSymbol] = true;
    }
}

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

void aprilslamcpp::ISAM2Optimise() {    
    if (batchOptimisation_) {
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
        keyframeEstimates_ = batchOptimizer.optimize();
        batchOptimisation_ = false;
    }

    isam_.update(keyframeGraph_, keyframeEstimates_);
    
    keyframeEstimates_.clear();
    keyframeGraph_.resize(0);
}

gtsam::Values aprilslamcpp::SAMOptimise() {    
    gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    gtsam::Values result = batchOptimizer.optimize();
    return result;
}

void aprilslamcpp::checkLoopClosure(const std::set<gtsam::Symbol>& detectedLandmarksCurrentPos) {
    if (useloopclosure) {
        gtsam::Symbol currentPoseIndex =  gtsam::Symbol('X', index_of_pose);
        gtsam::Pose3 currentPose =  keyframeEstimates_.at<gtsam::Pose3>(currentPoseIndex);
        
        for (const auto& entry : poseToLandmarks) {
            gtsam::Symbol keyframeSymbol = entry.first;
            const std::set<gtsam::Symbol>& keyframeLandmarks = entry.second;

            gtsam::Pose3 keyframePose = keyframeEstimates_.at<gtsam::Pose3>(keyframeSymbol);
            int keyframeIndex = keyframeSymbol.index();

            // 3D distance
            double distance = (currentPose.translation() - keyframePose.translation()).norm();

            if (distance < historyKeyframeSearchRadius && (currentPoseIndex.index() - keyframeIndex) > historyKeyframeSearchNum) {
                std::set<gtsam::Symbol> intersection;
                std::set_intersection(detectedLandmarksCurrentPos.begin(), detectedLandmarksCurrentPos.end(),
                                      keyframeLandmarks.begin(), keyframeLandmarks.end(),
                                      std::inserter(intersection, intersection.begin()));

                int reobservedLandmarks = intersection.size();

                if (reobservedLandmarks >= requiredReobservedLandmarks) {
                    ROS_INFO("found LC");
                    keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                        keyframeSymbol, currentPoseIndex, keyframePose.between(currentPose), loopClosureNoise));

                    visualizeLoopClosure(lc_pub_, currentPose, keyframePose, currentPoseIndex.index(), map_frame_id);

                    break;
                }
            }
        }
    }
}

bool aprilslam::aprilslamcpp::movementExceedsThreshold(const gtsam::Pose3& poseSE3) {
    gtsam::Point3 currentPos = poseSE3.translation();
    gtsam::Point3 lastPos = lastPoseSE3_.translation();
    
    double position_change = (currentPos - lastPos).norm();
    
    gtsam::Rot3 rotationDelta = lastPoseSE3_.rotation().between(poseSE3.rotation());
    double rotation_change = rotationDelta.axisAngle().second;
    
    return position_change >= stationary_position_threshold || rotation_change >= stationary_rotation_threshold;
}

void aprilslam::aprilslamcpp::initializeFirstPose(const gtsam::Pose3& poseSE3, gtsam::Pose3& pose0) {
    lastPoseSE3_ = poseSE3;
    lastPoseSE3_vis = poseSE3;
    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('X', 1), pose0, priorNoise));
    keyframeEstimates_.insert(gtsam::Symbol('X', 1), pose0);
    Estimates_visulisation.insert(gtsam::Symbol('X', 1), pose0);
    lastPose_ = pose0;
    lastPose_for_jump = pose0;
    
    if (usepriortagtable) {
        for (const auto& landmark : savedLandmarks) {
            gtsam::Symbol landmarkKey('L', landmark.first);
            keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point3>(landmarkKey, landmark.second, pointNoise));
            keyframeEstimates_.insert(landmarkKey, landmark.second);
            landmarkEstimates.insert(landmarkKey, landmark.second);
        }
    }
    Key_previous_pos = pose0;
    previousKeyframeSymbol = gtsam::Symbol('X', 1);
}

gtsam::Pose3 aprilslam::aprilslamcpp::predictNextPose(const gtsam::Pose3& poseSE3) {
    gtsam::Pose3 odometry = lastPoseSE3_.between(poseSE3);
    return lastPose_.compose(odometry);
}

void aprilslam::aprilslamcpp::updateOdometryPose(const gtsam::Pose3& poseSE3) {
    gtsam::Pose3 odometry = lastPoseSE3_vis.between(poseSE3);
    gtsam::Pose3 newPose = Estimates_visulisation.at<gtsam::Pose3>(gtsam::Symbol('X', index_of_pose - 1)).compose(odometry);
    Estimates_visulisation.insert(gtsam::Symbol('X', index_of_pose), newPose);
    lastPoseSE3_vis = poseSE3;
}

void aprilslam::aprilslamcpp::generate2bePublished() {
    if (useisam2) {
        auto result = isam_.calculateEstimate();

        std::map<int, gtsam::Point3> landmarks;
        for (const auto& key_value : result) {
            gtsam::Key key = key_value.key;
            if (gtsam::Symbol(key).chr() == 'L') {
                gtsam::Point3 point = result.at<gtsam::Point3>(key);
                landmarks[gtsam::Symbol(key).index()] = point;
            }
        }

        aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);
        Estimates_visulisation.insert(previousKeyframeSymbol, result.at<gtsam::Pose3>(previousKeyframeSymbol));
    } 
    else {
        std::map<int, gtsam::Point3> landmarks;
        for (const auto& key_value : keyframeEstimates_) {
            gtsam::Key key = key_value.key;
            if (gtsam::Symbol(key).chr() == 'L') {
                gtsam::Point3 point = keyframeEstimates_.at<gtsam::Point3>(key);
                landmarks[gtsam::Symbol(key).index()] = point;
            }
        }

        aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);
        Estimates_visulisation.insert(previousKeyframeSymbol, keyframeEstimates_.at<gtsam::Pose3>(previousKeyframeSymbol));
    }
}

std::set<gtsam::Symbol> aprilslam::aprilslamcpp::updateGraphWithLandmarks(
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos, 
    const std::pair<std::vector<int>, std::vector<Eigen::Vector3d>>& detections) {

    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector3d>& tagPos = detections.second;

    if (!Id.empty()) {
        for (size_t n = 0; n < Id.size(); ++n) {
            int tag_number = Id[n];        
            Eigen::Vector3d landSE3 = tagPos[n];

            if (usepriortagtable && savedLandmarks.find(tag_number) == savedLandmarks.end()) {
                continue;
            }

            // Compute bearing, elevation, and range for 3D measurement
            double range = landSE3.norm();
            double bearing = std::atan2(landSE3(1), landSE3(0));
            double elevation = std::atan2(landSE3(2), std::sqrt(landSE3(0)*landSE3(0) + landSE3(1)*landSE3(1)));

            gtsam::Symbol landmarkKey('L', tag_number);  

            if (detectedLandmarksHistoric.find(landmarkKey) != detectedLandmarksHistoric.end()) {
                // Existing landmark - use full 3D bearing
                gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey, 
                    gtsam::Unit3(landSE3), range, brNoise
                );
                gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);

                if (error.norm() < add2graph_threshold) 
                    keyframeGraph_.add(factor);

                detectedLandmarksCurrentPos.insert(landmarkKey);
            } else {
                // New landmark - compute prior location using full 3D transformation
                gtsam::Point3 landmarkInRobotFrame(landSE3(0), landSE3(1), landSE3(2));
                gtsam::Point3 priorLand = lastPose_.transformFrom(landmarkInRobotFrame);

                if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                    detectedLandmarksHistoric.insert(landmarkKey);

                    if (!keyframeEstimates_.exists(landmarkKey)) {
                        keyframeEstimates_.insert(landmarkKey, priorLand);
                    }

                    if (!landmarkEstimates.exists(landmarkKey)) {
                        landmarkEstimates.insert(landmarkKey, priorLand);
                    }

                    keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point3>(
                        landmarkKey, priorLand, pointNoise)
                    );
                }

                // Add bearing-range factor with full 3D bearing
                gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3> factor(
                    gtsam::Symbol('X', index_of_pose), landmarkKey,
                    gtsam::Unit3(landSE3), range, brNoise
                );
                keyframeGraph_.add(factor);
                detectedLandmarksCurrentPos.insert(landmarkKey);
            }
        }
    }
    return detectedLandmarksCurrentPos;
}

void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    
    if (usePFinitialise) {
        if (!pfInitialized_) {
            return;
        }
    }    

    double current_time = ros::Time::now().toSec();
    ros::WallTime start_loop, end_loop;
    double elapsed;

    gtsam::Pose3 poseSE3 = translateOdomMsg(msg);
    
    double raw_time = ros::Time::now().toSec();
    raw_odom_csv << std::fixed << std::setprecision(6)
                << raw_time << ","
                << poseSE3.translation().x() << ","
                << poseSE3.translation().y() << ","
                << poseSE3.translation().z() << ","
                << poseSE3.rotation().roll() << ","
                << poseSE3.rotation().pitch() << ","
                << poseSE3.rotation().yaw() << std::endl;
                
    aprilslam::publishMapToOdomTF(tf_broadcaster, Estimates_visulisation, index_of_pose, poseSE3, map_frame_id, odom_frame, robot_frame); 
    
    if (!movementExceedsThreshold(poseSE3)) return;

    index_of_pose += 1;
    if (index_of_pose == 2) initializeFirstPose(poseSE3, pose0);

    gtsam::Pose3 predictedPose = predictNextPose(poseSE3);

    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);

    std::set<gtsam::Symbol> detectedLandmarksCurrentPos;
    std::set<gtsam::Symbol> oldlandmarks;
    oldlandmarks = detectedLandmarksHistoric; 

    if (shouldAddKeyframe(Key_previous_pos, predictedPose, oldlandmarks, detectedLandmarksCurrentPos) || !usekeyframe) {
        keyframeEstimates_.insert(gtsam::Symbol('X', index_of_pose), predictedPose);
        if (previousKeyframeSymbol) {
            gtsam::Pose3 relativePose = Key_previous_pos.between(predictedPose);
            keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
        }
         
        lastPose_ = predictedPose;
        landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), predictedPose);

        start_loop = ros::WallTime::now();
        // FIXED: Pass 'true' for 3D mode
        auto detections = getCamDetections(camera_infos_, camera_detections_, true);
        if (!detections.first.empty()) {
            detectedLandmarksCurrentPos = updateGraphWithLandmarks(detectedLandmarksCurrentPos, detections);
        } 
        poseToLandmarks[gtsam::Symbol('X', index_of_pose)] = detectedLandmarksCurrentPos;

        end_loop = ros::WallTime::now();
        elapsed = (end_loop - start_loop).toSec();

        start_loop = ros::WallTime::now();
        if (index_of_pose % 1 == 0) {
            if (useisam2) {
                ISAM2Optimise();
            }
            else {
                gtsam::Values result = SAMOptimise();

                gtsam::Symbol currentPoseSymbol('X', index_of_pose);
                gtsam::Pose3 newPose = result.at<gtsam::Pose3>(currentPoseSymbol);
                gtsam::Pose3 oldPose = lastPose_for_jump;

                double poseJump = computePoseDelta(oldPose, newPose);

                if (index_of_pose < outlierRemovalStartIndex_) {
                    keyframeEstimates_ = result;
                } else {        
                    if (poseJump > jumpCombinedThreshold) {
                        if (useoutlierremoval) {
                            ROS_WARN("Large pose jump detected (%.3f). Reverting to odometry!", poseJump);
                            gtsam::Pose3 odometry = lastPoseSE3_.between(poseSE3);
                            gtsam::Pose3 newPose = lastPose_for_jump.compose(odometry);
                            keyframeEstimates_.update(gtsam::Symbol('X', index_of_pose), newPose);
                        }
                    } else {
                        keyframeEstimates_ = result;
                        if (useprunebysize) {
                            pruneGraphByPoseCount(maxfactors);    
                        }  
                    }
                }
            }
            end_loop = ros::WallTime::now();
            elapsed = (end_loop - start_loop).toSec();
            ROS_INFO("optimisation: %f seconds", elapsed);
        }
        lastPose_for_jump = keyframeEstimates_.at<gtsam::Pose3>(currentKeyframeSymbol);
        lastPoseSE3_ = poseSE3;
        
        Key_previous_pos = predictedPose;
        previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);  
        checkLoopClosure(detectedLandmarksCurrentPos);
        generate2bePublished();
    }
    else {
        updateOdometryPose(poseSE3);
    }
    
    if (usetrajsmoothing && !usekeyframe) {
        if (index_of_pose >= smoothingStartIndex_) {
            smoothTrajectory(smoothingwindow);
        } 
    }
    
    publishRefinedOdom(odom_traj_pub_, Estimates_visulisation, index_of_pose, map_frame_id, robot_frame, refined_odom_csv, ros::Time::now());
    aprilslam::publishPath(path_pub_, Estimates_visulisation, index_of_pose, map_frame_id);
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "april_slam_3d_full");
    ros::NodeHandle nh;
    aprilslam::aprilslamcpp slamNode(nh);
    ros::spin();
    return 0;
}