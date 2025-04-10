#include "aprilslamheader.h"
#include "publishing_utils.h"

namespace aprilslam {

gtsam::Pose2 odometryDirection(const gtsam::Pose2& odometry, double cmd_vel_linear_x){
    if (cmd_vel_linear_x == 0.0) {
        return odometry;
    }
    // Identify if vehicle is moving forward with cmd_vel direction
    double dx = odometry.x();
    if (cmd_vel_linear_x < 0) {
        dx = -std::abs(dx);  // Moving backward
    } else {
        dx = std::abs(dx);   // Moving forward
    }
    // Create a new odometry pose with adjusted dx
    gtsam::Pose2 adjustedOdometry(dx, odometry.y(), odometry.theta());
    return gtsam::Pose2(dx, odometry.y(), odometry.theta());
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
    // nh_.getParam("ud_frame", ud_frame);


    // Read batch optimization flag
    nh_.getParam("batch_optimisation", batchOptimisation_);

    // Read noise models
    std::vector<double> odometry_noise, prior_noise, bearing_range_noise, point_noise, loop_ClosureNoise;
    nh_.getParam("noise_models/odometry", odometry_noise);
    nh_.getParam("noise_models/prior", prior_noise);
    nh_.getParam("noise_models/bearing_range", bearing_range_noise);
    nh_.getParam("noise_models/point", point_noise);
    nh_.getParam("noise_models/loopClosureNoise", loop_ClosureNoise);

    // Read error thershold for a landmark to be added to the graph
    nh_.getParam("add2graph_threshold", add2graph_threshold);

    // Read Prune conditions
    nh_.getParam("maxfactors", maxfactors);
    nh_.getParam("useprunebysize", useprunebysize);

    // Read initilisation conditions
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

    // Stationay conditions
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

    // Camera transformation parameters
    nh_.getParam("camera_parameters/xyTrans_lcam_baselink", xyTrans_lcam_baselink);
    nh_.getParam("camera_parameters/xyTrans_rcam_baselink", xyTrans_rcam_baselink);
    nh_.getParam("camera_parameters/xyTrans_mcam_baselink", xyTrans_mcam_baselink);
    // Convert to Eigen::Vector3d
    mcam_baselink_transform = Eigen::Vector3d(xyTrans_mcam_baselink[0], xyTrans_mcam_baselink[1], xyTrans_mcam_baselink[2]);
    rcam_baselink_transform = Eigen::Vector3d(xyTrans_rcam_baselink[0], xyTrans_rcam_baselink[1], xyTrans_rcam_baselink[2]);
    lcam_baselink_transform = Eigen::Vector3d(xyTrans_lcam_baselink[0], xyTrans_lcam_baselink[1], xyTrans_lcam_baselink[2]);

    // Load camera topics
    nh_.getParam("camera_subscribers/lCam_subscriber/topic", lCam_topic);
    nh_.getParam("camera_subscribers/rCam_subscriber/topic", rCam_topic);
    nh_.getParam("camera_subscribers/mCam_subscriber/topic", mCam_topic);

    // Load outlier removal conditons
<<<<<<< HEAD
    nh_.getParam("useoutlierremoval", useoutlierremoval); 
    nh_.getParam("jumpCombinedThreshold", jumpCombinedThreshold); 
    nh_.getParam("outlierRemovalStartIndex_", outlierRemovalStartIndex_);

    // Load outlier removal conditons
    nh_.getParam("usetrajsmoothing", usetrajsmoothing); 
    nh_.getParam("smoothingwindow", smoothingwindow); 
    nh_.getParam("smoothingStartIndex_", smoothingStartIndex_);

=======
    nh_.param("max_optimisation_translation", jumpTranslationThreshold, 1.5); // meters
    nh_.param("max_optimisation_rotation", jumpRotationThreshold, 1.0); // radians
    nh_.param("max_optimisation_jump", jumpCombinedThreshold, 1.0); 
    nh_.param("outlier_removal_start_index", outlierRemovalStartIndex_, 20); // e.g. 10 timesteps
    
>>>>>>> f78b9e945b4ec65f3fe13d78359fb8d2be762843
    // save localisation result
    refined_odom_csv.open("/home/shuoyuan/catkin_slam_ws/src/aprilslamcpp/refined_odometry.csv", std::ios::out);
    raw_odom_csv.open("/home/shuoyuan/catkin_slam_ws/src/aprilslamcpp/raw_odometry.csv", std::ios::out);
    refined_odom_csv << "time,x,y,theta\n";  // Write header
    raw_odom_csv << "time,x,y,theta\n";  // Write header

    // Load saveLandmarks
    savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);

    // Initialize noise models
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << odometry_noise[0], odometry_noise[1], odometry_noise[2]).finished());
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << prior_noise[0], prior_noise[1], prior_noise[2]).finished());
    brNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << bearing_range_noise[0], bearing_range_noise[1]).finished());
    pointNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << point_noise[0], point_noise[1]).finished());
    loopClosureNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << loop_ClosureNoise[0], loop_ClosureNoise[1], loop_ClosureNoise[2]).finished());

    // Optimiser selection
    nh_.getParam("useisam2", useisam2);

    // Total number of IDs
    int total_tags;
    nh_.getParam("total_tags", total_tags);
    // Predefined tags to search for in the environment.
    for (int j = 0; j < total_tags; ++j) {
        possibleIds_.push_back("tag_" + std::to_string(j));
    }
    
    ROS_INFO("Parameters loaded.");

    // Initialize GTSAM components
    initializeGTSAM();
    // Index to keep track of the sequential pose.
    index_of_pose = 1;
    previousframeSymbol = index_of_pose;
    // Initialize the factor graphs
    keyframeGraph_ = gtsam::NonlinearFactorGraph();

    // Initialize camera subscribers
    mCam_subscriber = nh_.subscribe(mCam_topic, 1000, &aprilslamcpp::mCamCallback, this);
    rCam_subscriber = nh_.subscribe(rCam_topic, 1000, &aprilslamcpp::rCamCallback, this);
    lCam_subscriber = nh_.subscribe(lCam_topic, 1000, &aprilslamcpp::lCamCallback, this);
    
    // Initialise pose0 using particle filter, set a timer to ensure the initilisation is done properly
    if (usePFinitialise) {
        pf_init_timer_ = nh_.createTimer(ros::Duration(0.5), &aprilslamcpp::pfInitCallback, this);
    } else {
        pose0 = gtsam::Pose2(0.0, 0.0, 0.0);
    }
    
    // Subscriptions and Publications
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
    lc_pub_ = nh_.advertise<visualization_msgs::Marker>("loop_closure_markers", 1);
    landmark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
    path.header.frame_id = map_frame_id; 
    odom_traj_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom_tag", 1, true);
}

double aprilslamcpp::computePoseDelta(const gtsam::Pose2& oldPose, const gtsam::Pose2& newPose){
    double dx = newPose.x() - oldPose.x();
    double dy = newPose.y() - oldPose.y();

    // Get the old heading
    double oldTheta = oldPose.theta();

    // Project the (dx,dy) onto the axis perpendicular to oldTheta
    // i.e. dot it with [ -sin(theta), cos(theta) ]
    double lat = dx * (-std::sin(oldTheta)) + dy * std::cos(oldTheta);

    // Return absolute value to get the magnitude of the lateral jump
    return std::fabs(lat);
}

void aprilslamcpp::pfInitCallback(const ros::TimerEvent& event) {
    // Initial debug message for function entry
    ROS_INFO("PF Running");
    // If PF initialization already completed, stop the timer and return.
    if (pfInitialized_) {
        pf_init_timer_.stop();
        return;
    }

    // Attempt to get camera detections
    auto detections = getCamDetections(mCam_msg, rCam_msg, lCam_msg,
                                       mcam_baselink_transform, rcam_baselink_transform, lcam_baselink_transform);
    const std::vector<int>& Id = detections.first;
    const std::vector<Eigen::Vector2d>& tagPos = detections.second;

    // If no tags detected, cannot start or continue PF initialization
    if (Id.empty()) {
        ROS_INFO("No tags detected, waiting for detections...");
        return;
    }

    ROS_INFO("Number of tags observed: %zu", Id.size());

    double currentTime = ros::Time::now().toSec();

    // Start PF initialization if not started yet
    if (!pfInitInProgress_) {
        pfInitInProgress_ = true;
        pfInitStartTime_ = currentTime;

        // Initialize particles from the first detected tag
        x_P_pf_ = initParticlesFromFirstTag(Id, tagPos, savedLandmarks, PFWaitTime);

        ROS_INFO("PF initialization started.");
    }

    double elapsed = currentTime - pfInitStartTime_;

    if (elapsed < PFWaitTime) {
        // Within the PF init duration, run PF update
        x_P_pf_ = particleFilter(Id, tagPos, savedLandmarks, x_P_pf_, PFWaitTime, rngVar_, brngVar_);
    } else {
        // PF initialization time is up. Run PF one last time to get final estimate
        x_P_pf_ = particleFilter(Id, tagPos, savedLandmarks, x_P_pf_, PFWaitTime, rngVar_, brngVar_);

        // Compute x_est as mean of particles
        Eigen::Vector3d sum_states(0,0,0);
        for (const auto& particle : x_P_pf_) {
            sum_states += particle;
        }
        Eigen::Vector3d x_est_pf = sum_states / static_cast<double>(PFWaitTime);

        // Report the initialization result
        ROS_INFO("PF initialization result: x = %f, y = %f, theta = %f", x_est_pf(0), x_est_pf(1), 0.0);
        ROS_INFO("PLEASE DONT MOVE THE ROBOT!!!");

        // Ask user for confirmation
        std::string userInput;
        ROS_INFO("Are you satisfied with the PF initialization result? (yes/no): ");
        std::cin >> userInput;

        if (userInput == "yes") {
            // Finalize PF initialization using the PF-derived initial pose
            pose0 = gtsam::Pose2(x_est_pf(0), x_est_pf(1), 0);
            pfInitialized_ = true;
            pfInitInProgress_ = false;

            // Stop the timer now that initialization is complete
            pf_init_timer_.stop();

            // Free up memory
            x_P_pf_.clear();

            ROS_INFO("PF initialization finalized successfully.");
        } else {
            // Restart initialization process
            pfInitInProgress_ = false;
            ROS_WARN("PF initialization rejected. Restarting initialization process.");
            x_P_pf_.clear();
        }
    }
}

// Camera callback functions
void aprilslamcpp::mCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    mCam_msg = msg;
}

void aprilslamcpp::rCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    rCam_msg = msg;
}

void aprilslamcpp::lCamCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    lCam_msg = msg;
}

// Applies a moving average filter to smooth the trajectory
void aprilslamcpp::smoothTrajectory(int window_size) {
    // If empty, nothing to do
    if (Estimates_visulisation.empty()) {
        return;
    }

    // Collect all X(...) poses in a vector
    std::vector<std::pair<gtsam::Symbol, gtsam::Pose2>> xPoses;
    xPoses.reserve(Estimates_visulisation.size()); // minor optimization

    for (const auto& key_value : Estimates_visulisation) {
        gtsam::Symbol key(key_value.key);
        if (key.chr() == 'X') {
            // Convert the stored value to Pose2
            gtsam::Pose2 pose = key_value.value.cast<gtsam::Pose2>();
            xPoses.emplace_back(key, pose);
        }
    }

    // If fewer than window_size, skip smoothing
    if (xPoses.size() < static_cast<size_t>(window_size)) {
        return;
    }

    // Sort by Symbol index so we know which one is "last"
    std::sort(xPoses.begin(), xPoses.end(),
              [](const auto& a, const auto& b) {
                  return a.first.index() < b.first.index();
              });

    // Sum up the y-coordinates from the last `window_size` poses
    double sumY = 0.0;
    double sumX = 0.0;
    for (size_t i = xPoses.size() - window_size; i < xPoses.size(); ++i) {
        sumY += xPoses[i].second.y();
        sumX += xPoses[i].second.x();
    }

    // Compute the average y
    double avgY = sumY / window_size;
    double avgX = sumX / window_size;

    // Grab the final pose's x and theta exactly as-is
    gtsam::Pose2 lastPose = xPoses.back().second;    // the last stored pose
    double keepTheta = lastPose.theta();             // no smoothing for orientation

    // Construct a "smoothed" pose for the last key
    gtsam::Pose2 smoothedPose(avgX, avgY, keepTheta);

    // Overwrite the last pose in the values with our new partial-smooth version
    gtsam::Symbol lastKey = xPoses.back().first;
    if (Estimates_visulisation.exists(lastKey)) {
        Estimates_visulisation.update(lastKey, smoothedPose);
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

aprilslamcpp::~aprilslamcpp() {
        // Empty destructor, no resources to clean up.
        ROS_INFO("Shutting down aprilslamcpp.");
}

bool aprilslamcpp::shouldAddKeyframe(
    const gtsam::Pose2& lastPose, 
    const gtsam::Pose2& currentPose, 
    std::set<gtsam::Symbol> oldlandmarks, 
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos) {
    // Calculate the distance between the current pose and the last keyframe pose
    double distance = lastPose.range(currentPose);
    // Iterate over detectedLandmarksCurrentPos, add key if new tag is detected
    for (const auto& landmark : detectedLandmarksCurrentPos) {
        // If the landmark is not found in oldLandmarks, return true
        if (oldlandmarks.find(landmark) == oldlandmarks.end()) {
            return true;
        }
    }
    // Calculate the difference in orientation (theta) between the current pose and the last keyframe pose
    double angleDifference = std::abs(wrapToPi(currentPose.theta() - lastPose.theta()));

    // Check if either the distance moved or the rotation exceeds the threshold
    if (distance > distanceThreshold || angleDifference > rotationThreshold) {
        return true;  // Add a new keyframe
    }
    return false;  // Do not add a keyframe
}

void aprilslamcpp::pruneGraphByPoseCount(int maxPoses) {
    // Extract all pose keys from the graph
    std::set<gtsam::Key> poseKeys;
    for (const auto& factor : keyframeGraph_) {
        for (const auto& key : factor->keys()) {
            gtsam::Symbol symbol(key);
            if (symbol.chr() == 'X') { // Assuming 'X' represents pose variables
                poseKeys.insert(key);
            }
        }
    }

    // Check if pruning is needed
    if (poseKeys.size() <= maxPoses) {
        // No pruning needed
        return;
    }

    // Sort pose keys by their indices
    std::vector<gtsam::Key> sortedPoseKeys(poseKeys.begin(), poseKeys.end());
    std::sort(sortedPoseKeys.begin(), sortedPoseKeys.end(), [](gtsam::Key a, gtsam::Key b) {
        return gtsam::Symbol(a).index() < gtsam::Symbol(b).index();
    });

    // Identify poses to remove (the oldest ones)
    std::set<gtsam::Key> keysToRemove(sortedPoseKeys.begin(), sortedPoseKeys.begin() + (poseKeys.size() - maxPoses));

    // Build new graph and estimates without the poses to remove
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

    // Update the internal state
    keyframeGraph_ = newGraph;
    keyframeEstimates_ = newEstimates;

    // Add a prior to the oldest remaining pose if not already added
    // Get the oldest remaining pose key
    gtsam::Key oldestPoseKey = *std::min_element(sortedPoseKeys.begin() + (poseKeys.size() - maxPoses), sortedPoseKeys.end(), [](gtsam::Key a, gtsam::Key b) {
        return gtsam::Symbol(a).index() < gtsam::Symbol(b).index();
    });

    gtsam::Symbol oldestPoseSymbol(oldestPoseKey);

    if (!priorAddedToPose[oldestPoseSymbol]) {
        // Get the current estimate of the pose
        gtsam::Pose2 oldestPoseEstimate = keyframeEstimates_.at<gtsam::Pose2>(oldestPoseKey);
        // Add a prior factor
        keyframeGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(
            oldestPoseKey, oldestPoseEstimate, priorNoise));
        // Keep track that we added a prior to this pose
        priorAddedToPose[oldestPoseSymbol] = true;
    }
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

void aprilslamcpp::ISAM2Optimise() {    
    if (batchOptimisation_) {
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
        keyframeEstimates_ = batchOptimizer.optimize();
        batchOptimisation_ = false; // Only do this once
    }

    // Update the iSAM2 instance with the new measurements
    isam_.update(keyframeGraph_, keyframeEstimates_);
    
    keyframeEstimates_.clear();
    keyframeGraph_.resize(0);
}

gtsam::Values aprilslamcpp::SAMOptimise() {    
    // Perform batch optimization using Levenberg-Marquardt optimizer
    gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    gtsam::Values result = batchOptimizer.optimize();
    return result;
}

void aprilslamcpp::checkLoopClosure(const std::set<gtsam::Symbol>& detectedLandmarksCurrentPos) {
    if (useloopclosure) {
        // Get the current pose index
        gtsam::Symbol currentPoseIndex =  gtsam::Symbol('X', index_of_pose);
        gtsam::Pose2 currentPose =  keyframeEstimates_.at<gtsam::Pose2>(currentPoseIndex);
        // Loop through each keyframe stored in poseToLandmarks
        for (const auto& entry : poseToLandmarks) {
            gtsam::Symbol keyframeSymbol = entry.first;  // Symbol representing the keyframe
            const std::set<gtsam::Symbol>& keyframeLandmarks = entry.second;  // Landmarks associated with the keyframe

            // Get the keyframe's pose and its index
            gtsam::Pose2 keyframePose = keyframeEstimates_.at<gtsam::Pose2>(keyframeSymbol);
            int keyframeIndex = keyframeSymbol.index();  // Assuming index is accessible from the symbol

            // Compute the spatial distance between the current pose and the keyframe pose
            double distance = lastPose_.range(keyframePose);

            // Check if the spatial distance and index difference meet the loop closure criteria
            if (distance < historyKeyframeSearchRadius && (currentPoseIndex - keyframeIndex) > historyKeyframeSearchNum) {
                // Find the intersection of landmarks re-observed at the current pose and the keyframe's landmarks
                std::set<gtsam::Symbol> intersection;
                std::set_intersection(detectedLandmarksCurrentPos.begin(), detectedLandmarksCurrentPos.end(),
                                      keyframeLandmarks.begin(), keyframeLandmarks.end(),
                                      std::inserter(intersection, intersection.begin()));

                // Count the number of re-observed landmarks
                int reobservedLandmarks = intersection.size();

                // If the number of re-observed landmarks meets the required threshold, trigger loop closure
                if (reobservedLandmarks >= requiredReobservedLandmarks) {
                    ROS_INFO("found LC");
                    // Add a loop closure constraint between the current pose and the keyframe
                    keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(keyframeSymbol, currentPoseIndex, relPoseFG(keyframePose, currentPose), loopClosureNoise));

                    // Visualize the loop closure
                    visualizeLoopClosure(lc_pub_, currentPose, keyframePose, currentPoseIndex, map_frame_id);

                    break;  // Exit after adding one loop closure constraint
                }
            }
        }
    }
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
    lastPose_for_jump = pose0; // For outlier removal
    // Load calibrated landmarks as priors if available
    if (usepriortagtable) {
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
    return lastPose_.compose(odometry);
}

// Update odometry without adding a keyframe
void aprilslam::aprilslamcpp::updateOdometryPose(const gtsam::Pose2& poseSE2) {
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_vis, poseSE2);
    // gtsam::Pose2 adjustedOdometry = odometryDirection(odometry, linear_x_velocity_);
    gtsam::Pose2 newPose = Estimates_visulisation.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose - 1)).compose(odometry);
    Estimates_visulisation.insert(gtsam::Symbol('X', index_of_pose), newPose);
    lastPoseSE2_vis = poseSE2;
<<<<<<< HEAD
=======
    ROS_INFO_STREAM("key:,  X(" << index_of_pose << ") = [x=" 
        << newPose.x() << ", y=" << newPose.y() 
        << ", theta=" << newPose.theta() << "]");
>>>>>>> f78b9e945b4ec65f3fe13d78359fb8d2be762843
}

void aprilslam::aprilslamcpp::generate2bePublished() {
    if (useisam2) {
        // Calculate the current estimate using iSAM2
        auto result = isam_.calculateEstimate();

        // Extract landmark estimates from the result
        std::map<int, gtsam::Point2> landmarks;
        for (const auto& key_value : result) {
            gtsam::Key key = key_value.key;  // Get the key
            if (gtsam::Symbol(key).chr() == 'L') {
                gtsam::Point2 point = result.at<gtsam::Point2>(key); // Directly access the Point2 value
                landmarks[gtsam::Symbol(key).index()] = point;
            }
        }

        // Publish the landmarks
        aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);

        // Update the visualized estimates with the current pose
        Estimates_visulisation.insert(previousKeyframeSymbol, result.at<gtsam::Pose2>(previousKeyframeSymbol));
    } 
    else {
        // Extract landmark estimates from keyframe estimates
        std::map<int, gtsam::Point2> landmarks;
        for (const auto& key_value : keyframeEstimates_) {
            gtsam::Key key = key_value.key;  // Get the key
            if (gtsam::Symbol(key).chr() == 'L') {
                gtsam::Point2 point = keyframeEstimates_.at<gtsam::Point2>(key);  // Access the Point2 value
                landmarks[gtsam::Symbol(key).index()] = point;
            }
        }

        // Publish the landmarks
        aprilslam::publishLandmarks(landmark_pub_, landmarks, map_frame_id);

        // Update the visualized estimates with the current pose
        Estimates_visulisation.insert(previousKeyframeSymbol, keyframeEstimates_.at<gtsam::Pose2>(previousKeyframeSymbol));
    }
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

            // If using prior table and the current tag_number is not found in savedLandmarks, skip it.
            if (usepriortagtable && savedLandmarks.find(tag_number) == savedLandmarks.end()) {
                // This tag is not in the prior table, do not add it to the graph
                continue;
            }

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
                if (fabs(error[0]) < add2graph_threshold) 
                    keyframeGraph_.add(factor);

                detectedLandmarksCurrentPos.insert(landmarkKey);
            } else {
                // Compute prior location of the landmark using the current robot pose
                double theta = lastPose_.theta();
                Eigen::Rotation2Dd rotation(theta);  // Create a 2D rotation matrix
                Eigen::Vector2d rotatedPosition = rotation * landSE2;  // Rotate the position into the robot's frame
                gtsam::Point2 priorLand(rotatedPosition.x() + lastPose_.x(), rotatedPosition.y() + lastPose_.y());

                // If the current landmark was not detected in the calibration run 
                // Or it's on calibration mode
                if (!landmarkEstimates.exists(landmarkKey) || !usepriortagtable) {
                    // New landmark detected
                    detectedLandmarksHistoric.insert(landmarkKey);

                    // Insert initial estimate if not already present
                    if (!keyframeEstimates_.exists(landmarkKey)) {
                        keyframeEstimates_.insert(landmarkKey, priorLand);
                    }

                    if (!landmarkEstimates.exists(landmarkKey)) {
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
                detectedLandmarksCurrentPos.insert(landmarkKey);
            }
        }
    }
    return detectedLandmarksCurrentPos;
}

void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    
    // Ignoring odometry because PF is not done yet
    if (usePFinitialise) {
        if (!pfInitialized_) {
            return;
        }
    }    

    double current_time = ros::Time::now().toSec();
    ros::WallTime start_loop, end_loop; // Declare variables to hold start and end times
    double elapsed;

    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);
    
    double raw_time = msg->header.stamp.toSec();
    raw_odom_csv << std::fixed << std::setprecision(6)
                << raw_time << ","
                << poseSE2.x() << ","
                << poseSE2.y() << ","
                << poseSE2.theta() << std::endl;
                
    // Publish tf
    aprilslam::publishMapToOdomTF(tf_broadcaster, Estimates_visulisation, index_of_pose, poseSE2, map_frame_id, odom_frame, robot_frame); 
    // Check if the movement exceeds the thresholds
    if (!movementExceedsThreshold(poseSE2)) return;

    index_of_pose += 1; // Increment the pose index for each new odometry message
    // Initrialisation of the factor node and variable node
    if (index_of_pose == 2) initializeFirstPose(poseSE2, pose0);

    // Predict the next pose based on odometry and add it as an initial estimate
    gtsam::Pose2 predictedPose = predictNextPose(poseSE2);

    // Determine if this pose should be a keyframe
    gtsam::Symbol currentKeyframeSymbol('X', index_of_pose);

    // Loop closure detection setup
    std::set<gtsam::Symbol> detectedLandmarksCurrentPos;
    std::set<gtsam::Symbol> oldlandmarks;
    oldlandmarks = detectedLandmarksHistoric; 

    // Add odometry factor if keyframe
    if (shouldAddKeyframe(Key_previous_pos, predictedPose, oldlandmarks, detectedLandmarksCurrentPos) || !usekeyframe) {
        keyframeEstimates_.insert(gtsam::Symbol('X', index_of_pose), predictedPose);
        if (previousKeyframeSymbol) {
            gtsam::Pose2 relativePose = Key_previous_pos.between(predictedPose);
            keyframeGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previousKeyframeSymbol, currentKeyframeSymbol, relativePose, odometryNoise));
        }
         
        // Update the last pose and initial estimates for the next iteration
        lastPose_ = predictedPose;
        landmarkEstimates.insert(gtsam::Symbol('X', index_of_pose), predictedPose);

        // Iterate through all landmark detected IDs
        start_loop = ros::WallTime::now();
        if (mCam_msg && rCam_msg && lCam_msg) {  // Ensure the messages have been received
            auto detections = getCamDetections(mCam_msg, rCam_msg, lCam_msg, mcam_baselink_transform, rcam_baselink_transform, lcam_baselink_transform);
            detectedLandmarksCurrentPos = updateGraphWithLandmarks(detectedLandmarksCurrentPos, detections);
        }

        // Update the pose to landmarks mapping (for LC conditions)
        poseToLandmarks[gtsam::Symbol('X', index_of_pose)] = detectedLandmarksCurrentPos;

        // Loging for optimisation time
        end_loop = ros::WallTime::now();
        elapsed = (end_loop - start_loop).toSec();

        start_loop = ros::WallTime::now();
        // ROS_INFO("number of timesteps: %d", index_of_pose);
        if (index_of_pose % 1 == 0) {
            if (useisam2) {ISAM2Optimise();}
            else {gtsam::Values result = SAMOptimise();

                // Retrieve CURRENT optimised pose
                gtsam::Symbol currentPoseSymbol('X', index_of_pose);
                gtsam::Pose2 newPose = result.at<gtsam::Pose2>(currentPoseSymbol);

                // Retrieve estimate before current
                gtsam::Pose2 oldPose = lastPose_for_jump;

                // Compute jumps 
                double poseJump = computePoseDelta(oldPose, newPose);

                if (index_of_pose < outlierRemovalStartIndex_) {
                    keyframeEstimates_ = result;
                } else {        
                    if (poseJump > jumpCombinedThreshold) {
<<<<<<< HEAD
                        if (useoutlierremoval) {
                            ROS_WARN("Large pose jump detected (%.3f). Reverting to odometry or previous estimate for this step!", poseJump);
                            ROS_WARN("Discarding the newly optimized solution and trusting the old estimate.");
                            gtsam::Pose2 oldPose = keyframeEstimates_.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose));
                            gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
                            gtsam::Pose2 newPose = lastPose_for_jump.compose(odometry);
                            keyframeEstimates_.update(gtsam::Symbol('X', index_of_pose), newPose);
                            gtsam::Pose2 retrievedPose = keyframeEstimates_.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose));
                        }
                    } else {
                        keyframeEstimates_ = result;
                        if (useprunebysize) {
                            pruneGraphByPoseCount(maxfactors);    
                        }  
                    }
                }
=======
                        ROS_WARN("Large pose jump detected (%.3f). Reverting to odometry or previous estimate for this step!", poseJump);
                        ROS_WARN("Discarding the newly optimized solution and trusting the old estimate.");
   
                        // 1. Print the existing pose before the update
                        gtsam::Pose2 oldPose = keyframeEstimates_.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose));
                    
                        // 2. Compute the pose we want to revert to
                        gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_, poseSE2);
                        gtsam::Pose2 newPose = lastPose_for_jump.compose(odometry);
                    
                        // 3. Update the keyframeEstimates_ with the new pose
                        keyframeEstimates_.update(gtsam::Symbol('X', index_of_pose), newPose);
                    
                        // 4. Print the newly updated pose
                        gtsam::Pose2 retrievedPose = keyframeEstimates_.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose));

                    } else {
                        keyframeEstimates_ = result;
                        if (useprunebysize) {
                            pruneGraphByPoseCount(maxfactors);
                        }
                    }
                }  
>>>>>>> f78b9e945b4ec65f3fe13d78359fb8d2be762843
            }
            end_loop = ros::WallTime::now();
            elapsed = (end_loop - start_loop).toSec();
            ROS_INFO("optimisation: %f seconds", elapsed);
        }
        lastPose_for_jump = keyframeEstimates_.at<gtsam::Pose2>(currentKeyframeSymbol);
        // ROS_INFO("transform total: %f seconds", elapsed);
        lastPoseSE2_ = poseSE2;
        
    Key_previous_pos = predictedPose;
    previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);  
<<<<<<< HEAD
=======
    ROS_INFO_STREAM("key:,  X(" << index_of_pose << ") = [x=" 
        << lastPose_for_jump.x() << ", y=" << lastPose_for_jump.y() 
        << ", theta=" << lastPose_for_jump.theta() << "]");
>>>>>>> f78b9e945b4ec65f3fe13d78359fb8d2be762843
    checkLoopClosure(detectedLandmarksCurrentPos);
    generate2bePublished();
    }
    // Use Odometry for pose estimation when not a keyframe, landmarks not updated
    else{
        updateOdometryPose(poseSE2);  // Update pose without adding a keyframe
    }
    // Smooth the trajectory
<<<<<<< HEAD
    if (usetrajsmoothing && !usekeyframe) {
        if (index_of_pose >= smoothingStartIndex_) {
            smoothTrajectory(smoothingwindow);
        } 
=======
    if (index_of_pose >= 20) {
        smoothTrajectory(5);
    } else {
        ROS_WARN("Skipping trajectory smoothing: only %d keyframes available.", index_of_pose);
>>>>>>> f78b9e945b4ec65f3fe13d78359fb8d2be762843
    }
    // Publish path, landmarks, and odometry for visulisation
    publishRefinedOdom(odom_traj_pub_, Estimates_visulisation, index_of_pose, map_frame_id, robot_frame, refined_odom_csv);
    aprilslam::publishPath(path_pub_, Estimates_visulisation, index_of_pose, map_frame_id);
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