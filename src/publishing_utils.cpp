// publishing_utils.cpp

#include "publishing_utils.h"

namespace aprilslam {

// Utility function to normalize an angle to the range [-pi, pi]
double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    return angle - M_PI;
}   

// Computes the relative pose between two Pose2 objects
gtsam::Pose2 relPoseFG(const gtsam::Pose2& lastPoseSE2, const gtsam::Pose2& PoseSE2) {
    double dx = PoseSE2.x() - lastPoseSE2.x();
    double dy = PoseSE2.y() - lastPoseSE2.y();
    double dtheta = wrapToPi(PoseSE2.theta() - lastPoseSE2.theta());

    // Compute the distance moved along the robot's forward direction
    double distance = std::sqrt(dx * dx + dy * dy);
    double direction = std::atan2(dy, dx);
    // return gtsam::Pose2(distance, 0, dtheta);

    // Adjust the distance based on the robot's heading to account for backward movement
    double theta = lastPoseSE2.theta();
    double dx_body = std::cos(theta) * dx + std::sin(theta) * dy;
    double dy_body = -std::sin(theta) * dx + std::cos(theta) * dy;

    // Return the relative pose assuming robot cant move sideways: dy = 0
    return gtsam::Pose2(dx_body, dy_body, dtheta);
} 

void publishLandmarks(ros::Publisher& landmark_pub, const std::map<int, gtsam::Point2>& landmarks, const std::string& frame_id) {
    visualization_msgs::MarkerArray markers;
    int id = 0;
    for (const auto& landmark : landmarks) {
        // Sphere marker for each landmark
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "landmarks";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = landmark.second.x();
        marker.pose.position.y = landmark.second.y();
        marker.pose.position.z = 0;  // Assuming the landmarks are on the ground plane
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);

        // Marker IDs
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "landmark_ids";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = landmark.second.x();
        text_marker.pose.position.y = landmark.second.y();
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.2;
        text_marker.text = std::to_string(landmark.first);
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;

        markers.markers.push_back(text_marker);
    }

    landmark_pub.publish(markers);
}

void publishPath(ros::Publisher& path_pub, const gtsam::Values& result, int max_index, const std::string& frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();
    path.poses.clear();  // Clear any existing poses

    for (int i = 1; i <= max_index; i++) {
        gtsam::Symbol sym('X', i);
        if (result.exists(sym)) {
            gtsam::Pose2 pose = result.at<gtsam::Pose2>(sym);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = pose.x();
            pose_msg.pose.position.y = pose.y();
            pose_msg.pose.position.z = 0;  // Assuming 2D

            tf2::Quaternion quat;
            quat.setRPY(0, 0, pose.theta());
            pose_msg.pose.orientation = tf2::toMsg(quat);

            path.poses.push_back(pose_msg);
        }
    }

    path_pub.publish(path);
}

void publishMapToOdomTF(tf2_ros::TransformBroadcaster& tf_broadcaster, 
                        const gtsam::Values& result, int latest_index, 
                        const gtsam::Pose2& poseSE2, 
                        const std::string& map_frame, const std::string& odom_frame, const std::string& base_link_frame) {
    // Define the symbol for the latest pose
    gtsam::Symbol sym('X', latest_index);

    // Check if the latest pose exists in the result
    if (result.exists(sym)) {
        // Extract the pose from the SLAM result (map -> base_link)
        gtsam::Pose2 slamPose = result.at<gtsam::Pose2>(sym);

        // Create the transform for map -> base_link
        tf2::Transform map_to_base_link;
        tf2::Quaternion slam_quat;
        slam_quat.setRPY(0, 0, slamPose.theta());  // Rotation around Z-axis
        map_to_base_link.setOrigin(tf2::Vector3(slamPose.x(), slamPose.y(), 0)); // Translation
        map_to_base_link.setRotation(slam_quat);

        // Convert odometry pose (odom -> base_link) into a transform
        tf2::Transform odom_to_base_link;
        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, poseSE2.theta());  // Rotation around Z-axis
        odom_to_base_link.setOrigin(tf2::Vector3(poseSE2.x(), poseSE2.y(), 0)); // Translation
        odom_to_base_link.setRotation(odom_quat);

        // Compute the map -> odom transform
        tf2::Transform map_to_odom = map_to_base_link * odom_to_base_link.inverse();

        // Publish the map -> odom transform
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = map_frame;  // map
        transform_stamped.child_frame_id = odom_frame; // odom

        // Set translation
        transform_stamped.transform.translation.x = map_to_odom.getOrigin().x();
        transform_stamped.transform.translation.y = map_to_odom.getOrigin().y();
        transform_stamped.transform.translation.z = map_to_odom.getOrigin().z();

        // Set rotation
        transform_stamped.transform.rotation = tf2::toMsg(map_to_odom.getRotation());

        // Broadcast the transform
        tf_broadcaster.sendTransform(transform_stamped);
    }
}

void publishRefinedOdom(ros::Publisher& odom_pub,
                        const gtsam::Values& Estimates_visulisation,
                        int index_of_pose,
                        const std::string& odom_frame,
                        const std::string& base_link_frame,
                        std::ofstream& refined_odom_csv,
                        const ros::Time& stamp)
{
    gtsam::Symbol sym('X', index_of_pose);

    // Make sure the pose exists
    if (!Estimates_visulisation.exists(sym)) {
        ROS_WARN("publishRefinedOdom: Pose not found in Estimates_visulisation for X%d", index_of_pose);
        return;
    }

    // 1) Retrieve the GTSAM pose (assuming it's odom->base_link)
    gtsam::Pose2 refinedPose = Estimates_visulisation.at<gtsam::Pose2>(sym);

    // 2) Convert to quaternion
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, refinedPose.theta());

    // 3) Fill nav_msgs::Odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame;       // e.g. "odom"
    odom_msg.child_frame_id  = base_link_frame;  // e.g. "base_link"

    // Position
    odom_msg.pose.pose.position.x = refinedPose.x();
    odom_msg.pose.pose.position.y = refinedPose.y();
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation
    odom_msg.pose.pose.orientation = tf2::toMsg(quat);

    // 4) Publish
    odom_pub.publish(odom_msg);

    // 5) Save to a csv
    double time = stamp.toSec();
    refined_odom_csv << std::fixed << std::setprecision(6)
                    << time << ","
                    << refinedPose.x() << ","
                    << refinedPose.y() << ","
                    << refinedPose.theta() << std::endl;

}

void saveLandmarksToCSV(const std::map<int, gtsam::Point2>& landmarks, const std::string& filename) {
    std::ofstream file;
    file.open(filename, std::ios::out); // Open file in write mode

    if (!file) {
        std::cerr << "Failed to open the file!" << std::endl;
        return;
    }
    // Write the header line
    file << "id,x,y\n";
    
    for (const auto& landmark : landmarks) {
        int id = landmark.first;
        gtsam::Point2 point = landmark.second;
        file << id << "," << point.x() << "," << point.y() << "\n";
    }

    file.close();
}

std::map<int, gtsam::Point2> loadLandmarksFromCSV(const std::string& filename) {
    std::map<int, gtsam::Point2> landmarks;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open the file!" << std::endl;
        return landmarks;
    }

    std::string line;

    // Skip the header line
    if (std::getline(file, line)) {
        // You can print or log the header if needed
        // std::cout << "Header: " << line << std::endl;
    }

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string id_str, x_str, y_str;
        if (std::getline(ss, id_str, ',') && std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            try {
                int id = std::stoi(id_str);
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                landmarks[id] = gtsam::Point2(x, y);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
            }
        }
    }

    file.close();
    return landmarks;
}

// funtion for computing tag locations from coordinate transformation
void processDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr& cam_msg, 
                       const Eigen::Vector3d& xyTrans_cam_baselink,
                       std::vector<int>& Ids, 
                       std::vector<Eigen::Vector2d>& tagPoss) {
    if (cam_msg) {
        for (const auto& detection : cam_msg->detections) {
            Ids.push_back(detection.id[0]);
            double rotTheta = xyTrans_cam_baselink(2);
            Eigen::Matrix2d R;
            R << cos(rotTheta), -sin(rotTheta),
                 sin(rotTheta),  cos(rotTheta);
            Eigen::Vector2d rotP = R * Eigen::Vector2d(detection.pose.pose.pose.position.z, -detection.pose.pose.pose.position.x);
            tagPoss.push_back(rotP + xyTrans_cam_baselink.head<2>());
        }
    }
}

// funtion for processing tag detection topics into IDs and TagPoss
std::pair<std::vector<int>, std::vector<Eigen::Vector2d>> getCamDetections(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& mCam_msg,
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& rCam_msg,
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& lCam_msg,
    const Eigen::Vector3d& xyTrans_mcam_baselink,
    const Eigen::Vector3d& xyTrans_rcam_baselink,
    const Eigen::Vector3d& xyTrans_lcam_baselink) {

    std::vector<int> Ids;
    std::vector<Eigen::Vector2d> tagPoss;

    // Process detections from each camera
    processDetections(mCam_msg, xyTrans_mcam_baselink, Ids, tagPoss);
    processDetections(rCam_msg, xyTrans_rcam_baselink, Ids, tagPoss);
    processDetections(lCam_msg, xyTrans_lcam_baselink, Ids, tagPoss);

    return std::make_pair(Ids, tagPoss);
}

void visualizeLoopClosure(ros::Publisher& lc_pub, const gtsam::Pose2& currentPose, const gtsam::Pose2& keyframePose, int currentPoseIndex, const std::string& frame_id) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "loop_closure_line";
    line_marker.id = currentPoseIndex;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;

    // Set the color to green
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    // Set line thickness
    line_marker.scale.x = 0.05;

    // Add points to the marker for the keyframe and current pose
    geometry_msgs::Point p1, p2;
    p1.x = keyframePose.x();
    p1.y = keyframePose.y();
    p1.z = 0;

    p2.x = currentPose.x();
    p2.y = currentPose.y();
    p2.z = 0;

    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    // Publish the line marker
    lc_pub.publish(line_marker);
}

// Particle filter as position initialisation function
std::vector<Eigen::Vector3d> particleFilter(
        const std::vector<int>& Id,
        const std::vector<Eigen::Vector2d>& tagPos,
        const std::map<int, gtsam::Point2>& savedLandmarks,
        std::vector<Eigen::Vector3d>& x_P,
        int N,
        double rngVar,
        double brngVar) {

    int numLandmarks_detected = (int)Id.size();

    // z: each landmark measurement as [range, bearing]
    std::vector<Eigen::Vector2d> z(numLandmarks_detected);

    // Preprocess measurements
    // Original logic: range = atan2(y, x), bearing = sqrt(x² + y²)
    for (int n = 0; n < numLandmarks_detected; ++n) {
        Eigen::Vector2d landSE2 = tagPos[n];
        double range = std::atan2(landSE2(1), landSE2(0));
        double brng = std::sqrt(landSE2(0)*landSE2(0) + landSE2(1)*landSE2(1));
        z[n](0) = range;
        z[n](1) = brng;
    }

    // Extract landmark positions (egoPos)
    std::vector<Eigen::Vector2d> egoPos(numLandmarks_detected);
    for (int n = 0; n < numLandmarks_detected; ++n) {
        int tag_number_detected = Id[n];
        auto it = savedLandmarks.find(tag_number_detected);
        if (it != savedLandmarks.end()) {
            const gtsam::Point2& Lpos = it->second;
            egoPos[n](0) = Lpos.x();
            egoPos[n](1) = Lpos.y();
        } else {
            egoPos[n](0) = std::numeric_limits<double>::quiet_NaN();
            egoPos[n](1) = std::numeric_limits<double>::quiet_NaN();
        }
    }

    // Randomizer setup
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> nd(0.0, 1.0);
    std::uniform_real_distribution<double> ud(0.0, 1.0);

    // Temporary updates and weights
    std::vector<Eigen::Vector3d> x_P_update(N);
    std::vector<double> P_w(N);

    // Particle Filter Update
    for (int i = 0; i < N; ++i) {
        // State prediction
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
        Q(0,0) = 1.0;
        Q(1,1) = 1.0;
        Q(2,2) = 0.4;

        Eigen::Vector3d noise(nd(gen), nd(gen), nd(gen));
        Eigen::Vector3d xplus = A * x_P[i] + Q * noise;
        xplus(2) = wrapToPi(xplus(2));
        x_P_update[i] = xplus;

        // Measurement update
        double weight = 0.0;
        for (int l = 0; l < numLandmarks_detected; ++l) {
            if (std::isnan(egoPos[l](0)) || std::isnan(egoPos[l](1))) {
                // Landmark unknown, skip
                continue;
            }

            Eigen::Vector2d targetPos = xplus.head<2>();
            Eigen::Vector2d error = egoPos[l] - targetPos;
            double range_pred = error.norm();
            double bearing_pred = std::atan2(error(1), error(0)) - xplus(2);
            bearing_pred = std::atan2(std::sin(bearing_pred), std::cos(bearing_pred));

            double bearingError = z[l](1) - bearing_pred;
            if (bearingError < -M_PI) bearingError += 2*M_PI;
            else if (bearingError > M_PI) bearingError -= 2*M_PI;

            double rangeError = range_pred - z[l](0);
            double sr = std::sqrt(rngVar);
            double sb = std::sqrt(brngVar);

            double rl = std::exp(-0.5 * std::pow(rangeError/sr,2)) / (sr * std::sqrt(2*M_PI));
            double bl = std::exp(-0.5 * std::pow(bearingError/sb,2)) / (sb * std::sqrt(2*M_PI));

            weight += (rl * bl);
        }
        P_w[i] = weight;
    }

    // Normalize weights
    double sumW = 0.0;
    for (double w : P_w) sumW += w;

    if (sumW == 0) {
        for (int i = 0; i < N; ++i)
            P_w[i] = 1.0 / N;
    } else {
        for (int i = 0; i < N; ++i)
            P_w[i] /= sumW;
    }

    // Resampling
    std::vector<double> cumsum(N);
    cumsum[0] = P_w[0];
    for (int i = 1; i < N; ++i) {
        cumsum[i] = cumsum[i-1] + P_w[i];
    }

    std::vector<Eigen::Vector3d> x_P_new(N);
    for (int i = 0; i < N; ++i) {
        double r = ud(gen);
        auto it = std::lower_bound(cumsum.begin(), cumsum.end(), r);
        int idx = (int)std::distance(cumsum.begin(), it);
        if (idx >= N) idx = N-1;
        x_P_new[i] = x_P_update[idx];
    }

    // x_est calculation removed since it caused reference issues and wasn't in snippet
    // If needed, compute x_est here by averaging particles.

    // Update x_P reference
    x_P = x_P_new;
    return x_P_new; // return by value is safe since x_P_new is a local variable
}

std::vector<Eigen::Vector3d> initParticles(int Ninit) {
    // Define grid boundaries
    double xmin = -3.0, xmax = 6.0;
    double ymin = -25.0, ymax = 145.0;

    // Compute grid dimensions
    double ratio = (xmax - xmin) / (ymax - ymin);
    int Nx = static_cast<int>(std::round(ratio * std::sqrt(Ninit)));
    int Ny = static_cast<int>(std::round(double(Ninit) / Nx));
    int N = Nx * Ny;

    // Generate linear spacing
    Eigen::VectorXd X_lin = Eigen::VectorXd::LinSpaced(Nx, xmin, xmax);
    Eigen::VectorXd Y_lin = Eigen::VectorXd::LinSpaced(Ny, ymin, ymax);

    // Random orientation generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> d_theta(0.0, M_PI);

    // Prepare vector to store particles
    std::vector<Eigen::Vector3d> particles;
    particles.reserve(N);

    for (int i = 0; i < Ny; ++i) {
        for (int j = 0; j < Nx; ++j) {
            Eigen::Vector3d particle;
            particle(0) = X_lin(j);
            particle(1) = Y_lin(i);
            double new_theta = d_theta(gen);
            particle(2) = wrapToPi(new_theta); // Assuming wrapToPi is defined

            particles.push_back(particle);
        }
    }

    return particles;
}

std::vector<Eigen::Vector3d> initParticlesFromFirstTag(
        const std::vector<int>& Id,
        const std::vector<Eigen::Vector2d>& tagPos,
        const std::map<int, gtsam::Point2>& savedLandmarks,
        int Ninit) {

    // If no tags, return empty or handle as needed
    if (Id.empty()) {
        return std::vector<Eigen::Vector3d>();
    }

    // Randomly pick a tag from the detected list
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist_idx(0, (int)Id.size() - 1);

    int random_index = dist_idx(gen);
    int tag_id = Id[random_index];
    Eigen::Vector2d landSE2 = tagPos[random_index];

    // Attempt to find the chosen tag in the saved landmarks
    auto it = savedLandmarks.find(tag_id);
    if (it == savedLandmarks.end()) {
        // If the tag isn't found in the landmark table, return empty or fallback to another strategy
        return std::vector<Eigen::Vector3d>();
    }

    gtsam::Point2 tag_global = it->second;

    // Compute range and bearing from measurement
    double range = std::sqrt(landSE2(0)*landSE2(0) + landSE2(1)*landSE2(1));
    double bearing = std::atan2(landSE2(1), landSE2(0));

    // Estimate robot position:
    double robot_x = tag_global.x() - range * std::cos(bearing);
    double robot_y = tag_global.y() - range * std::sin(bearing);

    // Standard deviations for spreading out particles around the estimated location
    double stddev_pos = 5.0;      // meters, adjust as needed
    double stddev_theta = M_PI/4; // radians, adjust as needed

    // Random distributions
    std::normal_distribution<double> dist_x(robot_x, stddev_pos);
    std::normal_distribution<double> dist_y(robot_y, stddev_pos);
    std::normal_distribution<double> dist_theta(0.0, stddev_theta);

    // Generate particles
    std::vector<Eigen::Vector3d> particles;
    particles.reserve(Ninit);

    for (int i = 0; i < Ninit; ++i) {
        Eigen::Vector3d particle;
        particle(0) = dist_x(gen);
        particle(1) = dist_y(gen);
        double new_theta = dist_theta(gen);
        particle(2) = wrapToPi(new_theta);  // Ensure wrapToPi is defined somewhere

        particles.push_back(particle);
    }

    return particles;
}

} // namespace aprilslamcpp


