// publishing_utils.cpp

#include "publishing_utils.h"

namespace aprilslam {

// Utility function to normalize an angle to the range [-pi, pi]
double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    return angle - M_PI;
}   

gtsam::Pose3 relPoseFG(const gtsam::Pose3& lastPoseSE3, const gtsam::Pose3& PoseSE3) {
    // Compute relative transformation in the global frame
    gtsam::Pose3 relativePose = lastPoseSE3.between(PoseSE3);
    return relativePose;
} 

void publishLandmarks(ros::Publisher& landmark_pub, const std::map<int, gtsam::Point3>& landmarks, const std::string& frame_id) {
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
        marker.pose.position.z = landmark.second.z();  
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
        text_marker.pose.position.z = landmark.second.z() + 0.5;  
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
    path.poses.clear();

    for (int i = 1; i <= max_index; i++) {
        gtsam::Symbol sym('X', i);
        if (result.exists(sym)) {
            gtsam::Pose3 pose = result.at<gtsam::Pose3>(sym);  

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = pose.translation().x();
            pose_msg.pose.position.y = pose.translation().y();
            pose_msg.pose.position.z = pose.translation().z();

            // Convert rotation to quaternion
            gtsam::Quaternion gtsam_quat = pose.rotation().toQuaternion();
            pose_msg.pose.orientation.w = gtsam_quat.w();
            pose_msg.pose.orientation.x = gtsam_quat.x();
            pose_msg.pose.orientation.y = gtsam_quat.y();
            pose_msg.pose.orientation.z = gtsam_quat.z();

            path.poses.push_back(pose_msg);
        }
    }

    path_pub.publish(path);
}

void publishMapToOdomTF(tf2_ros::TransformBroadcaster& tf_broadcaster, 
                        const gtsam::Values& result, int latest_index, 
                        const gtsam::Pose3& poseSE3,  
                        const std::string& map_frame, const std::string& odom_frame, const std::string& base_link_frame) {
    gtsam::Symbol sym('X', latest_index);

    if (result.exists(sym)) {
        // Extract the pose from the SLAM result (map -> base_link)
        gtsam::Pose3 slamPose = result.at<gtsam::Pose3>(sym);

        // Create the transform for map -> base_link
        tf2::Transform map_to_base_link;
        gtsam::Quaternion slam_quat = slamPose.rotation().toQuaternion();
        tf2::Quaternion tf_slam_quat(slam_quat.x(), slam_quat.y(), slam_quat.z(), slam_quat.w());
        map_to_base_link.setOrigin(tf2::Vector3(
            slamPose.translation().x(), 
            slamPose.translation().y(), 
            slamPose.translation().z()));
        map_to_base_link.setRotation(tf_slam_quat);

        // Convert odometry pose (odom -> base_link) into a transform
        tf2::Transform odom_to_base_link;
        gtsam::Quaternion odom_quat = poseSE3.rotation().toQuaternion();
        tf2::Quaternion tf_odom_quat(odom_quat.x(), odom_quat.y(), odom_quat.z(), odom_quat.w());
        odom_to_base_link.setOrigin(tf2::Vector3(
            poseSE3.translation().x(), 
            poseSE3.translation().y(), 
            poseSE3.translation().z()));
        odom_to_base_link.setRotation(tf_odom_quat);

        // Compute the map -> odom transform
        tf2::Transform map_to_odom = map_to_base_link * odom_to_base_link.inverse();

        // Publish the map -> odom transform
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = map_frame;
        transform_stamped.child_frame_id = odom_frame;

        transform_stamped.transform.translation.x = map_to_odom.getOrigin().x();
        transform_stamped.transform.translation.y = map_to_odom.getOrigin().y();
        transform_stamped.transform.translation.z = map_to_odom.getOrigin().z();

        transform_stamped.transform.rotation = tf2::toMsg(map_to_odom.getRotation());

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

    if (!Estimates_visulisation.exists(sym)) {
        ROS_WARN("publishRefinedOdom: Pose not found in Estimates_visulisation for X%d", index_of_pose);
        return;
    }

    // Retrieve the GTSAM Pose3
    gtsam::Pose3 refinedPose = Estimates_visulisation.at<gtsam::Pose3>(sym);

    // Convert to quaternion
    gtsam::Quaternion gtsam_quat = refinedPose.rotation().toQuaternion();

    // Fill nav_msgs::Odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id  = base_link_frame;

    // Position
    odom_msg.pose.pose.position.x = refinedPose.translation().x();
    odom_msg.pose.pose.position.y = refinedPose.translation().y();
    odom_msg.pose.pose.position.z = refinedPose.translation().z();

    // Orientation
    odom_msg.pose.pose.orientation.w = gtsam_quat.w();
    odom_msg.pose.pose.orientation.x = gtsam_quat.x();
    odom_msg.pose.pose.orientation.y = gtsam_quat.y();
    odom_msg.pose.pose.orientation.z = gtsam_quat.z();

    odom_pub.publish(odom_msg);

    // Save to CSV with roll, pitch, yaw
    double time = stamp.toSec();
    static int seq = 0;
    refined_odom_csv << std::fixed << std::setprecision(6)
                    << time << "\t"
                    << seq++ << "\t"
                    << time << "\t"
                    << odom_frame << "\t"
                    << base_link_frame << "\t"
                    << refinedPose.translation().x() << "\t"
                    << refinedPose.translation().y() << "\t"
                    << refinedPose.translation().z() << "\t"
                    << gtsam_quat.x() << "\t"
                    << gtsam_quat.y() << "\t"
                    << gtsam_quat.z() << "\t"
                    << gtsam_quat.w() << "\t"
                    // 36 pose covariances (all zero)
                    << "0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t"
                    << "0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t"
                    << "0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t"
                    // 6 twist linear/angular (all zero)
                    << "0\t0\t0\t0\t0\t0\t"
                    // 36 twist covariances (all zero)
                    << "0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t"
                    << "0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t"
                    << "0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\t0\n";
}

void saveLandmarksToCSV(const std::map<int, gtsam::Point3>& landmarks, const std::string& filename) {
    std::ofstream file;
    file.open(filename, std::ios::out);

    if (!file) {
        std::cerr << "Failed to open the file!" << std::endl;
        return;
    }
    
    // Write the header line with z coordinate
    file << "id,x,y,z\n";
    
    for (const auto& landmark : landmarks) {
        int id = landmark.first;
        gtsam::Point3 point = landmark.second;
        file << id << "," << point.x() << "," << point.y() << "," << point.z() << "\n";
    }

    file.close();
}

std::map<int, gtsam::Point3> loadLandmarksFromCSV(const std::string& filename) {
    std::map<int, gtsam::Point3> landmarks;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open the file!" << std::endl;
        return landmarks;
    }

    std::string line;

    // Skip the header line
    if (std::getline(file, line)) {
        // Header line skipped
    }

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string id_str, x_str, y_str, z_str;
        if (std::getline(ss, id_str, ',') && 
            std::getline(ss, x_str, ',') && 
            std::getline(ss, y_str, ',') &&
            std::getline(ss, z_str, ',')) {
            try {
                int id = std::stoi(id_str);
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                double z = std::stod(z_str);
                landmarks[id] = gtsam::Point3(x, y, z);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
            }
        }
    }

    file.close();
    return landmarks;
}

void processDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr& cam_msg, 
                       const gtsam::Pose3& pose_cam_baselink,  // Now full Pose3
                       std::vector<int>& Ids, 
                       std::vector<Eigen::Vector3d>& tagPoss) {  // Now 3D
    if (cam_msg) {
        for (const auto& detection : cam_msg->detections) {
            Ids.push_back(detection.id[0]);
            
            // Get detection position in camera frame
            // AprilTag detection: x is right, y is down, z is forward
            gtsam::Point3 tag_in_camera(
                detection.pose.pose.pose.position.x,
                detection.pose.pose.pose.position.y,
                detection.pose.pose.pose.position.z
            );
            
            // Transform to base_link frame
            gtsam::Point3 tag_in_baselink = pose_cam_baselink.transformFrom(tag_in_camera);
            
            // Store as Eigen::Vector3d
            Eigen::Vector3d tagPos;
            tagPos << tag_in_baselink.x(), tag_in_baselink.y(), tag_in_baselink.z();
            tagPoss.push_back(tagPos);
        }
    }
}

std::pair<std::vector<int>, std::vector<Eigen::Vector3d>> getCamDetections(
    const std::vector<CameraInfo>& camera_infos,
    const std::map<std::string, apriltag_ros::AprilTagDetectionArray::ConstPtr>& camera_detections) {

    std::vector<int> Ids;
    std::vector<Eigen::Vector3d> tagPoss;

    for (const auto& cam : camera_infos) {
        auto it = camera_detections.find(cam.name);
        if (it == camera_detections.end()) {
            continue;
        }

        processDetections(it->second, cam.transform, Ids, tagPoss);
    }
    return std::make_pair(Ids, tagPoss);
}

void visualizeLoopClosure(ros::Publisher& lc_pub, 
                         const gtsam::Pose3& currentPose, 
                         const gtsam::Pose3& keyframePose, 
                         int currentPoseIndex, 
                         const std::string& frame_id) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "loop_closure_line";
    line_marker.id = currentPoseIndex;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;

    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    line_marker.scale.x = 0.05;

    geometry_msgs::Point p1, p2;
    p1.x = keyframePose.translation().x();
    p1.y = keyframePose.translation().y();
    p1.z = keyframePose.translation().z();

    p2.x = currentPose.translation().x();
    p2.y = currentPose.translation().y();
    p2.z = currentPose.translation().z();

    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    lc_pub.publish(line_marker);
}

std::vector<Eigen::Matrix<double, 6, 1>> particleFilter(
        const std::vector<int>& Id,
        const std::vector<Eigen::Vector3d>& tagPos,
        const std::map<int, gtsam::Point3>& savedLandmarks,
        std::vector<Eigen::Matrix<double, 6, 1>>& x_P,
        int N,
        double rngVar,
        double brngVar) {

    int numLandmarks_detected = (int)Id.size();

    // z: each landmark measurement as [bearing, elevation, range]
    std::vector<Eigen::Vector3d> z(numLandmarks_detected);

    // Preprocess measurements
    for (int n = 0; n < numLandmarks_detected; ++n) {
        Eigen::Vector3d landSE3 = tagPos[n];
        double range = landSE3.norm();
        double bearing = std::atan2(landSE3(1), landSE3(0));
        double elevation = std::atan2(landSE3(2), std::sqrt(landSE3(0)*landSE3(0) + landSE3(1)*landSE3(1)));
        z[n](0) = bearing;
        z[n](1) = elevation;
        z[n](2) = range;
    }

    // Extract landmark positions
    std::vector<Eigen::Vector3d> egoPos(numLandmarks_detected);
    for (int n = 0; n < numLandmarks_detected; ++n) {
        int tag_number_detected = Id[n];
        auto it = savedLandmarks.find(tag_number_detected);
        if (it != savedLandmarks.end()) {
            const gtsam::Point3& Lpos = it->second;
            egoPos[n](0) = Lpos.x();
            egoPos[n](1) = Lpos.y();
            egoPos[n](2) = Lpos.z();
        } else {
            egoPos[n].setConstant(std::numeric_limits<double>::quiet_NaN());
        }
    }

    // Randomizer setup
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> nd(0.0, 1.0);
    std::uniform_real_distribution<double> ud(0.0, 1.0);

    // Temporary updates and weights
    std::vector<Eigen::Matrix<double, 6, 1>> x_P_update(N);
    std::vector<double> P_w(N);

    // Particle Filter Update
    for (int i = 0; i < N; ++i) {
        // State prediction with 6DOF
        Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        Q(0,0) = 1.0;  // x
        Q(1,1) = 1.0;  // y
        Q(2,2) = 1.0;  // z
        Q(3,3) = 0.4;  // roll
        Q(4,4) = 0.4;  // pitch
        Q(5,5) = 0.4;  // yaw

        Eigen::Matrix<double, 6, 1> noise;
        for (int j = 0; j < 6; ++j) noise(j) = nd(gen);
        
        Eigen::Matrix<double, 6, 1> xplus = A * x_P[i] + Q * noise;
        
        // Wrap angles
        xplus(3) = wrapToPi(xplus(3));  // roll
        xplus(4) = wrapToPi(xplus(4));  // pitch
        xplus(5) = wrapToPi(xplus(5));  // yaw
        
        x_P_update[i] = xplus;

        // Measurement update
        double weight = 0.0;
        for (int l = 0; l < numLandmarks_detected; ++l) {
            if (std::isnan(egoPos[l](0))) continue;

            Eigen::Vector3d targetPos = xplus.head<3>();
            Eigen::Vector3d error = egoPos[l] - targetPos;
            double range_pred = error.norm();
            double bearing_pred = std::atan2(error(1), error(0)) - xplus(5);  // relative to yaw
            double elevation_pred = std::atan2(error(2), std::sqrt(error(0)*error(0) + error(1)*error(1)));
            
            bearing_pred = wrapToPi(bearing_pred);

            double bearingError = wrapToPi(z[l](0) - bearing_pred);
            double elevationError = wrapToPi(z[l](1) - elevation_pred);
            double rangeError = range_pred - z[l](2);
            
            double sr = std::sqrt(rngVar);
            double sb = std::sqrt(brngVar);

            double rl = std::exp(-0.5 * std::pow(rangeError/sr,2)) / (sr * std::sqrt(2*M_PI));
            double bl = std::exp(-0.5 * std::pow(bearingError/sb,2)) / (sb * std::sqrt(2*M_PI));
            double el = std::exp(-0.5 * std::pow(elevationError/sb,2)) / (sb * std::sqrt(2*M_PI));

            weight += (rl * bl * el);
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

    std::vector<Eigen::Matrix<double, 6, 1>> x_P_new(N);
    for (int i = 0; i < N; ++i) {
        double r = ud(gen);
        auto it = std::lower_bound(cumsum.begin(), cumsum.end(), r);
        int idx = (int)std::distance(cumsum.begin(), it);
        if (idx >= N) idx = N-1;
        x_P_new[i] = x_P_update[idx];
    }

    x_P = x_P_new;
    return x_P_new;
}

std::vector<Eigen::Matrix<double, 6, 1>> initParticles(int Ninit) {
    // Define grid boundaries for x, y, z
    double xmin = -3.0, xmax = 6.0;
    double ymin = -25.0, ymax = 145.0;
    double zmin = 0.0, zmax = 2.0;  // Assuming robot stays near ground

    // Compute grid dimensions (simplified 2D grid for x,y)
    double ratio = (xmax - xmin) / (ymax - ymin);
    int Nx = static_cast<int>(std::round(ratio * std::sqrt(Ninit)));
    int Ny = static_cast<int>(std::round(double(Ninit) / Nx));
    int N = Nx * Ny;

    // Generate linear spacing
    Eigen::VectorXd X_lin = Eigen::VectorXd::LinSpaced(Nx, xmin, xmax);
    Eigen::VectorXd Y_lin = Eigen::VectorXd::LinSpaced(Ny, ymin, ymax);

    // Random generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> d_z(zmin, zmax);
    std::uniform_real_distribution<double> d_theta(-M_PI, M_PI);

    // Prepare vector to store 6DOF particles
    std::vector<Eigen::Matrix<double, 6, 1>> particles;
    particles.reserve(N);

    for (int i = 0; i < Ny; ++i) {
        for (int j = 0; j < Nx; ++j) {
            Eigen::Matrix<double, 6, 1> particle;
            particle(0) = X_lin(j);      // x
            particle(1) = Y_lin(i);      // y
            particle(2) = d_z(gen);      // z
            particle(3) = 0.0;           // roll (assuming mostly upright)
            particle(4) = 0.0;           // pitch (assuming mostly upright)
            particle(5) = d_theta(gen);  // yaw (random orientation)

            particles.push_back(particle);
        }
    }

    return particles;
}

std::vector<Eigen::Matrix<double, 6, 1>> initParticlesFromFirstTag(
        const std::vector<int>& Id,
        const std::vector<Eigen::Vector3d>& tagPos,  // Now 3D
        const std::map<int, gtsam::Point3>& savedLandmarks,  // Now 3D
        int Ninit) {

    if (Id.empty()) {
        return std::vector<Eigen::Matrix<double, 6, 1>>();
    }

    // Randomly pick a tag
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist_idx(0, (int)Id.size() - 1);

    int random_index = dist_idx(gen);
    int tag_id = Id[random_index];
    Eigen::Vector3d landSE3 = tagPos[random_index];

    auto it = savedLandmarks.find(tag_id);
    if (it == savedLandmarks.end()) {
        return std::vector<Eigen::Matrix<double, 6, 1>>();
    }

    gtsam::Point3 tag_global = it->second;

    // Compute range and bearing from measurement
    double range = landSE3.norm();
    double bearing = std::atan2(landSE3(1), landSE3(0));
    double elevation = std::atan2(landSE3(2), std::sqrt(landSE3(0)*landSE3(0) + landSE3(1)*landSE3(1)));

    // Estimate robot position (simplified, assuming robot is at ground level)
    double robot_x = tag_global.x() - range * std::cos(elevation) * std::cos(bearing);
    double robot_y = tag_global.y() - range * std::cos(elevation) * std::sin(bearing);
    double robot_z = tag_global.z() - range * std::sin(elevation);

    // Standard deviations
    double stddev_pos = 5.0;
    double stddev_theta = M_PI/4;

    // Random distributions
    std::normal_distribution<double> dist_x(robot_x, stddev_pos);
    std::normal_distribution<double> dist_y(robot_y, stddev_pos);
    std::normal_distribution<double> dist_z(robot_z, stddev_pos);
    std::normal_distribution<double> dist_theta(0.0, stddev_theta);

    // Generate 6DOF particles
    std::vector<Eigen::Matrix<double, 6, 1>> particles;
    particles.reserve(Ninit);

    for (int i = 0; i < Ninit; ++i) {
        Eigen::Matrix<double, 6, 1> particle;
        particle(0) = dist_x(gen);           // x
        particle(1) = dist_y(gen);           // y
        particle(2) = dist_z(gen);           // z
        particle(3) = dist_theta(gen) * 0.1; // roll (small)
        particle(4) = dist_theta(gen) * 0.1; // pitch (small)
        particle(5) = dist_theta(gen);       // yaw
        
        // Wrap angles
        particle(3) = wrapToPi(particle(3));
        particle(4) = wrapToPi(particle(4));
        particle(5) = wrapToPi(particle(5));

        particles.push_back(particle);
    }

    return particles;
}

} // namespace aprilslam