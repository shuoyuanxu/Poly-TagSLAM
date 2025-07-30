# Poly-TagSLAM: Low-Cost Robot Localisation and Mapping in Polytunnels with Fiducial Markers

## Overview

This project implements a ROS-based Simultaneous Localization and Mapping (SLAM) system that uses AprilTags as landmarks and GTSAM for graph-based optimization. The system estimates the robot’s trajectory and maps landmarks using multiple cameras. To balance computational cost and localization and mapping accuracy, we employ two similar yet distinct strategies: one for landmark mapping and another for localization.

### Calibration

Calibration is performed using SAM (Smoothing and Mapping), where all poses, landmarks, and observations are incorporated into a factor graph. A single optimization process is then carried out to achieve a globally optimal solution.

### Localization

Localization utilizes prior knowledge of relatively accurate landmark positions. It is worth noting that the localization algorithm, **if disabled 'using prior landmarks'**, is capable of performing **real time SLAM**. Various optimization techniques and strategies can be employed to balance accuracy and efficiency, which will be discussed in detail in the following sections. 
Run it with this when dealing with recorded bag
```roslaunch aprilslamcpp run_localisation.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)```

## Table of Contents

1. [Installation](#1-installation)
   1. [Common Errors and Fixes](#1-common-errors-and-fixes)
2. [Core Components](#2-core-components)
   1. [AprilTag Detection](#1-apriltag-detection)
   2. [Odometry](#2-odometry)
   3. [GTSAM Optimization](#3-gtsam-optimization)
3. [Mathematical Foundation](#3-mathematical-foundation)
   1. [Assumptions](#1-assumptions)
   2. [Graph-Based SLAM](#2-graph-based-slam)
4. [Key Functions and Code Structure](#4-key-functions-and-code-structure)
   1. [relPoseFG](#1-relposefg)
   2. [Poly-TagSLAM Node Initialization](#2-aprilslam-node-initialization)
   3. [Optimization](#3-optimization)
   4. [Odometry Processing](#4-odometry-processing)
   5. [Calibration](#5-calibration)
   6. [Localization](#6-localization)
   7. [Keyframe](#7-Keyframe)
   8. [Stationary Condition](#8-stationary-condition)
5. [How to Run](#5-how-to-run)
6. [Future Work](#6-future-work)

---

## **1. Installation**

Ensure that the following dependencies are installed:

- **ROS** (Robot Operating System) - Noetic or Melodic
- **GTSAM** (Georgia Tech Smoothing And Mapping library)
- **Eigen** - For matrix computations

Download the code and put it into your catkin workspace, then run catkin_make to run it.

### **1. Common Errors and Fixes:** 

1. error: ‘optional’ in namespace ‘std’ does not name a template type
	std::optional is c++17 only, add this line to your cmake file:

```set(CMAKE_CXX_STANDARD 17)```

2. error: static assertion failed: Error: GTSAM was built against a different version of Eigen

	need to rebuild:
```cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..```

3. error: gtsam_wrapper.mexa64 unable to find libgtsam.so.4

The default search directory of gtsam_wrapper.mexa64 is /usr/lib/ yet all related libs are installed to /usr/local/lib. All corresponding files (the ones mentioned in Matlab error message) needs to be copied to /usr/lib/

```
sudo cp /usr/local/lib/libgtsam.so.4 /usr/lib/
sudo cp /usr/local/lib/libgtsam.so.4 /usr/lib/
```
		
4. Matlab toolbox: cmake -D GTSAM_INSTALL_MATLAB_TOOLBOX=1 ..
	copy the toolbox from usr/local/ to work directory, then add the folder to path in Matlab

5. To hide "Warning: TF_REPEATED_DATA ignoring data with redundant timestamp" error in terminal
```
source devel/setup.bash
rosrun aprilslamcpp aprilslamcpp 2> >(grep -v TF_REPEATED_DATA buffer_core)
rosbag play --pause rerecord_3_HDL.bag
```
6. Cant link to GTSAM, try to use the following 2 approach, **only one may work**
    (1) Give your CMakeLists.txt the absolute directory of GTSAM
    ```
    set(GTSAM_DIR "/home/.../GTSAM/build")
    ```
    
    (2) Ask it to find it 
    ```
    find_package(GTSAM REQUIRED)
    ```

---

## **2. Core Components**

### **1. AprilTag Detection**

The system uses AprilTags for robust feature detection. Three camera topics (`mCam`, `rCam`, `lCam`) are subscribed to detect AprilTags in their respective fields of view.

### **2. Odometry**

The system utilizes odometry data from various sensors, including wheel encoders, IMU, GPS, or LiDAR, to compute relative poses for constructing the factor graph. For optimal results, we recommend using accurate odometry sources, such as LiDAR, during the calibration phase. As for localization, other odometry sources can be effectively, as long as the sensor provides reasonably reliable and consistent data. This flexibility allows the system to accommodate different sensor configurations, making it adaptable to various environments and use cases.

### **3. Graph Building**

Formulate a factor graph with robot poses, landmarks, priors and observations.

### **4. GTSAM Optimization**

GTSAM performs factor graph-based optimization using:

- Between factors between poses (from Odometry) 
- Bearing-Range factors for landmarks
- Priors of initial pose and landmarks

---

## **3. Mathematical Foundation**

### **1. Assumptions**

The algorithm operates on a 2D plane, assuming that vertical differences do not impact performance. The robot's pose is represented using `gtsam::Pose2`, which includes:

- `(x, y)`: The robot's position in the 2D plan
- `θ`: The robot's orientation
- The function `relPoseFG` calculates the relative pose between two `Pose2` objects, returning the relative distance and adjusting for orientation. It assumes that **the robot cannot move sideways**.

### **2. Graph-Based SLAM**

The system applies odometry constraints (between consecutive poses) and bearing-range constraints (between poses and landmarks). Both SAM and ISAM2 optimizers can be utilized. Here’s a simple comparison between these two optimizers:

| Feature                | SAM (Batch Optimization)                     | iSAM2 (Incremental Optimization)        |
|------------------------|----------------------------------------------|-----------------------------------------|
| **Optimization Type**   | Batch (whole graph at once)                  | Incremental (updates relevant portions) |
| **Computation Time**    | High (grows with graph size)                 | Low (optimized for real-time updates)   |
| **Real-time Suitability**| No                                           | Yes                                     |
| **Memory Usage**        | High (stores entire graph)                   | Lower (incremental updates)             |
| **Algorithm**           | Batch least squares (e.g., Levenberg-Marquardt, Gauss-Newton) | Incremental smoothing with selective relinearization |
| **Use Case**            | Best for offline or small-scale optimization | Ideal for real-time applications like SLAM |

**SAM**: Optimizing the whole graph

`min_{x, l} ∑ || z_i - h(x_i, l_i) ||^2_Σ_i`

**ISAM2**: Incremental Update:

`x_{t+1} = x_t + \Delta x`

- Add New Factors: New factors (e.g., new odometry or landmark observations) are added to the factor graph when a new measurement is received.

- Relinearization: Only a subset of variables is relinearized, meaning that only variables affected (determined by ISAM2) by the new measurements are recalculated. This significantly reduces computational complexity compared to recalculating all variables.

- Bayes Tree Update: iSAM2 uses a Bayes Tree to represent the factor graph and efficiently update the system. The Bayes Tree organizes the factor graph into cliques, allowing for fast updates when new measurements are added.

---

## **4. Key Functions and Code Structure**

### **1. relPoseFG**

Our odometry does not give relative pose directly, instead it gives pose estimations. Therefore, a function computes the relative pose between two `gtsam::Pose2` objects is required:

- **Input**: Two `Pose2` objects (`lastPoseSE2` and `PoseSE2`)
- **Output**: Relative `Pose2` that represents the robot's motion from `lastPoseSE2` to `PoseSE2`

```cpp
gtsam::Pose2 relPoseFG(const gtsam::Pose2& lastPoseSE2, const gtsam::Pose2& PoseSE2) {
    double dx = PoseSE2.x() - lastPoseSE2.x();
    double dy = PoseSE2.y() - lastPoseSE2.y();
    double dtheta = wrapToPi(PoseSE2.theta() - lastPoseSE2.theta());

    double theta = lastPoseSE2.theta();
    double dx_body = std::cos(theta) * dx + std::sin(theta) * dy;
    return gtsam::Pose2(dx_body, dy_body, dtheta);
}
```

### **2. Poly-TagSLAM Node Initialization**

In the constructor of `aprilslamcpp`, multiple parameters are read from ROS parameters to configure the system, such as:

- Noise models
- Thresholds for stationary detection
- Paths for saving/loading landmarks
- Subscriber and publisher topics

```cpp
aprilslamcpp::aprilslamcpp(ros::NodeHandle node_handle)
    : nh_(node_handle), tf_listener_(tf_buffer_) {
    nh_.getParam("odom_topic", odom_topic);
    nh_.getParam("trajectory_topic", trajectory_topic);
    nh_.getParam("frame_id", frame_id);

    // Noise models
    nh_.getParam("noise_models/odometry", odometry_noise);
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << odometry_noise[0], odometry_noise[1], odometry_noise[2]).finished());

    // Subscribers and Publishers
    mCam_subscriber = nh_.subscribe(mCam_topic, 1000, &aprilslamcpp::mCamCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1, true);
}
```

### **3. Optimization**

The `SAMOptimise` function performs batch optimization of the factor graph using GTSAM’s Levenberg-Marquardt optimizer. After the optimization, the input estimates are updated and will be used for the next iteration. Due to the fact that SAM is normally computationally heavy, we recommand to use our pruning function 'pruneGraphByPoseCount' to manage the total size of the factor graph when trying to run it in real time.

```cpp
void aprilslamcpp::SAMOptimise() {
    gtsam::Levenberg-MarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
    keyframeEstimates_ = batchOptimizer.optimize();

    // Prune the graph based on the number of poses
    if (useprunebysize) {
    pruneGraphByPoseCount(maxfactors);
    }
}
```
The `ISAM2Optimise` function performs incremental optimization of the factor graph using GTSAM’s iSAM2 optimizer. Estimates on all poses and landmarks plus the entire graph is stored in a 'gtsam::ISAM2 isam_' variable, which can only be updated but not trimed. Therefore, after every ISAM2 iteration, historic estimates and graph needs to be cleaned to avoid repetitive data appeared in isam_ variable. Since ISAM2 is not computationally heavy, no graph management approach is required

```cpp
void aprilslamcpp::ISAM2Optimise() {    
    // Perform batch optimization once if required
    if (batchOptimisation_) {
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(keyframeGraph_, keyframeEstimates_);
        keyframeEstimates_ = batchOptimizer.optimize();
        batchOptimisation_ = false; // Only do this once
    }

    // Incrementally update the iSAM2 instance with new measurements
    isam_.update(keyframeGraph_, keyframeEstimates_);

    // Clear the graph and estimates for the next iteration
    keyframeEstimates_.clear();
    keyframeGraph_.resize(0);
}
```

### **4. Odometry Processing**

This function is where the factor graph is built and corresponding estimates are inserted whenever a new steam of odometry data comes in:

```cpp
    odom_sub_ = nh_.subscribe(odom_topic, 10, &aprilslamcpp::addOdomFactor, this);
```

Once the odom are received, the soonest tag detections will be used to formulate the factor graph, once this function is finished running with the current odometry, a estimates (`keyframeEstimates_`) containing all the pos and landmarks, as well as a factor graph (`keyframeGraph_`) containing pose prior, landmark prior (`PriorFactor`), lanmark observation (`BearingRangeFactor`), and pos to pos factor (`BetweenFactor`) are formulated

```cpp
void aprilslam::aprilslamcpp::addOdomFactor(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_time = ros::Time::now().toSec();
    ros::WallTime start_loop, end_loop; // Declare variables to hold start and end times
    double elapsed;

    // Convert the incoming odometry message to a simpler (x, y, theta) format using a previously defined method
    gtsam::Pose2 poseSE2 = translateOdomMsg(msg);

    // Check if the movement exceeds the thresholds
    if (!movementExceedsThreshold(poseSE2)) return;

    index_of_pose += 1; // Increment the pose index for each new odometry message

    // Store the initial pose for relative calculations
    if (index_of_pose == 2) initializeFirstPose(poseSE2);

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
        // ROS_INFO("transform total: %f seconds", elapsed);
        lastPoseSE2_ = poseSE2;
        start_loop = ros::WallTime::now();
        ROS_INFO("number of timesteps: %d", index_of_pose);
        if (index_of_pose % 1 == 0) {
            if (useisam2) {ISAM2Optimise();}
            else {SAMOptimise();}
            end_loop = ros::WallTime::now();
            elapsed = (end_loop - start_loop).toSec();
            ROS_INFO("optimisation: %f seconds", elapsed);
        }
    
    Key_previous_pos = predictedPose;
    previousKeyframeSymbol = gtsam::Symbol('X', index_of_pose);  
    checkLoopClosure(detectedLandmarksCurrentPos);
    generate2bePublished();
    }
    // Use Odometry for pose estimation when not a keyframe, landmarks not updated
    else{
        updateOdometryPose(poseSE2);  // Update pose without adding a keyframe
    }
    // Publish path, landmarks, and tf for visulisation
    aprilslam::publishPath(path_pub_, Estimates_visulisation, index_of_pose, frame_id);
    aprilslam::publishOdometryTrajectory(odom_traj_pub_, tf_broadcaster, Estimates_visulisation, index_of_pose, frame_id, ud_frame);
}
```
It is worth noting that `gtsam::Vector error = factor.unwhitenedError(landmarkEstimates);` this line is used for identifying whjether a landmark is too much of an outliner to be added to the factor graph.


### **5. Calibration**

Code will wait until the bag finished playing and a graph containing all pose, odometry, landmarks, and landmark detections is built. Then the SAMOptimize function will run once to obtain the landmark locations.
![image](https://github.com/user-attachments/assets/33a27ead-4368-49e7-b587-ae3cf211938c)

The condition for bag finished is trigger by a preset time interval that no new detections are received. Result can be compared with GT using the `align.py` script in `~/src/aprilslamcpp/config`, some example: 

![Screenshot from 2024-10-03 16-21-09](https://github.com/user-attachments/assets/76e1654a-3b47-4bf6-8302-f8ae3b699367)
![Screenshot from 2024-10-03 16-20-48](https://github.com/user-attachments/assets/beef65a7-bc4e-4616-b8a9-77fe90ddefb5)
![image](https://github.com/user-attachments/assets/54f23e6e-a070-47b9-81f6-06dca0350bf4)


### **6. Localization**

The localization feature leverages previously mapped landmark locations to estimate the robot’s pose in real-time. Here's a breakdown of how the localization is implemented in the system:
![image](https://github.com/user-attachments/assets/1e83bbe0-50d9-4fd5-beff-386e27deba49)

The system can load pre-mapped landmark locations from a CSV file, which can be used as priors for localization. When initializing the SLAM system, the pre-mapped landmarks are loaded and incorporated as priors into the GTSAM factor graph.

```cpp
// During the constructor of aprilslamcpp
if (usepriortagtable) {
    std::map<int, gtsam::Point2> savedLandmarks = loadLandmarksFromCSV(pathtoloadlandmarkcsv);
    for (const auto& landmark : savedLandmarks) {
        gtsam::Symbol landmarkKey('L', landmark.first);
        keyframeGraph_.add(gtsam::PriorFactor<gtsam::Point2>(landmarkKey, landmark.second, pointNoise));
        keyframeEstimates_.insert(landmarkKey, landmark.second);
        landmarkEstimates.insert(landmarkKey, landmark.second);
    }
}
```

There are various confugurations can be applied in Localization algorithm to balance the efficiency and accuracy, so far the SAM with pruning works the best, the optimal settings for localisation alorithm: 

```
useprunebysize: true # no point of using it with ISAM2
useisam2: false # true for ISAM2, false for SAM
useloopclosure: false
usekeyframe: true
```

Some preliminary result: 
![image](https://github.com/user-attachments/assets/f80f839f-2006-434a-98a0-f52385e00243)

**Real time localization testing:**

https://github.com/user-attachments/assets/75063024-7650-4e20-ae3e-a789640560e6

**Reduced tag density**
[Screencast from 05-11-24 10:20:36.webm](https://github.com/user-attachments/assets/4b76c97e-623f-4718-8c1e-357900edc693)


**Real time SLAM**
[Screencast from 31-10-24 20:07:57.webm](https://github.com/user-attachments/assets/35de5278-d3cb-4e0d-b2d2-0d8881075a22)

### **7. Keyframe**

This function `shouldAddKeyframe` determines whether a new keyframe should be added based on:

1. The robot moves a certain distance.
2. The robot rotates beyond a certain angle.
3. New landmarks are detected.

```cpp
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
```

### **8. Stationary Condition**

This section of code determines not to build a graph when robot is stationay to save computational cost:

```cpp
// Check if movement exceeds the stationary thresholds
bool aprilslam::aprilslamcpp::movementExceedsThreshold(const gtsam::Pose2& poseSE2) {
    double position_change = std::hypot(poseSE2.x() - lastPoseSE2_.x(), poseSE2.y() - lastPoseSE2_.y());
    double rotation_change = std::abs(wrapToPi(poseSE2.theta() - lastPoseSE2_.theta()));
    return position_change >= stationary_position_threshold || rotation_change >= stationary_rotation_threshold;
}
```

### **9. Update with odometry only**

This function is for handling non-keyframes. To save computational cost, a non-key pose is update with only the odometry.

```cpp
// Update odometry without adding a keyframe
void aprilslam::aprilslamcpp::updateOdometryPose(const gtsam::Pose2& poseSE2) {
    gtsam::Pose2 odometry = relPoseFG(lastPoseSE2_vis, poseSE2);
    // gtsam::Pose2 adjustedOdometry = odometryDirection(odometry, linear_x_velocity_);
    gtsam::Pose2 newPose = Estimates_visulisation.at<gtsam::Pose2>(gtsam::Symbol('X', index_of_pose - 1)).compose(odometry);
    Estimates_visulisation.insert(gtsam::Symbol('X', index_of_pose), newPose);
    lastPoseSE2_vis = poseSE2;
}
```

### **10. Update with odometry only**

We added outlier removal and trajectory smoothing functionality to improve the localisation. Please dont use outlier removal when odometry is super unreliable (wheel), settings can be found in the params_localisation.yaml.


## **5. How to Run**

Launch the SLAM node:

```bash
roslaunch aprilslamcpp run_localisation.launch 

roslaunch aprilslamcpp run_calibration.launch 
```

Start the cameras and odometry data sources or start replaying the bag file (e.g. rosbag play matt_DLO.bag).

View the output in RViz.

---

## **6. Future Work**

- **Loop Closure Enhancements**: Current loop closure detection is based on re-observing landmarks. We can integrate smarter logic for more robust detection.

![LC_result](https://github.com/user-attachments/assets/cde22ab3-ba0d-4a79-8ee0-9077fb1eb258)









