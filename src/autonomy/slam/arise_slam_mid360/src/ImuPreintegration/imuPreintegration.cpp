//
// Created by shibo zhao on 2020-09-27.
//
#include "arise_slam_mid360/ImuPreintegration/imuPreintegration.h"
#include <fstream>

// For TransformFusion
// static double lidarOdomTime2=-1;
deque<nav_msgs::msg::Odometry> imuOdomQueue;
static Eigen::Affine3f lidarOdomAffine;
namespace arise_slam {

    imuPreintegration::imuPreintegration(const rclcpp::NodeOptions & options)
    : Node("imu_preintegration_node", options) {
    }
    
    void imuPreintegration::initInterface() {
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        if (!readGlobalparam(shared_from_this())) {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::imuPreintegration] Could not read global parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readParameters()) {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::imuPreintegration] Could not read local parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::imuPreintegration] Could not read calibration parameters. Exiting...");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "[AriseSlam::imuPreintegration] use_imu_roll_pitch:  %d", config_.use_imu_roll_pitch);
        RCLCPP_INFO(this->get_logger(), "[AriseSlam::imuPreintegration] lidar_flip:  %d", config_.lidar_flip);

        // Load saved gravity offsets for localization mode consistency.
        // Priority: 1) manual params, 2) companion file from map_dir, 3) compute fresh
        double saved_gravity_roll = this->get_parameter("saved_gravity_roll_offset").as_double();
        double saved_gravity_pitch = this->get_parameter("saved_gravity_pitch_offset").as_double();
        if (saved_gravity_roll != 0.0 || saved_gravity_pitch != 0.0) {
            imu_Init->use_saved_gravity_offsets = true;
            imu_Init->saved_roll_offset = saved_gravity_roll * M_PI / 180.0;
            imu_Init->saved_pitch_offset = saved_gravity_pitch * M_PI / 180.0;
            RCLCPP_WARN(this->get_logger(),
                "[AriseSlam::imuPreintegration] Using manual gravity offsets - roll: %f deg, pitch: %f deg",
                saved_gravity_roll, saved_gravity_pitch);
        } else {
            // Try auto-loading from companion gravity file alongside the PCD map
            std::string map_dir = this->get_parameter("map_dir").as_string();
            if (!map_dir.empty()) {
                std::string gravity_file = map_dir;
                size_t pcd_pos = gravity_file.rfind(".pcd");
                if (pcd_pos != std::string::npos) {
                    gravity_file.replace(pcd_pos, 4, "_gravity.txt");
                } else {
                    gravity_file += "_gravity.txt";
                }
                std::ifstream ifs(gravity_file);
                if (ifs.is_open()) {
                    double roll_deg, pitch_deg;
                    if (ifs >> roll_deg >> pitch_deg) {
                        imu_Init->use_saved_gravity_offsets = true;
                        imu_Init->saved_roll_offset = roll_deg * M_PI / 180.0;
                        imu_Init->saved_pitch_offset = pitch_deg * M_PI / 180.0;
                        RCLCPP_WARN(this->get_logger(),
                            "[AriseSlam::imuPreintegration] Loaded gravity offsets from %s - roll: %f deg, pitch: %f deg",
                            gravity_file.c_str(), roll_deg, pitch_deg);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                            "[AriseSlam::imuPreintegration] Failed to parse gravity file: %s", gravity_file.c_str());
                    }
                    ifs.close();
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "[AriseSlam::imuPreintegration] No gravity file found at %s, will compute from IMU", gravity_file.c_str());
                }
            }
        }

        //subscribe and publish relevant topics
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, 10,
            std::bind(&imuPreintegration::imuHandler, this,
                        std::placeholders::_1), sub_options);
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            ProjectName+"/laser_odometry", 5,
            std::bind(&imuPreintegration::laserodometryHandler, this,
                        std::placeholders::_1), sub_options);
        // subVisualOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "vins_estimator/imu_propagate", 10,
        //     std::bind(&imuPreingration::visualodometryHandler, this,
        //                 std::palceholders::_1));

        // Loop closure: subscribe to registered scan for keyframe clouds
        if (lc_enabled_) {
            subRegisteredScan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                ProjectName + "/registered_scan", rclcpp::SensorDataQoS(),
                std::bind(&imuPreintegration::registeredScanHandler, this,
                          std::placeholders::_1), sub_options);

            latest_cloud_.reset(new pcl::PointCloud<PointType>());
            kf_position_tree_.reset(new pcl::KdTreeFLANN<PointType>());
            kf_positions_.reset(new pcl::PointCloud<PointType>());
        }

        // pubImuOdometry = this->create_publisher<nav_msgs::msg::Odometry>(
        //     ProjectName+"/integrated_to_init_incremental", 10);
        pubImuOdometry2 = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/state_estimation", 10);
        pubHealthStatus = this->create_publisher<std_msgs::msg::Bool>(
            ProjectName+"/state_estimation_health", 1);
        pubImuPath = this->create_publisher<nav_msgs::msg::Path>(
            ProjectName+"/imuodom_path", 1);
        // pubImuOdometrySmooth = this->create_publisher<nav_msgs::msg::Odometry>(
        //     ProjectName+"/integrated_to_init2", 2000);

        // Gravity offsets publisher with transient_local QoS so late subscribers get the value
        auto gravity_qos = rclcpp::QoS(1).transient_local();
        pubGravityOffsets = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gravity_offsets", gravity_qos);

        tfMap2Odom = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfOdom2BaseLink = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // set relevant parameter
        std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(config_.imuGravity);
        
        // std::shared_ptr<gtsam::PreintegrationParams> p(std::make_shared<gtsam::PreintegrationParams>(config_.imuGravity));
        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(config_.imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) *
                                   pow(1e-4, 2); // error committed in integrating position from velocities

        gtsam::imuBias::ConstantBias prior_imu_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVisualPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());         // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);                      // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6,
                                                             1e-1);                    // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, config_.lidar_correction_noise); // meter
       // correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m

        noMotionNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished()); // rad,rad,rad,m, m, m
        noVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);

        noiseModelBetweenBias = (gtsam::Vector(6)
                << config_.imuAccBiasN,
                config_.imuAccBiasN, config_.imuAccBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN, config_.imuGyrBiasN)
                .finished();
        imuIntegratorImu_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias); // setting up the IMU integration for IMU message
        imuIntegratorOpt_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias); // setting up the IMU integration for optimization

        // Start loop closure thread
        if (lc_enabled_) {
            lc_thread_ = std::thread(&imuPreintegration::loopClosureThread, this);
            RCLCPP_INFO(this->get_logger(), "Loop closure thread started");
        }

        //set extrinsic matrix for laser and imu
        if (PROVIDE_IMU_LASER_EXTRINSIC) {
            imu2Lidar = gtsam::Pose3(gtsam::Rot3(imu_laser_R), gtsam::Point3(imu_laser_T));
            lidar2Imu = imu2Lidar.inverse();
        } else {
            imu2cam = gtsam::Pose3(gtsam::Rot3(imu_camera_R), gtsam::Point3(imu_camera_T));
            cam2Lidar = gtsam::Pose3(gtsam::Rot3(cam_laser_R), gtsam::Point3(cam_laser_T));
            imu2Lidar = imu2cam.compose(cam2Lidar);
            lidar2Imu = imu2Lidar.inverse();
        }

    }
    

    bool imuPreintegration::readParameters()
    {
        this->declare_parameter<float>("acc_n", 1e-3);
        this->declare_parameter<float>("acc_w", 1e-3);
        this->declare_parameter<float>("gyr_n", 1e-6);
        this->declare_parameter<float>("gyr_w", 1e-6);
        this->declare_parameter<float>("g_norm", 9.8);
        this->declare_parameter<float>("lidar_correction_noise",0.01);
        this->declare_parameter<float>("smooth_factor",0.9);
        this->declare_parameter<bool>("use_imu_roll_pitch",true);
        this->declare_parameter<bool>("lidar_flip",false);
        this->declare_parameter<std::string>("sensor");
        this->declare_parameter<double>("imu_acc_x_offset", 0.0);
        this->declare_parameter<double>("imu_acc_y_offset", 0.0);
        this->declare_parameter<double>("imu_acc_z_offset", 0.0);
        this->declare_parameter<double>("saved_gravity_roll_offset", 0.0);
        this->declare_parameter<double>("saved_gravity_pitch_offset", 0.0);
        this->declare_parameter<std::string>("map_dir", "");
        this->declare_parameter<double>("imu_acc_x_limit", 1.0);
        this->declare_parameter<double>("imu_acc_y_limit", 1.0);
        this->declare_parameter<double>("imu_acc_z_limit", 1.0);

        // Loop closure parameters
        this->declare_parameter<bool>("loop_closure_enabled", false);
        this->declare_parameter<double>("lc_search_radius", 15.0);
        this->declare_parameter<double>("lc_time_diff", 30.0);
        this->declare_parameter<int>("lc_history_search_num", 25);
        this->declare_parameter<double>("lc_fitness_threshold", 0.3);
        this->declare_parameter<double>("lc_downsample_res", 0.3);
        this->declare_parameter<double>("lc_keyframe_dist", 1.0);
        this->declare_parameter<double>("lc_keyframe_angle", 0.2);

        config_.imuAccNoise = this->get_parameter("acc_n").as_double();
        config_.imuAccBiasN = this->get_parameter("acc_w").as_double();
        config_.imuGyrNoise = this->get_parameter("gyr_n").as_double();
        config_.imuGyrBiasN = this->get_parameter("gyr_w").as_double();
        config_.imuGravity = this->get_parameter("g_norm").as_double();
        config_.lidar_correction_noise = this->get_parameter("lidar_correction_noise").as_double();
        config_.smooth_factor = this->get_parameter("smooth_factor").as_double();
        config_.use_imu_roll_pitch = this->get_parameter("use_imu_roll_pitch").as_bool();
        config_.lidar_flip = this->get_parameter("lidar_flip").as_bool();
        config_.imu_acc_x_offset = this->get_parameter("imu_acc_x_offset").as_double();
        config_.imu_acc_y_offset = this->get_parameter("imu_acc_y_offset").as_double();
        config_.imu_acc_z_offset = this->get_parameter("imu_acc_z_offset").as_double();
        config_.imu_acc_x_limit = this->get_parameter("imu_acc_x_limit").as_double();
        config_.imu_acc_y_limit = this->get_parameter("imu_acc_y_limit").as_double();
        config_.imu_acc_z_limit = this->get_parameter("imu_acc_z_limit").as_double();

        // Loop closure parameters
        lc_enabled_ = this->get_parameter("loop_closure_enabled").as_bool();
        lc_search_radius_ = this->get_parameter("lc_search_radius").as_double();
        lc_time_diff_ = this->get_parameter("lc_time_diff").as_double();
        lc_history_search_num_ = this->get_parameter("lc_history_search_num").as_int();
        lc_fitness_threshold_ = this->get_parameter("lc_fitness_threshold").as_double();
        lc_downsample_res_ = this->get_parameter("lc_downsample_res").as_double();
        lc_keyframe_dist_ = this->get_parameter("lc_keyframe_dist").as_double();
        lc_keyframe_angle_ = this->get_parameter("lc_keyframe_angle").as_double();
        RCLCPP_INFO(this->get_logger(), "Loop closure enabled: %d, search_radius: %f, time_diff: %f",
                    lc_enabled_, lc_search_radius_, lc_time_diff_);

        RCLCPP_INFO(this->get_logger(), "config_.imu_acc_x_limit: %f", config_.imu_acc_x_limit);
        RCLCPP_INFO(this->get_logger(), "config_.imu_acc_y_limit: %f", config_.imu_acc_y_limit);
        RCLCPP_INFO(this->get_logger(), "config_.imu_acc_z_limit: %f", config_.imu_acc_z_limit);

        RCLCPP_INFO(this->get_logger(), "config_.use_imu_roll_pitch: %d", config_.use_imu_roll_pitch);
        RCLCPP_INFO(this->get_logger(), "config_.lidar_flip: %d", config_.lidar_flip);

        RCLCPP_INFO(this->get_logger(), "imuAccNoise: %f  imuAccBiasN: %f imuGyrNoise %f imuGyrBiasN %f imuGravity %f",
                    config_.imuAccNoise, config_.imuAccBiasN, config_.imuGyrNoise, config_.imuGyrBiasN, config_.imuGravity);

        std::string sensorType;
        if (!this->get_parameter("sensor", sensorType))
        {
            RCLCPP_ERROR(this->get_logger(), "no sensor parameters");
            return false;
        }
         if (sensorType == "velodyne")
        {
            config_.sensor = SensorType::VELODYNE;
            std::cerr << "sensor = VELODYNE" << std::endl;
        }
        else if (sensorType == "ouster")
        {
            config_.sensor = SensorType::OUSTER;
            std::cerr << "sensor=OUSTER" << std::endl;
        }
        else if (sensorType == "livox")
        {
            config_.sensor = SensorType::LIVOX;
            std::cerr << "sensor=LIVOX" << std::endl;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                    "Invalid sensor type: %s. (must be either 'velodyne' or 'ouster')", sensorType.c_str());
            rclcpp::shutdown();
        }

        return true;

    }



    Eigen::Affine3f imuPreintegration::odom2affine(nav_msgs::msg::Odometry odom) {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf2::Quaternion orientation;
        tf2::fromMsg(odom.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void imuPreintegration::resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void imuPreintegration::resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void imuPreintegration::addNoMotionFactor(const FrameId &from_id, const FrameId &to_id)
    {

        Eigen::Matrix3d relative_motion;
        Eigen::Vector3d relative_trans;

        graphFactors.add(
                std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                        gtsam::Symbol('x', from_id),
                        gtsam::Symbol('x', to_id),
                        gtsam::Pose3(gtsam::Rot3(relative_motion.setIdentity()),
                                     gtsam::Point3(relative_trans.setZero())),
                        noMotionNoise));

        // debug_info_.numAddedNoMotionF_++;
        // if (VLOG_IS_ON(10)) {
        //     std::cout << "No motion detected, adding no relative motion prior"
        //               << std::endl;
        // }
    }

    void imuPreintegration::addZeroVelocityPrior(const FrameId &frame_id) {
        // VLOG(10) << "No motion detected, adding zero velocity prior.";
        graphFactors.add(
                std::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
                        gtsam::Symbol('v', frame_id),
                        gtsam::Vector3::Zero(),
                        noVelocityNoise));
    }

    void imuPreintegration::reset_graph() {

        // get updated noise before reset
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise, updatedVelNoise, updatedBiasNoise;

        try
        {
            updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to reset graph. No marginal covariance key");
            updatedPoseNoise = priorPoseNoise;
            updatedVelNoise = priorVelNoise;
            updatedBiasNoise = priorBiasNoise;
        }

        // reset graph
        resetOptimization();
        // add pose
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   updatedPoseNoise);
        graphFactors.add(priorPose);
        // add velocity
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    updatedVelNoise);
        graphFactors.add(priorVel);
        // add bias
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, updatedBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        key = 1;
    }

    void imuPreintegration::initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose) {

        // 0. initialize system
        resetOptimization();
        //TODO: it seems that we don't need this  process if we use ringbuffer


        while (!imuQueOpt.empty())
        {
            if (secs(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
            {
                lastImuT_opt = secs(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }


        // initial pose
        prevPose_ = lidarPose.compose(lidar2Imu);

        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                   priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                    priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        // addZeroVelocityPrior(0);
        //     addNoMotionFactor(key - 1, key);

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        key = 1;
        systemInitialized = true;

    }

    void imuPreintegration::integrate_imumeasurement(double currentCorrectionTime) {
        // 1. integrate imu data and optimize

        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = secs(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_opt);
                lastImuT_opt = imuTime;

                if(dt < 0.001 || dt > 0.5) 
                    dt = 0.005;
               
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                imuQueOpt.pop_front();
            }
            else
                break;
        }

    }


    bool imuPreintegration::build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp) {


        // add laser pose prior factor

        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);

        // insert predicted values
        gtsam::NavState propState_ =
                imuIntegratorOpt_->predict(prevState_, prevBias_);
        // auto diff  = curPose.translation() - propState_.pose().translation();
        // RCLCPP_WARN(this->get_logger(), "diff norm: %f", diff.norm());

        // if (diff.norm() > 0.2)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Large difference between imu and lidar pose estimation");
        //     // resetParams();
        //     // return;
        //     correctionNoise =  gtsam::noiseModel::Isotropic::Sigma(6, lidar_correction_noise*100); // meter
        // }else{
        //     correctionNoise =  gtsam::noiseModel::Isotropic::Sigma(6, lidar_correction_noise); // meter

        // }
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose,
                                                     correctionNoise);
        graphFactors.add(pose_factor);
                // add imu factor to graph
        
        const gtsam::PreintegratedImuMeasurements &preint_imu =
                dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(
                        *imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key),
                                    B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(
                        sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        
  
        // optimize
        bool systemSolvedSuccessfully = false;
        try {
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
            systemSolvedSuccessfully = true;
        }
        catch (const gtsam::IndeterminantLinearSystemException &) {
            systemSolvedSuccessfully = false;
            RCLCPP_WARN(this->get_logger(), "Update failed due to underconstrained call to isam2 in imuPreintegration");
        }

        graphFactors.resize(0);
        graphValues.clear();

        if (systemSolvedSuccessfully) {

            // RCLCPP_INFO_STREAM(this->get_logger(), "key: " << key);
            // RCLCPP_INFO_STREAM(this->get_logger(), "X shape" <<  X);
            // Overwrite the beginning of the preintegration for the next step.
            gtsam::Values result = optimizer.calculateEstimate();
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
            // Reset the optimization preintegration object.
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);


        }        
        return systemSolvedSuccessfully;
    }

    void imuPreintegration::repropagate_imuodometry(double currentCorrectionTime) {
        // 2. after optimization, re-propagate imu odometry preintegration

        // Capture pre-correction predicted lidar pose (for correction spreading)
        gtsam::NavState preState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
        gtsam::Pose3 preLidar = gtsam::Pose3(gtsam::Rot3(preState.quaternion()), preState.position()).compose(imu2Lidar);

        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && secs(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = secs(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = secs(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 200.0) :(imuTime - lastImuQT);
                lastImuQT = imuTime;

                if(dt < 0.001 || dt > 0.5)
                    dt = 0.005;

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        // Capture post-correction predicted lidar pose and accumulate the correction delta.
        // The IMU callback will gradually apply this correction instead of jumping instantly.
        gtsam::NavState postState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
        gtsam::Pose3 postLidar = gtsam::Pose3(gtsam::Rot3(postState.quaternion()), postState.position()).compose(imu2Lidar);

        correction_pos_ += (postLidar.translation() - preLidar.translation());
        Eigen::Quaterniond preQ(preLidar.rotation().toQuaternion().w(), preLidar.rotation().toQuaternion().x(),
                                preLidar.rotation().toQuaternion().y(), preLidar.rotation().toQuaternion().z());
        Eigen::Quaterniond postQ(postLidar.rotation().toQuaternion().w(), postLidar.rotation().toQuaternion().x(),
                                 postLidar.rotation().toQuaternion().y(), postLidar.rotation().toQuaternion().z());
        correction_rot_ = correction_rot_ * (preQ.inverse() * postQ);
        correction_rot_.normalize();

    }

    void imuPreintegration::process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose) {

        // reset graph for speed (300 to allow loop closure corrections to propagate)
        if (key > 300) {
            reset_graph();
        }

        // 1. integrate_imumeasurement


        integrate_imumeasurement(currentCorrectionTime);

        lidarodom_w_cur = relativePose;


        // 2. build_graph
        bool successOptimization = build_graph(lidarodom_w_cur, currentCorrectionTime);
        
        // 3. check optimization
        if (failureDetection(prevVel_, prevBias_) || !successOptimization) {
            RCLCPP_WARN(this->get_logger(), "failureDetected");
            resetParams();
            return;
        }

    //    RCLCPP_INFO_STREAM(this->get_logger(), "key: " << key);
        // 4. reprogate_imuodometry
        repropagate_imuodometry(currentCorrectionTime);
        ++key;

        doneFirstOpt = true;
    }

    bool imuPreintegration::failureDetection(const gtsam::Vector3 &velCur,
                                             const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            RCLCPP_WARN(this->get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }



        // if (use_no_motion_prior && vel.norm() < 0.03) {
        //     addZeroVelocityPrior(key);
        //     addNoMotionFactor(key - 1, key);
        //     RCLCPP_INFO(this->get_logger(), " Detect small velocity and Reset velocity to zero ! ");
        // }


        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                           biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                           biasCur.gyroscope().z());

        // RCLCPP_WARN(this->get_logger(), " ba.norm: %f | bg.norm: %f", ba.norm(), bg.norm());
        if (ba.norm() > 2.0 || bg.norm() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

# if 0
    void imuPreintegration::synchronize_measurements(std::deque<nav_msgs::msg::Odometry::SharedPtr> &laserodomQue,
                                                     std::deque<nav_msgs::msg::Odometry::SharedPtr> &visualodomQue,
                                                     std::deque<sensor_msgs::msg::Imu> &imuQue)
    {

        use_visualodom = false;
        use_laserodom = false;
        use_onlyimu = false;
        health_status = false;

        if (imuQue.empty())
            return;

        if (laserodomQue.empty() && visualodomQue.empty())
        {
            use_onlyimu = true;
            health_status = false;
            return;
        }

        if (!laserodomQue.empty() && !visualodomQue.empty() && !imuQue.empty())
        {
            mBuf.lock();
            health_status = true;
            while (!visualodomQue.empty() &&
                   secs(visualodomQue.front()) < secs(laserodomQue.front()) - 0.05)
                visualodomQue.pop_front();

            if (visualodomQue.empty())
            {
                mBuf.unlock();
                use_visualodom = false;
            }
            else
            {
                use_visualodom = true;
            }

            while (!laserodomQue.empty() && secs(visualodomQue.front()) 
                                            > secs(laserodomQue.front()) + 0.05)
            {
                laserodomQue.pop_front();
            }

            if (laserodomQue.empty())
            {
                mBuf.unlock();
                use_laserodom = false;
            }
            else
            {
                use_laserodom = true;
            }

            if (!laserodomQue.empty() && !visualodomQue.empty())
            {
                double timelaserodom = secs(&laserodomQue.front());
                double timevisualodom = secs(&visualodomQue.front());

                if (abs(timelaserodom - timevisualodom) > 0.05)
                {
                    mBuf.unlock();
                    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> unsync laserodom and visual odom.\033[0m");
                }
            }
            mBuf.unlock();
        }
        else if (!laserodomQue.empty() && !imuQue.empty())
        {
            use_laserodom = true;
            health_status = true;
        }
        else if (!visualodomQue.empty() && !imuQue.empty())
        {
            use_visualodom = true;
            health_status = false;
        }

        // RCLCPP_WARN(this->get_logger(), "health_status %d", health_status);
    }

#endif

    void imuPreintegration::laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        // RCLCPP_INFO(this->get_logger(), "laserOdometryCB from /aft_mapped_to_init_incremental");
        std::lock_guard<std::mutex> lock(mBuf);


        cur_frame = odomMsg;
        double lidarOdomTime = secs(odomMsg);


        if (imuQueOpt.empty())
            return;


        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        // bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
                                              gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (systemInitialized == false) {

            initial_system(lidarOdomTime, lidarPose);

            return;
        }


        TicToc Optimization_time;
        //1. process imu odometry
        process_imu_odometry(lidarOdomTime, lidarPose);

        // Loop closure: keyframe selection and correction application
        if (lc_enabled_ && doneFirstOpt) {
            gtsam::Pose3 current_lidar_pose = prevPose_.compose(imu2Lidar);
            if (shouldAddKeyframe(current_lidar_pose)) {
                addKeyframe(current_lidar_pose, lidarOdomTime);
            }
            applyLoopClosure();
        }

#if 1
        // 2. safe lannding process
        double latest_imu_time;

        latest_imu_time = secs(&imuQueImu.back());


      //    LOG(INFO) << "abs(latest_laser_time - latest_imu_time (including process)): " << abs(lidarOdomTime - latest_imu_time);

        if (lidarOdomTime - latest_imu_time < imu_laser_timedelay) {
            RESULT = IMU_STATE::SUCCESS;
            health_status = true;
          
          if((int)odomMsg->pose.covariance[0] == 1)
           {
             RESULT = IMU_STATE::FAIL;
           }
         
        } else {
            health_status = false;
            if (cur_frame != nullptr && last_frame != nullptr) {
                Eigen::Vector3d velocity_curr;
                velocity_curr.x() =
                    (cur_frame->pose.pose.position.x - last_frame->pose.pose.position.x) /
                    (secs(cur_frame) - secs(last_frame));
                velocity_curr.y() = (cur_frame->pose.pose.position.y - last_frame->pose.pose.position.y) /
                                    (secs(cur_frame) - secs(last_frame));
                velocity_curr.z() = (cur_frame->pose.pose.position.z - last_frame->pose.pose.position.z) /
                                    (secs(cur_frame) - secs(last_frame));

                RCLCPP_INFO(this->get_logger(), "LOOSE CONNECTION WITH IMU DRIVER, PLEASE CHECK HARDWARE!!");
                RESULT = IMU_STATE::FAIL;
                health_status = false;
                nav_msgs::msg::Odometry odometry;

                // odometry.header.stamp = odomMsg->header.stamp;
                // odometry.header.frame_id = WORLD_FRAME;
                // odometry.child_frame_id = SENSOR_FRAME;

                // odometry.pose.pose.position.x = odomMsg->pose.pose.position.x;
                // odometry.pose.pose.position.y = odomMsg->pose.pose.position.y;
                // odometry.pose.pose.position.z = odomMsg->pose.pose.position.z;
                // odometry.pose.pose.orientation.x = odomMsg->pose.pose.orientation.x;
                // odometry.pose.pose.orientation.y = odomMsg->pose.pose.orientation.y;
                // odometry.pose.pose.orientation.z = odomMsg->pose.pose.orientation.z;
                // odometry.pose.pose.orientation.w = odomMsg->pose.pose.orientation.w;

                // odometry.twist.twist.linear.x = velocity_curr.x();
                // odometry.twist.twist.linear.y = velocity_curr.y();
                // odometry.twist.twist.linear.z = velocity_curr.z();
                // odometry.pose.covariance[0] = double(RESULT);
                // pubImuOdometrySmooth->publish(odometry);

                std_msgs::msg::Bool health_status_msg;
                health_status_msg.data = health_status;
                pubHealthStatus->publish(health_status_msg);
            }
        }

        last_frame = cur_frame;
#endif
        //   receive_first_laserodom=true;
        last_processed_lidar_time = lidarOdomTime;
    }

    void imuPreintegration::visualodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        RCLCPP_INFO(this->get_logger(), "visualodometryHandler from");
        std::lock_guard<std::mutex> lock(mBuf);
        if (imuQueOpt.empty())
            return;

        visualOdomBuf.addMeas(odomMsg, secs(odomMsg));

    }



    //TODO: need to consider the extrinsic matrix of imu and lidar
    sensor_msgs::msg::Imu imuPreintegration::imuConverter(const sensor_msgs::msg::Imu &imu_in) {
        sensor_msgs::msg::Imu imu_out = imu_in;
        
        Eigen::Matrix3d imu_laser_R_Gravity;
        imu_laser_R_Gravity=imu_Init->imu_laser_R_Gravity;

        Eigen::Vector3d rpy;
        rpy=imu_Init->rotationMatrixToRPY(imu_laser_R_Gravity);
 

        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                            imu_in.angular_velocity.z);
        gyr=imu_laser_R_Gravity*gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();


        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                            imu_in.linear_acceleration.y,
                            imu_in.linear_acceleration.z);

        acc=imu_laser_R_Gravity*acc;
        acc = acc + ((gyr - gyr_pre) * 200).cross(- imu_laser_T) + gyr.cross(gyr.cross(-imu_laser_T));
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();


        // rotate roll pitch yaw
        Eigen::Quaterniond q(imu_in.orientation.w, imu_in.orientation.x,
                             imu_in.orientation.y, imu_in.orientation.z);

        q.normalize();


        Eigen::Quaterniond q_extrinsic;
        q_extrinsic=Eigen::Quaterniond(imu_laser_R_Gravity);

        Eigen::Quaterniond q_new;

        q_new=q*q_extrinsic;

        q_new.normalize();

        imu_out.orientation.x = q_new.x();
        imu_out.orientation.y = q_new.y();
        imu_out.orientation.z = q_new.z();
        imu_out.orientation.w = q_new.w();

        gyr_pre = gyr;

        if (imu_init_success == true) // limit the accelerations if successfully init
        {
            Eigen::Vector3d acc_mean = imu_Init->acc_mean;
            Eigen::Vector3d gyr_mean = imu_Init->gyr_mean;
            acc_mean = imu_Init->imu_laser_R_Gravity * acc_mean;
            gyr_mean = imu_Init->imu_laser_R_Gravity * gyr_mean;
            Eigen::Vector3d acc_diff = acc - acc_mean;
            // Eigen::Vector3d gyr_diff = gyr - gyr_mean;

            if (abs(acc_diff(0)) > config_.imu_acc_x_limit){
                // RCLCPP_WARN(this->get_logger(), "IMU acc x limit exceeded: %f", acc_diff(0));
                acc_diff(0) = acc_diff(0) > 0 ? config_.imu_acc_x_limit : -config_.imu_acc_x_limit;
            }
            if (abs(acc_diff(1)) > config_.imu_acc_y_limit){
                // RCLCPP_WARN(this->get_logger(), "IMU acc y limit exceeded: %f", acc_diff(1));
                acc_diff(1) = acc_diff(1) > 0 ? config_.imu_acc_y_limit : -config_.imu_acc_y_limit;
            }
            if (abs(acc_diff(2)) > config_.imu_acc_z_limit){
                // RCLCPP_WARN(this->get_logger(), "IMU acc z limit exceeded: %f", acc_diff(2));
                acc_diff(2) = acc_diff(2) > 0 ? config_.imu_acc_z_limit : -config_.imu_acc_z_limit;
            }

            imu_out.linear_acceleration.x = acc_mean(0) + acc_diff(0);
            imu_out.linear_acceleration.y = acc_mean(1) + acc_diff(1);
            imu_out.linear_acceleration.z = acc_mean(2) + acc_diff(2);

            // imu_out.linear_acceleration.x = 0;
            // imu_out.linear_acceleration.y = 0;
            // imu_out.linear_acceleration.z = imu_out.linear_acceleration.z;
        }

        return imu_out;
    }

    void imuPreintegration::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw) 
    {
        imu_raw->linear_acceleration.x += config_.imu_acc_x_offset;
        imu_raw->linear_acceleration.y += config_.imu_acc_y_offset;
        imu_raw->linear_acceleration.z += config_.imu_acc_z_offset;

        if (config_.lidar_flip) 
        {
          imu_raw->linear_acceleration.y *= -1.0;
          imu_raw->linear_acceleration.z *= -1.0;
          imu_raw->angular_velocity.y *= -1.0;
          imu_raw->angular_velocity.z *= -1.0;
        }
     
        std::lock_guard<std::mutex> lock(mBuf);
        
        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);
        
        assert(imu_raw->linear_acceleration.x!=thisImu.linear_acceleration.x); 


        if(config_.sensor==SensorType::LIVOX and imu_init_success == false)
        {
          Imu::Ptr imudata = std::make_shared<Imu>();
          imudata->time = imu_raw->header.stamp.sec + imu_raw->header.stamp.nanosec * 1e-9;
          imudata->acc = Eigen::Vector3d(imu_raw->linear_acceleration.x,imu_raw->linear_acceleration.y,imu_raw->linear_acceleration.z);
          imudata->gyr = Eigen::Vector3d(imu_raw->angular_velocity.x,imu_raw->angular_velocity.y,imu_raw->angular_velocity.z);
          imudata->q_w_i=Eigen::Quaterniond(imu_raw->orientation.w, 
          imu_raw->orientation.x, imu_raw->orientation.y, imu_raw->orientation.z);

          imuBuf.addMeas(imudata, imudata->time);

          double first_imu_time = 0.0; 
          imuBuf.getFirstTime(first_imu_time);

          if (imudata->time - first_imu_time > 1.0 and imu_init_success == false)
            {   
                //TODO: IMUInit might be not necessary since it is only for accleration
                imu_Init->imuInit(imuBuf);
                imu_init_success=true;
                imuBuf.clean(imudata->time);
                std::cout<<"IMU Initialization Process Finish! "<<std::endl;

                // Publish gravity offsets so visualization_tools can save them with the map
                std_msgs::msg::Float64MultiArray gravity_msg;
                gravity_msg.data.push_back(imu_Init->roll_offset_gravity * 180.0 / M_PI);
                gravity_msg.data.push_back(imu_Init->pitch_offset_gravity * 180.0 / M_PI);
                pubGravityOffsets->publish(gravity_msg);
            }
        }
        



        if(config_.sensor==SensorType::LIVOX)
        {
           double gravity = 9.8105;
           Eigen::Vector3d acc;
           acc=Eigen::Vector3d(thisImu.linear_acceleration.x,thisImu.linear_acceleration.y,thisImu.linear_acceleration.z);
           acc= acc*gravity/imu_Init->acc_mean.norm();
           thisImu.linear_acceleration.x=acc.x();
           thisImu.linear_acceleration.y=acc.y();
           thisImu.linear_acceleration.z=acc.z();
        }
        
        if(imu_init_success==false) 
        {
            return;
        }
        

        //Add the IMU Initialization for LIVOX LiDAR
        double imuTime = secs(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 200.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;
        
        if(dt < 0.001 || dt > 0.5) 
            dt = 0.005;
            // return;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        double roll, pitch, yaw;
        tf2::Quaternion orientation;
        tf2::fromMsg(thisImu.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger(), "Aroll, pitch, yaw %f, %f, %f", roll, pitch, yaw);

        tf2::Quaternion yaw_quat;
        yaw_quat.setRPY(0,0,-yaw);

        orientation = yaw_quat*orientation;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // ROS_INFO_THROTTLE(this->get_logger(), 1.0, "roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);


        if (doneFirstOpt == false)
            return;



        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(thisImu.linear_acceleration.x,
                               thisImu.linear_acceleration.y,
                               thisImu.linear_acceleration.z),
                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y,
                               thisImu.angular_velocity.z),
                dt);

        // predict odometry
        gtsam::NavState currentState =
                imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        tf2::Quaternion state_q(currentState.quaternion().x(), currentState.quaternion().y(), currentState.quaternion().z(), currentState.quaternion().w());

        tf2::Matrix3x3(state_q).getRPY(roll, pitch, yaw);
        // tf2::Quaternion yaw_quat;
        yaw_quat.setRPY(0,0,yaw);
        orientation = yaw_quat*orientation;

        // publish odometry
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = WORLD_FRAME;
        odometry.child_frame_id = SENSOR_FRAME;
        
        // TODO: convert current Velocity State from world frame to current frame
        Eigen::Quaterniond q_w_curr;
        if (config_.use_imu_roll_pitch)
        {
            // RCLCPP_WARN(this->get_logger(), "imuPreintegration: use imu roll pitch");
            q_w_curr = Eigen::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z());
            // q_w_curr = Eigen::Quaterniond(currentState.quaternion().w(), currentState.quaternion().x(),
            //                               currentState.quaternion().y(), currentState.quaternion().z());
        }
        else
        {
            q_w_curr = Eigen::Quaterniond(currentState.quaternion().w(), currentState.quaternion().x(),
                                          currentState.quaternion().y(), currentState.quaternion().z());
        }
        // Eigen::Quaterniond q_w_curr(currentState.quaternion().w(), currentState.quaternion().x(),
        //                             currentState.quaternion().y(), currentState.quaternion().z());
        gtsam::Rot3 imuRot(q_w_curr);
            // transform imu pose to ldiar
        gtsam::Pose3 imuPose =
                gtsam::Pose3(imuRot, currentState.position());
        gtsam::Pose3 lidarPoseOpt = imuPose.compose(imu2Lidar);

        Eigen::Vector3d velocity_w_curr(currentState.velocity().x(), currentState.velocity().y(),
                                        currentState.velocity().z());

        //     Eigen::Vector3d velocity_w_curr(0.05*currentState.velocity().x()+0.95*prevStateOdom.velocity().x(),
        //                                        0.05*currentState.velocity().y()+0.95*prevStateOdom.velocity().y(),
        //                                        0.1*currentState.velocity().z()+0.9*prevStateOdom.velocity().z());

        Eigen::Vector3d velocity_curr = currentState.quaternion().inverse() * velocity_w_curr;



        // odometry.pose.pose.position.x = currentState.position().x();
        // odometry.pose.pose.position.y = currentState.position().y();
        // odometry.pose.pose.position.z = currentState.position().z();

        // odometry.pose.pose.orientation.x = currentState.quaternion().x();
        // odometry.pose.pose.orientation.y = currentState.quaternion().y();
        // odometry.pose.pose.orientation.z = currentState.quaternion().z();
        // odometry.pose.pose.orientation.w = currentState.quaternion().w();

        // odometry.pose.pose.orientation.x = q_w_curr.x();
        // odometry.pose.pose.orientation.y = q_w_curr.y();
        // odometry.pose.pose.orientation.z = q_w_curr.z();
        // odometry.pose.pose.orientation.w = q_w_curr.w();

        // odometry.twist.twist.linear.x = velocity_curr.x();
        // odometry.twist.twist.linear.y = velocity_curr.y();
        // odometry.twist.twist.linear.z = velocity_curr.z();
        // odometry.twist.twist.angular.x =
        //         thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        // odometry.twist.twist.angular.y =
        //         thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        // odometry.twist.twist.angular.z =
        //         thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        // odometry.pose.covariance[0] = double(RESULT);
        // pubImuOdometry->publish(odometry);


        Eigen::Quaterniond q_w_lidar(lidarPoseOpt.rotation().toQuaternion().w(), lidarPoseOpt.rotation().toQuaternion().x(),
                                     lidarPoseOpt.rotation().toQuaternion().y(), lidarPoseOpt.rotation().toQuaternion().z());
        q_w_lidar.normalize();

        // Gradually apply SLAM corrections to avoid discontinuities.
        // correction_pos_/rot_ hold the portion of the latest correction not yet applied.
        // Decay them toward zero each frame; subtract from raw pose so the output
        // transitions smoothly from pre-correction to post-correction trajectory.
        // Zero lag during smooth IMU integration (correction_ stays at zero).
        constexpr double correction_decay = 0.06;  // ~17 frames to reach 95% at 200Hz (~85ms)
        correction_pos_ *= (1.0 - correction_decay);
        correction_rot_ = correction_rot_.slerp(correction_decay, Eigen::Quaterniond::Identity());
        correction_rot_.normalize();

        Eigen::Vector3d pub_pos = lidarPoseOpt.translation() - correction_pos_;
        Eigen::Quaterniond pub_rot = q_w_lidar * correction_rot_.inverse();
        pub_rot.normalize();

        nav_msgs::msg::Odometry odometry2;
        odometry2.header.stamp = thisImu.header.stamp;
        odometry2.header.frame_id = WORLD_FRAME;
        odometry2.child_frame_id = SENSOR_FRAME;

        odometry2.pose.pose.position.x = pub_pos.x();
        odometry2.pose.pose.position.y = pub_pos.y();
        odometry2.pose.pose.position.z = pub_pos.z();
        odometry2.pose.pose.orientation.x = pub_rot.x();
        odometry2.pose.pose.orientation.y = pub_rot.y();
        odometry2.pose.pose.orientation.z = pub_rot.z();
        odometry2.pose.pose.orientation.w = pub_rot.w();

        odometry2.twist.twist.linear.x = velocity_curr.x();
        odometry2.twist.twist.linear.y = velocity_curr.y();
        odometry2.twist.twist.linear.z = velocity_curr.z();
        odometry2.twist.twist.angular.x =
                thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry2.twist.twist.angular.y =
                thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry2.twist.twist.angular.z =
                thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        odometry2.pose.covariance[0] = double(RESULT);

        odometry2.pose.covariance[1] = prevBiasOdom.accelerometer().x();
        odometry2.pose.covariance[2] = prevBiasOdom.accelerometer().y();
        odometry2.pose.covariance[3] = prevBiasOdom.accelerometer().z();
        odometry2.pose.covariance[4] = prevBiasOdom.gyroscope().x();
        odometry2.pose.covariance[5] = prevBiasOdom.gyroscope().y();
        odometry2.pose.covariance[6] = prevBiasOdom.gyroscope().z();
        odometry2.pose.covariance[7] = config_.imuGravity;

        frame_count++;
        if(frame_count%4==0)
            pubImuOdometry2->publish(odometry2);

        std_msgs::msg::Bool health_status_msg;
        health_status_msg.data = health_status;
        pubHealthStatus->publish(health_status_msg);

        geometry_msgs::msg::TransformStamped transform_stamped_;
        tf2::Transform transform;
        transform_stamped_.header.stamp  = thisImu.header.stamp;
        transform_stamped_.header.frame_id = WORLD_FRAME;
        transform_stamped_.child_frame_id = SENSOR_FRAME;

        tf2::Quaternion q;
        transform.setOrigin(tf2::Vector3(pub_pos.x(),
            pub_pos.y(), pub_pos.z()));

        q.setW(pub_rot.w());
        q.setX(pub_rot.x());
        q.setY(pub_rot.y());
        q.setZ(pub_rot.z());
        transform.setRotation(q);
        transform_stamped_.transform = tf2::toMsg(transform);
        if(frame_count%4==0)
            tfOdom2BaseLink->sendTransform(transform_stamped_);

        // publish IMU path
        static nav_msgs::msg::Path imuPath;
        static double last_path_time = -1;
        double curimuTime = secs(&thisImu);
        if (curimuTime - last_path_time > 0.1)
        {
            last_path_time = curimuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = thisImu.header.stamp;
            pose_stamped.header.frame_id = WORLD_FRAME;
            pose_stamped.pose = odometry2.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while (!imuPath.poses.empty() &&
                   abs(secs(&imuPath.poses.front()) -
                       secs(&imuPath.poses.back())) > 3.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath->get_subscription_count() != 0)
            {
                imuPath.header.stamp = thisImu.header.stamp;
                imuPath.header.frame_id = WORLD_FRAME;
                pubImuPath->publish(imuPath);
            }
        }

        use_laserodom = false;
        use_visualodom = false;

    }

    // ======================== Loop Closure Implementation ========================

    void imuPreintegration::registeredScanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_->clear();
        pcl::fromROSMsg(*msg, *latest_cloud_);
    }

    bool imuPreintegration::shouldAddKeyframe(const gtsam::Pose3& pose) {
        if (global_kf_count_ == 0) return true;

        double dist = (pose.translation() - last_keyframe_pose_.translation()).norm();
        double angle = last_keyframe_pose_.rotation().inverse().compose(pose.rotation()).axisAngle().second;
        return dist > lc_keyframe_dist_ || std::abs(angle) > lc_keyframe_angle_;
    }

    void imuPreintegration::addKeyframe(const gtsam::Pose3& pose, double timestamp) {
        pcl::PointCloud<PointType>::Ptr cloud_copy(new pcl::PointCloud<PointType>());
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (latest_cloud_->empty()) return;

            // The registered scan is in world frame; transform to body frame for storage
            Eigen::Affine3f pose_inv = Eigen::Affine3f::Identity();
            Eigen::Matrix4f mat = pose.matrix().cast<float>();
            pose_inv.matrix() = mat.inverse();
            pcl::transformPointCloud(*latest_cloud_, *cloud_copy, pose_inv);
        }

        // Downsample
        pcl::PointCloud<PointType>::Ptr cloud_dwz(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> dwz;
        dwz.setLeafSize(lc_downsample_res_, lc_downsample_res_, lc_downsample_res_);
        dwz.setInputCloud(cloud_copy);
        dwz.filter(*cloud_dwz);

        Keyframe kf;
        kf.index = global_kf_count_;
        kf.timestamp = timestamp;
        kf.lidar_pose = pose;
        kf.cloud = cloud_dwz;

        {
            std::lock_guard<std::mutex> lock(lc_mutex_);
            keyframe_history_.push_back(kf);

            // Add position to KD-tree cloud
            PointType pt;
            pt.x = pose.translation().x();
            pt.y = pose.translation().y();
            pt.z = pose.translation().z();
            pt.intensity = global_kf_count_;
            kf_positions_->push_back(pt);
        }

        last_keyframe_pose_ = pose;
        global_kf_count_++;
    }

    void imuPreintegration::loopClosureThread() {
        rclcpp::Rate rate(1);  // 1 Hz
        while (!lc_shutdown_) {
            rate.sleep();
            if (rclcpp::ok() == false) break;

            int latest_idx;
            {
                std::lock_guard<std::mutex> lock(lc_mutex_);
                latest_idx = (int)keyframe_history_.size() - 1;
            }
            if (latest_idx < 1 || latest_idx == last_lc_check_idx_) continue;
            last_lc_check_idx_ = latest_idx;

            int match_idx = -1;
            Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
            if (detectAndVerifyLoop(latest_idx, match_idx, correction)) {
                std::lock_guard<std::mutex> lock(lc_mutex_);
                // Compute corrected pose: ICP gives transform from source(current) to target(historical)
                // corrected_world_pose = correction * current_world_pose
                Eigen::Matrix4f current_mat = keyframe_history_[latest_idx].lidar_pose.matrix().cast<float>();
                Eigen::Matrix4f corrected_mat = correction * current_mat;

                gtsam::Pose3 corrected_pose(corrected_mat.cast<double>());

                LoopResult result;
                result.from_idx = match_idx;
                result.to_idx = latest_idx;
                result.corrected_pose = corrected_pose;
                result.fitness = 0.0;  // set by detectAndVerifyLoop via log
                lc_queue_.push_back(result);

                RCLCPP_WARN(this->get_logger(),
                    "Loop closure detected! keyframe %d <-> %d", match_idx, latest_idx);
            }
        }
    }

    bool imuPreintegration::detectAndVerifyLoop(int current_kf_idx, int& match_idx, Eigen::Matrix4f& correction) {
        // Lock to read keyframe data
        Keyframe current_kf;
        {
            std::lock_guard<std::mutex> lock(lc_mutex_);
            if (current_kf_idx >= (int)keyframe_history_.size()) return false;
            current_kf = keyframe_history_[current_kf_idx];

            if (kf_positions_->size() < 2) return false;
        }

        // Build KD-tree and search for candidates
        pcl::KdTreeFLANN<PointType> tree;
        pcl::PointCloud<PointType>::Ptr positions_copy;
        {
            std::lock_guard<std::mutex> lock(lc_mutex_);
            positions_copy.reset(new pcl::PointCloud<PointType>(*kf_positions_));
        }
        tree.setInputCloud(positions_copy);

        PointType search_pt;
        search_pt.x = current_kf.lidar_pose.translation().x();
        search_pt.y = current_kf.lidar_pose.translation().y();
        search_pt.z = current_kf.lidar_pose.translation().z();

        std::vector<int> indices;
        std::vector<float> distances;
        tree.radiusSearch(search_pt, lc_search_radius_, indices, distances);

        // Find best candidate: within radius AND sufficient time difference
        int best_candidate = -1;
        float best_dist = std::numeric_limits<float>::max();
        {
            std::lock_guard<std::mutex> lock(lc_mutex_);
            for (int idx : indices) {
                if (idx >= (int)keyframe_history_.size()) continue;
                double time_diff = std::abs(current_kf.timestamp - keyframe_history_[idx].timestamp);
                if (time_diff < lc_time_diff_) continue;  // too recent, skip
                if (distances[&idx - &indices[0]] < best_dist) {
                    best_dist = distances[&idx - &indices[0]];
                    best_candidate = idx;
                }
            }
        }

        if (best_candidate < 0) return false;

        // Build target cloud from historical keyframes around the candidate
        pcl::PointCloud<PointType>::Ptr target_cloud(new pcl::PointCloud<PointType>());
        {
            std::lock_guard<std::mutex> lock(lc_mutex_);
            int half = lc_history_search_num_ / 2;
            int start = std::max(0, best_candidate - half);
            int end = std::min((int)keyframe_history_.size() - 1, best_candidate + half);

            for (int i = start; i <= end; i++) {
                // Skip keyframes too close in time to current (they may have same drift)
                if (std::abs(keyframe_history_[i].timestamp - current_kf.timestamp) < lc_time_diff_)
                    continue;

                pcl::PointCloud<PointType>::Ptr transformed(new pcl::PointCloud<PointType>());
                Eigen::Matrix4f pose_mat = keyframe_history_[i].lidar_pose.matrix().cast<float>();
                pcl::transformPointCloud(*keyframe_history_[i].cloud, *transformed, pose_mat);
                *target_cloud += *transformed;
            }
        }

        if (target_cloud->empty()) return false;

        // Downsample target
        pcl::PointCloud<PointType>::Ptr target_dwz(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> dwz;
        dwz.setLeafSize(lc_downsample_res_, lc_downsample_res_, lc_downsample_res_);
        dwz.setInputCloud(target_cloud);
        dwz.filter(*target_dwz);

        // Build source cloud: current keyframe in world frame
        pcl::PointCloud<PointType>::Ptr source_cloud(new pcl::PointCloud<PointType>());
        {
            Eigen::Matrix4f pose_mat = current_kf.lidar_pose.matrix().cast<float>();
            pcl::transformPointCloud(*current_kf.cloud, *source_cloud, pose_mat);
        }

        // Run ICP
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(lc_search_radius_ * 2.0);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_dwz);

        pcl::PointCloud<PointType> aligned;
        icp.align(aligned);

        if (!icp.hasConverged()) {
            RCLCPP_INFO(this->get_logger(), "Loop ICP did not converge, kf %d <-> %d",
                        best_candidate, current_kf_idx);
            return false;
        }

        float fitness = icp.getFitnessScore();
        if (fitness > lc_fitness_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Loop ICP fitness too high: %f > %f, kf %d <-> %d",
                        fitness, lc_fitness_threshold_, best_candidate, current_kf_idx);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loop ICP converged! fitness: %f, kf %d <-> %d",
                    fitness, best_candidate, current_kf_idx);

        match_idx = best_candidate;
        correction = icp.getFinalTransformation();  // transforms source to align with target
        return true;
    }

    void imuPreintegration::applyLoopClosure() {
        LoopResult result;
        {
            std::lock_guard<std::mutex> lock(lc_mutex_);
            if (lc_queue_.empty()) return;
            result = lc_queue_.front();
            lc_queue_.pop_front();
        }

        // Apply corrected pose as a tight prior on the current GTSAM key
        gtsam::Pose3 corrected_imu_pose = result.corrected_pose.compose(lidar2Imu);
        auto lc_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());

        graphFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(key - 1), corrected_imu_pose, lc_noise));

        // Run ISAM2 update to incorporate the correction
        try {
            optimizer.update(graphFactors, graphValues);
            optimizer.update();  // extra iteration for convergence
            graphFactors.resize(0);
            graphValues.clear();

            gtsam::Values result_vals = optimizer.calculateEstimate();
            prevPose_ = result_vals.at<gtsam::Pose3>(X(key - 1));
            prevVel_ = result_vals.at<gtsam::Vector3>(V(key - 1));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result_vals.at<gtsam::imuBias::ConstantBias>(B(key - 1));

            // Reset IMU preintegration with updated bias
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            RCLCPP_WARN(this->get_logger(), "Loop closure correction applied! kf %d <-> %d",
                        result.from_idx, result.to_idx);
        } catch (const gtsam::IndeterminantLinearSystemException& e) {
            RCLCPP_WARN(this->get_logger(), "Loop closure ISAM2 update failed: %s", e.what());
            graphFactors.resize(0);
            graphValues.clear();
        }
    }

} // end namespace arise_slam
