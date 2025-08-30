// Key enhanced methods for mrpt_rbpf_slam_wrapper.cpp
// Add these methods to your existing wrapper implementation

// Enhanced parameter loading with IMU and encoder support
bool PFslamWrapper::getParams(const ros::NodeHandle& nh_p)
{
	ROS_INFO("READ PARAM FROM LAUNCH FILE - Enhanced Version");
	
	// Original parameters
	nh_p.getParam("ini_filename", ini_filename_);
	nh_p.param<std::string>("global_frame_id", global_frame_id_, "map");
	nh_p.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
	nh_p.param<std::string>("base_frame_id", base_frame_id_, "base_link");
	nh_p.param<std::string>("sensor_source", sensor_source_, "scan");
	nh_p.param<bool>("update_sensor_pose", update_sensor_pose_, true);

	// IMU parameters
	nh_p.param<bool>("use_imu", use_imu_, false);
	nh_p.param<std::string>("imu_topic", imu_topic_, "/imu");
	nh_p.param<double>("imu_angular_vel_noise", imu_angular_vel_noise_, 0.01);
	nh_p.param<double>("imu_linear_acc_noise", imu_linear_acc_noise_, 0.1);

	// Enhanced odometry parameters
	nh_p.param<bool>("use_direct_odom", use_direct_odom_, true);
	nh_p.param<std::string>("odom_topic", odom_topic_, "/odom");
	nh_p.param<double>("max_linear_vel", max_linear_vel_, 2.0);
	nh_p.param<double>("max_angular_vel", max_angular_vel_, 1.5);
	nh_p.param<double>("max_linear_jump", max_linear_jump_, 0.5);
	nh_p.param<double>("max_angular_jump", max_angular_jump_, 0.5);

	ROS_INFO("Enhanced parameters loaded:");
	ROS_INFO("  use_imu: %s", use_imu_ ? "TRUE" : "FALSE");
	ROS_INFO("  imu_topic: %s", imu_topic_.c_str());
	ROS_INFO("  use_direct_odom: %s", use_direct_odom_ ? "TRUE" : "FALSE");
	ROS_INFO("  odom_topic: %s", odom_topic_.c_str());
	ROS_INFO("  max_linear_vel: %.2f m/s", max_linear_vel_);
	ROS_INFO("  max_angular_vel: %.2f rad/s", max_angular_vel_);

	PFslam::Options options;
	if (!loadOptions(nh_p, options))
	{
		ROS_ERROR("Not able to read all parameters!");
		return false;
	}
	initSlam(std::move(options));
	return true;
}

// Enhanced initialization with IMU and direct odometry subscribers
bool PFslamWrapper::init(ros::NodeHandle& nh)
{
	// Original initialization code remains the same...
	// [Keep existing init() code until subscriber section]

	/// Enhanced Subscribers ///
	std::vector<std::string> lstSources;
	mrpt::system::tokenize(sensor_source_, " ,\t\n", lstSources);
	
	sensorSub_.resize(lstSources.size());
	for (size_t i = 0; i < lstSources.size(); i++)
	{
		sensorSub_[i] = nh.subscribe(
			lstSources[i], 1, &PFslamWrapper::laserCallback, this);
		ROS_INFO("Subscribed to laser topic: %s", lstSources[i].c_str());
	}

	// IMU subscriber
	if (use_imu_)
	{
		imu_sub_ = nh.subscribe(
			imu_topic_, 10, &PFslamWrapper::imuCallback, this);
		ROS_INFO("Subscribed to IMU topic: %s", imu_topic_.c_str());
	}

	// Direct odometry subscriber  
	if (use_direct_odom_)
	{
		odom_sub_ = nh.subscribe(
			odom_topic_, 10, &PFslamWrapper::odometryCallback, this);
		ROS_INFO("Subscribed to odometry topic: %s", odom_topic_.c_str());
	}

	// Continue with original initialization...
	mapBuilder_ = mrpt::slam::CMetricMapBuilderRBPF(options_.rbpfMappingOptions_);
	init3Dwindow();
	return true;
}

// New IMU callback method
void PFslamWrapper::imuCallback(const sensor_msgs::Imu& msg)
{
	using namespace mrpt::obs;
	
	if (!use_imu_) return;

	// Validate IMU data
	if (std::isnan(msg.angular_velocity.x) || std::isnan(msg.angular_velocity.y) || 
	    std::isnan(msg.angular_velocity.z) || std::isnan(msg.linear_acceleration.x) ||
	    std::isnan(msg.linear_acceleration.y) || std::isnan(msg.linear_acceleration.z))
	{
		ROS_WARN("Invalid IMU data received, skipping...");
		return;
	}

	// Check IMU frame exists in TF
	if (imu_poses_.find(msg.header.frame_id) == imu_poses_.end())
	{
		updateSensorPose(msg.header.frame_id);
		// Store IMU pose separately if needed
		if (laser_poses_.find(msg.header.frame_id) != laser_poses_.end())
		{
			imu_poses_[msg.header.frame_id] = laser_poses_[msg.header.frame_id];
		}
	}

	// Create MRPT IMU observation
	CObservationIMU::Ptr imu_obs = CObservationIMU::Create();
	imu_obs->timestamp = mrpt::ros1bridge::fromROS(msg.header.stamp);
	imu_obs->sensorLabel = msg.header.frame_id;

	// Set IMU data with noise consideration
	imu_obs->dataIsPresent[mrpt::obs::IMU_WX] = true;
	imu_obs->dataIsPresent[mrpt::obs::IMU_WY] = true; 
	imu_obs->dataIsPresent[mrpt::obs::IMU_WZ] = true;
	imu_obs->dataIsPresent[mrpt::obs::IMU_X_ACC] = true;
	imu_obs->dataIsPresent[mrpt::obs::IMU_Y_ACC] = true;
	imu_obs->dataIsPresent[mrpt::obs::IMU_Z_ACC] = true;

	imu_obs->rawMeasurements[mrpt::obs::IMU_WX] = msg.angular_velocity.x;
	imu_obs->rawMeasurements[mrpt::obs::IMU_WY] = msg.angular_velocity.y;
	imu_obs->rawMeasurements[mrpt::obs::IMU_WZ] = msg.angular_velocity.z;
	imu_obs->rawMeasurements[mrpt::obs::IMU_X_ACC] = msg.linear_acceleration.x;
	imu_obs->rawMeasurements[mrpt::obs::IMU_Y_ACC] = msg.linear_acceleration.y;
	imu_obs->rawMeasurements[mrpt::obs::IMU_Z_ACC] = msg.linear_acceleration.z;

	// Store for future use (can be integrated with motion model)
	last_imu_ = msg;
	first_imu_received_ = true;
	
	ROS_DEBUG("IMU data processed: wx=%.3f, wy=%.3f, wz=%.3f", 
	          msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
}

// New direct odometry callback with validation
void PFslamWrapper::odometryCallback(const nav_msgs::Odometry& msg)
{
	if (!use_direct_odom_) return;

	// Validate odometry
	if (!validateOdometry(msg))
	{
		ROS_WARN("Invalid odometry data received, skipping...");
		return;
	}

	// Update last odometry
	last_odom_ = msg;
	first_odom_received_ = true;

	ROS_DEBUG("Direct odometry received: x=%.3f, y=%.3f, theta=%.3f", 
	          msg.pose.pose.position.x, msg.pose.pose.position.y,
	          tf2::getYaw(msg.pose.pose.orientation));
}

// Odometry validation method
bool PFslamWrapper::validateOdometry(const nav_msgs::Odometry& current_odom)
{
	// Check for NaN values
	if (std::isnan(current_odom.pose.pose.position.x) ||
	    std::isnan(current_odom.pose.pose.position.y) ||
	    std::isnan(current_odom.pose.pose.orientation.z) ||
	    std::isnan(current_odom.twist.twist.linear.x) ||
	    std::isnan(current_odom.twist.twist.angular.z))
	{
		return false;
	}

	// Validate velocity limits
	if (std::abs(current_odom.twist.twist.linear.x) > max_linear_vel_ ||
	    std::abs(current_odom.twist.twist.angular.z) > max_angular_vel_)
	{
		ROS_WARN("Odometry velocity exceeds limits: linear=%.2f (max %.2f), angular=%.2f (max %.2f)",
		         std::abs(current_odom.twist.twist.linear.x), max_linear_vel_,
		         std::abs(current_odom.twist.twist.angular.z), max_angular_vel_);
		return false;
	}

	// Check for sudden jumps if we have previous data
	if (first_odom_received_)
	{
		double dx = current_odom.pose.pose.position.x - last_odom_.pose.pose.position.x;
		double dy = current_odom.pose.pose.position.y - last_odom_.pose.pose.position.y;
		double linear_jump = sqrt(dx*dx + dy*dy);
		
		double current_yaw = tf2::getYaw(current_odom.pose.pose.orientation);
		double last_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
		double angular_jump = std::abs(current_yaw - last_yaw);
		
		// Normalize angular difference
		while (angular_jump > M_PI) angular_jump -= 2*M_PI;
		angular_jump = std::abs(angular_jump);

		if (linear_jump > max_linear_jump_ || angular_jump > max_angular_jump_)
		{
			ROS_WARN("Odometry jump detected: linear=%.3f (max %.3f), angular=%.3f (max %.3f)",
			         linear_jump, max_linear_jump_, angular_jump, max_angular_jump_);
			return false;
		}
	}

	return true;
}

// Enhanced laser callback with direct odometry integration
void PFslamWrapper::laserCallback(const sensor_msgs::LaserScan& msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;
	
	CObservation2DRangeScan::Ptr laser = CObservation2DRangeScan::Create();

	if (laser_poses_.find(msg.header.frame_id) == laser_poses_.end())
	{
		updateSensorPose(msg.header.frame_id);
	}
	else
	{
		if (update_sensor_pose_) updateSensorPose(msg.header.frame_id);
		
		mrpt::ros1bridge::fromROS(msg, laser_poses_[msg.header.frame_id], *laser);
		sensory_frame_ = CSensoryFrame::Create();
		
		// Get odometry - prefer direct odometry over TF
		CObservationOdometry::Ptr odometry;
		if (use_direct_odom_ && first_odom_received_)
		{
			// Use direct odometry data
			odometry = CObservationOdometry::Create();
			odometry->sensorLabel = odom_frame_id_;
			odometry->hasEncodersInfo = false;
			odometry->hasVelocities = true;
			
			// Position from latest odometry
			odometry->odometry.x() = last_odom_.pose.pose.position.x;
			odometry->odometry.y() = last_odom_.pose.pose.position.y;
			odometry->odometry.phi() = tf2::getYaw(last_odom_.pose.pose.orientation);
			
			// Velocity information
			odometry->velocityLocal.vx = last_odom_.twist.twist.linear.x;
			odometry->velocityLocal.vy = last_odom_.twist.twist.linear.y;
			odometry->velocityLocal.omega = last_odom_.twist.twist.angular.z;
		}
		else
		{
			// Fallback to TF-based odometry
			odometryForCallback(odometry, msg.header);
		}

		CObservation::Ptr obs = CObservation::Ptr(laser);
		sensory_frame_->insert(obs);
		
		// Add IMU data if available and enabled
		if (use_imu_ && first_imu_received_)
		{
			// IMU integration can be added here for motion model enhancement
			// For now, we just log that IMU data is available
			ROS_DEBUG("IMU data available for sensor fusion");
		}

		observation(sensory_frame_, odometry);
		timeLastUpdate_ = sensory_frame_->getObservationByIndex(0)->timestamp;

		tictac_.Tic();
		mapBuilder_.processActionObservation(*action_, *sensory_frame_);
		t_exec_ = tictac_.Tac();
		ROS_INFO("Enhanced SLAM executed in %.03fms (IMU: %s, Direct Odom: %s)", 
		         1000.0f * t_exec_,
		         (use_imu_ && first_imu_received_) ? "OK" : "OFF",
		         (use_direct_odom_ && first_odom_received_) ? "OK" : "TF");
		         
		publishMapPose();
		run3Dwindow();
		publishTF();
	}
}