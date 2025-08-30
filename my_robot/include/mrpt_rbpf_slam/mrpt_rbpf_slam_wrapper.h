/*
 * File: mrpt_rbpf_slam_wrapper.h
 * Author: Vladislav Tananaev
 * Enhanced with IMU integration and encoder validation
 */

#pragma once

#include <iostream>	 // std::cout
#include <fstream>	// std::ifstream
#include <string>
#include "mrpt_rbpf_slam/mrpt_rbpf_slam.h"

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
// add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>  // IMU support added
#include <nav_msgs/Odometry.h>  // Direct odometry support
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// mrpt msgs
// mrpt bridge libs
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/ros1bridge/logging.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/time.h>

#include <mrpt/obs/CObservationIMU.h>  // IMU observation support

namespace mrpt_rbpf_slam
{
/**
 * @brief The PFslamWrapper class provides  the ROS wrapper for
 *Rao-Blackwellized Particle filter SLAM from MRPT libraries.
 *Enhanced with IMU integration and encoder validation
 */
class PFslamWrapper : public PFslam
{
   public:
	PFslamWrapper();
	~PFslamWrapper() = default;

	/**
	 * @brief Read the parameters from launch file
	 */
	bool getParams(const ros::NodeHandle& nh_p);

	/**
	 * @brief Initialize publishers subscribers and RBPF slam
	 */
	bool init(ros::NodeHandle& nh);

	/**
	 * @brief Play rawlog file
	 *
	 * @return true if rawlog file exists and played
	 */
	bool rawlogPlay();

	/**
	 * @brief Publish beacon or grid map and robot pose
	 */
	void publishMapPose();

	/**
	 * @brief Callback function for the beacons
	 */
	void laserCallback(const sensor_msgs::LaserScan& msg);

	/**
	 * @brief Callback function for IMU data
	 * 
	 * @param msg IMU sensor data
	 */
	void imuCallback(const sensor_msgs::Imu& msg);

	/**
	 * @brief Callback function for direct odometry data
	 * 
	 * @param msg Odometry data from encoders
	 */
	void odometryCallback(const nav_msgs::Odometry& msg);

	/**
	 * @brief Wait for transform between odometry frame and the robot frame
	 */
	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const ros::Time& time,
		const ros::Duration& timeout,
		const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

	/**
	 * @brief Get the odometry for received observation (legacy TF-based)
	 */
	void odometryForCallback(
		mrpt::obs::CObservationOdometry::Ptr& odometry,
		const std_msgs::Header& msg_header);

	/**
	 * @brief Validate encoder odometry data
	 * 
	 * @param current_odom Current odometry reading
	 * @return true if odometry is valid
	 */
	bool validateOdometry(const nav_msgs::Odometry& current_odom);

	/**
	 * @brief Update the pose of the sensor with respect to the robot
	 */
	void updateSensorPose(const std::string& frame_id);

	/**
	 * @brief Publish tf tree
	 */
	void publishTF();

	/**
	 * @brief Correct visualization for ro slam
	 */
   private:

	std::string ini_filename_;	///< name of ini file
	std::string global_frame_id_;  ///< /map frame
	std::string odom_frame_id_;	 ///< /odom frame
	std::string base_frame_id_;	 ///< robot frame

	// Sensor sources
	std::string sensor_source_;	 ///< 2D laser scans
	std::string imu_topic_;      ///< IMU topic name
	std::string odom_topic_;     ///< Direct odometry topic name
	bool update_sensor_pose_;  ///< on true the sensor pose is updated on every sensor reading
	bool use_imu_;             ///< Enable IMU integration
	bool use_direct_odom_;     ///< Use direct odometry instead of TF

	// Odometry validation parameters
	double max_linear_vel_;      ///< Max linear velocity (m/s)
	double max_angular_vel_;     ///< Max angular velocity (rad/s)
	double max_linear_jump_;     ///< Max linear jump between readings (m)
	double max_angular_jump_;    ///< Max angular jump between readings (rad)
	
	// IMU parameters
	double imu_angular_vel_noise_;   ///< IMU gyroscope noise
	double imu_linear_acc_noise_;    ///< IMU accelerometer noise

	std::map<std::string, mrpt::poses::CPose3D>
		laser_poses_;  ///< laser scan poses with respect to the map
	std::map<std::string, mrpt::poses::CPose3D>
		beacon_poses_;	///< beacon poses with respect to the map
	std::map<std::string, mrpt::poses::CPose3D>
		imu_poses_;    ///< IMU poses with respect to the robot

	// Subscribers
	std::vector<ros::Subscriber> sensorSub_;  ///< list of sensors topics
	ros::Subscriber imu_sub_;                 ///< IMU subscriber
	ros::Subscriber odom_sub_;                ///< Direct odometry subscriber

	// Last readings for validation
	nav_msgs::Odometry last_odom_;            ///< Last odometry reading
	sensor_msgs::Imu last_imu_;               ///< Last IMU reading
	bool first_odom_received_{false};         ///< First odometry flag
	bool first_imu_received_{false};          ///< First IMU flag

	// read rawlog file
	// vector of pairs of actions and obsrvations from rawlog file
	std::vector<
		std::pair<mrpt::obs::CActionCollection, mrpt::obs::CSensoryFrame>>
		data_;

	std::vector<mrpt::opengl::CEllipsoid3D::Ptr> viz_beacons_;

	// publishers for map and pose particles
	ros::Publisher pub_map_, pub_metadata_, pub_particles_,
		pub_particles_beacons_, beacon_viz_pub_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener listenerTF_{tf_buffer_};
	tf2_ros::TransformBroadcaster tf_broadcaster_;	///< transform broadcaster

	mrpt::system::CTicTac tictac_;	///< timer for SLAM performance evaluation
	float t_exec_;	///< the time which take one SLAM update execution
};
}  // namespace mrpt_rbpf_slam