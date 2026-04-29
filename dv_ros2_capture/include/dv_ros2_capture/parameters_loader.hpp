#pragma once

#include <rclcpp/rclcpp.hpp>

#include <filesystem>

namespace dv_ros2_node {
/**
 * Struct containing the parameters
 */
struct Params {
	int64_t timeIncrement = 1000;
	bool frames           = true;
	bool events           = true;
	bool imu              = true;
	bool triggers         = true;
	std::string cameraName;
	std::filesystem::path aedat4FilePath;
	std::filesystem::path cameraCalibrationFilePath;
	std::string cameraFrameName    = "camera";
	std::string imuFrameName       = "imu";
	bool transformImuToCameraFrame = true;
	bool unbiasedImuData           = true;
	bool noiseFiltering            = false;
	int64_t noiseBATime            = 2000;
	bool undistortEvents           = false;

	std::vector<std::string> syncDeviceList;
	bool waitForSync = false;

	// DAVIS-specific settings
	int colorMode    = 0; // 0=DEFAULT, 1=GRAYSCALE, 2=ORIGINAL, 3=ORIGINAL_SPLIT
	int readoutMode  = 0; // 0=both, 1=events only, 2=frames only
	bool autoExposure = true;
	int exposureUs    = 20000; // microseconds

	// DVXplorer-specific settings
	bool globalHold         = true;
	int contrastThresholdOn  = 9;
	int contrastThresholdOff = 9;

	// Static IMU bias overrides. Empty = fall back to calibration file values.
	// Order: [x, y, z]. Accel in m/s^2, gyro in rad/s. These are the values
	// that get *subtracted* from raw IMU samples when unbiasedImuData is true.
	std::vector<double> accelerometerBias;
	std::vector<double> gyroscopeBias;
};

/**
 * Read the parameters from ROS2 parameter server.
 */
class ParametersLoader {
public:
	/**
	 * Load parameters from ROS2 node parameter declarations.
	 *
	 * @param node rclcpp::Node reference
	 */
	explicit ParametersLoader(rclcpp::Node &node);

	/**
	 * Return the parameters read from the parameter server.
	 *
	 * @return dv_ros2_node::Params
	 */
	Params getParams();

	/**
	 * Log read data with RCLCPP_INFO
	 */
	void printConfiguration();

private:
	Params params_;
	rclcpp::Node &node_;
};

} // namespace dv_ros2_node
