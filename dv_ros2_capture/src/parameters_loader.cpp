#include <dv_ros2_capture/parameters_loader.hpp>

#include <fmt/core.h>
#include <fmt/format.h>

#include <iostream>
#include <string>

using namespace dv_ros2_node;

ParametersLoader::ParametersLoader(rclcpp::Node &node) : node_(node) {
	node_.declare_parameter("frames", params_.frames);
	node_.declare_parameter("events", params_.events);
	node_.declare_parameter("imu", params_.imu);
	node_.declare_parameter("triggers", params_.triggers);
	node_.declare_parameter("cameraName", params_.cameraName);
	node_.declare_parameter("timeIncrement", static_cast<int>(params_.timeIncrement));
	node_.declare_parameter("cameraFrameName", params_.cameraFrameName);
	node_.declare_parameter("imuFrameName", params_.imuFrameName);
	node_.declare_parameter("transformImuToCameraFrame", params_.transformImuToCameraFrame);
	node_.declare_parameter("unbiasedImuData", params_.unbiasedImuData);
	node_.declare_parameter("noiseFiltering", params_.noiseFiltering);
	node_.declare_parameter("noiseBackgroundActivityTime", static_cast<int>(params_.noiseBATime));
	node_.declare_parameter("undistortEvents", params_.undistortEvents);
	node_.declare_parameter("cameraCalibrationFilePath", std::string(""));
	node_.declare_parameter("opencvCalibrationFilePath", std::string(""));
	node_.declare_parameter("aedat4FilePath", std::string(""));
	node_.declare_parameter("syncDevices", std::vector<std::string>{});
	node_.declare_parameter("waitForSync", params_.waitForSync);

	// Camera hardware settings
	node_.declare_parameter("colorMode", params_.colorMode);
	node_.declare_parameter("readoutMode", params_.readoutMode);
	node_.declare_parameter("autoExposure", params_.autoExposure);
	node_.declare_parameter("exposureUs", params_.exposureUs);
	node_.declare_parameter("globalHold", params_.globalHold);
	node_.declare_parameter("contrastThresholdOn", params_.contrastThresholdOn);
	node_.declare_parameter("contrastThresholdOff", params_.contrastThresholdOff);

	node_.declare_parameter("accelerometerBias", std::vector<double>{});
	node_.declare_parameter("gyroscopeBias", std::vector<double>{});

	// DVXplorer/DVXplorerM IMU rate and oversampling filter settings
	node_.declare_parameter("imuAccelDataRate", params_.imuAccelDataRate);
	node_.declare_parameter("imuGyroDataRate", params_.imuGyroDataRate);
	node_.declare_parameter("imuAccelFilter", params_.imuAccelFilter);
	node_.declare_parameter("imuGyroFilter", params_.imuGyroFilter);

	params_.frames = node_.get_parameter("frames").as_bool();
	params_.events = node_.get_parameter("events").as_bool();
	params_.imu    = node_.get_parameter("imu").as_bool();
	params_.triggers = node_.get_parameter("triggers").as_bool();
	params_.cameraName = node_.get_parameter("cameraName").as_string();

	int timeIncrement = node_.get_parameter("timeIncrement").as_int();
	params_.timeIncrement = static_cast<int64_t>(timeIncrement);

	params_.cameraFrameName = node_.get_parameter("cameraFrameName").as_string();
	params_.imuFrameName = node_.get_parameter("imuFrameName").as_string();
	params_.transformImuToCameraFrame = node_.get_parameter("transformImuToCameraFrame").as_bool();
	params_.unbiasedImuData = node_.get_parameter("unbiasedImuData").as_bool();
	params_.noiseFiltering = node_.get_parameter("noiseFiltering").as_bool();

	int noiseBATime = node_.get_parameter("noiseBackgroundActivityTime").as_int();
	params_.noiseBATime = static_cast<int64_t>(noiseBATime);
	params_.undistortEvents = node_.get_parameter("undistortEvents").as_bool();

	std::string tmp = node_.get_parameter("cameraCalibrationFilePath").as_string();
	params_.cameraCalibrationFilePath = static_cast<std::filesystem::path>(tmp);

	params_.opencvCalibrationFilePath
		= static_cast<std::filesystem::path>(node_.get_parameter("opencvCalibrationFilePath").as_string());

	tmp = node_.get_parameter("aedat4FilePath").as_string();
	if (!tmp.empty()) {
		params_.aedat4FilePath = tmp;
		if (!params_.aedat4FilePath.empty() && !std::filesystem::exists(params_.aedat4FilePath)) {
			throw std::invalid_argument(
				fmt::format("File {0} not found. Please provide a correct path in the configuration file.",
					params_.aedat4FilePath.string()));
		}
	}

	params_.syncDeviceList = node_.get_parameter("syncDevices").as_string_array();
	params_.waitForSync = node_.get_parameter("waitForSync").as_bool();

	params_.colorMode            = static_cast<int>(node_.get_parameter("colorMode").as_int());
	params_.readoutMode          = static_cast<int>(node_.get_parameter("readoutMode").as_int());
	params_.autoExposure         = node_.get_parameter("autoExposure").as_bool();
	params_.exposureUs           = static_cast<int>(node_.get_parameter("exposureUs").as_int());
	params_.globalHold           = node_.get_parameter("globalHold").as_bool();
	params_.contrastThresholdOn  = static_cast<int>(node_.get_parameter("contrastThresholdOn").as_int());
	params_.contrastThresholdOff = static_cast<int>(node_.get_parameter("contrastThresholdOff").as_int());

	params_.accelerometerBias = node_.get_parameter("accelerometerBias").as_double_array();
	params_.gyroscopeBias     = node_.get_parameter("gyroscopeBias").as_double_array();

	params_.imuAccelDataRate = node_.get_parameter("imuAccelDataRate").as_string();
	params_.imuGyroDataRate  = node_.get_parameter("imuGyroDataRate").as_string();
	params_.imuAccelFilter   = node_.get_parameter("imuAccelFilter").as_string();
	params_.imuGyroFilter    = node_.get_parameter("imuGyroFilter").as_string();
	if (!params_.accelerometerBias.empty() && params_.accelerometerBias.size() != 3) {
		throw std::invalid_argument(
			fmt::format("accelerometerBias must have exactly 3 entries [x, y, z], got {}",
				params_.accelerometerBias.size()));
	}
	if (!params_.gyroscopeBias.empty() && params_.gyroscopeBias.size() != 3) {
		throw std::invalid_argument(
			fmt::format("gyroscopeBias must have exactly 3 entries [x, y, z], got {}",
				params_.gyroscopeBias.size()));
	}
}

Params ParametersLoader::getParams() {
	return params_;
}

void ParametersLoader::printConfiguration() {
	RCLCPP_INFO(node_.get_logger(), ">>>>>> Capture node parameter settings: <<<<<<");
	RCLCPP_INFO(node_.get_logger(), "Frames enabled: %s", params_.frames ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "Events enabled: %s", params_.events ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "Imu enabled: %s", params_.imu ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "Convert IMU to camera frame: %s", params_.transformImuToCameraFrame ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "Triggers enabled: %s", params_.triggers ? "yes" : "no");
	if (!params_.aedat4FilePath.empty()) {
		RCLCPP_INFO(node_.get_logger(), "aedat4FilePath: %s", params_.aedat4FilePath.c_str());
	}
	if (!params_.cameraName.empty()) {
		RCLCPP_INFO(node_.get_logger(), "cameraName: %s", params_.cameraName.c_str());
	}
	RCLCPP_INFO(node_.get_logger(), "cameraCalibrationFilePath: %s", params_.cameraCalibrationFilePath.c_str());
	RCLCPP_INFO(node_.get_logger(), "opencvCalibrationFilePath: %s", params_.opencvCalibrationFilePath.c_str());
	RCLCPP_INFO(node_.get_logger(), "noiseFiltering: %s", params_.noiseFiltering ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "noiseBackgroundActivityTime: %ld", params_.noiseBATime);
	RCLCPP_INFO(node_.get_logger(), "undistortEvents: %s", params_.undistortEvents ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "syncDevices: [%s]", fmt::format("{}", fmt::join(params_.syncDeviceList, ", ")).c_str());
	RCLCPP_INFO(node_.get_logger(), "waitForSync: %s", params_.waitForSync ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "colorMode: %d", params_.colorMode);
	RCLCPP_INFO(node_.get_logger(), "readoutMode: %d", params_.readoutMode);
	RCLCPP_INFO(node_.get_logger(), "autoExposure: %s", params_.autoExposure ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "exposureUs: %d", params_.exposureUs);
	RCLCPP_INFO(node_.get_logger(), "globalHold: %s", params_.globalHold ? "yes" : "no");
	RCLCPP_INFO(node_.get_logger(), "contrastThresholdOn: %d", params_.contrastThresholdOn);
	RCLCPP_INFO(node_.get_logger(), "contrastThresholdOff: %d", params_.contrastThresholdOff);
	RCLCPP_INFO(node_.get_logger(), "imuAccelDataRate: %s (filter: %s)",
		params_.imuAccelDataRate.c_str(), params_.imuAccelFilter.c_str());
	RCLCPP_INFO(node_.get_logger(), "imuGyroDataRate:  %s (filter: %s)",
		params_.imuGyroDataRate.c_str(), params_.imuGyroFilter.c_str());
	if (params_.accelerometerBias.size() == 3) {
		RCLCPP_INFO(node_.get_logger(), "accelerometerBias (m/s^2): [%+.6f, %+.6f, %+.6f]",
			params_.accelerometerBias[0], params_.accelerometerBias[1], params_.accelerometerBias[2]);
	}
	if (params_.gyroscopeBias.size() == 3) {
		RCLCPP_INFO(node_.get_logger(), "gyroscopeBias (rad/s):    [%+.6f, %+.6f, %+.6f]",
			params_.gyroscopeBias[0], params_.gyroscopeBias[1], params_.gyroscopeBias[2]);
	}
	RCLCPP_INFO(node_.get_logger(), ">>>>>> End of parameters <<<<<<");
}
