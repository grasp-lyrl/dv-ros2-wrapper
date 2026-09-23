#pragma once

#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/core/core.hpp>
#include <dv-processing/io/camera/davis.hpp>
#include <dv-processing/io/camera/dvxplorer.hpp>
#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>

#include <dv_ros2_messaging/messaging.hpp>

#include "parameters_loader.hpp"

#include <boost/lockfree/spsc_queue.hpp>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <dv_ros2_capture/msg/camera_discovery.hpp>
#include <dv_ros2_capture/srv/set_imu_biases.hpp>
#include <dv_ros2_capture/srv/set_imu_info.hpp>
#include <dv_ros2_capture/srv/synchronize_camera.hpp>

#include <thread>

namespace dv_capture_node {
using TimestampQueue = boost::lockfree::spsc_queue<int64_t, boost::lockfree::capacity<1000>>;

using TransformMessage  = geometry_msgs::msg::TransformStamped;
using TransformsMessage = tf2_msgs::msg::TFMessage;
using DiscoveryMessage  = dv_ros2_capture::msg::CameraDiscovery;

namespace fs = std::filesystem;

/**
 * Initialize ros publishers to stream data from a DV camera or a aedat4 file.
 */
class CaptureNode : public rclcpp::Node {
public:
	/**
	 * Initialize the publisher according to node options.
	 * Parameters are loaded from the ROS2 parameter server.
	 * @param options Node options forwarded from the component container.
	 */
	explicit CaptureNode(const rclcpp::NodeOptions &options);

	/**
	 * Stop the running threads.
	 */
	~CaptureNode();

	/**
	 * Start the threads for reading the data.
	 */
	void startCapture();

	/**
	 * Stop the running threads.
	 */
	void stop();

	/**
	 * Check whether read threads are still running
	 * @return True if capture threads are still running, false otherwise.
	 */
	[[nodiscard]] bool isRunning() const;

private:
	// Publishers declaration.
	rclcpp::Publisher<dv_ros2_msgs::ImageMessage>::SharedPtr mFramePublisher;
	rclcpp::Publisher<dv_ros2_msgs::CameraInfoMessage>::SharedPtr mCameraInfoPublisher;
	rclcpp::Publisher<dv_ros2_msgs::ImuMessage>::SharedPtr mImuPublisher;
	rclcpp::Publisher<dv_ros2_msgs::EventArrayMessage>::SharedPtr mEventArrayPublisher;
	rclcpp::Publisher<dv_ros2_msgs::TriggerMessage>::SharedPtr mTriggerPublisher;
	rclcpp::Publisher<TransformsMessage>::SharedPtr mTransformPublisher;
	rclcpp::Publisher<DiscoveryMessage>::SharedPtr mDiscoveryPublisher;

	rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr mCameraService;
	rclcpp::Service<dv_ros2_capture::srv::SetImuInfo>::SharedPtr mImuInfoService;
	rclcpp::Service<dv_ros2_capture::srv::SetImuBiases>::SharedPtr mImuBiasesService;
	rclcpp::Service<dv_ros2_capture::srv::SynchronizeCamera>::SharedPtr mSyncServerService;

	// Declare the Reader to read from the device or from a recording.
	std::unique_ptr<dv::io::InputBase> mReader;

	// Parameters read from the configuration file. Set to true the type of data that needs to be streamed.
	dv_ros2_node::Params mParams;

	// Camera info message buffer
	dv_ros2_msgs::CameraInfoMessage mCameraInfoMsg;
	dv_ros2_msgs::CameraInfoMessage mRawCameraInfoMsg;
	cv::Mat mUndistortMap;
	std::unique_ptr<std::thread> mCameraInfoThread = nullptr;

	// threads related
	std::thread mFrameThread;
	TimestampQueue mFrameQueue;
	std::thread mImuThread;
	TimestampQueue mImuQueue;
	std::thread mEventsThread;
	TimestampQueue mEventsQueue;
	std::thread mTriggerThread;
	TimestampQueue mTriggerQueue;
	std::atomic<bool> mSpinThread = true;
	std::thread mClock;
	std::thread mSyncThread;
	std::unique_ptr<std::thread> mDiscoveryThread = nullptr;
	std::recursive_mutex mReaderMutex;

	std::atomic<bool> mSynchronized;

	std::unique_ptr<dv::noise::BackgroundActivityNoiseFilter<>> mNoiseFilter = nullptr;
	void updateNoiseFilter(const bool enable, const int64_t backgroundActivityTime);

	dv::camera::CalibrationSet mCalibration;

	int64_t mImuTimeOffset      = 0;
	Eigen::Vector3f mAccBiases  = Eigen::Vector3f::Zero();
	Eigen::Vector3f mGyroBiases = Eigen::Vector3f::Zero();
	rclcpp::Time startupTime;
	std::atomic<int64_t> mCurrentSeek;
	std::optional<TransformsMessage> mImuToCamTransforms = std::nullopt;
	dv::kinematics::Transformationf mImuToCamTransform
		= dv::kinematics::Transformationf(0, Eigen::Vector3f::Zero(), Eigen::Quaternion<float>::Identity());

	/**
	 * Publish the images and camera info if mFrameBool is True
	 */
	void framePublisher();
	void setCameraInfo(
		const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
		std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> rsp);

	/**
	 * Publish the imu message if mImuBool is True
	 */
	void imuPublisher();
	void setImuInfo(
		const std::shared_ptr<dv_ros2_capture::srv::SetImuInfo::Request> req,
		std::shared_ptr<dv_ros2_capture::srv::SetImuInfo::Response> rsp);
	void setImuBiases(
		const std::shared_ptr<dv_ros2_capture::srv::SetImuBiases::Request> req,
		std::shared_ptr<dv_ros2_capture::srv::SetImuBiases::Response> rsp);

	/**
	 * Publish the events if mEventsBool is True
	 */
	void eventsPublisher();

	/**
	 * Publish the triggers if mTriggerBool is True
	 */
	void triggerPublisher();

	/**
	 * Start a clock thread that gives time synchronization to all the other threads.
	 */
	void clock(int64_t start, int64_t end, int64_t timeIncrement);

	/**
	 * Create a camera info message.
	 */
	void populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry);

	/**
	 * Update the event undistortion lookup table and the published camera intrinsics.
	 */
	void updateEventUndistortionParams();

	/**
	 * Replace the camera intrinsics with those in an OpenCV FileStorage calibration and
	 * rebuild the undistortion map from them.
	 * @param path OpenCV XML/YAML holding camera_matrix and distortion_coefficients.
	 */
	void loadOpenCvCalibration(const fs::path &path);

	/**
	 * Undistort event coordinates using the stored lookup table.
	 */
	[[nodiscard]] dv::EventStore undistortEvents(const dv::EventStore &events);

	/**
	 * Convert the imu message frame into the camera frame if the transformation exists.
	 */
	[[nodiscard]] inline dv_ros2_msgs::ImuMessage transformImuFrame(dv_ros2_msgs::ImuMessage &&imu);

	/**
	 * Generate the CalibrationSet with the data from the Set Camera Info and the set IMU services.
	 */
	void updateCalibrationSet();

	/**
	 * Stores the calibration data into a new file.
	 */
	[[nodiscard]] fs::path saveCalibration();

	/**
	 * Write current capture node calibration parameters into an active calibration file.
	 */
	void generateActiveCalibrationFile();

	[[nodiscard]] fs::path getActiveCalibrationPath() const;

	fs::path getCameraCalibrationDirectory(bool createDirectories = true) const;

	void runDiscovery(const std::string &syncServiceName);

	void synchronizeCamera(
		const std::shared_ptr<dv_ros2_capture::srv::SynchronizeCamera::Request> req,
		std::shared_ptr<dv_ros2_capture::srv::SynchronizeCamera::Response> rsp);

	[[nodiscard]] std::map<std::string, std::string> discoverSyncDevices();

	void sendSyncCalls(const std::map<std::string, std::string> &serviceNames);

	void synchronizationThread();

	/**
	 * Apply initial camera hardware settings and set up parameter change callback.
	 */
	void configureCameraHardware();

	/**
	 * Callback for dynamic parameter changes (exposure, thresholds, noise, etc.).
	 */
	rcl_interfaces::msg::SetParametersResult onParameterChange(
		const std::vector<rclcpp::Parameter> &parameters);

	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mParamCallbackHandle;
};

} // namespace dv_capture_node
