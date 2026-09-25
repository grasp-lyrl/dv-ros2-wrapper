#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace dv_monodepth_node {

/// Node parameters, all settable as ROS2 parameters.
struct MetricDepthParams {
	/// OpenCV FileStorage calibration; empty loads dv_ros2_capture's calib_40deg.xml.
	std::string calibrationFile = "";
	/// Camera height above the floor, in metres, until a height message arrives.
	double cameraHeight = 1.0;
	/// sensor_msgs/Range to take the height from; empty keeps cameraHeight.
	std::string heightTopic = "";
	/// Written where depth is invalid.
	double invalidDepth = 0.1;
	/// Depths beyond this are invalid; 0 keeps them all.
	double maxDepth      = 5.0;
	int ransacIterations = 64;
	/// Inlier band, as a fraction of the floor candidates' 2-98 percentile disparity spread.
	double ransacTolerance = 0.026;
	/// How long the last fit stands in for frames without one; 0 drops them.
	int fitHoldMs = 1000;
};

/// The floor line `d = slope * (r·g) + shift`; the depth scale is `slope * height`.
struct FloorFit {
	double slope = 0.0;
	double shift = 0.0;
	/// Disparity band around the line that counts as floor.
	double tolerance = 0.0;
};

/**
 * Metric depth from relative disparity `d = s / z + t`, with `s` and `t` fitted per frame by
 * RANSAC on the floor seen by a level camera at a known height.
 */
class MetricDepthNode : public rclcpp::Node {
public:
	explicit MetricDepthNode(const rclcpp::NodeOptions &options);

private:
	void readParameters();

	/// Unproject every pixel once, through the camera's distortion.
	void unprojectPixels();

	void heightCallback(const sensor_msgs::msg::Range::ConstSharedPtr &range);

	void disparityCallback(const sensor_msgs::msg::Image::ConstSharedPtr &disparity);

	/// Fit the floor line; empty if no line has enough support.
	[[nodiscard]] std::optional<FloorFit> fitFloor(const float *disparity);

	void reportStats();

	MetricDepthParams mParams;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDisparitySubscriber;
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr mHeightSubscriber;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDepthPublisher;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mDepthInfoPublisher;

	sensor_msgs::msg::CameraInfo mCameraInfo;
	double mHeight = 0.0;

	/// Per pixel, r·g for its z = 1 ray; 0 where the ray cannot reach the floor.
	std::vector<float> mFloorRay;
	/// Pixels whose ray can reach the floor.
	std::vector<uint32_t> mFloorPixels;

	std::vector<float> mCandidateW;
	std::vector<float> mCandidateD;
	std::vector<uint32_t> mScored;
	std::vector<float> mSpread;

	std::mt19937 mRng{0};

	std::optional<FloorFit> mLastFit;
	int64_t mLastFitNs = 0;

	// Fit stats, logged periodically.
	std::chrono::steady_clock::time_point mLastReport;
	double mVoteShare     = 0.0;
	double mVoteShareSum  = 0.0;
	size_t mFramesFitted  = 0;
	size_t mFramesHeld    = 0;
	size_t mFramesDropped = 0;
};

} // namespace dv_monodepth_node
