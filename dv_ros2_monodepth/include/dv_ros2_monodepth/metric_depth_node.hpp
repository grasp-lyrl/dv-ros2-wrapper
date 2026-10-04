#pragma once

#include <opencv2/core.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
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
	/// nav_msgs/Odometry for the floor fit's gravity and the vertical height; empty assumes a level camera.
	std::string attitudeTopic = "";
	/// NOTE: 1 mm so kr_demo_planner skips these pixels (under depth_filter_mindist); 0 would mean free space.
	double invalidDepth = 0.001;
	/// Depths beyond this are invalid; 0 keeps them all.
	double maxDepth      = 5.0;
	int ransacIterations = 64;
	/// Inlier band, as a fraction of the floor candidates' 2-98 percentile disparity spread.
	double ransacTolerance = 0.026;
	/// Share of the scored floor candidates the best line must explain.
	double minInlierShare = 0.25;
	/// Fewest floor pixels a fit may rest on, before RANSAC and after the final line fit.
	int minInliers = 300;
	/// How long the last fit stands in for frames without one; 0 drops them.
	int fitHoldMs = 1000;
	/// Publish depth resampled to a pinhole image with the calibration's K and no distortion.
	bool undistortDepth = false;
	/// Frames taken below this height, in metres, are dropped; 0 keeps them all.
	double minHeight = 0.0;
	/// Reject a fit whose slope strays this fraction from the recent median, or whose shift is positive; 0 accepts all.
	double maxFitChange = 0.0;
};

/// The floor line `d = slope * (r·g) + shift`; the depth scale is `slope * height`.
struct FloorFit {
	double slope = 0.0;
	double shift = 0.0;
	/// Disparity band around the line that counts as floor.
	double tolerance = 0.0;
};

/// Gravity in the optical frame of a level camera: straight down the image.
inline const cv::Vec3d kLevelGravity(0.0, 1.0, 0.0);

/// Gravity direction in the camera's optical frame at an odometry stamp.
struct GravitySample {
	int64_t stampNs     = 0;
	cv::Vec3d direction = kLevelGravity;
};

/**
 * Metric depth from relative disparity `d = s / z + t`, with `s` and `t` fitted per frame by
 * RANSAC on the floor below a camera at a known height, level unless odometry gives its attitude.
 */
class MetricDepthNode : public rclcpp::Node {
public:
	explicit MetricDepthNode(const rclcpp::NodeOptions &options);

private:
	void readParameters();

	/// Unproject every pixel once, through the camera's distortion.
	void unprojectPixels();

	/// Floor candidates and their r·g for a camera with this gravity direction.
	void aimFloorRays(const cv::Vec3d &gravity);

	/// Map each pixel of the undistorted pinhole image to the distorted pixel it samples.
	void buildUndistortion();

	/// Resample a depth image from the distorted grid onto the undistorted one.
	void undistort(sensor_msgs::msg::Image &depth);

	void heightCallback(const sensor_msgs::msg::Range::ConstSharedPtr &range);

	void attitudeCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);

	/// The buffered gravity nearest a stamp.
	[[nodiscard]] cv::Vec3d gravityAt(int64_t stampNs) const;

	void disparityCallback(const sensor_msgs::msg::Image::ConstSharedPtr &disparity);

	/// Fit the floor line; empty if no line has enough support.
	[[nodiscard]] std::optional<FloorFit> fitFloor(const float *disparity);

	/// Whether a fit agrees with the recently accepted ones.
	[[nodiscard]] bool isConsistent(const FloorFit &fit) const;

	void reportStats();

	MetricDepthParams mParams;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDisparitySubscriber;
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr mHeightSubscriber;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mAttitudeSubscriber;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDepthPublisher;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mDepthInfoPublisher;
	/// The scale and shift each published depth frame used, sent only while subscribed.
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mScalePublisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mShiftPublisher;

	sensor_msgs::msg::CameraInfo mCameraInfo;
	/// What depth/camera_info carries: the calibration, or its K without distortion when undistorting.
	sensor_msgs::msg::CameraInfo mPublishedInfo;
	double mHeight = 0.0;
	/// Gravity from the latest odometry messages, oldest first; a level camera until the first arrives.
	std::deque<GravitySample> mGravity = {GravitySample{}};

	/// Per undistorted pixel, the distorted pixel it samples; -1 where that falls off the sensor.
	std::vector<int32_t> mUndistortSource;
	std::vector<uint8_t> mUndistortBuffer;

	/// Per pixel, its z = 1 ray through the lens.
	std::vector<cv::Point2f> mRays;
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
	/// Slopes of the latest accepted fits, cleared once none is recent enough to hold.
	std::deque<double> mRecentSlopes;

	// Fit stats, logged periodically.
	std::chrono::steady_clock::time_point mLastReport;
	double mVoteShare     = 0.0;
	double mVoteShareSum  = 0.0;
	size_t mFramesFitted  = 0;
	size_t mFramesHeld    = 0;
	size_t mFramesDropped = 0;
	size_t mFramesLow     = 0;
	size_t mFitsRejected  = 0;
};

} // namespace dv_monodepth_node
