#include "dv_ros2_monodepth/metric_depth_node.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/distortion_models.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

using namespace std::chrono_literals;

namespace dv_monodepth_node {

namespace {
/// Rays at least this far below horizontal are floor candidates.
constexpr double kFloorMinAngle = 8.0 * std::numbers::pi / 180.0;
/// Pixels each hypothesis is scored against.
constexpr size_t kScoredPixels      = 4000;
constexpr double kMinInlierFraction = 0.25;
constexpr size_t kMinInliers        = 300;

/// The camera node is found by content, since it is named for a serial that need not match this camera.
sensor_msgs::msg::CameraInfo readCalibration(const std::string &path) {
	cv::FileStorage store(path, cv::FileStorage::READ);
	if (!store.isOpened()) {
		throw std::invalid_argument("Cannot read calibration: " + path);
	}
	cv::FileNode camera;
	for (const auto &node : store.root()) {
		if (node.isMap() && !node["camera_matrix"].empty()) {
			camera = node;
			break;
		}
	}
	cv::Mat cameraMatrix;
	cv::Mat distortion;
	int width  = 0;
	int height = 0;
	if (!camera.empty()) {
		camera["camera_matrix"] >> cameraMatrix;
		camera["distortion_coefficients"] >> distortion;
		camera["image_width"] >> width;
		camera["image_height"] >> height;
	}
	if (cameraMatrix.total() != 9 || distortion.empty() || width <= 0 || height <= 0) {
		throw std::invalid_argument(
			"Calibration lacks a camera_matrix, distortion_coefficients or image size: " + path);
	}
	cameraMatrix.convertTo(cameraMatrix, CV_64F);
	distortion.convertTo(distortion, CV_64F);
	const auto fisheyeNode = store["use_fisheye_model"];
	const bool fisheye     = !fisheyeNode.empty() && static_cast<int>(fisheyeNode) != 0;

	sensor_msgs::msg::CameraInfo info;
	info.width  = static_cast<uint32_t>(width);
	info.height = static_cast<uint32_t>(height);
	info.distortion_model
		= fisheye ? sensor_msgs::distortion_models::EQUIDISTANT : sensor_msgs::distortion_models::PLUMB_BOB;
	info.d.assign(distortion.begin<double>(), distortion.end<double>());
	std::copy(cameraMatrix.begin<double>(), cameraMatrix.end<double>(), info.k.begin());
	info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	for (size_t row = 0; row < 3; ++row) {
		std::copy_n(info.k.begin() + 3 * row, 3, info.p.begin() + 4 * row);
	}
	return info;
}
} // namespace

MetricDepthNode::MetricDepthNode(const rclcpp::NodeOptions &options) :
	rclcpp::Node("metric_depth_node", options) {
	readParameters();
	mCameraInfo = readCalibration(mParams.calibrationFile);
	unprojectPixels();

	mDepthPublisher     = this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
	mDepthInfoPublisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 10);

	mDisparitySubscriber = this->create_subscription<sensor_msgs::msg::Image>("disparity",
		rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Image::ConstSharedPtr &disparity) {
			this->disparityCallback(disparity);
		});

	mHeight     = mParams.cameraHeight;
	mLastReport = std::chrono::steady_clock::now();
	if (mParams.heightTopic.empty()) {
		RCLCPP_INFO(this->get_logger(), "Metric depth node ready: camera %.2f m above the floor.",
			mParams.cameraHeight);
		return;
	}
	mHeightSubscriber = this->create_subscription<sensor_msgs::msg::Range>(mParams.heightTopic,
		rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Range::ConstSharedPtr &range) {
			this->heightCallback(range);
		});
	RCLCPP_INFO(this->get_logger(), "Metric depth node ready: height from [%s], %.2f m until it arrives.",
		mParams.heightTopic.c_str(), mParams.cameraHeight);
}

void MetricDepthNode::readParameters() {
	mParams.calibrationFile  = this->declare_parameter("calibration_file", mParams.calibrationFile);
	mParams.cameraHeight     = this->declare_parameter("camera_height", mParams.cameraHeight);
	mParams.heightTopic      = this->declare_parameter("height_topic", mParams.heightTopic);
	mParams.invalidDepth     = this->declare_parameter("invalid_depth", mParams.invalidDepth);
	mParams.maxDepth         = this->declare_parameter("max_depth", mParams.maxDepth);
	mParams.ransacIterations = this->declare_parameter("ransac_iterations", mParams.ransacIterations);
	mParams.ransacTolerance  = this->declare_parameter("ransac_tolerance", mParams.ransacTolerance);
	mParams.fitHoldMs        = this->declare_parameter("fit_hold_ms", mParams.fitHoldMs);

	if (mParams.calibrationFile.empty()) {
		mParams.calibrationFile
			= ament_index_cpp::get_package_share_directory("dv_ros2_capture") + "/config/calib_40deg.xml";
	}
	if (mParams.cameraHeight <= 0.0) {
		throw std::invalid_argument("camera_height must be positive");
	}
	if (mParams.maxDepth < 0.0) {
		throw std::invalid_argument("max_depth must be zero (no cap) or positive");
	}
	if (mParams.ransacIterations <= 0 || mParams.ransacTolerance <= 0.0) {
		throw std::invalid_argument("ransac_iterations and ransac_tolerance must be positive");
	}
	if (mParams.fitHoldMs < 0) {
		throw std::invalid_argument("fit_hold_ms must be zero (no hold) or positive");
	}
}

void MetricDepthNode::unprojectPixels() {
	const auto &info = mCameraInfo;
	std::vector<cv::Point2f> pixels;
	pixels.reserve(static_cast<size_t>(info.width) * static_cast<size_t>(info.height));
	for (uint32_t v = 0; v < info.height; ++v) {
		for (uint32_t u = 0; u < info.width; ++u) {
			pixels.emplace_back(static_cast<float>(u), static_cast<float>(v));
		}
	}

	const cv::Matx33d cameraMatrix(info.k.data());
	std::vector<cv::Point2f> rays;
	if (info.distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
		cv::fisheye::undistortPoints(pixels, rays, cameraMatrix, info.d);
	}
	else {
		cv::undistortPoints(pixels, rays, cameraMatrix, info.d, cv::noArray(), cv::noArray(),
			cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9));
	}

	mFloorRay.assign(rays.size(), 0.0f);
	mFloorPixels.clear();
	const double minSine = std::sin(kFloorMinAngle);
	for (size_t i = 0; i < rays.size(); ++i) {
		const double x = rays[i].x;
		const double y = rays[i].y;
		// Level camera: gravity is +y, so r·g = y for the z = 1 ray.
		if (y / std::sqrt(x * x + y * y + 1.0) >= minSine) {
			mFloorRay[i] = static_cast<float>(y);
			mFloorPixels.push_back(static_cast<uint32_t>(i));
		}
	}

	RCLCPP_INFO(this->get_logger(), "Camera %ux%u (%s) from [%s]: %zu pixels can see the floor.", info.width,
		info.height, info.distortion_model.c_str(), mParams.calibrationFile.c_str(), mFloorPixels.size());
}

void MetricDepthNode::heightCallback(const sensor_msgs::msg::Range::ConstSharedPtr &range) {
	const float value     = range->range;
	const bool withinSpec = range->max_range <= range->min_range
						 || (value >= range->min_range && value <= range->max_range);
	if (std::isfinite(value) && value > 0.0f && withinSpec) {
		mHeight = value;
	}
}

void MetricDepthNode::disparityCallback(const sensor_msgs::msg::Image::ConstSharedPtr &disparity) {
	const auto &info = mCameraInfo;
	if (disparity->encoding != "32FC1" || disparity->width != info.width || disparity->height != info.height
		|| disparity->step != disparity->width * sizeof(float)) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"Expected a %ux%u 32FC1 disparity, got %ux%u %s.", info.width, info.height, disparity->width,
			disparity->height, disparity->encoding.c_str());
		return;
	}
	const auto *values = reinterpret_cast<const float *>(disparity->data.data());

	const int64_t stampNs = rclcpp::Time(disparity->header.stamp).nanoseconds();
	const int64_t holdNs  = static_cast<int64_t>(mParams.fitHoldMs) * 1'000'000;
	auto fit              = fitFloor(values);
	mVoteShareSum += mVoteShare;
	if (fit.has_value()) {
		mLastFit   = fit;
		mLastFitNs = stampNs;
		++mFramesFitted;
	}
	else if (mLastFit.has_value() && stampNs - mLastFitNs <= holdNs) {
		fit = mLastFit;
		++mFramesHeld;
	}
	else {
		++mFramesDropped;
		reportStats();
		return;
	}

	auto depth    = std::make_unique<sensor_msgs::msg::Image>();
	depth->header = disparity->header;
	depth->height       = disparity->height;
	depth->width        = disparity->width;
	depth->encoding     = "32FC1";
	depth->is_bigendian = false;
	depth->step         = disparity->step;
	depth->data.resize(disparity->data.size());
	auto *out = reinterpret_cast<float *>(depth->data.data());

	const double slope     = fit->slope;
	const double shift     = fit->shift;
	const double tolerance = fit->tolerance;
	const double scale     = slope * mHeight;
	const auto invalid     = static_cast<float>(mParams.invalidDepth);
	const double minGap    = mParams.maxDepth > 0.0 ? scale / mParams.maxDepth : 0.0;
	const size_t count     = static_cast<size_t>(disparity->width) * static_cast<size_t>(disparity->height);
	for (size_t i = 0; i < count; ++i) {
		const double d         = values[i];
		const double gap       = d - shift;
		const float w          = mFloorRay[i];
		const bool beyondFloor = w > 0.0f && d < slope * w + shift - tolerance;
		const bool valid       = std::isfinite(d) && gap > 0.0 && gap >= minGap && !beyondFloor;

		out[i] = valid ? static_cast<float>(scale / gap) : invalid;
	}

	auto depthInfo    = std::make_unique<sensor_msgs::msg::CameraInfo>(info);
	depthInfo->header = depth->header;
	mDepthPublisher->publish(std::move(depth));
	mDepthInfoPublisher->publish(std::move(depthInfo));
	reportStats();
}

std::optional<FloorFit> MetricDepthNode::fitFloor(const float *disparity) {
	mVoteShare = 0.0;
	mCandidateW.clear();
	mCandidateD.clear();
	for (const uint32_t pixel : mFloorPixels) {
		if (std::isfinite(disparity[pixel])) {
			mCandidateW.push_back(mFloorRay[pixel]);
			mCandidateD.push_back(disparity[pixel]);
		}
	}
	const size_t count = mCandidateW.size();
	if (count < kMinInliers) {
		return std::nullopt;
	}

	std::uniform_int_distribution<size_t> pick(0, count - 1);
	const size_t scoredCount = std::min(count, kScoredPixels);
	mScored.resize(scoredCount);
	mSpread.resize(scoredCount);
	for (size_t i = 0; i < scoredCount; ++i) {
		mScored[i] = static_cast<uint32_t>(pick(mRng));
		mSpread[i] = mCandidateD[mScored[i]];
	}
	const auto percentile = [this, scoredCount](const double fraction) {
		const auto nth = mSpread.begin()
					   + static_cast<std::ptrdiff_t>(fraction * static_cast<double>(scoredCount - 1));
		std::nth_element(mSpread.begin(), nth, mSpread.end());
		return static_cast<double>(*nth);
	};
	FloorFit fit;
	fit.tolerance = mParams.ransacTolerance * std::max(percentile(0.98) - percentile(0.02), 1e-6);

	size_t bestVotes     = 0;
	double bestSlope     = 0.0;
	double bestIntercept = 0.0;
	for (int iteration = 0; iteration < mParams.ransacIterations; ++iteration) {
		const size_t a  = pick(mRng);
		const size_t b  = pick(mRng);
		const double dw = static_cast<double>(mCandidateW[b]) - mCandidateW[a];
		if (std::abs(dw) < 1e-3) {
			continue;
		}
		const double slope     = (static_cast<double>(mCandidateD[b]) - mCandidateD[a]) / dw;
		const double intercept = mCandidateD[a] - slope * mCandidateW[a];
		size_t votes           = 0;
		for (const uint32_t index : mScored) {
			votes += std::abs(slope * mCandidateW[index] + intercept - mCandidateD[index]) < fit.tolerance;
		}
		if (votes > bestVotes) {
			bestVotes     = votes;
			bestSlope     = slope;
			bestIntercept = intercept;
		}
	}
	mVoteShare = static_cast<double>(bestVotes) / static_cast<double>(scoredCount);
	if (mVoteShare < kMinInlierFraction) {
		return std::nullopt;
	}

	double sumW    = 0.0;
	double sumD    = 0.0;
	double sumWW   = 0.0;
	double sumWD   = 0.0;
	size_t inliers = 0;
	for (size_t i = 0; i < count; ++i) {
		const double w = mCandidateW[i];
		const double d = mCandidateD[i];
		if (std::abs(bestSlope * w + bestIntercept - d) < fit.tolerance) {
			sumW += w;
			sumD += d;
			sumWW += w * w;
			sumWD += w * d;
			++inliers;
		}
	}
	const auto n             = static_cast<double>(inliers);
	const double denominator = n * sumWW - sumW * sumW;
	if (inliers < kMinInliers || denominator <= 0.0) {
		return std::nullopt;
	}
	fit.slope = (n * sumWD - sumW * sumD) / denominator;
	fit.shift = (sumD - fit.slope * sumW) / n;
	if (fit.slope <= 0.0) {
		return std::nullopt;
	}
	return fit;
}

void MetricDepthNode::reportStats() {
	const auto now = std::chrono::steady_clock::now();
	if (now - mLastReport < 5s) {
		return;
	}
	const size_t frames = mFramesFitted + mFramesHeld + mFramesDropped;
	if (frames > 0) {
		RCLCPP_INFO(this->get_logger(),
			"%zu fitted, %zu held, %zu dropped | floor inliers %.0f%% | h %.2f m | s %.3f t %.3f", mFramesFitted,
			mFramesHeld, mFramesDropped, 100.0 * mVoteShareSum / static_cast<double>(frames), mHeight,
			mLastFit.has_value() ? mLastFit->slope * mHeight : 0.0, mLastFit.has_value() ? mLastFit->shift : 0.0);
	}
	mLastReport   = now;
	mVoteShareSum = 0.0;
	mFramesFitted = mFramesHeld = mFramesDropped = 0;
}

} // namespace dv_monodepth_node

RCLCPP_COMPONENTS_REGISTER_NODE(dv_monodepth_node::MetricDepthNode)
