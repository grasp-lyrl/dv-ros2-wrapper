#include "../include/dv_ros2_capture/event_undistortion.hpp"

#include <dv_ros2_messaging/messaging.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>

namespace dv_capture_node {

/// Undistorts a recorded event stream with the same calibration and lookup as the capture node.
class EventUndistortNode : public rclcpp::Node {
public:
	explicit EventUndistortNode(const rclcpp::NodeOptions &options) : rclcpp::Node("event_undistort_node", options) {
		const auto inputTopic  = this->declare_parameter("input_topic", std::string("events"));
		const auto outputTopic = this->declare_parameter("output_topic", std::string("events_undistorted"));
		const auto path        = this->declare_parameter("calibration_file", std::string(""));
		if (path.empty()) {
			throw std::invalid_argument("calibration_file is required: an OpenCV FileStorage calibration");
		}
		mCalibration = readOpenCvCalibration(path);

		const auto &K = mCalibration.cameraMatrix;
		RCLCPP_INFO(this->get_logger(), "Loaded OpenCV calibration [%s]: f=(%.1f, %.1f), c=(%.1f, %.1f), %s.",
			path.c_str(), K.at<double>(0, 0), K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2),
			mCalibration.fisheye ? "fisheye" : "plumb-bob");

		// Reliable, matching the capture node's /events.
		mPublisher  = this->create_publisher<dv_ros2_msgs::EventArrayMessage>(outputTopic, 100);
		mSubscriber = this->create_subscription<dv_ros2_msgs::EventArrayMessage>(
			inputTopic, 100, [this](const dv_ros2_msgs::EventArrayMessage::ConstSharedPtr &events) {
				this->onEvents(events);
			});

		RCLCPP_INFO(this->get_logger(), "Undistorting %s -> %s", inputTopic.c_str(), outputTopic.c_str());
	}

private:
	/// Built on the first message, in case the calibration lacks a resolution.
	void buildMap(const cv::Size &sensor) {
		const cv::Size size = mCalibration.resolution.empty() ? sensor : mCalibration.resolution;
		if (size != sensor) {
			RCLCPP_WARN(this->get_logger(),
				"Calibration is for %dx%d but the stream is %dx%d; undistortion will be wrong.", size.width,
				size.height, sensor.width, sensor.height);
		}
		const auto undistortion = buildUndistortionMap(
			mCalibration.cameraMatrix, mCalibration.distortion, size, mCalibration.fisheye);
		mRemap = undistortion.remap;

		const auto &Knew = undistortion.newCameraMatrix;
		RCLCPP_INFO(this->get_logger(), "Rectified %dx%d frame: f=(%.1f, %.1f), c=(%.1f, %.1f).", size.width,
			size.height, Knew.at<double>(0, 0), Knew.at<double>(1, 1), Knew.at<double>(0, 2), Knew.at<double>(1, 2));
	}

	void onEvents(const dv_ros2_msgs::EventArrayMessage::ConstSharedPtr &msg) {
		if (mRemap.empty()) {
			buildMap(cv::Size(static_cast<int>(msg->width), static_cast<int>(msg->height)));
		}

		auto out    = std::make_unique<dv_ros2_msgs::EventArrayMessage>();
		out->header = msg->header;
		out->width  = static_cast<uint32_t>(mRemap.width());
		out->height = static_cast<uint32_t>(mRemap.height());
		out->events.reserve(msg->events.size());

		for (const auto &event : msg->events) {
			uint16_t ux = 0;
			uint16_t uy = 0;
			if (mRemap(event.x, event.y, ux, uy)) {
				auto &undistorted = out->events.emplace_back(event);
				undistorted.x     = ux;
				undistorted.y     = uy;
			}
		}

		mPublisher->publish(std::move(out));
	}

	OpenCvCalibration mCalibration;
	PixelRemap mRemap;
	rclcpp::Publisher<dv_ros2_msgs::EventArrayMessage>::SharedPtr mPublisher;
	rclcpp::Subscription<dv_ros2_msgs::EventArrayMessage>::SharedPtr mSubscriber;
};

} // namespace dv_capture_node

RCLCPP_COMPONENTS_REGISTER_NODE(dv_capture_node::EventUndistortNode)
