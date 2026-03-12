#pragma once

#include <dv-processing/core/core.hpp>
#include <dv-processing/core/frame.hpp>
#include <dv-processing/data/frame_base.hpp>
#include <dv-processing/exception/exception.hpp>

#include <dv_ros2_msgs/msg/event_array.hpp>
#include <dv_ros2_msgs/msg/trigger.hpp>

#include <opencv2/core.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <builtin_interfaces/msg/time.hpp>

namespace dv_ros2_msgs {

namespace _detail {
[[nodiscard]] inline int32_t imageTypeFromEncoding(const std::string &encoding) {
	if (encoding.find("bayer") != std::string::npos) {
		throw dv::exceptions::RuntimeError("Bayer image encoding is not supported for conversion!");
	}

	// Determine channels and depth from encoding string
	int channels = 1;
	int depth    = 8;
	if (encoding == sensor_msgs::image_encodings::MONO8) {
		channels = 1;
		depth    = 8;
	}
	else if (encoding == sensor_msgs::image_encodings::MONO16) {
		channels = 1;
		depth    = 16;
	}
	else if (encoding == sensor_msgs::image_encodings::BGR8 || encoding == sensor_msgs::image_encodings::RGB8) {
		channels = 3;
		depth    = 8;
	}
	else if (encoding == sensor_msgs::image_encodings::BGRA8 || encoding == sensor_msgs::image_encodings::RGBA8) {
		channels = 4;
		depth    = 8;
	}
	else {
		throw dv::exceptions::InvalidArgument<std::string>("Unsupported image encoding", encoding);
	}

	switch (depth) {
		case 8: {
			return CV_MAKETYPE(CV_8U, channels);
		}
		case 16: {
			return CV_MAKETYPE(CV_16U, channels);
		}
		default:
			throw dv::exceptions::InvalidArgument<int>("Unsupported image bit depth", depth);
	}
}
} // namespace _detail

/// A dv::processing compatible alias for `sensor_msgs::msg::Imu`.
using ImuMessage = sensor_msgs::msg::Imu;

/// A dv::processing compatible alias for `sensor_msgs::msg::Image`.
using ImageMessage = sensor_msgs::msg::Image;

/// A dv::processing compatible alias for `sensor_msgs::msg::CameraInfo`.
using CameraInfoMessage = sensor_msgs::msg::CameraInfo;

/// A dv::processing compatible alias for `dv_ros2_msgs::msg::EventArray`.
using EventArrayMessage = dv_ros2_msgs::msg::EventArray;

/// A dv::processing compatible alias for `dv_ros2_msgs::msg::Event`.
using EventMessage = dv_ros2_msgs::msg::Event;

/// A dv::processing compatible alias for `dv_ros2_msgs::msg::Trigger`.
using TriggerMessage = dv_ros2_msgs::msg::Trigger;

/**
 * Converts UNIX microsecond timestamp into builtin_interfaces::msg::Time format.
 * @param timestamp	DV format UNIX microsecond timestamp
 * @return ROS2 timestamp
 */
[[nodiscard]] inline builtin_interfaces::msg::Time toRosTime(const int64_t timestamp) {
	builtin_interfaces::msg::Time t;
	t.sec     = static_cast<int32_t>(timestamp / 1'000'000);
	t.nanosec = static_cast<uint32_t>((timestamp % 1'000'000) * 1'000);
	return t;
}

/**
 * Convert builtin_interfaces::msg::Time into UNIX microsecond timestamp
 * @param timestamp ROS2 timestamp
 * @return DV format UNIX microsecond timestamp
 */
[[nodiscard]] inline int64_t toDvTime(const builtin_interfaces::msg::Time &timestamp) {
	return (static_cast<int64_t>(timestamp.sec) * 1'000'000) + (timestamp.nanosec / 1'000);
}

/**
 * Convert OpenCV image into ROS2 image message. Supports only single channel 8-bit, three channel 8-bit BGR images,
 * and continuous and non-continuous memory.
 * Performs deep data copy.
 * @param image OpenCV Image
 * @return ROS2 image (sensor_msgs::msg::Image)
 * @throws RuntimeError If image data layout is not supported
 */
[[nodiscard]] inline ImageMessage toRosImageMessage(const cv::Mat &image) {
	ImageMessage msg;

	msg.height = image.rows;
	msg.width  = image.cols;

	if (image.empty()) {
		return msg;
	}

	switch (image.type()) {
		case CV_8UC1:
			msg.encoding = sensor_msgs::image_encodings::MONO8;
			break;
		case CV_8UC3:
			msg.encoding = sensor_msgs::image_encodings::BGR8;
			break;
		default:
			throw dv::exceptions::RuntimeError("Received unsupported image type");
	}

	msg.is_bigendian  = false;
	msg.step          = msg.width * image.elemSize();
	const size_t size = msg.step * msg.height;
	msg.data.resize(size);

	if (image.isContinuous()) {
		memcpy((char *) (&msg.data[0]), image.data, size);
	}
	else {
		auto ros_data_ptr  = (uchar *) (&msg.data[0]);
		uchar *cv_data_ptr = image.data;
		for (int i = 0; i < image.rows; ++i) {
			memcpy(ros_data_ptr, cv_data_ptr, msg.step);
			ros_data_ptr += msg.step;
			cv_data_ptr += image.step;
		}
	}
	return msg;
}

/**
 * Converts dv::Frame into sensor_msgs::msg::Image.
 * @param frame DV Frame containing an image.
 * @return ROS2 image (sensor_msgs::msg::Image)
 * @throws RuntimeError If image data layout is not supported
 */
[[nodiscard]] inline ImageMessage frameToRosImageMessage(const dv::Frame &frame) {
	ImageMessage imageMessage = toRosImageMessage(frame.image);
	imageMessage.header.stamp = toRosTime(frame.timestamp);
	return imageMessage;
}

/**
 * Convert dv::IMU into sensor_msgs::msg::Imu
 * @param imu DV IMU measurement
 * @return ROS2 Imu message
 */
[[nodiscard]] inline ImuMessage toRosImuMessage(const dv::IMU &imu) {
	ImuMessage imuMessage;
	imuMessage.header.stamp = toRosTime(imu.timestamp);

	constexpr float deg2rad = std::numbers::pi_v<float> / 180.0f;
	constexpr float earthG  = 9.81007f;

	imuMessage.angular_velocity.x    = imu.gyroscopeX * deg2rad;
	imuMessage.angular_velocity.y    = imu.gyroscopeY * deg2rad;
	imuMessage.angular_velocity.z    = imu.gyroscopeZ * deg2rad;
	imuMessage.linear_acceleration.x = imu.accelerometerX * earthG;
	imuMessage.linear_acceleration.y = imu.accelerometerY * earthG;
	imuMessage.linear_acceleration.z = imu.accelerometerZ * earthG;

	return imuMessage;
}

/**
 * Convert dv::Trigger into dv_ros2_msgs::msg::Trigger
 * @param trigger DV Trigger
 * @return ROS2 Trigger message
 */
[[nodiscard]] inline TriggerMessage toRosTriggerMessage(const dv::Trigger &trigger) {
	TriggerMessage msg;
	msg.timestamp = toRosTime(trigger.timestamp);
	msg.type      = static_cast<int8_t>(trigger.type);
	return msg;
}

[[nodiscard]] inline EventArrayMessage toRosEventsMessage(const dv::EventStore &events, const cv::Size &resolution) {
	EventArrayMessage msg;
	builtin_interfaces::msg::Time time = toRosTime(events.getLowestTime());

	int64_t secInMicro = static_cast<int64_t>(time.sec) * 1'000'000;
	msg.header.stamp   = toRosTime(events.getHighestTime());
	msg.events.reserve(events.size());
	for (const auto &event : events) {
		int64_t time_diff = event.timestamp() - secInMicro;
		if (time_diff < 1'000'000) {
			// We are in the same second, we only need to update the nano-second part
			time.nanosec = static_cast<uint32_t>(time_diff * 1'000);
		}
		else {
			time       = toRosTime(event.timestamp());
			secInMicro = static_cast<int64_t>(time.sec) * 1'000'000;
		}
		auto &e    = msg.events.emplace_back();
		e.x        = event.x();
		e.y        = event.y();
		e.polarity = event.polarity();
		e.ts       = time;
	}

	msg.width  = resolution.width;
	msg.height = resolution.height;
	return msg;
}

/**
 * Convert an array message into an event store.
 * @param message Event array message
 * @return DV Event store
 */
[[nodiscard]] inline dv::EventStore toEventStore(const EventArrayMessage &message) {
	if (message.events.empty()) {
		return {};
	}
	int32_t seconds                              = message.events.front().ts.sec;
	int64_t timestamp                            = static_cast<int64_t>(seconds) * 1'000'000;
	std::shared_ptr<dv::EventPacket> eventPacket = std::make_shared<dv::EventPacket>();
	eventPacket->elements.reserve(message.events.size());
	for (const auto &event : message.events) {
		if (event.ts.sec != seconds) {
			seconds   = event.ts.sec;
			timestamp = static_cast<int64_t>(seconds) * 1'000'000;
		}
		const int64_t eventTimestamp = timestamp + static_cast<int64_t>(event.ts.nanosec / 1000);

		dv::runtime_assert(
			[event, eventTimestamp]() {
				return eventTimestamp == toDvTime(event.ts);
			},
			[]() {
				return "Timestamp conversion failed!";
			});

		eventPacket->elements.emplace_back(eventTimestamp, event.x, event.y, event.polarity);
	}
	dv::EventStore store(std::const_pointer_cast<const dv::EventPacket>(eventPacket));
	return store;
}

/**
 * Convert an image message from ROS2 into a dv::Frame. Allocates the memory for the image and performs
 * deep data copy.
 * @param imageMsg Message to be converted.
 * @return A copy of the image in a dv::Frame.
 * @throws runtime_error Exception is thrown if encoding of the source image is not supported.
 */
[[nodiscard]] inline dv::Frame toDvFrame(const ImageMessage &imageMsg) {
	const std::string encoding(imageMsg.encoding);
	cv::Mat image(static_cast<int32_t>(imageMsg.height), static_cast<int32_t>(imageMsg.width),
		_detail::imageTypeFromEncoding(encoding), const_cast<uchar *>(&imageMsg.data[0]), imageMsg.step);
	return {toDvTime(imageMsg.header.stamp), image.clone()};
}

/**
 * A small convenience class that allows image memory access through cv::Mat mapping. The underlying data is valid
 * as long as the instance of this class exists, since it does not perform deep memory copy, just shares the
 * ownership by keeping a copy of a smart pointer to the actual memory, so the memory is not deallocated.
 */
class FrameMap {
public:
	/**
	 * A smart pointer to the original memory owner.
	 */
	ImageMessage::ConstSharedPtr message;

	/**
	 * This frame is a mapping to the contents of image message. This value should not be used outside
	 * of this map class and it is only valid as long as the parent class instance exists.
	 */
	dv::Frame frame;

	/**
	 * Construct the mapping, it will construct the mapped frame in this class instance.
	 * @param msg Image message for mapping.
	 */
	explicit FrameMap(const ImageMessage::ConstSharedPtr &msg) : message(msg) {
		const std::string encoding(msg->encoding);
		cv::Mat image(static_cast<int32_t>(msg->height), static_cast<int32_t>(msg->width),
			_detail::imageTypeFromEncoding(encoding), const_cast<uchar *>(&msg->data[0]), msg->step);
		frame = dv::Frame(toDvTime(msg->header.stamp), image);
	}
};

} // namespace dv_ros2_msgs
