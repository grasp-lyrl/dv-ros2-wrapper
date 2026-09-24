#pragma once

#include <dv-processing/core/core.hpp>

#include <dv_ros2_messaging/messaging.hpp>

#include "latest_value.hpp"

#include <torch/torch.h>
#include <torch/csrc/inductor/aoti_package/model_package_loader.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace dv_monodepth_node {

/// Node parameters, all settable as ROS2 parameters.
struct Params {
	std::string inputTopic = "events";
	/// AOTI package (.pt2) from neurosim's `scripts/export_f3_aoti.py --module depth`.
	std::string modelPath = "";
	/// Must match the `T` the model was exported with.
	int windowMs     = 50;
	int sensorWidth  = 640;
	int sensorHeight = 480;
	/// Run on every Nth window and drop the rest.
	int windowStride = 1;
	/// Decimate windows above this many events; 0 keeps them all.
	int maxEvents             = 0;
	int deviceId              = 0;
	bool publishVisualization = true;
	/// Disparity range of the preview colormap; with max <= min each frame is normalized alone.
	double disparityMin = 0.0;
	double disparityMax = 2.5;
};

/**
 * Monocular disparity from an event stream, using a fused F3 + DepthAnythingV2 AOTI package.
 * Events are sliced into windows on the executor thread; inference runs on its own thread,
 * always on the newest window.
 */
class MonoDepthNode : public rclcpp::Node {
public:
	explicit MonoDepthNode(const rclcpp::NodeOptions &options);

	~MonoDepthNode();

	/// Stop the inference thread.
	void stop();

	[[nodiscard]] bool isRunning() const;

private:
	void readParameters();

	/// Allocate the event buffers and load the AOTI package.
	void setupInference();

	/// Grow the event buffers to hold at least `rows` events.
	void ensureCapacity(int64_t rows);

	void eventCallback(const dv_ros2_msgs::EventArrayMessage::ConstSharedPtr &events);

	/// Hands a completed window to the inference thread.
	void windowCallback(const dv::EventStore &events);

	void inferenceLoop();

	/// Pack a window into the model's input layout and copy it to the device.
	[[nodiscard]] torch::Tensor packEvents(const dv::EventStore &events);

	/// Publish disparity and/or its preview. At least one must be requested: the readback is
	/// what syncs the input copies packEvents queued.
	void publishDisparity(const torch::Tensor &disparity, int64_t timestamp, bool wantDisparity,
		bool wantPreview);

	Params mParams;

	rclcpp::Subscription<dv_ros2_msgs::EventArrayMessage>::SharedPtr mEventSubscriber;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDisparityPublisher;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mVisualizationPublisher;

	/// Preview range, retunable live through parameters.
	std::atomic<double> mDisparityMin = 0.0;
	std::atomic<double> mDisparityMax = 2.5;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mParamCallback;

	dv::EventStreamSlicer mSlicer;
	std::optional<int> mJobId;
	int64_t mWindowCount = 0;

	LatestValue<dv::EventStore> mPendingWindow;

	std::thread mInferenceThread;
	std::atomic<bool> mSpinThread = false;

	std::optional<torch::inductor::AOTIModelPackageLoader> mModel;

	/// Pinned host and device event buffers, preallocated and narrowed per window.
	torch::Tensor mHostEvents;
	torch::Tensor mDeviceEvents;
	/// The model's [1] int32 event-count input.
	torch::Tensor mHostCounts;
	torch::Tensor mDeviceCounts;
	int64_t mCapacity = 0;

	int64_t mWindowUs    = 0;
	float mInverseWidth  = 0.0f;
	float mInverseHeight = 0.0f;

	std::optional<torch::Device> mDevice;

	// Throughput stats, logged periodically.
	std::chrono::steady_clock::time_point mLastReport;
	double mFramesDone = 0.0;
	double mPackMs     = 0.0;
	double mRunMs      = 0.0;
	double mEventsSeen = 0.0;
};

} // namespace dv_monodepth_node
