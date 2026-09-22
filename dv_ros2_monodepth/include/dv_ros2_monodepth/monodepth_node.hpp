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

/**
 * Parameters, all settable from the component's ROS2 parameters.
 */
struct Params {
	/// Event stream to subscribe to.
	std::string inputTopic = "events";
	/// AOTI package (.pt2) exported by neurosim's `scripts/export_f3_aoti.py --module depth`.
	std::string modelPath = "";
	/// Accumulation window. Must equal the `T` the model was trained and exported with,
	/// since the age column is normalized by it.
	int windowMs = 50;
	/// Sensor resolution the model expects; the DVXplorer Micro and F3 both use 640x480.
	int sensorWidth  = 640;
	int sensorHeight = 480;
	/// Upper bound on events handed to the model in one window. 0 keeps every event.
	/// The export marks the event count dynamic, so this is a throughput knob for
	/// smaller hardware rather than a requirement.
	int maxEvents = 0;
	/// CUDA device index.
	int deviceId = 0;
	/// Also publish a colourized preview. Costs a colormap per frame, and is skipped
	/// anyway when nothing subscribes to it.
	bool publishVisualization = true;
};

/**
 * Turns an event stream into monocular disparity using a single fused F3 + DepthAnythingV2
 * AOTI package.
 *
 * The event topic delivers roughly a thousand messages a second while the network runs at
 * a few tens of hertz, so the subscription callback only accumulates into a slicer and the
 * network runs on its own thread, fed by a latest-value slot that drops whatever inference
 * could not keep up with.
 */
class MonoDepthNode : public rclcpp::Node {
public:
	explicit MonoDepthNode(const rclcpp::NodeOptions &options);

	~MonoDepthNode();

	/// Stop the inference thread.
	void stop();

	[[nodiscard]] bool isRunning() const;

private:
	/// Read and validate parameters into mParams.
	void readParameters();

	/// Allocate the pinned staging and device buffers, and load the AOTI package.
	void setupInference();

	/// Grow the staging and device buffers if this window needs more rows than they hold.
	void ensureCapacity(int64_t rows);

	/// Subscription callback. Runs on the executor thread ~1 kHz, so it only converts
	/// and accumulates.
	void eventCallback(const dv_ros2_msgs::EventArrayMessage::ConstSharedPtr &events);

	/// Slicer callback, once per window. Hands the window to the inference thread.
	void windowCallback(const dv::EventStore &events);

	/// Inference thread body.
	void inferenceLoop();

	/**
	 * Pack an event window into the pinned staging buffer in the layout the model was
	 * trained on, and copy it to the device.
	 * @return A view of the device buffer holding exactly the packed rows.
	 */
	[[nodiscard]] torch::Tensor packEvents(const dv::EventStore &events);

	/// Publish disparity as 32FC1, and optionally a colourized preview. At least one of
	/// the two flags must be set: the device-to-host read is what waits on the copies
	/// queued in packEvents, so the staging buffers are only safe to reuse afterwards.
	void publishDisparity(const torch::Tensor &disparity, int64_t timestamp, bool wantDisparity,
		bool wantPreview);

	Params mParams;

	rclcpp::Subscription<dv_ros2_msgs::EventArrayMessage>::SharedPtr mEventSubscriber;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDisparityPublisher;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mVisualizationPublisher;

	dv::EventStreamSlicer mSlicer;
	std::optional<int> mJobId;

	/// Newest complete window awaiting inference; older ones are dropped here.
	LatestValue<dv::EventStore> mPendingWindow;

	std::thread mInferenceThread;
	std::atomic<bool> mSpinThread = false;

	std::optional<torch::inductor::AOTIModelPackageLoader> mModel;

	/// Page-locked host staging, allocated once at capacity and narrowed per window.
	torch::Tensor mHostEvents;
	/// Device-side event buffer, likewise narrowed per window.
	torch::Tensor mDeviceEvents;
	/// The [1] int32 event count the model takes alongside the events.
	torch::Tensor mHostCounts;
	torch::Tensor mDeviceCounts;

	/// Rows the preallocated buffers can hold.
	int64_t mCapacity = 0;

	int64_t mWindowUs    = 0;
	float mInverseWidth  = 0.0f;
	float mInverseHeight = 0.0f;

	std::optional<torch::Device> mDevice;

	// Rolling throughput accounting, reported from the inference thread every few seconds.
	std::chrono::steady_clock::time_point mLastReport;
	double mFramesDone = 0.0;
	double mPackMs     = 0.0;
	double mRunMs      = 0.0;
	double mEventsSeen = 0.0;
};

} // namespace dv_monodepth_node
