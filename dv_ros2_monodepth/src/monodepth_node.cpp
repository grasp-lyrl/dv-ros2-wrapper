#include "dv_ros2_monodepth/monodepth_node.hpp"

#include <cuda_runtime.h>

#include <opencv2/imgproc.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

namespace dv_monodepth_node {

namespace {
/// Rows to allocate before the first window arrives. Grown on demand, so this only
/// decides whether the first few windows reallocate.
constexpr int64_t kInitialCapacity = 512 * 1024;

/// How long the inference thread sleeps when no window is ready. Windows arrive every
/// `windowMs`, so this bounds the handoff delay at a small fraction of one window.
constexpr auto kIdlePoll = 1ms;
} // namespace

MonoDepthNode::MonoDepthNode(const rclcpp::NodeOptions &options) : rclcpp::Node("monodepth_node", options) {
	readParameters();

	mWindowUs       = static_cast<int64_t>(mParams.windowMs) * 1000;
	mInverseWidth   = 1.0f / static_cast<float>(mParams.sensorWidth);
	mInverseHeight  = 1.0f / static_cast<float>(mParams.sensorHeight);

	setupInference();

	// Best effort on the way out as well: a consumer that cannot keep up should miss
	// frames rather than hold the inference thread back.
	mDisparityPublisher
		= this->create_publisher<sensor_msgs::msg::Image>("disparity", rclcpp::SensorDataQoS());
	if (mParams.publishVisualization) {
		mVisualizationPublisher
			= this->create_publisher<sensor_msgs::msg::Image>("disparity_image", rclcpp::SensorDataQoS());
	}

	// The preview range can be retuned live with `ros2 param set`, since what reads well
	// depends on the scene and the checkpoint.
	mDisparityMin  = mParams.disparityMin;
	mDisparityMax  = mParams.disparityMax;
	mParamCallback = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
		for (const auto &param : params) {
			if (param.get_name() == "disparity_min") {
				mDisparityMin = param.as_double();
			}
			else if (param.get_name() == "disparity_max") {
				mDisparityMax = param.as_double();
			}
		}
		rcl_interfaces::msg::SetParametersResult result;
		result.successful = true;
		return result;
	});

	// Best effort: the event stream is a sensor feed, and a window we could not keep up
	// with is worth less than the one behind it.
	mEventSubscriber = this->create_subscription<dv_ros2_msgs::EventArrayMessage>(
		mParams.inputTopic, rclcpp::SensorDataQoS(),
		[this](const dv_ros2_msgs::EventArrayMessage::ConstSharedPtr &events) {
			this->eventCallback(events);
		});

	mJobId = mSlicer.doEveryTimeInterval(std::chrono::milliseconds(mParams.windowMs),
		[this](const dv::EventStore &events) {
			this->windowCallback(events);
		});

	mSpinThread      = true;
	mInferenceThread = std::thread(&MonoDepthNode::inferenceLoop, this);

	RCLCPP_INFO(this->get_logger(), "Monodepth node ready: %d ms windows on [%s], %dx%d.",
		mParams.windowMs, mParams.inputTopic.c_str(), mParams.sensorWidth, mParams.sensorHeight);
}

MonoDepthNode::~MonoDepthNode() {
	stop();
}

void MonoDepthNode::stop() {
	if (mSpinThread.exchange(false) && mInferenceThread.joinable()) {
		mInferenceThread.join();
	}
}

bool MonoDepthNode::isRunning() const {
	return mSpinThread.load(std::memory_order_relaxed);
}

void MonoDepthNode::readParameters() {
	mParams.inputTopic           = this->declare_parameter("input_topic", mParams.inputTopic);
	mParams.modelPath            = this->declare_parameter("model_path", mParams.modelPath);
	mParams.windowMs             = this->declare_parameter("window_ms", mParams.windowMs);
	mParams.sensorWidth          = this->declare_parameter("sensor_width", mParams.sensorWidth);
	mParams.sensorHeight         = this->declare_parameter("sensor_height", mParams.sensorHeight);
	mParams.maxEvents            = this->declare_parameter("max_events", mParams.maxEvents);
	mParams.deviceId             = this->declare_parameter("device_id", mParams.deviceId);
	mParams.publishVisualization = this->declare_parameter("publish_visualization", mParams.publishVisualization);
	mParams.disparityMin         = this->declare_parameter("disparity_min", mParams.disparityMin);
	mParams.disparityMax         = this->declare_parameter("disparity_max", mParams.disparityMax);

	if (mParams.modelPath.empty()) {
		throw std::invalid_argument("model_path is required: point it at the .pt2 from export_f3_aoti.py");
	}
	if (mParams.windowMs <= 0) {
		throw std::invalid_argument("window_ms must be positive");
	}
	if (mParams.sensorWidth <= 0 || mParams.sensorHeight <= 0) {
		throw std::invalid_argument("sensor_width and sensor_height must be positive");
	}
	if (mParams.maxEvents < 0) {
		throw std::invalid_argument("max_events must be zero (keep every event) or positive");
	}
}

void MonoDepthNode::setupInference() {
	// Ask the driver to sleep on GPU waits rather than spin. The default spins, and since
	// the inference thread blocks on the device-to-host read until the whole model has
	// run, that burned a core per ~13 ms of inference doing nothing. This has to happen
	// before anything creates the CUDA context, so it sits ahead of the torch call below.
	if (const cudaError_t status = cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
		status != cudaSuccess) {
		RCLCPP_WARN(this->get_logger(), "Could not select blocking GPU waits (%s); the "
			"inference thread will spin while the model runs.", cudaGetErrorString(status));
	}

	if (!torch::cuda::is_available()) {
		throw std::runtime_error("CUDA is not available; the AOTI package is compiled for the GPU");
	}
	mDevice.emplace(torch::kCUDA, static_cast<c10::DeviceIndex>(mParams.deviceId));

	const auto pinned = torch::TensorOptions().dtype(torch::kInt32).pinned_memory(true);
	mHostCounts       = torch::empty({1}, pinned);
	mDeviceCounts     = torch::empty({1}, torch::TensorOptions().dtype(torch::kInt32).device(*mDevice));

	ensureCapacity(kInitialCapacity);

	RCLCPP_INFO(this->get_logger(), "Loading AOTI package [%s]...", mParams.modelPath.c_str());
	const auto start = std::chrono::steady_clock::now();
	// Exactly one thread ever calls run(), so the runner can skip its own locking. The
	// device is selected here rather than by setting a current device, which keeps this
	// thread-agnostic.
	mModel.emplace(mParams.modelPath, "model", /*run_single_threaded=*/true, /*num_runners=*/1,
		static_cast<c10::DeviceIndex>(mParams.deviceId));
	const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - start);
	RCLCPP_INFO(this->get_logger(), "Loaded in %ld ms.", elapsed.count());
}

void MonoDepthNode::ensureCapacity(int64_t rows) {
	if (rows <= mCapacity) {
		return;
	}
	// Half again, so a steadily rising event rate stops reallocating instead of doing it
	// on every window.
	const int64_t capacity = rows + rows / 2;
	mHostEvents            = torch::empty({capacity, 4},
        torch::TensorOptions().dtype(torch::kFloat32).pinned_memory(true));
	mDeviceEvents = torch::empty({capacity, 4},
		torch::TensorOptions().dtype(torch::kFloat32).device(*mDevice));
	mCapacity = capacity;
	RCLCPP_INFO(this->get_logger(), "Event buffers sized to %ld events (%.1f MiB per copy).",
		capacity, static_cast<double>(capacity * 4 * sizeof(float)) / (1024.0 * 1024.0));
}

void MonoDepthNode::eventCallback(const dv_ros2_msgs::EventArrayMessage::ConstSharedPtr &events) {
	try {
		mSlicer.accept(dv_ros2_msgs::toEventStore(*events));
	}
	catch (const std::out_of_range &exception) {
		RCLCPP_WARN(this->get_logger(), "Dropping out-of-order event batch: %s", exception.what());
	}
}

void MonoDepthNode::windowCallback(const dv::EventStore &events) {
	if (events.isEmpty()) {
		return;
	}
	mPendingWindow.write(dv::EventStore(events));
}

void MonoDepthNode::inferenceLoop() {
	// No autograd bookkeeping anywhere on this thread.
	c10::InferenceMode guard;

	dv::EventStore window;
	mLastReport = std::chrono::steady_clock::now();
	while (mSpinThread.load(std::memory_order_relaxed)) {
		if (!mPendingWindow.read(window)) {
			std::this_thread::sleep_for(kIdlePoll);
			continue;
		}
		// The export declares the event count dynamic with a floor of two, so a window
		// that quiet has no valid shape to run at.
		if (window.size() < 2) {
			continue;
		}

		// Decide once, and skip the GPU entirely when nothing is listening. This also
		// keeps the pinned staging buffer safe: the only thing that waits on the copies
		// queued in packEvents is the device-to-host read below, so a window that
		// produced no output must not run at all.
		const bool wantDisparity = mDisparityPublisher->get_subscription_count() > 0;
		const bool wantPreview
			= mVisualizationPublisher != nullptr && mVisualizationPublisher->get_subscription_count() > 0;
		if (!wantDisparity && !wantPreview) {
			continue;
		}

		try {
			// The window is anchored on its newest event, which is the instant the
			// disparity describes.
			const int64_t timestamp = window.getHighestTime();

			const auto packStart = std::chrono::steady_clock::now();
			auto events          = packEvents(window);
			const auto runStart  = std::chrono::steady_clock::now();

			auto outputs = mModel->run({events, mDeviceCounts});
			if (outputs.empty()) {
				RCLCPP_ERROR(this->get_logger(), "Model returned no outputs.");
				continue;
			}
			// The device-to-host read inside this is what the queued work is waited on,
			// so the clock after it covers the model end to end.
			publishDisparity(outputs.front(), timestamp, wantDisparity, wantPreview);
			const auto done = std::chrono::steady_clock::now();

			const auto milliseconds = [](auto from, auto to) {
				return std::chrono::duration<double, std::milli>(to - from).count();
			};
			mPackMs += milliseconds(packStart, runStart);
			mRunMs  += milliseconds(runStart, done);
			mEventsSeen += static_cast<double>(window.size());
			++mFramesDone;

			const double sinceReport = milliseconds(mLastReport, done);
			if (sinceReport >= 5000.0) {
				RCLCPP_INFO(this->get_logger(),
					"%.1f Hz | %.0f kev/window | pack %.1f ms | model %.1f ms",
					(mFramesDone * 1000.0) / sinceReport, (mEventsSeen / mFramesDone) / 1000.0,
					mPackMs / mFramesDone, mRunMs / mFramesDone);
				mLastReport = done;
				mFramesDone = 0;
				mPackMs = mRunMs = mEventsSeen = 0.0;
			}
		}
		catch (const c10::Error &exception) {
			RCLCPP_ERROR(this->get_logger(), "Inference failed: %s", exception.what());
		}
		catch (const std::exception &exception) {
			RCLCPP_ERROR(this->get_logger(), "Inference failed: %s", exception.what());
		}
	}
}

torch::Tensor MonoDepthNode::packEvents(const dv::EventStore &events) {
	const int64_t total = static_cast<int64_t>(events.size());
	// The export marks the event count dynamic, so by default every event goes to the
	// model. `max_events` decimates with a stride rather than truncating, which keeps the
	// window's age distribution intact -- and age is the only temporal signal the model
	// gets.
	const bool decimate = mParams.maxEvents > 0 && total > static_cast<int64_t>(mParams.maxEvents);
	const int64_t kept  = decimate ? static_cast<int64_t>(mParams.maxEvents) : total;

	ensureCapacity(kept);

	// Age, not elapsed time: the training loader writes (t_anchor - t) / window, so the
	// newest event in a window is 0 and the oldest approaches 1. Feeding forward time
	// here reverses the field the model was trained on.
	const int64_t anchor       = events.getHighestTime();
	const float inverseWindow  = 1.0f / static_cast<float>(mWindowUs);
	float *const rows          = mHostEvents.data_ptr<float>();

	const auto writeRow = [&](int64_t index, const dv::Event &event) {
		float *row = rows + (index * 4);
		row[0]     = static_cast<float>(event.x()) * mInverseWidth;
		row[1]     = static_cast<float>(event.y()) * mInverseHeight;
		row[2]     = std::clamp(static_cast<float>(anchor - event.timestamp()) * inverseWindow, 0.0f, 1.0f);
		row[3]     = event.polarity() ? 1.0f : 0.0f;
	};

	if (decimate) {
		const double stride = static_cast<double>(total) / static_cast<double>(kept);
		double cursor       = 0.0;
		for (int64_t i = 0; i < kept; ++i) {
			writeRow(i, events.at(static_cast<size_t>(cursor)));
			cursor += stride;
		}
	}
	else {
		int64_t i = 0;
		for (const auto &event : events) {
			writeRow(i++, event);
		}
	}

	// Page-locked source, so this overlaps instead of staging through a driver bounce
	// buffer. The model run that follows is queued on the same stream, which orders it
	// after the copy.
	auto host   = mHostEvents.narrow(0, 0, kept);
	auto device = mDeviceEvents.narrow(0, 0, kept);
	device.copy_(host, /*non_blocking=*/true);

	mHostCounts.data_ptr<int32_t>()[0] = static_cast<int32_t>(kept);
	mDeviceCounts.copy_(mHostCounts, /*non_blocking=*/true);

	return device;
}

void MonoDepthNode::publishDisparity(
	const torch::Tensor &disparity, int64_t timestamp, bool wantDisparity, bool wantPreview) {
	// The model emits [1, H, W] at full sensor resolution.
	const auto frame     = disparity.squeeze(0);
	const int64_t height = frame.size(0);
	const int64_t width  = frame.size(1);
	const auto stamp     = dv_ros2_msgs::toRosTime(timestamp);

	if (wantDisparity) {
		auto message          = std::make_unique<sensor_msgs::msg::Image>();
		message->header.stamp = stamp;
		message->height       = static_cast<uint32_t>(height);
		message->width        = static_cast<uint32_t>(width);
		message->encoding     = "32FC1";
		message->is_bigendian = false;
		message->step         = static_cast<uint32_t>(width * sizeof(float));
		message->data.resize(static_cast<size_t>(message->step) * static_cast<size_t>(height));

		// Copy off the device straight into the message's storage, so the frame is not
		// staged through a temporary on the way out.
		auto destination = torch::from_blob(message->data.data(), {height, width},
			torch::TensorOptions().dtype(torch::kFloat32));
		destination.copy_(frame);

		mDisparityPublisher->publish(std::move(message));
	}

	if (wantPreview) {
		const auto host = frame.to(torch::kCPU, torch::kFloat32).contiguous();
		const cv::Mat raw(static_cast<int>(height), static_cast<int>(width), CV_32FC1,
			const_cast<float *>(host.data_ptr<float>()));

		// A fixed range, so a colour means the same disparity from one frame to the next.
		// Normalizing each frame on its own, as the training previews do, makes the whole
		// colormap jump whenever the nearest surface changes.
		double low  = mDisparityMin.load(std::memory_order_relaxed);
		double high = mDisparityMax.load(std::memory_order_relaxed);
		if (high <= low) {
			cv::minMaxLoc(raw, &low, &high);
		}
		const double span = (high - low) > 1e-9 ? (high - low) : 1.0;

		cv::Mat normalized;
		raw.convertTo(normalized, CV_8UC1, 255.0 / span, -255.0 * low / span);
		cv::Mat coloured;
		cv::applyColorMap(normalized, coloured, cv::COLORMAP_INFERNO);

		auto message          = std::make_unique<sensor_msgs::msg::Image>(
            dv_ros2_msgs::toRosImageMessage(coloured));
		message->header.stamp = stamp;
		mVisualizationPublisher->publish(std::move(message));
	}
}

} // namespace dv_monodepth_node

RCLCPP_COMPONENTS_REGISTER_NODE(dv_monodepth_node::MonoDepthNode)
