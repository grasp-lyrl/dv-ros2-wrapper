#include <dv_ros2_capture/capture_node.hpp>

#include <thread>

using namespace dv_capture_node;

int main(int argc, char **argv) {
	using namespace std::chrono_literals;

	// Initialize ROS2
	rclcpp::init(argc, argv);

	// Create the capture node (params loaded and capture started in constructor)
	auto node = std::make_shared<CaptureNode>(rclcpp::NodeOptions());

	// Spin ROS2
	while (rclcpp::ok() && node->isRunning()) {
		rclcpp::spin_some(node);
		std::this_thread::sleep_for(1ms);
	}

	rclcpp::shutdown();
	return 0;
}
