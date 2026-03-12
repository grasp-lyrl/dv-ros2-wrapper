#include <dv-processing/processing.hpp>

#include <dv_ros2_messaging/messaging.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>

namespace dv_visualization_node {

class VisualizationNode : public rclcpp::Node {
public:
	explicit VisualizationNode(const rclcpp::NodeOptions &options) :
		rclcpp::Node("visualization_node", options) {
		using namespace std::chrono_literals;
		framePublisher_ = this->create_publisher<dv_ros2_msgs::ImageMessage>("image", 10);

		eventSubscriber_ = this->create_subscription<dv_ros2_msgs::EventArrayMessage>(
			"events", 200,
			[this](const dv_ros2_msgs::EventArrayMessage::SharedPtr events) {
				if (visualizer_ == nullptr) {
					visualizer_ = std::make_unique<dv::visualization::EventVisualizer>(
						cv::Size(events->width, events->height));
				}

				try {
					slicer_.accept(dv_ros2_msgs::toEventStore(*events));
				}
				catch (std::out_of_range &exception) {
					RCLCPP_WARN(this->get_logger(), "%s", exception.what());
				}
			});

		slicer_.doEveryTimeInterval(33ms, [this](const dv::EventStore &events) {
			if (visualizer_ != nullptr) {
				cv::Mat image                 = visualizer_->generateImage(events);
				dv_ros2_msgs::ImageMessage msg = dv_ros2_msgs::toRosImageMessage(image);
				msg.header.stamp              = dv_ros2_msgs::toRosTime(events.getLowestTime());
				framePublisher_->publish(msg);
			}
		});
	}

private:
	dv::EventStreamSlicer slicer_;
	std::unique_ptr<dv::visualization::EventVisualizer> visualizer_ = nullptr;
	rclcpp::Publisher<dv_ros2_msgs::ImageMessage>::SharedPtr framePublisher_;
	rclcpp::Subscription<dv_ros2_msgs::EventArrayMessage>::SharedPtr eventSubscriber_;
};

} // namespace dv_visualization_node

RCLCPP_COMPONENTS_REGISTER_NODE(dv_visualization_node::VisualizationNode)
