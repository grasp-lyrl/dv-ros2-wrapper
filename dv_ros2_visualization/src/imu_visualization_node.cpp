#include <dv_ros2_messaging/messaging.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <mutex>

namespace dv_visualization_node {

class ImuVisualizationNode : public rclcpp::Node {
public:
	explicit ImuVisualizationNode(const rclcpp::NodeOptions &options) :
		rclcpp::Node("imu_visualization_node", options) {
		using namespace std::chrono_literals;

		framePublisher_ = this->create_publisher<dv_ros2_msgs::ImageMessage>("imu_image", 10);

		imuSubscriber_ = this->create_subscription<dv_ros2_msgs::ImuMessage>(
			"imu", rclcpp::SensorDataQoS(),
			[this](const dv_ros2_msgs::ImuMessage::SharedPtr msg) {
				std::lock_guard<std::mutex> lock(mutex_);

				const double stamp = static_cast<double>(msg->header.stamp.sec)
								   + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

				if (lastStamp_ > 0.0) {
					const double dt = stamp - lastStamp_;
					if (dt > 0.0 && dt < 0.1) {
						mahonyUpdate(msg->angular_velocity.x, msg->angular_velocity.y,
									 msg->angular_velocity.z, msg->linear_acceleration.x,
									 msg->linear_acceleration.y, msg->linear_acceleration.z, dt);
					}
				}
				lastStamp_ = stamp;
				latestImu_ = *msg;
				haveImu_   = true;
			});

		renderTimer_ = this->create_wall_timer(33ms, [this]() { render(); });
	}

private:
	// Mahony complementary filter on SO(3).
	// State: unit quaternion q (body-to-world), integral error for gyro bias.
	// Camera convention: +x right, +y up, +z forward.
	// Gyro (RHR): x -> pitch, y -> yaw, z -> roll.
	// At rest accel reads (0, +g, 0) since +y is up.
	void mahonyUpdate(double gx, double gy, double gz,
					  double ax, double ay, double az, double dt) {
		const double norm = std::sqrt(ax * ax + ay * ay + az * az);
		if (norm > 1.0) {
			// Normalise accel to unit vector.
			ax /= norm; ay /= norm; az /= norm;

			// Estimated direction of world +y (up) expressed in body frame,
			// derived from the current quaternion (world-to-body rotation of (0,1,0)).
			// v_body = R^T * (0,1,0) where R is the body-to-world rotation matrix.
			const double vx = 2.0 * (qx_ * qy_ + qw_ * qz_);
			const double vy = qw_*qw_ - qx_*qx_ + qy_*qy_ - qz_*qz_;
			const double vz = 2.0 * (qy_ * qz_ - qw_ * qx_);

			// Error: cross product of measured and estimated gravity direction.
			// No atan2, no singularities.
			const double ex = ay * vz - az * vy;
			const double ey = az * vx - ax * vz;
			const double ez = ax * vy - ay * vx;

			// Integral feedback (eliminates gyro bias over time).
			ix_ += Ki_ * ex * dt;
			iy_ += Ki_ * ey * dt;
			iz_ += Ki_ * ez * dt;

			gx += Kp_ * ex + ix_;
			gy += Kp_ * ey + iy_;
			gz += Kp_ * ez + iz_;
		}

		// Integrate quaternion: dq/dt = 0.5 * q ⊗ [0, gx, gy, gz]
		const double dw = 0.5 * (-qx_ * gx - qy_ * gy - qz_ * gz) * dt;
		const double dx = 0.5 * ( qw_ * gx + qy_ * gz - qz_ * gy) * dt;
		const double dy = 0.5 * ( qw_ * gy - qx_ * gz + qz_ * gx) * dt;
		const double dz = 0.5 * ( qw_ * gz + qx_ * gy - qy_ * gx) * dt;

		qw_ += dw; qx_ += dx; qy_ += dy; qz_ += dz;

		// Renormalise.
		const double qnorm = std::sqrt(qw_*qw_ + qx_*qx_ + qy_*qy_ + qz_*qz_);
		qw_ /= qnorm; qx_ /= qnorm; qy_ /= qnorm; qz_ /= qnorm;
	}

	// Extract Euler angles from quaternion. All computed from the smooth quaternion
	// state — atan2 is only applied here, never to raw accel.
	void getEuler(double &roll, double &pitch, double &yaw) const {
		// Estimated world +y (up) in body frame — same as in mahonyUpdate.
		const double vx = 2.0 * (qx_ * qy_ + qw_ * qz_);
		const double vy = qw_*qw_ - qx_*qx_ + qy_*qy_ - qz_*qz_;
		const double vz = 2.0 * (qy_ * qz_ - qw_ * qx_);

		// Roll (about +z): tilts +y toward +x. roll = atan2(vx, vy).
		roll  = std::atan2(vx, vy);
		// Pitch (about +x): tilts +y toward -z. pitch = atan2(-vz, |vxy|).
		pitch = std::atan2(-vz, std::sqrt(vx * vx + vy * vy));

		// Yaw: angle of camera forward (+z body) projected onto world horizontal plane.
		// Forward in world = R * (0,0,1): R[0][2]=2(xz+wy), R[2][2]=1-2(x^2+y^2).
		yaw = std::atan2(2.0 * (qx_ * qz_ + qw_ * qy_),
						 1.0 - 2.0 * (qx_ * qx_ + qy_ * qy_));
	}

	void render() {
		dv_ros2_msgs::ImuMessage imu;
		double roll, pitch, yaw;
		bool have;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			have = haveImu_;
			imu  = latestImu_;
			getEuler(roll, pitch, yaw);
		}
		if (!have) {
			return;
		}

		constexpr int W = 640;
		constexpr int H = 480;
		cv::Mat image(H, W, CV_8UC3, cv::Scalar(20, 20, 20));

		drawArtificialHorizon(image, cv::Rect(20, 20, 260, 260), roll, pitch);
		drawCompass(image, cv::Point(410, 150), 100, yaw);
		drawBars(image, cv::Rect(20, 300, 260, 160), "Accel (m/s^2)",
				 {imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z},
				 20.0);
		drawBars(image, cv::Rect(300, 300, 320, 160), "Gyro (rad/s)",
				 {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z}, 5.0);
		drawReadouts(image, cv::Point(300, 20), roll, pitch, yaw, imu);

		auto msg            = dv_ros2_msgs::toRosImageMessage(image);
		msg.header.stamp    = imu.header.stamp;
		msg.header.frame_id = imu.header.frame_id;
		framePublisher_->publish(msg);
	}

	static void drawArtificialHorizon(cv::Mat &img, const cv::Rect &roi, double roll, double pitch) {
		const cv::Point2f center(roi.width * 0.5f, roi.height * 0.5f);
		const float radius = std::min(roi.width, roi.height) * 0.45f;

		// Positive pitch (RHR about +x) = nose-down -> horizon moves up in view.
		const float pitchPx = static_cast<float>(pitch * 180.0 / M_PI) * 2.0f;
		// Positive roll (RHR about +z, CCW viewed from behind) -> world rotates CW
		// in image -> right end of horizon goes down.
		const float cosR = std::cos(roll);
		const float sinR = std::sin(roll);

		cv::Mat canvas(roi.size(), CV_8UC3, cv::Scalar(20, 20, 20));
		for (int y = 0; y < roi.height; ++y) {
			for (int x = 0; x < roi.width; ++x) {
				const float dx = x - center.x;
				const float dy = y - center.y;
				if (dx * dx + dy * dy > radius * radius) continue;
				const float ry = -sinR * dx + cosR * dy + pitchPx;
				canvas.at<cv::Vec3b>(y, x) = (ry < 0) ? cv::Vec3b(200, 130, 60) : cv::Vec3b(40, 90, 140);
			}
		}
		cv::line(canvas, cv::Point2f(center.x - cosR * radius, center.y - sinR * radius - pitchPx),
				 cv::Point2f(center.x + cosR * radius, center.y + sinR * radius - pitchPx),
				 cv::Scalar(255, 255, 255), 2);
		cv::line(canvas, {int(center.x) - 30, int(center.y)}, {int(center.x) - 8, int(center.y)},
				 cv::Scalar(0, 255, 255), 3);
		cv::line(canvas, {int(center.x) + 8, int(center.y)}, {int(center.x) + 30, int(center.y)},
				 cv::Scalar(0, 255, 255), 3);
		cv::circle(canvas, center, 3, cv::Scalar(0, 255, 255), -1);
		cv::circle(canvas, center, int(radius), cv::Scalar(200, 200, 200), 1);

		canvas.copyTo(img(roi));
		cv::putText(img, "Attitude", {roi.x, roi.y - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
					cv::Scalar(220, 220, 220), 1);
	}

	static void drawCompass(cv::Mat &img, const cv::Point &center, int radius, double yaw) {
		cv::circle(img, center, radius, cv::Scalar(80, 80, 80), 1);
		cv::circle(img, center, radius - 4, cv::Scalar(40, 40, 40), -1);
		const double angle = -yaw - M_PI_2;
		const cv::Point tip(center.x + static_cast<int>(std::cos(angle) * (radius - 8)),
							center.y + static_cast<int>(std::sin(angle) * (radius - 8)));
		cv::arrowedLine(img, center, tip, cv::Scalar(0, 200, 255), 2, 8, 0, 0.2);
		cv::putText(img, "N", {center.x - 6, center.y - radius - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
					cv::Scalar(220, 220, 220), 1);
		cv::putText(img, "Yaw", {center.x - 20, center.y + radius + 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
					cv::Scalar(220, 220, 220), 1);
	}

	static void drawBars(cv::Mat &img, const cv::Rect &roi, const std::string &title,
						 const std::array<double, 3> &values, double fullScale) {
		cv::rectangle(img, roi, cv::Scalar(60, 60, 60), 1);
		cv::putText(img, title, {roi.x + 4, roi.y - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
					cv::Scalar(220, 220, 220), 1);

		const char *labels[3]       = {"x", "y", "z"};
		const cv::Scalar colors[3]  = {{80, 80, 255}, {80, 220, 80}, {255, 180, 80}};
		const int rowH  = roi.height / 3;
		const int zeroX = roi.x + roi.width / 2;
		const int halfW = roi.width / 2 - 20;
		for (int i = 0; i < 3; ++i) {
			const int cy   = roi.y + i * rowH + rowH / 2;
			const double v = std::clamp(values[i] / fullScale, -1.0, 1.0);
			const int bw   = static_cast<int>(v * halfW);
			cv::line(img, {zeroX, roi.y + i * rowH + 4}, {zeroX, roi.y + (i + 1) * rowH - 4},
					 cv::Scalar(120, 120, 120), 1);
			cv::rectangle(img, cv::Rect(std::min(zeroX, zeroX + bw), cy - 8, std::abs(bw), 16),
						  colors[i], -1);
			cv::putText(img, labels[i], {roi.x + 6, cy + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
						cv::Scalar(220, 220, 220), 1);
			char buf[32];
			std::snprintf(buf, sizeof(buf), "%+.2f", values[i]);
			cv::putText(img, buf, {roi.x + roi.width - 60, cy + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
						cv::Scalar(220, 220, 220), 1);
		}
	}

	static void drawReadouts(cv::Mat &img, const cv::Point &origin, double roll, double pitch,
							 double yaw, const dv_ros2_msgs::ImuMessage &imu) {
		char buf[64];
		constexpr double r2d = 180.0 / M_PI;
		auto line            = [&](int row, const std::string &s) {
            cv::putText(img, s, {origin.x, origin.y + 20 + row * 20}, cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(220, 220, 220), 1);
		};
		line(0, "Orientation (deg)");
		std::snprintf(buf, sizeof(buf), "  roll  %+7.2f", roll * r2d);  line(1, buf);
		std::snprintf(buf, sizeof(buf), "  pitch %+7.2f", pitch * r2d); line(2, buf);
		std::snprintf(buf, sizeof(buf), "  yaw   %+7.2f", yaw * r2d);   line(3, buf);
		line(5, "Accel (m/s^2)");
		std::snprintf(buf, sizeof(buf), "  %+6.2f %+6.2f %+6.2f", imu.linear_acceleration.x,
					  imu.linear_acceleration.y, imu.linear_acceleration.z);
		line(6, buf);
		line(8, "Gyro (rad/s)");
		std::snprintf(buf, sizeof(buf), "  %+6.2f %+6.2f %+6.2f", imu.angular_velocity.x,
					  imu.angular_velocity.y, imu.angular_velocity.z);
		line(9, buf);
	}

	// Mahony filter gains (standard starting values from Mahony et al. 2008).
	static constexpr double Kp_ = 2.0;    // proportional: convergence speed
	static constexpr double Ki_ = 0.005;  // integral: gyro bias correction

	rclcpp::Publisher<dv_ros2_msgs::ImageMessage>::SharedPtr framePublisher_;
	rclcpp::Subscription<dv_ros2_msgs::ImuMessage>::SharedPtr imuSubscriber_;
	rclcpp::TimerBase::SharedPtr renderTimer_;

	std::mutex mutex_;
	dv_ros2_msgs::ImuMessage latestImu_;
	bool haveImu_   = false;
	double lastStamp_ = 0.0;

	// Quaternion state (body-to-world). Initialised to identity (level, facing +z).
	double qw_ = 1.0, qx_ = 0.0, qy_ = 0.0, qz_ = 0.0;
	// Mahony integral error for bias estimation.
	double ix_ = 0.0, iy_ = 0.0, iz_ = 0.0;
};

} // namespace dv_visualization_node

RCLCPP_COMPONENTS_REGISTER_NODE(dv_visualization_node::ImuVisualizationNode)
