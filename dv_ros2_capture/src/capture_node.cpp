#include "../include/dv_ros2_capture/capture_node.hpp"

#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/io/camera/discovery.hpp>
#include <dv-processing/kinematics/transformation.hpp>

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp_components/register_node_macro.hpp>

using namespace dv_capture_node;
using namespace dv_ros2_msgs;
using namespace std::chrono_literals;

namespace {

dv::io::camera::imu::BoschBMI160AccelDataRate parseAccelDataRate(const std::string &s) {
	using R = dv::io::camera::imu::BoschBMI160AccelDataRate;
	if (s == "12.5Hz") return R::RATE_12_5HZ;
	if (s == "25Hz")   return R::RATE_25HZ;
	if (s == "50Hz")   return R::RATE_50HZ;
	if (s == "100Hz")  return R::RATE_100HZ;
	if (s == "200Hz")  return R::RATE_200HZ;
	if (s == "400Hz")  return R::RATE_400HZ;
	if (s == "800Hz")  return R::RATE_800HZ;
	if (s == "1600Hz") return R::RATE_1600HZ;
	throw std::invalid_argument(fmt::format(
		"imuAccelDataRate '{}' invalid. Use one of: 12.5Hz, 25Hz, 50Hz, 100Hz, 200Hz, 400Hz, 800Hz, 1600Hz", s));
}

dv::io::camera::imu::BoschBMI160GyroDataRate parseGyroDataRate(const std::string &s) {
	using R = dv::io::camera::imu::BoschBMI160GyroDataRate;
	if (s == "25Hz")   return R::RATE_25HZ;
	if (s == "50Hz")   return R::RATE_50HZ;
	if (s == "100Hz")  return R::RATE_100HZ;
	if (s == "200Hz")  return R::RATE_200HZ;
	if (s == "400Hz")  return R::RATE_400HZ;
	if (s == "800Hz")  return R::RATE_800HZ;
	if (s == "1600Hz") return R::RATE_1600HZ;
	if (s == "3200Hz") return R::RATE_3200HZ;
	throw std::invalid_argument(fmt::format(
		"imuGyroDataRate '{}' invalid. Use one of: 25Hz, 50Hz, 100Hz, 200Hz, 400Hz, 800Hz, 1600Hz, 3200Hz", s));
}

dv::io::camera::imu::BoschBMI160AccelFilter parseAccelFilter(const std::string &s) {
	using F = dv::io::camera::imu::BoschBMI160AccelFilter;
	if (s == "normal") return F::FILTER_NORMAL;
	if (s == "osr2")   return F::FILTER_OSR2;
	if (s == "osr4")   return F::FILTER_OSR4;
	throw std::invalid_argument(fmt::format(
		"imuAccelFilter '{}' invalid. Use one of: normal, osr2, osr4", s));
}

dv::io::camera::imu::BoschBMI160GyroFilter parseGyroFilter(const std::string &s) {
	using F = dv::io::camera::imu::BoschBMI160GyroFilter;
	if (s == "normal") return F::FILTER_NORMAL;
	if (s == "osr2")   return F::FILTER_OSR2;
	if (s == "osr4")   return F::FILTER_OSR4;
	throw std::invalid_argument(fmt::format(
		"imuGyroFilter '{}' invalid. Use one of: normal, osr2, osr4", s));
}

} // namespace

CaptureNode::CaptureNode(const rclcpp::NodeOptions &options) :
	rclcpp::Node("capture_node", options) {
	auto loadParams = dv_ros2_node::ParametersLoader(*this);
	loadParams.printConfiguration();
	mParams = loadParams.getParams();

	mSpinThread = true;
	if (mParams.aedat4FilePath.empty()) {
		mReader = dv::io::camera::open(mParams.cameraName);
	}
	else {
		mReader = std::make_unique<dv::io::MonoCameraRecording>(mParams.aedat4FilePath, mParams.cameraName);
	}
	startupTime = this->now();
	if (mParams.frames && !mReader->isFrameStreamAvailable()) {
		mParams.frames = false;
		RCLCPP_WARN(this->get_logger(), "Frame stream is not available!");
	}
	if (mParams.events && !mReader->isEventStreamAvailable()) {
		mParams.events = false;
		RCLCPP_WARN(this->get_logger(), "Event stream is not available!");
	}
	if (mParams.imu && !mReader->isImuStreamAvailable()) {
		mParams.imu = false;
		RCLCPP_WARN(this->get_logger(), "Imu data stream is not available!");
	}
	if (mParams.triggers && !mReader->isTriggerStreamAvailable()) {
		mParams.triggers = false;
		RCLCPP_WARN(this->get_logger(), "Trigger data stream is not available!");
	}

	if (mParams.frames) {
		mFramePublisher = this->create_publisher<ImageMessage>("camera/image", 10);
	}

	if (mParams.imu) {
		mImuPublisher = this->create_publisher<ImuMessage>("imu", 1000);
	}

	if (mParams.events) {
		mEventArrayPublisher = this->create_publisher<EventArrayMessage>("events", 1000);
	}

	if (mParams.triggers) {
		mTriggerPublisher = this->create_publisher<TriggerMessage>("triggers", 10);
	}

	if (!mParams.frames && !mParams.events) {
		// Camera info is not published if frames nor events is enabled
		return;
	}

	mCameraInfoPublisher = this->create_publisher<CameraInfoMessage>("camera_info", 10);
	mCameraService = this->create_service<sensor_msgs::srv::SetCameraInfo>(
		"set_camera_info",
		std::bind(&CaptureNode::setCameraInfo, this, std::placeholders::_1, std::placeholders::_2));
	mImuInfoService = this->create_service<dv_ros2_capture::srv::SetImuInfo>(
		"set_imu_info",
		std::bind(&CaptureNode::setImuInfo, this, std::placeholders::_1, std::placeholders::_2));
	mImuBiasesService = this->create_service<dv_ros2_capture::srv::SetImuBiases>(
		"set_imu_biases",
		std::bind(&CaptureNode::setImuBiases, this, std::placeholders::_1, std::placeholders::_2));

	fs::path calibrationPath = getActiveCalibrationPath();
	if (!mParams.cameraCalibrationFilePath.empty()) {
		RCLCPP_INFO(this->get_logger(), "Loading user supplied calibration at path [%s]", mParams.cameraCalibrationFilePath.c_str());
		if (!fs::exists(mParams.cameraCalibrationFilePath)) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"User supplied calibration file does not exist!", mParams.cameraCalibrationFilePath);
		}
		RCLCPP_INFO(this->get_logger(), "Loading calibration data from %s...", mParams.cameraCalibrationFilePath.c_str());
		calibrationPath = mParams.cameraCalibrationFilePath; // load file directly instead of copying to active calibration path
	}

	if (fs::exists(calibrationPath)) {
		RCLCPP_INFO(this->get_logger(), "Loading calibration file [%s]", calibrationPath.c_str());
		mCalibration                 = dv::camera::CalibrationSet::LoadFromFile(calibrationPath);
		const std::string cameraName = mReader->getCameraName();
		auto cameraCalibration       = mCalibration.getCameraCalibrationByName(cameraName);
		if (const auto &imuCalib = mCalibration.getImuCalibrationByName(cameraName); imuCalib.has_value()) {
			mTransformPublisher = this->create_publisher<TransformsMessage>("/tf", 100);
			mImuTimeOffset      = imuCalib->timeOffsetMicros;

			TransformMessage msg;
			msg.header.frame_id = mParams.imuFrameName;
			msg.child_frame_id  = mParams.cameraFrameName;

			mImuToCamTransform = dv::kinematics::Transformationf(0, imuCalib->transformationToC0.getTransform());

			mAccBiases.x() = imuCalib->accOffsetAvg.x;
			mAccBiases.y() = imuCalib->accOffsetAvg.y;
			mAccBiases.z() = imuCalib->accOffsetAvg.z;

			mGyroBiases.x() = imuCalib->omegaOffsetAvg.x;
			mGyroBiases.y() = imuCalib->omegaOffsetAvg.y;
			mGyroBiases.z() = imuCalib->omegaOffsetAvg.z;

			const auto translation      = mImuToCamTransform.getTranslation<Eigen::Vector3d>();
			msg.transform.translation.x = translation.x();
			msg.transform.translation.y = translation.y();
			msg.transform.translation.z = translation.z();

			const auto rotation      = mImuToCamTransform.getQuaternion();
			msg.transform.rotation.x = rotation.x();
			msg.transform.rotation.y = rotation.y();
			msg.transform.rotation.z = rotation.z();
			msg.transform.rotation.w = rotation.w();

			mImuToCamTransforms = TransformsMessage();
			mImuToCamTransforms->transforms.push_back(msg);
		}
		if (cameraCalibration.has_value()) {
			populateInfoMsg(cameraCalibration->getCameraGeometry());
		}
		else {
			RCLCPP_ERROR(this->get_logger(), "Calibration in [%s] does not contain calibration for camera [%s]",
				calibrationPath.c_str(), cameraName.c_str());
			std::vector<std::string> names;
			for (const auto &calib : mCalibration.getCameraCalibrations()) {
				names.push_back(calib.second.name);
			}
			const std::string nameString = fmt::format("{}", fmt::join(names, "; "));
			RCLCPP_ERROR(this->get_logger(), "The file only contains calibrations for these cameras: [%s]", nameString.c_str());
			throw std::runtime_error("Calibration is not available!");
		}
	}
	else {
		RCLCPP_WARN(this->get_logger(), "[%s] No calibration was found, assuming ideal pinhole (no distortion).", mReader->getCameraName().c_str());
		std::optional<cv::Size> resolution;
		if (mReader->isFrameStreamAvailable()) {
			resolution = mReader->getFrameResolution();
		}
		else if (mReader->isEventStreamAvailable()) {
			resolution = mReader->getEventResolution();
		}
		if (resolution.has_value()) {
			const auto width = static_cast<float>(resolution->width);
			populateInfoMsg(dv::camera::CameraGeometry(
				width, width, width * 0.5f, static_cast<float>(resolution->height) * 0.5f, *resolution));
			generateActiveCalibrationFile();
		}
		else {
			throw std::runtime_error("Sensor resolution not available.");
		}
	}

	// IMPORTANT: User-supplied static IMU biases override anything
	// loaded from the camera calibration file.
	if (mParams.accelerometerBias.size() == 3) {
		mAccBiases.x() = static_cast<float>(mParams.accelerometerBias[0]);
		mAccBiases.y() = static_cast<float>(mParams.accelerometerBias[1]);
		mAccBiases.z() = static_cast<float>(mParams.accelerometerBias[2]);
		RCLCPP_INFO(this->get_logger(), "Applied accelerometer bias from yaml: [%+.4f, %+.4f, %+.4f]",
			mAccBiases.x(), mAccBiases.y(), mAccBiases.z());
	}
	if (mParams.gyroscopeBias.size() == 3) {
		mGyroBiases.x() = static_cast<float>(mParams.gyroscopeBias[0]);
		mGyroBiases.y() = static_cast<float>(mParams.gyroscopeBias[1]);
		mGyroBiases.z() = static_cast<float>(mParams.gyroscopeBias[2]);
		RCLCPP_INFO(this->get_logger(), "Applied gyroscope bias from yaml:     [%+.6f, %+.6f, %+.6f]",
			mGyroBiases.x(), mGyroBiases.y(), mGyroBiases.z());
	}

	// Configure camera-specific hardware settings and register dynamic parameter callback
	configureCameraHardware();

	updateNoiseFilter(mParams.noiseFiltering, mParams.noiseBATime);

	startCapture();
}

void CaptureNode::populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry) {
	mCameraInfoMsg.width  = cameraGeometry.getResolution().width;
	mCameraInfoMsg.height = cameraGeometry.getResolution().height;

	const auto distortion = cameraGeometry.getDistortion();

	switch (cameraGeometry.getDistortionModel()) {
		case dv::camera::DistortionModel::EQUIDISTANT: {
			mCameraInfoMsg.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
			mCameraInfoMsg.d.assign(distortion.begin(), distortion.end());
			break;
		}

		case dv::camera::DistortionModel::RADIAL_TANGENTIAL: {
			mCameraInfoMsg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
			mCameraInfoMsg.d.assign(distortion.begin(), distortion.end());
			if (mCameraInfoMsg.d.size() < 5) {
				mCameraInfoMsg.d.resize(5, 0.0);
			}
			break;
		}

		case dv::camera::DistortionModel::NONE: {
			mCameraInfoMsg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
			mCameraInfoMsg.d                = {0.0, 0.0, 0.0, 0.0, 0.0};
			break;
		}

		default:
			throw dv::exceptions::InvalidArgument<dv::camera::DistortionModel>(
				"Unsupported camera distortion model.", cameraGeometry.getDistortionModel());
	}

	auto cx = cameraGeometry.getCentralPoint().x;
	auto cy = cameraGeometry.getCentralPoint().y;
	auto fx = cameraGeometry.getFocalLength().x;
	auto fy = cameraGeometry.getFocalLength().y;

	mCameraInfoMsg.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
	mCameraInfoMsg.r = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
	mCameraInfoMsg.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0};
	mRawCameraInfoMsg = mCameraInfoMsg;
	updateEventUndistortionParams();
}

void CaptureNode::updateEventUndistortionParams() {
	mCameraInfoMsg = mRawCameraInfoMsg;
	mUndistortMap.release();

	if (!mParams.undistortEvents) {
		return;
	}

	if (mRawCameraInfoMsg.width == 0 || mRawCameraInfoMsg.height == 0 || mRawCameraInfoMsg.k.size() < 9) {
		return;
	}

	const int width  = static_cast<int>(mRawCameraInfoMsg.width);
	const int height = static_cast<int>(mRawCameraInfoMsg.height);

	cv::Mat Kdist = (cv::Mat_<double>(3, 3) <<
		mRawCameraInfoMsg.k[0], mRawCameraInfoMsg.k[1], mRawCameraInfoMsg.k[2],
		mRawCameraInfoMsg.k[3], mRawCameraInfoMsg.k[4], mRawCameraInfoMsg.k[5],
		mRawCameraInfoMsg.k[6], mRawCameraInfoMsg.k[7], mRawCameraInfoMsg.k[8]);

	if (mRawCameraInfoMsg.d.empty()) {
		return;
	}

	cv::Mat distCoeffs(1, static_cast<int>(mRawCameraInfoMsg.d.size()), CV_64F);
	for (int index = 0; index < static_cast<int>(mRawCameraInfoMsg.d.size()); ++index) {
		distCoeffs.at<double>(0, index) = mRawCameraInfoMsg.d[static_cast<size_t>(index)];
	}

	std::vector<cv::Point2f> pixels;
	pixels.reserve(static_cast<size_t>(width) * static_cast<size_t>(height));
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			pixels.emplace_back(static_cast<float>(x), static_cast<float>(y));
		}
	}

	const cv::Mat pixelsMat(pixels);
	cv::Mat undistortedPixels;
	cv::Mat Knew;
	const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
	const bool isFisheye = (mRawCameraInfoMsg.distortion_model == sensor_msgs::distortion_models::EQUIDISTANT);

	if (isFisheye) {
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
			Kdist, distCoeffs, cv::Size(width, height), identity, Knew, 0.0);
		cv::fisheye::undistortPoints(pixelsMat, undistortedPixels, Kdist, distCoeffs, identity, Knew);
	}
	else {
		Knew = cv::getOptimalNewCameraMatrix(Kdist, distCoeffs, cv::Size(width, height), 0.0);
		cv::undistortPoints(pixelsMat, undistortedPixels, Kdist, distCoeffs, cv::noArray(), Knew);
	}

	mUndistortMap = undistortedPixels.reshape(2, height);

	const double fxNew = Knew.at<double>(0, 0);
	const double fyNew = Knew.at<double>(1, 1);
	const double cxNew = Knew.at<double>(0, 2);
	const double cyNew = Knew.at<double>(1, 2);

	mCameraInfoMsg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	mCameraInfoMsg.d                = {0.0, 0.0, 0.0, 0.0, 0.0};
	mCameraInfoMsg.k                = {fxNew, 0.0, cxNew, 0.0, fyNew, cyNew, 0.0, 0.0, 1.0};
	mCameraInfoMsg.r                = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	mCameraInfoMsg.p                = {fxNew, 0.0, cxNew, 0.0, 0.0, fyNew, cyNew, 0.0, 0.0, 0.0, 1.0, 0.0};
}

dv::EventStore CaptureNode::undistortEvents(const dv::EventStore &events) {
	if (mUndistortMap.empty()) {
		return events;
	}

	const int width  = mUndistortMap.cols;
	const int height = mUndistortMap.rows;

	auto packet = std::make_shared<dv::EventPacket>();
	packet->elements.reserve(events.size());

	for (const auto &event : events) {
		const auto &uv = mUndistortMap.at<cv::Vec2f>(event.y(), event.x());
		const auto ux  = static_cast<int16_t>(std::round(uv[0]));
		const auto uy  = static_cast<int16_t>(std::round(uv[1]));

		if (ux < 0 || ux >= width || uy < 0 || uy >= height) {
			continue;
		}

		packet->elements.emplace_back(event.timestamp(), ux, uy, event.polarity());
	}

	return dv::EventStore(std::const_pointer_cast<const dv::EventPacket>(packet));
}

void CaptureNode::setCameraInfo(
	const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
	std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> rsp) {
	// Set camera info is usually called from a camera calibration pipeline, like the ros camera calibration.
	mCameraInfoMsg = req->camera_info;
	mRawCameraInfoMsg = mCameraInfoMsg;
	updateEventUndistortionParams();

	try {
		auto calibPath     = saveCalibration();
		rsp->success        = true;
		rsp->status_message = fmt::format("Calibration stored successfully in {0}.", calibPath);
	}
	catch (const std::exception &e) {
		rsp->success        = false;
		rsp->status_message = fmt::format("Error storing camera calibration.");
	}
}

void CaptureNode::setImuBiases(
	const std::shared_ptr<dv_ros2_capture::srv::SetImuBiases::Request> req,
	std::shared_ptr<dv_ros2_capture::srv::SetImuBiases::Response> rsp) {
	if (mParams.unbiasedImuData && (!mAccBiases.isZero() || !mGyroBiases.isZero())) {
		RCLCPP_ERROR(this->get_logger(), "Trying to set IMU biases on a camera capture node which publishes IMU data with biases subtracted.");
		RCLCPP_ERROR(this->get_logger(), "The received biases will be ignored");
		rsp->success        = false;
		rsp->status_message = "Failed to apply IMU biases since biases are already applied.";
		return;
	}

	RCLCPP_INFO(this->get_logger(), "Setting IMU biases...");
	mAccBiases  = Eigen::Vector3f(req->acc_biases.x, req->acc_biases.y, req->acc_biases.z);
	mGyroBiases = Eigen::Vector3f(req->gyro_biases.x, req->gyro_biases.y, req->gyro_biases.z);

	try {
		saveCalibration();
		rsp->success        = true;
		rsp->status_message = "IMU biases stored in calibration file.";
		RCLCPP_INFO(this->get_logger(), "Unbiasing output IMU messages.");
		mParams.unbiasedImuData = true;
	}
	catch (const std::exception &e) {
		rsp->success        = false;
		rsp->status_message = "Error storing Imu biases calibration.";
	}
}

void CaptureNode::synchronizeCamera(
	const std::shared_ptr<dv_ros2_capture::srv::SynchronizeCamera::Request> req,
	std::shared_ptr<dv_ros2_capture::srv::SynchronizeCamera::Response> rsp) {
	RCLCPP_INFO(this->get_logger(), "Received synchronization request from [%s]", req->master_camera_name.c_str());

	// Assume failure case
	rsp->success = false;

	auto *liveCapture = dynamic_cast<dv::io::camera::SyncCameraInputBase *>(mReader.get());
	if (!liveCapture) {
		RCLCPP_WARN(this->get_logger(), "Received synchronization request on a non-live/synchronizable camera!");
		return;
	}
	if (liveCapture->isRunning() && !liveCapture->isMaster()) {
		// Update the timestamp offset
		liveCapture->setTimestampOffset(std::chrono::microseconds{req->timestamp_offset});
		RCLCPP_INFO(this->get_logger(), "Camera [%s] synchronized: timestamp offset updated.", liveCapture->getCameraName().c_str());
		rsp->camera_name = liveCapture->getCameraName();
		rsp->success     = true;
		mSynchronized    = true;
	}
	else {
		RCLCPP_WARN(this->get_logger(), "Received synchronization request on a master camera, please check synchronization cable!");
	}
}

void CaptureNode::setImuInfo(
	const std::shared_ptr<dv_ros2_capture::srv::SetImuInfo::Request> req,
	std::shared_ptr<dv_ros2_capture::srv::SetImuInfo::Response> rsp) {
	mImuTimeOffset = req->imu_info.time_offset_micros;
	geometry_msgs::msg::TransformStamped stampedTransform;
	stampedTransform.transform         = req->imu_info.t_sc;
	stampedTransform.header.frame_id   = mParams.imuFrameName;
	stampedTransform.child_frame_id    = mParams.cameraFrameName;
	mImuToCamTransforms->transforms[0] = stampedTransform;

	Eigen::Quaternion<float> q(static_cast<float>(stampedTransform.transform.rotation.w),
		static_cast<float>(stampedTransform.transform.rotation.x),
		static_cast<float>(stampedTransform.transform.rotation.y),
		static_cast<float>(stampedTransform.transform.rotation.z));
	mImuToCamTransform = dv::kinematics::Transformationf(0, Eigen::Vector3f::Zero(), q);

	try {
		auto calibPath     = saveCalibration();
		rsp->success        = true;
		rsp->status_message = fmt::format("Calibration stored successfully in {0}.", calibPath);
	}
	catch (const std::exception &e) {
		rsp->success        = false;
		rsp->status_message = fmt::format("Error storing camera calibration.");
	}
}

fs::path CaptureNode::getCameraCalibrationDirectory(const bool createDirectories) const {
	const fs::path directory
		= fmt::format("{0}/.dv_camera/camera_calibration/{1}", std::getenv("HOME"), mReader->getCameraName());
	if (createDirectories && !fs::exists(directory)) {
		fs::create_directories(directory);
	}
	return directory;
}

fs::path CaptureNode::getActiveCalibrationPath() const {
	return getCameraCalibrationDirectory() / "active_calibration.json";
}

void CaptureNode::generateActiveCalibrationFile() {
	RCLCPP_INFO(this->get_logger(), "Generating active calibration file...");
	updateCalibrationSet();
	mCalibration.writeToFile(getActiveCalibrationPath());
}

fs::path CaptureNode::saveCalibration() {
	auto date = fmt::format("{:%Y_%m_%d_%H_%M_%S}", dv::toTimePoint(dv::now()));
	const std::string calibrationFileName
		= fmt::format("calibration_camera_{0}_{1}.json", mReader->getCameraName(), date);
	const fs::path calibPath = getCameraCalibrationDirectory() / calibrationFileName;
	updateCalibrationSet();
	mCalibration.writeToFile(calibPath);

	fs::copy_file(calibPath, getActiveCalibrationPath(), fs::copy_options::overwrite_existing);
	return calibPath;
}

void CaptureNode::updateCalibrationSet() {
	RCLCPP_INFO(this->get_logger(), "Generating calibration set...");
	const std::string cameraName = mReader->getCameraName();
	dv::camera::calibrations::CameraCalibration calib;
	bool calibrationExists = false;
	if (auto camCalibration = mCalibration.getCameraCalibrationByName(cameraName); camCalibration.has_value()) {
		calib             = *camCalibration;
		calibrationExists = true;
	}
	else {
		calib.name = cameraName;
	}
	calib.resolution = cv::Size(static_cast<int>(mCameraInfoMsg.width), static_cast<int>(mCameraInfoMsg.height));
	calib.distortion.clear();
	calib.distortion.assign(mCameraInfoMsg.d.begin(), mCameraInfoMsg.d.end());
	if (mCameraInfoMsg.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB) {
		calib.distortionModel = dv::camera::DistortionModel::RADIAL_TANGENTIAL;
	}
	else if (mCameraInfoMsg.distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
		calib.distortionModel = dv::camera::DistortionModel::EQUIDISTANT;
	}
	else {
		throw dv::exceptions::InvalidArgument<std::string>(
			"Unknown camera model.", mCameraInfoMsg.distortion_model);
	}
	calib.focalLength = cv::Point2f(static_cast<float>(mCameraInfoMsg.k[0]), static_cast<float>(mCameraInfoMsg.k[4]));
	calib.principalPoint
		= cv::Point2f(static_cast<float>(mCameraInfoMsg.k[2]), static_cast<float>(mCameraInfoMsg.k[5]));

	calib.transformationToC0 = dv::kinematics::Transformationf{};

	if (calibrationExists) {
		mCalibration.updateCameraCalibration(calib);
	}
	else {
		mCalibration.addCameraCalibration(calib);
	}

	dv::camera::calibrations::IMUCalibration imuCalibration;
	bool imuCalibrationExists = false;
	if (auto imuCalib = mCalibration.getImuCalibrationByName(cameraName); imuCalib.has_value()) {
		imuCalibration       = *imuCalib;
		imuCalibrationExists = true;
	}
	else {
		imuCalibration.name = cameraName;
	}
	bool imuHasValues = false;
	if ((mImuToCamTransforms.has_value() && !mImuToCamTransforms->transforms.empty())) {
		const Eigen::Matrix4f mat         = mImuToCamTransform.getTransform().transpose();
		imuCalibration.transformationToC0 = dv::kinematics::Transformationf{0, mat};
		imuHasValues                      = true;
	}

	if (!mAccBiases.isZero()) {
		imuCalibration.accOffsetAvg.x = mAccBiases.x();
		imuCalibration.accOffsetAvg.y = mAccBiases.y();
		imuCalibration.accOffsetAvg.z = mAccBiases.z();
		imuHasValues                  = true;
	}

	if (!mGyroBiases.isZero()) {
		imuCalibration.omegaOffsetAvg.x = mGyroBiases.x();
		imuCalibration.omegaOffsetAvg.y = mGyroBiases.y();
		imuCalibration.omegaOffsetAvg.z = mGyroBiases.z();
		imuHasValues                    = true;
	}

	if (mImuTimeOffset > 0) {
		imuCalibration.timeOffsetMicros = mImuTimeOffset;
		imuHasValues                    = true;
	}

	if (imuCalibrationExists) {
		mCalibration.updateImuCalibration(imuCalibration);
	}
	else if (imuHasValues) {
		mCalibration.addImuCalibration(imuCalibration);
	}
}

void CaptureNode::startCapture() {
	RCLCPP_INFO(this->get_logger(), "Spinning capture node.");

	auto *liveCapture = dynamic_cast<dv::io::camera::CameraInputBase *>(mReader.get());
	// If the pointer is valid - the reader is handling a live camera
	if (liveCapture) {
		mSynchronized = false;
		mSyncThread   = std::thread(&CaptureNode::synchronizationThread, this);
	}
	else {
		mSynchronized = true;
	}

	auto *fileCapture = dynamic_cast<dv::io::MonoCameraRecording *>(mReader.get());
	if (fileCapture) {
		const auto times = fileCapture->getTimeRange();
		mClock           = std::thread(&CaptureNode::clock, this, times.first, times.second, mParams.timeIncrement);
	}
	else {
		mClock = std::thread(&CaptureNode::clock, this, -1, -1, mParams.timeIncrement);
	}
	if (mParams.frames) {
		mFrameThread = std::thread(&CaptureNode::framePublisher, this);
	}
	if (mParams.imu) {
		mImuThread = std::thread(&CaptureNode::imuPublisher, this);
	}
	if (mParams.events) {
		mEventsThread = std::thread(&CaptureNode::eventsPublisher, this);
	}
	if (mParams.triggers) {
		mTriggerThread = std::thread(&CaptureNode::triggerPublisher, this);
	}

	if (mParams.events || mParams.frames) {
		mCameraInfoThread = std::make_unique<std::thread>([this] {
			rclcpp::Rate infoRate(25.0);
			while (mSpinThread.load(std::memory_order_relaxed)) {
				const auto currentTime = dv_ros2_msgs::toRosTime(mCurrentSeek);
				if (mCameraInfoPublisher->get_subscription_count() > 0) {
					mCameraInfoMsg.header.stamp = currentTime;
					mCameraInfoPublisher->publish(mCameraInfoMsg);
				}
				if (mImuToCamTransforms.has_value() && !mImuToCamTransforms->transforms.empty()) {
					mImuToCamTransforms->transforms.back().header.stamp = currentTime;
					mTransformPublisher->publish(*mImuToCamTransforms);
				}
				infoRate.sleep();
			}
		});
	}
}

void CaptureNode::updateNoiseFilter(const bool enable, const int64_t backgroundActivityTime) {
	if (enable) {
		// Create the filter and return
		if (mNoiseFilter == nullptr) {
			mNoiseFilter = std::make_unique<dv::noise::BackgroundActivityNoiseFilter<>>(
				mReader->getEventResolution().value(), dv::Duration(backgroundActivityTime));
			return;
		}

		// Noise filter is instantiated, just update the period
		mNoiseFilter->setBackgroundActivityDuration(dv::Duration(backgroundActivityTime));
	}
	else {
		// Destroy the filter
		mNoiseFilter = nullptr;
	}
}

void CaptureNode::clock(int64_t start, int64_t end, int64_t timeIncrement) {
	RCLCPP_INFO(this->get_logger(), "Spinning clock.");

	double frequency = 1.0 / (static_cast<double>(timeIncrement) * 1e-6);

	rclcpp::Rate sleepRate(frequency);
	if (start == -1) {
		start         = std::numeric_limits<int64_t>::max() - 1;
		end           = std::numeric_limits<int64_t>::max();
		timeIncrement = 0;
		RCLCPP_INFO(this->get_logger(), "Reading from camera [%s]...", mReader->getCameraName().c_str());
	}

	while (mSpinThread) {
		if (mSynchronized.load(std::memory_order_relaxed)) {
			if (mParams.frames) {
				mFrameQueue.push(start);
			}
			if (mParams.imu) {
				mImuQueue.push(start);
			}
			if (mParams.events) {
				mEventsQueue.push(start);
			}
			if (mParams.triggers) {
				mTriggerQueue.push(start);
			}
			start += timeIncrement;
		}

		sleepRate.sleep();
		// EOF or reader is disconnected
		if (start >= end || !mReader->isRunning()) {
			mSpinThread = false;
		}
	}
}

void CaptureNode::stop() {
	RCLCPP_INFO(this->get_logger(), "Stopping the capture node.");

	mSpinThread = false;
	mClock.join();
	if (mParams.frames) {
		mFrameThread.join();
	}
	if (mParams.imu) {
		mImuThread.join();
	}
	if (mParams.events) {
		mEventsThread.join();
	}
	if (mParams.triggers) {
		mTriggerThread.join();
	}
	if (mSyncThread.joinable()) {
		mSyncThread.join();
	}
	if (mCameraInfoThread != nullptr) {
		mCameraInfoThread->join();
	}
	if (mDiscoveryThread != nullptr) {
		mDiscoveryThread->join();
	}
}

void CaptureNode::framePublisher() {
	RCLCPP_INFO(this->get_logger(), "Spinning framePublisher");

	std::optional<dv::Frame> frame = std::nullopt;

	while (mSpinThread) {
		mFrameQueue.consume_all([&](const int64_t timestamp) {
			if (!frame.has_value()) {
				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				frame = mReader->getNextFrame();
			}
			while (frame.has_value() && timestamp >= frame->timestamp) {
				if (mFramePublisher->get_subscription_count() > 0) {
					ImageMessage msg = dv_ros2_msgs::frameToRosImageMessage(*frame);
					mFramePublisher->publish(msg);
				}

				mCurrentSeek = frame->timestamp;

				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				frame = mReader->getNextFrame();
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void CaptureNode::imuPublisher() {
	RCLCPP_INFO(this->get_logger(), "Spinning imuPublisher");

	std::optional<std::vector<dv::IMU>> imuData = std::nullopt;

	while (mSpinThread) {
		mImuQueue.consume_all([&](const int64_t timestamp) {
			if (!imuData.has_value()) {
				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				imuData = mReader->getNextImuBatch();
			}
			while (imuData.has_value() && !imuData->empty() && timestamp >= imuData->back().timestamp) {
				if (mImuPublisher->get_subscription_count() > 0) {
					for (auto &imu : *imuData) {
						imu.timestamp += mImuTimeOffset;
						mImuPublisher->publish(transformImuFrame(dv_ros2_msgs::toRosImuMessage(imu)));
					}
				}

				mCurrentSeek = imuData->back().timestamp;

				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				imuData = mReader->getNextImuBatch();
			}

			// If value present but empty, we don't want to keep it for later spins.
			if (imuData.has_value() && imuData->empty()) {
				imuData = std::nullopt;
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void CaptureNode::triggerPublisher() {
	RCLCPP_INFO(this->get_logger(), "Spinning triggerPublisher");

	std::optional<std::vector<dv::Trigger>> triggerData = std::nullopt;

	while (mSpinThread) {
		mTriggerQueue.consume_all([&](const int64_t timestamp) {
			if (!triggerData.has_value()) {
				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				triggerData = mReader->getNextTriggerBatch();
			}
			while (triggerData.has_value() && !triggerData->empty() && timestamp >= triggerData->back().timestamp) {
				if (mTriggerPublisher->get_subscription_count() > 0) {
					for (const auto &trigger : *triggerData) {
						mTriggerPublisher->publish(dv_ros2_msgs::toRosTriggerMessage(trigger));
					}
				}

				mCurrentSeek = triggerData->back().timestamp;

				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				triggerData = mReader->getNextTriggerBatch();
			}

			// If value present but empty, we don't want to keep it for later spins.
			if (triggerData.has_value() && triggerData->empty()) {
				triggerData = std::nullopt;
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void CaptureNode::eventsPublisher() {
	RCLCPP_INFO(this->get_logger(), "Spinning eventsPublisher");

	std::optional<dv::EventStore> events = std::nullopt;

	cv::Size resolution = mReader->getEventResolution().value();

	while (mSpinThread) {
		mEventsQueue.consume_all([&](const int64_t timestamp) {
			if (!events.has_value()) {
				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				events = mReader->getNextEventBatch();
			}
			while (events.has_value() && !events->isEmpty() && timestamp >= events->getHighestTime()) {
				dv::EventStore store;
				if (mNoiseFilter != nullptr) {
					mNoiseFilter->accept(*events);
					store = mNoiseFilter->generateEvents();
				}
				else {
					store = *events;
				}

				if (mParams.undistortEvents) {
                    auto start_time = std::chrono::high_resolution_clock::now();
					store = undistortEvents(store);
                    auto end_time = std::chrono::high_resolution_clock::now();
				}

				if (mEventArrayPublisher->get_subscription_count() > 0) {
					auto msg = dv_ros2_msgs::toRosEventsMessage(store, resolution);
					mEventArrayPublisher->publish(msg);
				}

				mCurrentSeek = events->getHighestTime();

				std::lock_guard<std::recursive_mutex> lockGuard(mReaderMutex);
				events = mReader->getNextEventBatch();
			}

			// If value present but empty, we don't want to keep it for later spins.
			if (events.has_value() && events->isEmpty()) {
				events = std::nullopt;
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

CaptureNode::~CaptureNode() {
	stop();
}

bool CaptureNode::isRunning() const {
	return mSpinThread.load(std::memory_order_relaxed);
}

void CaptureNode::runDiscovery(const std::string &syncServiceName) {
	auto *liveCapture = dynamic_cast<dv::io::camera::CameraInputBase *>(mReader.get());

	if (!liveCapture) {
		return;
	}

	auto *syncCamera = dynamic_cast<dv::io::camera::SyncCameraInputBase *>(mReader.get());

	mDiscoveryPublisher = this->create_publisher<DiscoveryMessage>("/dvs/discovery", 10);
	mDiscoveryThread    = std::make_unique<std::thread>([this, &liveCapture, &syncCamera, &syncServiceName] {
		DiscoveryMessage message;
		message.is_master           = (syncCamera) ? (syncCamera->isMaster()) : (true);
		message.name               = liveCapture->getCameraName();
		message.startup_time       = rclcpp::Time(startupTime);
		message.publishing_events   = mParams.events;
		message.publishing_frames   = mParams.frames;
		message.publishing_imu      = mParams.imu;
		message.publishing_triggers = mParams.triggers;
		message.sync_service_topic  = syncServiceName;
		// 5 Hz is enough
		rclcpp::Rate rate(5.0);
		while (mSpinThread) {
			if (mDiscoveryPublisher->get_subscription_count() > 0) {
				message.header.stamp = this->now();
				mDiscoveryPublisher->publish(message);
			}
			rate.sleep();
		}
	});
}

std::map<std::string, std::string> CaptureNode::discoverSyncDevices() {
	if (mParams.syncDeviceList.empty()) {
		return {};
	}

	RCLCPP_INFO(this->get_logger(), "Waiting for devices [%s] to be online",
		fmt::format("{}", fmt::join(mParams.syncDeviceList, ", ")).c_str());

	// List info about each sync device
	struct DiscoveryContext {
		std::map<std::string, std::string> serviceNames;
		std::atomic<bool> complete;
		std::vector<std::string> deviceList;

		void handleMessage(const DiscoveryMessage::SharedPtr message) {
			const std::string cameraName(message->name.c_str());
			if (serviceNames.contains(cameraName)) {
				return;
			}

			if (std::find(deviceList.begin(), deviceList.end(), cameraName) != deviceList.end()) {
				serviceNames.insert(std::make_pair(cameraName, message->sync_service_topic.c_str()));
				if (serviceNames.size() == deviceList.size()) {
					complete = true;
				}
			}
		}
	};

	DiscoveryContext context;
	context.deviceList = mParams.syncDeviceList;
	context.complete   = false;

	auto subscriber = this->create_subscription<DiscoveryMessage>(
		"/dvs/discovery", 10,
		[&context](const DiscoveryMessage::SharedPtr msg) {
			context.handleMessage(msg);
		});

	while (mSpinThread.load(std::memory_order_relaxed) && !context.complete.load(std::memory_order_relaxed)) {
		std::this_thread::sleep_for(1ms);
	}

	RCLCPP_INFO(this->get_logger(), "All sync devices are online.");

	return context.serviceNames;
}

void CaptureNode::sendSyncCalls(const std::map<std::string, std::string> &serviceNames) {
	if (serviceNames.empty()) {
		return;
	}

	auto *liveCapture = dynamic_cast<dv::io::camera::SyncCameraInputBase *>(mReader.get());

	if (!liveCapture) {
		return;
	}

	for (const auto &[cameraName, serviceName] : serviceNames) {
		if (serviceName.empty()) {
			RCLCPP_ERROR(this->get_logger(), "Camera [%s] can't be synchronized, synchronization service is unavailable, please check synchronization cable!", cameraName.c_str());
			continue;
		}

		auto client = this->create_client<dv_ros2_capture::srv::SynchronizeCamera>(serviceName);
		client->wait_for_service();
		auto request = std::make_shared<dv_ros2_capture::srv::SynchronizeCamera::Request>();
		request->timestamp_offset  = liveCapture->getTimestampOffset().count();
		request->master_camera_name = liveCapture->getCameraName();
		auto future = client->async_send_request(request);
		if (future.wait_for(5s) == std::future_status::ready) {
			auto response = future.get();
			if (response->success) {
				RCLCPP_INFO(this->get_logger(), "Successfully synchronized device [%s]", response->camera_name.c_str());
			}
			else {
				RCLCPP_ERROR(this->get_logger(), "Failed to synchronize device available on service [%s]", serviceName.c_str());
			}
		}
		else {
			RCLCPP_ERROR(this->get_logger(), "Timeout synchronizing device on service [%s]", serviceName.c_str());
		}
	}
}

void CaptureNode::synchronizationThread() {
	std::string serviceName;
	auto *liveCapture = dynamic_cast<dv::io::camera::SyncCameraInputBase *>(mReader.get());

	if (!liveCapture) {
		// Cameras without synchronization support can just be considered done immediately.
		if (dynamic_cast<dv::io::camera::CameraInputBase *>(mReader.get())) {
			mSynchronized = true;
		}
		return;
	}

	if (liveCapture->isMaster()) {
		// Wait for all cameras to show up
		const auto syncServiceList = discoverSyncDevices();
		runDiscovery(serviceName);
		sendSyncCalls(syncServiceList);
		mSynchronized = true;
	}
	else {
		mSyncServerService = this->create_service<dv_ros2_capture::srv::SynchronizeCamera>(
			fmt::format("{}/sync", liveCapture->getCameraName()),
			std::bind(&CaptureNode::synchronizeCamera, this, std::placeholders::_1, std::placeholders::_2));

		serviceName = fmt::format("{}/sync", liveCapture->getCameraName());
		runDiscovery(serviceName);

		// Wait for synchronization only if explicitly requested
		if (!mParams.waitForSync) {
			mSynchronized = true;
		}

		size_t iterations = 0;
		while (mSpinThread.load(std::memory_order_relaxed)) {
			std::this_thread::sleep_for(1ms);

			// Do not print warnings if it's synchronized
			if (mSynchronized.load(std::memory_order_relaxed)) {
				continue;
			}

			if (iterations > 2000) {
				RCLCPP_WARN(this->get_logger(), "[%s] Waiting for synchronization service call...", liveCapture->getCameraName().c_str());
				iterations = 0;
			}
			iterations++;
		}
	}
}

dv_ros2_msgs::ImuMessage CaptureNode::transformImuFrame(dv_ros2_msgs::ImuMessage &&imu) {
	if (mParams.unbiasedImuData) {
		imu.linear_acceleration.x -= mAccBiases.x();
		imu.linear_acceleration.y -= mAccBiases.y();
		imu.linear_acceleration.z -= mAccBiases.z();

		imu.angular_velocity.x -= mGyroBiases.x();
		imu.angular_velocity.y -= mGyroBiases.y();
		imu.angular_velocity.z -= mGyroBiases.z();
	}
	if (mParams.transformImuToCameraFrame) {
		const Eigen::Vector3<double> resW
			= mImuToCamTransform.rotatePoint<Eigen::Vector3<double>>(imu.angular_velocity);
		imu.angular_velocity.x = resW.x();
		imu.angular_velocity.y = resW.y();
		imu.angular_velocity.z = resW.z();

		const Eigen::Vector3<double> resV
			= mImuToCamTransform.rotatePoint<Eigen::Vector3<double>>(imu.linear_acceleration);
		imu.linear_acceleration.x = resV.x();
		imu.linear_acceleration.y = resV.y();
		imu.linear_acceleration.z = resV.z();
	}
	return imu;
}

void CaptureNode::configureCameraHardware() {
	auto *davis = dynamic_cast<dv::io::camera::DAVIS *>(mReader.get());
	if (davis != nullptr) {
		davis->setColorMode(static_cast<dv::io::camera::parser::DAVIS::ColorMode>(mParams.colorMode));

		if (mParams.readoutMode == 1) {
			davis->setEventsRunning(true);
			davis->setFramesRunning(false);
		}
		else if (mParams.readoutMode == 2) {
			davis->setEventsRunning(false);
			davis->setFramesRunning(true);
		}
		else {
			davis->setEventsRunning(true);
			davis->setFramesRunning(true);
		}

		if (mParams.autoExposure) {
			davis->setAutoExposure(true);
		}
		else {
			davis->setAutoExposure(false);
			davis->setExposureDuration(std::chrono::microseconds{mParams.exposureUs});
		}

		if (davis->isTriggerStreamAvailable()) {
			davis->setDetectorRisingEdges(true);
			davis->setDetectorFallingEdges(false);
			davis->setDetectorRunning(mParams.triggers);
		}

		davis->setTimeInterval(std::chrono::microseconds{mParams.timeIncrement});
		RCLCPP_INFO(this->get_logger(), "DAVIS camera configured.");
	}

	auto *dvxplorer = dynamic_cast<dv::io::camera::DVXplorer *>(mReader.get());
	if (dvxplorer != nullptr) {
		dvxplorer->setGlobalHold(mParams.globalHold);
		dvxplorer->setContrastThresholdOn(static_cast<uint8_t>(mParams.contrastThresholdOn));
		dvxplorer->setContrastThresholdOff(static_cast<uint8_t>(mParams.contrastThresholdOff));

		if (dvxplorer->isTriggerStreamAvailable()) {
			dvxplorer->setDetectorRisingEdges(true);
			dvxplorer->setDetectorFallingEdges(false);
			dvxplorer->setDetectorRunning(mParams.triggers);
		}

		dvxplorer->setImuAccelDataRate(parseAccelDataRate(mParams.imuAccelDataRate));
		dvxplorer->setImuGyroDataRate(parseGyroDataRate(mParams.imuGyroDataRate));
		dvxplorer->setImuAccelFilter(parseAccelFilter(mParams.imuAccelFilter));
		dvxplorer->setImuGyroFilter(parseGyroFilter(mParams.imuGyroFilter));

		dvxplorer->setTimeInterval(std::chrono::microseconds{mParams.timeIncrement});
		RCLCPP_INFO(this->get_logger(), "DVXplorer camera configured.");
	}

	auto *dvxplorerm = dynamic_cast<dv::io::camera::DVXplorerM *>(mReader.get());
	if (dvxplorerm != nullptr) {
		dvxplorerm->setGlobalHold(mParams.globalHold);
		dvxplorerm->setContrastThresholdOn(static_cast<uint8_t>(mParams.contrastThresholdOn));
		dvxplorerm->setContrastThresholdOff(static_cast<uint8_t>(mParams.contrastThresholdOff));

		dvxplorerm->setImuAccelDataRate(parseAccelDataRate(mParams.imuAccelDataRate));
		dvxplorerm->setImuGyroDataRate(parseGyroDataRate(mParams.imuGyroDataRate));
		dvxplorerm->setImuAccelFilter(parseAccelFilter(mParams.imuAccelFilter));
		dvxplorerm->setImuGyroFilter(parseGyroFilter(mParams.imuGyroFilter));

		dvxplorerm->setTimeInterval(std::chrono::microseconds{mParams.timeIncrement});
		RCLCPP_INFO(this->get_logger(), "DVXplorerM camera configured.");
	}

	// Register the parameter change callback for runtime tuning
	mParamCallbackHandle = this->add_on_set_parameters_callback(
		std::bind(&CaptureNode::onParameterChange, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult CaptureNode::onParameterChange(
	const std::vector<rclcpp::Parameter> &parameters) {
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;

	for (const auto &param : parameters) {
		const auto &name = param.get_name();

		if (name == "noiseFiltering" || name == "noiseBackgroundActivityTime") {
			bool filtering   = this->get_parameter("noiseFiltering").as_bool();
			int64_t baTime = this->get_parameter("noiseBackgroundActivityTime").as_int();
			if (name == "noiseFiltering") {
				filtering = param.as_bool();
			}
			if (name == "noiseBackgroundActivityTime") {
				baTime = param.as_int();
			}
			mParams.noiseFiltering = filtering;
			mParams.noiseBATime    = baTime;
			updateNoiseFilter(filtering, baTime);
			continue;
		}

			if (name == "undistortEvents") {
				mParams.undistortEvents = param.as_bool();
				updateEventUndistortionParams();
				continue;
			}

		auto *davis = dynamic_cast<dv::io::camera::DAVIS *>(mReader.get());
		if (davis != nullptr) {
			if (name == "colorMode") {
				int mode = static_cast<int>(param.as_int());
				davis->setColorMode(static_cast<dv::io::camera::parser::DAVIS::ColorMode>(mode));
				mParams.colorMode = mode;
			}
			else if (name == "readoutMode") {
				int mode = static_cast<int>(param.as_int());
				if (mode == 1) {
					davis->setEventsRunning(true);
					davis->setFramesRunning(false);
				}
				else if (mode == 2) {
					davis->setEventsRunning(false);
					davis->setFramesRunning(true);
				}
				else {
					davis->setEventsRunning(true);
					davis->setFramesRunning(true);
				}
				mParams.readoutMode = mode;
			}
			else if (name == "autoExposure") {
				bool autoExp = param.as_bool();
				if (autoExp) {
					davis->setAutoExposure(true);
				}
				else {
					davis->setAutoExposure(false);
					davis->setExposureDuration(std::chrono::microseconds{mParams.exposureUs});
				}
				mParams.autoExposure = autoExp;
			}
			else if (name == "exposureUs") {
				int exposure = static_cast<int>(param.as_int());
				if (!mParams.autoExposure) {
					davis->setExposureDuration(std::chrono::microseconds{exposure});
				}
				mParams.exposureUs = exposure;
			}
		}

		auto *dvxplorer = dynamic_cast<dv::io::camera::DVXplorer *>(mReader.get());
		if (dvxplorer != nullptr) {
			if (name == "globalHold") {
				dvxplorer->setGlobalHold(param.as_bool());
				mParams.globalHold = param.as_bool();
			}
			else if (name == "contrastThresholdOn") {
				dvxplorer->setContrastThresholdOn(static_cast<uint8_t>(param.as_int()));
				mParams.contrastThresholdOn = static_cast<int>(param.as_int());
			}
			else if (name == "contrastThresholdOff") {
				dvxplorer->setContrastThresholdOff(static_cast<uint8_t>(param.as_int()));
				mParams.contrastThresholdOff = static_cast<int>(param.as_int());
			}
		}

		auto *dvxplorerm = dynamic_cast<dv::io::camera::DVXplorerM *>(mReader.get());
		if (dvxplorerm != nullptr) {
			if (name == "globalHold") {
				dvxplorerm->setGlobalHold(param.as_bool());
				mParams.globalHold = param.as_bool();
			}
			else if (name == "contrastThresholdOn") {
				dvxplorerm->setContrastThresholdOn(static_cast<uint8_t>(param.as_int()));
				mParams.contrastThresholdOn = static_cast<int>(param.as_int());
			}
			else if (name == "contrastThresholdOff") {
				dvxplorerm->setContrastThresholdOff(static_cast<uint8_t>(param.as_int()));
				mParams.contrastThresholdOff = static_cast<int>(param.as_int());
			}
		}
	}

	return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(dv_capture_node::CaptureNode)
