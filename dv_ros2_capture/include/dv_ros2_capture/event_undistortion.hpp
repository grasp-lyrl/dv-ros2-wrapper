#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace dv_capture_node {

/// Intrinsics from an OpenCV calibration file.
struct OpenCvCalibration {
	cv::Mat cameraMatrix;
	cv::Mat distortion;
	/// Empty if the file does not record one.
	cv::Size resolution;
	bool fisheye = false;
};

/**
 * Read an OpenCV FileStorage calibration. The camera node is found by content, since it is
 * named for a serial that need not match this camera.
 * @throws std::invalid_argument If the file is missing, unreadable, or lacks intrinsics.
 */
[[nodiscard]] inline OpenCvCalibration readOpenCvCalibration(const std::filesystem::path &path) {
	if (!std::filesystem::exists(path)) {
		throw std::invalid_argument("OpenCV calibration file does not exist: " + path.string());
	}

	cv::FileStorage store(path.string(), cv::FileStorage::READ);
	if (!store.isOpened()) {
		throw std::invalid_argument("Cannot read OpenCV calibration: " + path.string());
	}

	cv::FileNode camera;
	for (const auto &node : store.root()) {
		if (node.isMap() && !node["camera_matrix"].empty()) {
			camera = node;
			break;
		}
	}
	if (camera.empty()) {
		throw std::invalid_argument("No node with a camera_matrix in OpenCV calibration: " + path.string());
	}

	OpenCvCalibration calibration;
	camera["camera_matrix"] >> calibration.cameraMatrix;
	camera["distortion_coefficients"] >> calibration.distortion;
	if (calibration.cameraMatrix.total() != 9 || calibration.distortion.empty()) {
		throw std::invalid_argument(
			"OpenCV calibration lacks a 3x3 camera_matrix or distortion_coefficients: " + path.string());
	}
	calibration.cameraMatrix.convertTo(calibration.cameraMatrix, CV_64F);
	calibration.distortion.convertTo(calibration.distortion, CV_64F);

	int width  = 0;
	int height = 0;
	camera["image_width"] >> width;
	camera["image_height"] >> height;
	if (width > 0 && height > 0) {
		calibration.resolution = cv::Size(width, height);
	}

	const auto fisheyeNode = store["use_fisheye_model"];
	calibration.fisheye    = !fisheyeNode.empty() && static_cast<int>(fisheyeNode) != 0;
	return calibration;
}

/// Precomputed rectified destination for every sensor pixel, so undistorting an event is one
/// table read.
class PixelRemap {
public:
	PixelRemap() = default;

	/// Build from a CV_32FC2 map of rectified coordinates, one per sensor pixel.
	explicit PixelRemap(const cv::Mat &map) :
		mWidth(static_cast<unsigned>(map.cols)),
		mHeight(static_cast<unsigned>(map.rows)),
		mTable(map.total(), kDropped) {
		CV_Assert(map.type() == CV_32FC2);
		for (int y = 0; y < map.rows; ++y) {
			const auto *row = map.ptr<cv::Vec2f>(y);
			for (int x = 0; x < map.cols; ++x) {
				const float rx = std::round(row[x][0]);
				const float ry = std::round(row[x][1]);
				// Checked before narrowing; the comparisons are false for NaN, dropping it too.
				if (rx >= 0.0f && rx < static_cast<float>(map.cols) && ry >= 0.0f
					&& ry < static_cast<float>(map.rows)) {
					mTable[static_cast<size_t>(y) * mWidth + static_cast<size_t>(x)]
						= (static_cast<uint32_t>(ry) << 16) | static_cast<uint32_t>(rx);
				}
			}
		}
	}

	[[nodiscard]] bool empty() const {
		return mTable.empty();
	}

	[[nodiscard]] int width() const {
		return static_cast<int>(mWidth);
	}

	[[nodiscard]] int height() const {
		return static_cast<int>(mHeight);
	}

	/// Rectified position of (x, y); false if the event should be dropped.
	[[nodiscard]] bool operator()(const int x, const int y, uint16_t &ux, uint16_t &uy) const {
		if (static_cast<unsigned>(x) >= mWidth || static_cast<unsigned>(y) >= mHeight) {
			return false;
		}
		const uint32_t packed = mTable[static_cast<size_t>(y) * mWidth + static_cast<size_t>(x)];
		if (packed == kDropped) {
			return false;
		}
		ux = static_cast<uint16_t>(packed & 0xFFFFu);
		uy = static_cast<uint16_t>(packed >> 16);
		return true;
	}

private:
	static constexpr uint32_t kDropped = 0xFFFFFFFFu;

	unsigned mWidth  = 0;
	unsigned mHeight = 0;
	/// (y << 16) | x per sensor pixel, or kDropped.
	std::vector<uint32_t> mTable;
};

struct UndistortionMap {
	PixelRemap remap;
	/// Intrinsics of the rectified frame.
	cv::Mat newCameraMatrix;
};

/// Build the rectification for a camera, keeping the sensor size, cropped to valid pixels.
[[nodiscard]] inline UndistortionMap buildUndistortionMap(
	const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Size &size, const bool fisheye) {
	std::vector<cv::Point2f> pixels;
	pixels.reserve(static_cast<size_t>(size.width) * static_cast<size_t>(size.height));
	for (int y = 0; y < size.height; ++y) {
		for (int x = 0; x < size.width; ++x) {
			pixels.emplace_back(static_cast<float>(x), static_cast<float>(y));
		}
	}

	const cv::Mat pixelsMat(pixels);
	cv::Mat undistortedPixels;
	UndistortionMap result;
	const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);

	if (fisheye) {
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
			cameraMatrix, distortion, size, identity, result.newCameraMatrix, 0.0);
		cv::fisheye::undistortPoints(
			pixelsMat, undistortedPixels, cameraMatrix, distortion, identity, result.newCameraMatrix);
	}
	else {
		result.newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distortion, size, 0.0);
		cv::undistortPoints(
			pixelsMat, undistortedPixels, cameraMatrix, distortion, cv::noArray(), result.newCameraMatrix);
	}

	result.remap = PixelRemap(undistortedPixels.reshape(2, size.height));
	return result;
}

} // namespace dv_capture_node
