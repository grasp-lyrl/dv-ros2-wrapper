#include <dv_ros2_messaging/messaging.hpp>

#include <gtest/gtest.h>

TEST(DvRosMessagingTests, TimestampConversionTests) {
	int64_t someTime               = 1644507922'623652;
	builtin_interfaces::msg::Time rosTime = dv_ros2_msgs::toRosTime(someTime);
	EXPECT_EQ(rosTime.sec, 1644507922);
	EXPECT_EQ(rosTime.nanosec, 623652000u);
	int64_t dvTime = dv_ros2_msgs::toDvTime(rosTime);
	EXPECT_EQ(dvTime, someTime);
}

TEST(DvRosMessagingTests, GrayImageConversionTests) {
	cv::Mat gray     = cv::Mat::ones(cv::Size(100, 100), CV_8UC1);
	int64_t someTime = 1644507922'623652;
	dv::Frame frame  = dv::Frame(someTime, gray);
	auto rosMsg      = dv_ros2_msgs::frameToRosImageMessage(frame);
	EXPECT_EQ(rosMsg.header.stamp.sec, 1644507922);
	EXPECT_EQ(rosMsg.header.stamp.nanosec, 623652000u);
	EXPECT_EQ(rosMsg.width, 100u);
	EXPECT_EQ(rosMsg.height, 100u);
	EXPECT_EQ(rosMsg.encoding, sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(rosMsg.data.front(), 1);
}

TEST(DvRosMessagingTests, ColorImageConversionTests) {
	cv::Mat color    = cv::Mat::ones(cv::Size(100, 100), CV_8UC3);
	int64_t someTime = 1644507922'623652;
	dv::Frame frame  = dv::Frame(someTime, color);
	auto rosMsg      = dv_ros2_msgs::frameToRosImageMessage(frame);
	EXPECT_EQ(rosMsg.header.stamp.sec, 1644507922);
	EXPECT_EQ(rosMsg.header.stamp.nanosec, 623652000u);
	EXPECT_EQ(rosMsg.width, 100u);
	EXPECT_EQ(rosMsg.height, 100u);
	EXPECT_EQ(rosMsg.encoding, sensor_msgs::image_encodings::BGR8);
	EXPECT_EQ(rosMsg.data.front(), 1);
}

TEST(DvRosMessagingTests, InvalidImageConversionTests) {
	cv::Mat floatingImage = cv::Mat::ones(cv::Size(100, 100), CV_32FC3);
	int64_t someTime      = 1644507922'623652;
	dv::Frame frame       = dv::Frame(someTime, floatingImage);
	EXPECT_ANY_THROW(dv_ros2_msgs::frameToRosImageMessage(frame));
}

TEST(DvRosMessagingTests, EventStoreConversions) {
	dv::EventStore events;
	int64_t someTime = 1644507922'623652;
	events.emplace_back(someTime, 0, 0, true);
	events.emplace_back(someTime + 1000, 0, 0, true);
	events.emplace_back(someTime + 2000, 0, 0, true);
	events.emplace_back(someTime + 3000, 0, 0, true);

	const auto rosEvents = dv_ros2_msgs::toRosEventsMessage(events, cv::Size(100, 100));

	dv::EventStore eventsBack = dv_ros2_msgs::toEventStore(rosEvents);

	EXPECT_EQ(rosEvents.events.size(), events.size());
	EXPECT_EQ(eventsBack.size(), events.size());
	EXPECT_EQ(eventsBack.getLowestTime(), events.getLowestTime());
	EXPECT_EQ(eventsBack.getHighestTime(), events.getHighestTime());
	for (size_t i = 0; i < eventsBack.size(); i++) {
		EXPECT_EQ(eventsBack.at(i).timestamp(), events.at(i).timestamp());
		EXPECT_EQ(eventsBack.at(i).x(), events.at(i).x());
		EXPECT_EQ(eventsBack.at(i).y(), events.at(i).y());
		EXPECT_EQ(eventsBack.at(i).polarity(), events.at(i).polarity());
	}
}

TEST(DvRosMessagingTests, EventStoreRemappedConversions) {
	// Straddles a second boundary.
	dv::EventStore events;
	const int64_t someTime = 1644507922'999000;
	for (int i = 0; i < 8; i++) {
		events.emplace_back(someTime + i * 400, static_cast<int16_t>(10 + i), static_cast<int16_t>(20 + i), i % 2 == 0);
	}

	// Drops odd x, shifts the rest.
	const auto remap = [](const int x, const int y, uint16_t &outX, uint16_t &outY) {
		if (x % 2 != 0) {
			return false;
		}
		outX = static_cast<uint16_t>(x + 100);
		outY = static_cast<uint16_t>(y + 200);
		return true;
	};
	const auto remapped = dv_ros2_msgs::toRosEventsMessage(events, cv::Size(640, 480), remap);

	ASSERT_EQ(remapped.events.size(), 4);
	EXPECT_EQ(remapped.width, 640);
	EXPECT_EQ(remapped.height, 480);
	for (size_t i = 0; i < remapped.events.size(); i++) {
		const auto &source = events.at(2 * i);
		EXPECT_EQ(remapped.events[i].x, source.x() + 100);
		EXPECT_EQ(remapped.events[i].y, source.y() + 200);
		EXPECT_EQ(remapped.events[i].polarity, source.polarity());
		EXPECT_EQ(dv_ros2_msgs::toDvTime(remapped.events[i].ts), source.timestamp());
	}
	EXPECT_EQ(dv_ros2_msgs::toDvTime(remapped.header.stamp), events.at(6).timestamp());

	const auto plain    = dv_ros2_msgs::toRosEventsMessage(events, cv::Size(640, 480));
	const auto identity = dv_ros2_msgs::toRosEventsMessage(
		events, cv::Size(640, 480), [](const int x, const int y, uint16_t &outX, uint16_t &outY) {
			outX = static_cast<uint16_t>(x);
			outY = static_cast<uint16_t>(y);
			return true;
		});
	EXPECT_EQ(identity, plain);
	EXPECT_EQ(dv_ros2_msgs::toDvTime(plain.header.stamp), events.getHighestTime());

	const auto none = dv_ros2_msgs::toRosEventsMessage(
		events, cv::Size(640, 480), [](int, int, uint16_t &, uint16_t &) {
			return false;
		});
	EXPECT_TRUE(none.events.empty());
	EXPECT_EQ(dv_ros2_msgs::toDvTime(none.header.stamp), 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
