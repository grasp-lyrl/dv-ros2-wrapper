#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace dv_monodepth_node {

/**
 * Single-producer/single-consumer triple buffer that keeps only the newest value, so the
 * reader never gets a stale one and the writer never blocks.
 */
template<typename T>
class LatestValue {
public:
	/// Publish a value. Wait-free, single writer.
	void write(T &&value) {
		mSlots[mWriteIndex] = std::move(value);
		const uint8_t previous = mReady.exchange(mWriteIndex | kFreshBit, std::memory_order_acq_rel);
		mWriteIndex            = previous & kIndexMask;
	}

	/// Take the newest value if one arrived since the last read. Wait-free, single reader.
	[[nodiscard]] bool read(T &out) {
		if ((mReady.load(std::memory_order_acquire) & kFreshBit) == 0) {
			return false;
		}
		const uint8_t previous = mReady.exchange(mReadIndex, std::memory_order_acq_rel);
		mReadIndex             = previous & kIndexMask;
		out                    = std::move(mSlots[mReadIndex]);
		return true;
	}

private:
	static constexpr uint8_t kIndexMask = 0x03;
	static constexpr uint8_t kFreshBit  = 0x04;

	std::array<T, 3> mSlots;
	uint8_t mWriteIndex           = 0;
	std::atomic<uint8_t> mReady   = 1; // slot 1, not fresh
	uint8_t mReadIndex            = 2;
};

} // namespace dv_monodepth_node
