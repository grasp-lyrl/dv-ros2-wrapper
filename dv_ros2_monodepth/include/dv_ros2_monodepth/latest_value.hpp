#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace dv_monodepth_node {

/**
 * A single-producer/single-consumer slot holding only the most recent value.
 *
 * Events arrive far faster than the network can consume them, so a queue would either
 * grow without bound or hand the model a window that is already stale. This is a triple
 * buffer instead: the writer never blocks and never drops *its* write, the reader always
 * gets the newest complete value, and anything the reader did not get to is overwritten.
 *
 * Three slots is the minimum that lets both sides hold one exclusively while a third
 * carries the handoff, so neither ever touches the other's slot.
 */
template<typename T>
class LatestValue {
public:
	/// Publish a value. Wait-free. Only one thread may call this.
	void write(T &&value) {
		mSlots[mWriteIndex] = std::move(value);
		// Hand our slot over and take whatever the reader left behind.
		const uint8_t previous = mReady.exchange(mWriteIndex | kFreshBit, std::memory_order_acq_rel);
		mWriteIndex            = previous & kIndexMask;
	}

	/**
	 * Move the newest value out, if one arrived since the last call. Wait-free.
	 * Only one thread may call this.
	 * @return False if no new value is available; `out` is then untouched.
	 */
	[[nodiscard]] bool read(T &out) {
		if ((mReady.load(std::memory_order_acquire) & kFreshBit) == 0) {
			return false;
		}
		// Clearing the fresh bit is what makes a second read return false.
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
