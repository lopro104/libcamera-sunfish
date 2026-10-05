/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Contrast detect autofocus for the simple IPA
 */

#pragma once

#include <vector>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Af : public Algorithm
{
public:
	Af() = default;
	~Af() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	enum class State {
		Warmup,
		Coarse,
		Fine,
		Converged,
	};

	void startSweep(IPAContext &context, int32_t from, int32_t to,
			unsigned int steps, State state);
	void moveTo(IPAContext &context, int32_t position);

	State state_ = State::Warmup;
	std::vector<int32_t> positions_;
	unsigned int index_ = 0;
	unsigned int settle_ = 0;
	unsigned int warmup_ = 0;
	unsigned int changed_ = 0;

	int32_t bestPosition_ = 0;
	double bestScore_ = 0;
	double refScore_ = 0;
	double refBrightness_ = 0;

	/* Tuning */
	unsigned int coarseSteps_ = 8;
	unsigned int fineSteps_ = 6;
	unsigned int settleFrames_ = 1;
	unsigned int warmupFrames_ = 6;
	double rescanDrop_ = 0.6;
	unsigned int rescanFrames_ = 6;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
