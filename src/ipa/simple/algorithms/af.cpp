/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Contrast detect autofocus for the simple IPA
 *
 * The soft ISP statistics carry a sharpness figure (sum of the absolute
 * differences between neighbouring green samples). Normalised by the green
 * sum it does not depend on exposure, so the lens is swept coarsely over its
 * whole range, then finely around the sharpest coarse position, and parked
 * at the best one. A large, lasting drop in sharpness or brightness (the
 * scene changed) starts a new sweep.
 */

#include "af.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftAf)

int Af::init([[maybe_unused]] IPAContext &context, const ValueNode &tuningData)
{
	coarseSteps_ = tuningData["coarseSteps"].get<unsigned int>().value_or(coarseSteps_);
	fineSteps_ = tuningData["fineSteps"].get<unsigned int>().value_or(fineSteps_);
	settleFrames_ = tuningData["settleFrames"].get<unsigned int>().value_or(settleFrames_);
	rescanDrop_ = tuningData["rescanDrop"].get<double>().value_or(rescanDrop_);

	return 0;
}

int Af::configure(IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	state_ = State::Warmup;
	warmup_ = warmupFrames_;
	context.activeState.af.apply = false;

	if (context.configuration.lens.available)
		LOG(IPASoftAf, Debug) << "Lens range "
				      << context.configuration.lens.min << "-"
				      << context.configuration.lens.max;
	return 0;
}

void Af::moveTo(IPAContext &context, int32_t position)
{
	context.activeState.af.position = position;
	context.activeState.af.apply = true;
	settle_ = settleFrames_;
}

void Af::startSweep(IPAContext &context, int32_t from, int32_t to,
		    unsigned int steps, State state)
{
	positions_.clear();
	for (unsigned int i = 0; i < steps; i++)
		positions_.push_back(from + (to - from) * static_cast<int64_t>(i) /
					    std::max(1u, steps - 1));
	index_ = 0;
	bestScore_ = -1;
	state_ = state;
	moveTo(context, positions_[0]);
}

void Af::process(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	const auto &lens = context.configuration.lens;

	if (!lens.available || !stats->valid)
		return;

	if (settle_) {
		settle_--;
		return;
	}

	double brightness = static_cast<double>(stats->sum_.g()) + 1.0;
	double score = static_cast<double>(stats->sharpness) / brightness;

	switch (state_) {
	case State::Warmup:
		/* Let the exposure settle first */
		if (warmup_) {
			warmup_--;
			return;
		}
		startSweep(context, lens.min, lens.max, coarseSteps_, State::Coarse);
		return;

	case State::Coarse:
	case State::Fine:
		if (score > bestScore_) {
			bestScore_ = score;
			bestPosition_ = positions_[index_];
		}

		if (++index_ < positions_.size()) {
			moveTo(context, positions_[index_]);
			return;
		}

		if (state_ == State::Coarse) {
			int32_t span = (lens.max - lens.min) /
				       std::max(1u, coarseSteps_ - 1);
			startSweep(context,
				   std::max(lens.min, bestPosition_ - span),
				   std::min(lens.max, bestPosition_ + span),
				   fineSteps_, State::Fine);
			return;
		}

		LOG(IPASoftAf, Debug) << "Focused at " << bestPosition_
				      << ", score " << bestScore_;
		moveTo(context, bestPosition_);
		state_ = State::Converged;
		refScore_ = 0;
		changed_ = 0;
		return;

	case State::Converged:
		if (refScore_ == 0) {
			refScore_ = score;
			refBrightness_ = brightness;
			return;
		}

		if (score < refScore_ * rescanDrop_ ||
		    brightness < refBrightness_ * 0.5 ||
		    brightness > refBrightness_ * 2.0)
			changed_++;
		else
			changed_ = 0;

		if (changed_ >= rescanFrames_) {
			LOG(IPASoftAf, Debug) << "Scene changed, refocusing";
			startSweep(context, lens.min, lens.max, coarseSteps_,
				   State::Coarse);
		}
		return;
	}
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
