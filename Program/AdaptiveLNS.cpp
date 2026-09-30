#include "AdaptiveLNS.h"
#include "SolutionValidator.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_set>

namespace
{
struct VisitBlock
{
	int route = -1;
	int origin = -1;
	std::vector<int> clients;
	double removalSaving = 0.0;
};

void rebuildChromosome(Individual& indiv)
{
	indiv.chromT.clear();
	for (const auto& route : indiv.chromR)
		indiv.chromT.insert(indiv.chromT.end(), route.begin(), route.end());
}

std::vector<VisitBlock> makeBlocks(const Individual& indiv, const Params& params)
{
	std::vector<VisitBlock> blocks;
	for (size_t r = 0; r < indiv.chromR.size(); ++r)
	{
		const auto& route = indiv.chromR[r];
		for (size_t i = 0; i < route.size();)
		{
			const size_t begin = i;
			const int origin = params.Corres_origin[route[i]];
			while (i < route.size() && params.Corres_origin[route[i]] == origin) ++i;
			VisitBlock block;
			block.route = static_cast<int>(r);
			block.origin = origin;
			block.clients.assign(route.begin() + begin, route.begin() + i);
			const int before = begin == 0 ? 0 : route[begin - 1];
			const int after = i == route.size() ? 0 : route[i];
			block.removalSaving = params.timeCost[before][route[begin]] +
				params.timeCost[route[i - 1]][after] - params.timeCost[before][after];
			blocks.push_back(block);
		}
	}
	return blocks;
}
}

AdaptiveLNS::AdaptiveLNS(Params& p) : params(p)
{
	removalWeight.fill(10.0);
	repairWeight.fill(10.0);
}

int AdaptiveLNS::roulette(const double* weights, int count)
{
	double total = std::accumulate(weights, weights + count, 0.0);
	std::uniform_real_distribution<double> pick(0.0, total);
	double value = pick(params.ran);
	for (int i = 0; i < count; ++i) if ((value -= weights[i]) <= 0.0) return i;
	return count - 1;
}

void AdaptiveLNS::updateWeight(double& weight, double reward)
{
	const double theta = params.ap.lnsReactionFactor;
	weight = (1.0 - theta) * weight + theta * reward;
}

bool AdaptiveLNS::ruinAndRepair(const Individual& source, Individual& candidate, int removalOp, int repairOp)
{
	if (params.deadlineReached()) return false;
	candidate = source;
	auto blocks = makeBlocks(source, params);
	if (blocks.size() < 2) return false;
	std::uniform_real_distribution<double> ratio(params.ap.lnsDestroyMin, params.ap.lnsDestroyMax);
	const int removeCount = std::max(1, std::min<int>(blocks.size() - 1,
		static_cast<int>(std::ceil(ratio(params.ran) * blocks.size()))));

	if (removalOp == 0) std::shuffle(blocks.begin(), blocks.end(), params.ran); // RaR
	else if (removalOp == 1) // BWR
		std::sort(blocks.begin(), blocks.end(), [](const VisitBlock& a, const VisitBlock& b) { return a.removalSaving > b.removalSaving; });
	else if (removalOp == 2) // RGR: distance-related to one random request
	{
		std::uniform_int_distribution<int> anchorPick(0, static_cast<int>(blocks.size()) - 1);
		const int anchor = blocks[anchorPick(params.ran)].clients.front();
		std::sort(blocks.begin(), blocks.end(), [&](const VisitBlock& a, const VisitBlock& b) {
			return params.timeCost[anchor][a.clients.front()] < params.timeCost[anchor][b.clients.front()];
		});
	}
	else if (removalOp == 3) // TCR
	{
		std::shuffle(blocks.begin(), blocks.end(), params.ran);
	}
	else // BiR: alternate high and low savings
	{
		std::sort(blocks.begin(), blocks.end(), [](const VisitBlock& a, const VisitBlock& b) { return a.removalSaving > b.removalSaving; });
		std::vector<VisitBlock> binary;
		for (size_t lo = 0, hi = blocks.size() - 1; lo <= hi; ++lo) {
			binary.push_back(blocks[lo]);
			if (lo < hi) binary.push_back(blocks[hi--]);
		}
		blocks.swap(binary);
	}
	const int actualRemove = removalOp == 3 ? std::min(2, removeCount) : removeCount;
	std::vector<VisitBlock> removed(blocks.begin(), blocks.begin() + actualRemove);

	for (const VisitBlock& block : removed)
	{
		if (params.deadlineReached()) return false;
		auto& route = candidate.chromR[block.route];
		route.erase(std::remove_if(route.begin(), route.end(), [&](int client) {
			return params.Corres_origin[client] == block.origin;
		}), route.end());
	}

	if (repairOp >= 2) // MR2I / MR3I: difficult (high-regret proxy) blocks first
		std::sort(removed.begin(), removed.end(), [](const VisitBlock& a, const VisitBlock& b) { return a.removalSaving > b.removalSaving; });

	for (const VisitBlock& block : removed)
	{
		struct Choice { double cost; int route; int pos; Individual solution; };
		std::vector<Choice> choices;
		for (int r = 0; r < params.nbVehicles; ++r)
		{
			if (params.deadlineReached()) return false;
			const auto baseRoute = candidate.chromR[r];
			for (int pos = 0; pos <= static_cast<int>(baseRoute.size()); ++pos)
			{
				if (params.deadlineReached()) return false;
				// Insert only at visit-block boundaries.
				if (pos > 0 && pos < static_cast<int>(baseRoute.size()) &&
					params.Corres_origin[baseRoute[pos - 1]] == params.Corres_origin[baseRoute[pos]]) continue;
				Individual trial(candidate);
				trial.chromR[r].insert(trial.chromR[r].begin() + pos, block.clients.begin(), block.clients.end());
				rebuildChromosome(trial);
				trial.evaluateCompleteCost(params);
				const ValidationResult valid = validateSolutionFromScratch(trial, params);
				if (valid.capacityFeasible && valid.timeWindowFeasible && valid.commodityFeasible && valid.visitBlocksValid)
					choices.push_back({ trial.eval.distance, r, pos, std::move(trial) });
			}
		}
		if (choices.empty()) return false;
		std::sort(choices.begin(), choices.end(), [](const Choice& a, const Choice& b) { return a.cost < b.cost; });
		int rank = 0;
		if (repairOp == 1 && choices.size() > 1) // MGI 0.72/0.20/0.08
		{
			std::uniform_real_distribution<double> draw(0.0, 1.0);
			const double p = draw(params.ran);
			rank = p < 0.72 ? 0 : (p < 0.92 ? 1 : 2);
			rank = std::min(rank, static_cast<int>(choices.size()) - 1);
		}
		candidate = std::move(choices[rank].solution); // BGI and regret variants use best insertion
	}
	rebuildChromosome(candidate);
	candidate.evaluateCompleteCost(params);
	return validateSolutionFromScratch(candidate, params).feasible();
}

bool AdaptiveLNS::run(const Individual& seed, Individual& result, double globalBest)
{
	Individual current(seed);
	Individual best(seed);
	double temperature = std::max(1.e-6, params.ap.lnsInitialTemperature);
	bool improved = false;
	for (int iteration = 0; iteration < params.ap.lnsIterations; ++iteration)
	{
		if (params.deadlineReached()) break;
		const int removal = roulette(removalWeight.data(), static_cast<int>(removalWeight.size()));
		const int repair = roulette(repairWeight.data(), static_cast<int>(repairWeight.size()));
		Individual candidate(params);
		if (!ruinAndRepair(current, candidate, removal, repair)) {
			updateWeight(removalWeight[removal], 0.0); updateWeight(repairWeight[repair], 0.0); continue;
		}
		const double delta = candidate.eval.distance - current.eval.distance;
		std::uniform_real_distribution<double> draw(0.0, 1.0);
		const bool accepted = delta < -MY_EPSILON || draw(params.ran) < std::exp(-delta / temperature);
		double reward = 0.0;
		if (candidate.eval.distance < globalBest - MY_EPSILON) reward = 50.0;
		else if (candidate.eval.distance < current.eval.distance - MY_EPSILON) reward = 20.0;
		else if (accepted) reward = 10.0;
		updateWeight(removalWeight[removal], reward);
		updateWeight(repairWeight[repair], reward);
		if (accepted) current = candidate;
		if (candidate.eval.distance < best.eval.distance - MY_EPSILON) { best = candidate; improved = true; }
		temperature *= 0.95;
	}
	result = best;
	return improved;
}
