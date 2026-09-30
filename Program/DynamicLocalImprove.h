#ifndef DYNAMIC_LOCAL_IMPROVE_H
#define DYNAMIC_LOCAL_IMPROVE_H

#include "Individual.h"
#include <optional>
#include <string>
#include <vector>

struct DynamicLocalImproveConfig
{
	int nearbyRoutes = 3;
	int maxPasses = 20;
	bool enforceGlobalDeadline = false;
};

struct DynamicLocalImproveResult
{
	std::optional<Individual> improved;
	std::vector<int> affectedRoutes;
	std::vector<int> searchRoutes;
	int evaluatedMoves = 0;
	int acceptedMoves = 0;
	int relocateMoves = 0;
	int swapMoves = 0;
	int twoOptMoves = 0;
	long long generatedRelocateMoves = 0;
	long long feasibleRelocateMoves = 0;
	long long improvingRelocateMoves = 0;
	long long generatedSwapMoves = 0;
	long long feasibleSwapMoves = 0;
	long long improvingSwapMoves = 0;
	long long generated2OptMoves = 0;
	long long feasible2OptMoves = 0;
	long long improving2OptMoves = 0;
	long long generatedCrossRouteRelocateMoves = 0;
	long long feasibleCrossRouteRelocateMoves = 0;
	long long improvingCrossRouteRelocateMoves = 0;
	long long feasibleUphillMoves1 = 0;
	long long feasibleUphillMoves3 = 0;
	long long feasibleUphillMoves5 = 0;
	double initialDistance = 0.0;
	double finalDistance = 0.0;
	bool validatorPassed = false;
	std::string reason;
};

DynamicLocalImproveResult improveDynamicQuantityLocally(
	const Individual& source,
	const Individual& splitCandidate,
	int resplitCustomer,
	Params& params,
	const DynamicLocalImproveConfig& config = DynamicLocalImproveConfig());

DynamicLocalImproveResult improveDynamicQuantityLocally(
	const Individual& source,
	const Individual& splitCandidate,
	const std::vector<int>& resplitCustomers,
	Params& params,
	const DynamicLocalImproveConfig& config = DynamicLocalImproveConfig());

#endif
