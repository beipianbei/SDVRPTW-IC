#ifndef BOUNDED_UPHILL_DYNAMIC_LS_H
#define BOUNDED_UPHILL_DYNAMIC_LS_H

#include "Individual.h"
#include <optional>
#include <string>
#include <vector>

struct BoundedUphillDynamicLSConfig
{
	double maxSingleDelta = 3.0;
	double maxCumulativeDelta = 5.0;
	int maxUphillMoves = 2;
	int recoveryMoves = 5;
	int beamWidth = 5;
	int nearbyRoutes = 3;
	bool enforceGlobalDeadline = false;
};

struct DynamicBridgeStats
{
	long long bridgeCandidatesTried = 0;
	long long bridgeTrajectoriesGenerated = 0;
	long long uphillMovesAcceptedForSearch = 0;
	long long depth1Trajectories = 0;
	long long depth2Trajectories = 0;
	long long recoveryRuns = 0;
	long long recoveryImprovingRuns = 0;
	long long netImprovingTrajectories = 0;
	double bestPeakExcursion = 0.0;
	double bestFinalDistance = 0.0;
	double bestNetImprovement = 0.0;
	bool hasFinalDistance = false;

	void add(const DynamicBridgeStats& other);
};

struct DynamicBridgeResult
{
	std::optional<Individual> bestImproving;
	DynamicBridgeStats stats;
	std::string uphillMove1;
	std::string uphillMove2;
	double delta1 = 0.0;
	double delta2 = 0.0;
	double sourceDistance = 0.0;
	double splitStartDistance = 0.0;
	double peakDistance = 0.0;
	double finalDistance = 0.0;
	int recoveryMoveCount = 0;
	int finalSplitCustomers = 0;
	bool validatorPassed = false;
};

DynamicBridgeResult runBoundedUphillDynamicLS(
	const Individual& source,
	const Individual& splitStart,
	const std::vector<int>& resplitCustomers,
	Params& params,
	const BoundedUphillDynamicLSConfig& config);

#endif
