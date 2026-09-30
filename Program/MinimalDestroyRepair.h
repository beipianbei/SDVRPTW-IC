#ifndef MINIMAL_DESTROY_REPAIR_H
#define MINIMAL_DESTROY_REPAIR_H

#include "Population.h"
#include <string>
#include <vector>

struct MinimalDestroyRepairConfig
{
	int trigger = 500;
	double removalRatio = 0.15;
	int survivalCheckDelay = 50;
};

struct MinimalDestroyRepairStats
{
	int triggers = 0;
	int successfulRuns = 0;
	int skippedNoFeasibleParent = 0;
	int feasibleRepairs = 0;
	int improvedParent = 0;
	int newGlobalBest = 0;
	int inserted = 0;
	int survivalChecks = 0;
	int survived = 0;
	double totalTime = 0.0;
};

class MinimalDestroyRepair
{
public:
	MinimalDestroyRepair(Params& params, Population& population, LocalSearch& localSearch,
		const MinimalDestroyRepairConfig& config);
	// Returns true only when this trigger produced a new fixed global best.
	bool onIteration(int iteration, int nonImprovingIterations);
	const MinimalDestroyRepairStats& statistics() const { return stats; }

private:
	struct PendingSurvival
	{
		int dueIteration = 0;
		int triggerIteration = 0;
		std::vector<std::vector<int>> routes;
	};

	Params& params;
	Population& population;
	LocalSearch& localSearch;
	MinimalDestroyRepairConfig config;
	MinimalDestroyRepairStats stats;
	int nextAllowedIteration = 0;
	std::vector<PendingSurvival> pendingSurvival;

	bool destroyAndRegret2Repair(const Individual& parent, Individual& candidate,
		std::vector<int>& removedCustomers, int& removedRouteCount);
	void processSurvivalChecks(int iteration);
};

#endif
