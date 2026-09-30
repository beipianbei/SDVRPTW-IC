#ifndef DYNAMIC_ELITE_H
#define DYNAMIC_ELITE_H

#include "BoundedUphillDynamicLS.h"
#include "Individual.h"
#include <optional>
#include <string>

struct DynamicEliteConfig
{
	int trigger = 200;
	double triggerBudgetSeconds = 2.0;
	int maxPairsPerTrigger = 1;
	int maxBridgeCandidatesPerTrigger = 5;
	double splitAcceptanceGap = 5.0;
	BoundedUphillDynamicLSConfig bridge;
};

struct DynamicEliteStats
{
	long long dynamicTriggers = 0;
	long long jointCandidates = 0;
	long long bridgeCandidates = 0;
	long long bridgeTrajectories = 0;
	long long dynamicEliteUpdates = 0;
};

class DynamicEliteController
{
public:
	DynamicEliteController(Params& params, const DynamicEliteConfig& config);
	void onIteration(int iteration, int nonImprovingIterations, const Individual* bestFixed);

	const std::optional<Individual>& bestDynamic() const { return dynamicElite; }
	const std::optional<Individual>& bestDynamicSplit() const { return dynamicSplitElite; }
	const DynamicEliteStats& statistics() const { return stats; }
	int bestDynamicSplitCustomers() const;
	bool bestDynamicValidatorPassed() const;

private:
	Params& params;
	DynamicEliteConfig config;
	DynamicEliteStats stats;
	std::optional<Individual> dynamicElite;
	std::optional<Individual> dynamicSplitElite;
	long long pairCursor = 0;

	double elapsedSeconds() const;
	bool globalTimeAvailable(double reserveSeconds = 0.0) const;
	int countSplitCustomers(const Individual& individual) const;
};

bool exportDynamicQuantitySolution(const Individual& individual, const std::string& fileName);

#endif
