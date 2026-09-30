#ifndef SOLUTION_VALIDATOR_H
#define SOLUTION_VALIDATOR_H

#include "Individual.h"
#include <string>

struct ValidationResult
{
	bool structureValid = true;
	bool capacityFeasible = true;
	bool timeWindowFeasible = true;
	bool commodityFeasible = true;
	bool visitBlocksValid = true;
	bool originalDemandValid = true;
	double distance = 0.0;
	double capacityExcess = 0.0;
	double timeWindowExcess = 0.0;
	int routesUsed = 0;
	std::string message;

	bool feasible() const
	{
		return structureValid && capacityFeasible && timeWindowFeasible &&
			commodityFeasible && visitBlocksValid && originalDemandValid;
	}
};

struct RouteDiagnostics
{
	double totalDemand = 0.0;
	double averageRouteLoad = 0.0;
	double averageUtilization = 0.0;
	double minimumRouteLoad = 0.0;
	double maximumRouteLoad = 0.0;
	int fullRoutes = 0;
	int routesUsed = 0;
};

ValidationResult validateSolutionFromScratch(const Individual& indiv, const Params& params);
RouteDiagnostics computeRouteDiagnostics(const Individual& indiv, const Params& params);

#endif

