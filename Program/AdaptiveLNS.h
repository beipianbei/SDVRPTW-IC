#ifndef ADAPTIVE_LNS_H
#define ADAPTIVE_LNS_H

#include "Individual.h"
#include <array>

class AdaptiveLNS
{
public:
	explicit AdaptiveLNS(Params& params);
	bool run(const Individual& seed, Individual& result, double globalBest);
	const std::array<double, 5>& removalWeights() const { return removalWeight; }
	const std::array<double, 4>& repairWeights() const { return repairWeight; }

private:
	Params& params;
	std::array<double, 5> removalWeight;
	std::array<double, 4> repairWeight;

	int roulette(const double* weights, int count);
	bool ruinAndRepair(const Individual& source, Individual& candidate, int removalOp, int repairOp);
	void updateWeight(double& weight, double reward);
};

#endif
