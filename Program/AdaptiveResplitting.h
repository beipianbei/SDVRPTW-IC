#ifndef ADAPTIVE_RESPLITTING_H
#define ADAPTIVE_RESPLITTING_H
#include "Individual.h"
#include <optional>
#include <string>
#include <vector>

struct ResplitReport {
 int customer = -1; double originalDemand = 0.0; double oldDistance = 0.0; double newDistance = 0.0;
 int splitRoutes = 0;
 std::string oldAllocation; std::string newAllocation; std::string reason; bool feasible = false; bool improving = false;
};

struct ResplitEnumerationStats {
	long long generatedCandidates = 0;
	long long generatedOneWayCandidates = 0;
	long long generatedTwoWayCandidates = 0;
	long long generatedThreeWayCandidates = 0;
	long long generatedFeasibleCandidates = 0;
	long long generatedFeasibleOneWayCandidates = 0;
	long long generatedFeasibleTwoWayCandidates = 0;
	long long generatedFeasibleThreeWayCandidates = 0;
	long long generatedFeasibleSplitCandidates = 0;
	long long generatedImprovingCandidates = 0;
	long long generatedImprovingSplitCandidates = 0;

	void add(const ResplitEnumerationStats& other);
};

struct SplitCandidateDiagnostic {
	int customer = -1;
	double demand = 0.0;
	int splitRoutes = 0;
	std::string allocation;
	std::string routes;
	double distance = 0.0;
	double delta = 0.0;
	bool distanceAvailable = false;
	bool capacityOK = false;
	bool twOK = false;
	bool conflictOK = false;
	bool validator = false;
	bool selectedAsBestSplit = false;
	std::string reason;
};

struct AdaptiveResplitResult {
	std::optional<Individual> bestOverall;
	std::optional<Individual> bestSplit;
	ResplitReport overallReport;
	ResplitReport splitReport;
	ResplitEnumerationStats stats;
	std::vector<SplitCandidateDiagnostic> splitDiagnostics;
};

std::vector<int> getAdaptiveResplitEligibleCustomers(const Individual& source, const Params& params);

AdaptiveResplitResult generateAdaptiveResplitCandidates(const Individual& source, Params& params,
	int candidateRoutes = 5, int maxDynamicSplits = 3, int forcedCustomer = -1,
	int maxSplitDiagnostics = 0);

// Compatibility wrapper for callers that only need the best overall candidate.
std::optional<Individual> generateAdaptiveResplitCandidate(const Individual& source, Params& params,
 ResplitReport& report, int candidateRoutes = 5, int maxDynamicSplits = 3);

class DynamicElite {
 std::optional<Individual> best;
public:
 bool consider(const Individual& candidate, const Params& params);
 const Individual* get() const { return best ? &*best : nullptr; }
};
#endif
