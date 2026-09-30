#ifndef JOINT_RESPLITTING_H
#define JOINT_RESPLITTING_H

#include "Individual.h"
#include <optional>
#include <string>
#include <utility>
#include <vector>

enum class JointCandidateType
{
	JOINT_RELOCATE,
	JOINT_ONE_SPLIT,
	JOINT_BOTH_SPLIT
};

enum class JointInsertionMode
{
	GREEDY_INSERTION,
	JOINT_INSERTION_COMBINATION,
	EXHAUSTIVE_FEASIBLE_INSERTION
};

const char* jointInsertionModeName(JointInsertionMode mode);

const char* jointCandidateTypeName(JointCandidateType type);
std::string jointAllocationText(const Individual& individual, int customer);
std::string jointCustomerRoutesText(const Individual& individual, int customerA, int customerB);

struct JointResplitConfig
{
	int candidateRoutes = 6;
	int maxPatternsPerCustomer = 8;
	int maxJointPatterns = 100;
	int maxCandidatesPerPair = 300;
	int maxDynamicSplits = 3;
	JointInsertionMode insertionMode = JointInsertionMode::GREEDY_INSERTION;
	int topInsertionPositions = 3;
	int patternReferenceRoutes = 0;
	double collectSplitCandidatesUpToDelta = -1.0;
	bool enforceGlobalDeadline = false;
};

struct JointResplitReport
{
	int customerA = -1;
	int customerB = -1;
	double demandA = 0.0;
	double demandB = 0.0;
	std::string oldA;
	std::string newA;
	std::string oldB;
	std::string newB;
	std::string routesBefore;
	std::string routesAfter;
	std::vector<int> routePool;
	JointCandidateType type = JointCandidateType::JOINT_RELOCATE;
	double sourceDistance = 0.0;
	double candidateDistance = 0.0;
	bool feasible = false;
	bool improving = false;
	std::string reason;
};

struct JointEnumerationStats
{
	long long generatedCandidates = 0;
	long long feasibleCandidates = 0;
	long long generatedSplitCandidates = 0;
	long long feasibleSplitCandidates = 0;
	long long generatedImprovingCandidates = 0;
	long long generatedImprovingSplitCandidates = 0;
	long long bothCustomersSplitCandidates = 0;
	long long oneSplitImprovingCandidates = 0;
	long long bothSplitImprovingCandidates = 0;
	long long relocateOnlyImprovingCandidates = 0;
	long long insertionPlacementsEvaluated = 0;
	long long feasibleInsertionPlacements = 0;
	long long nearNeutralSplitCandidates = 0;
	long long splitDeltaLE0 = 0;
	long long splitDelta0To1 = 0;
	long long splitDelta1To3 = 0;
	long long splitDelta3To5 = 0;
	long long splitDelta5To10 = 0;
	long long splitDeltaAbove10 = 0;

	void add(const JointEnumerationStats& other);
};

struct JointResplitResult
{
	struct CandidateRecord {
		Individual candidate;
		JointResplitReport report;
		CandidateRecord(const Individual& candidate, const JointResplitReport& report)
			: candidate(candidate), report(report) {}
	};
	std::optional<Individual> bestOverall;
	std::optional<Individual> bestSplit;
	JointResplitReport overallReport;
	JointResplitReport splitReport;
	JointEnumerationStats stats;
	std::vector<CandidateRecord> collectedSplitCandidates;
};

struct JointCustomerPair
{
	int customerA = -1;
	int customerB = -1;
	double score = 0.0;
};

std::vector<JointCustomerPair> selectJointCustomerPairs(
	const Individual& source,
	const Params& params,
	const std::vector<double>& singleBestSplitDelta,
	int maxPairs,
	double anchorGap = 5.0);

JointResplitResult generateJointResplitCandidate(
	const Individual& source,
	Params& params,
	int customerA,
	int customerB,
	const JointResplitConfig& config = JointResplitConfig());

#endif
