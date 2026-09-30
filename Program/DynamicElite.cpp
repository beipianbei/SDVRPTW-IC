#include "DynamicElite.h"
#include "AdaptiveResplitting.h"
#include "JointResplitting.h"
#include "SolutionValidator.h"
#include "NumericFormat.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <time.h>

DynamicEliteController::DynamicEliteController(Params& params, const DynamicEliteConfig& config)
	: params(params), config(config)
{
}

double DynamicEliteController::elapsedSeconds() const
{
	return static_cast<double>(clock() - params.startTime) / CLOCKS_PER_SEC;
}

bool DynamicEliteController::globalTimeAvailable(double reserveSeconds) const
{
	return !params.hasGlobalDeadline() || elapsedSeconds() + reserveSeconds < params.ap.timeLimit;
}

int DynamicEliteController::countSplitCustomers(const Individual& individual) const
{
	std::vector<int> routeCount(params.originalDemand.size(), 0);
	for (const auto& route : individual.routesDynamic) {
		std::set<int> seen;
		for (const RouteVisit& visit : route) seen.insert(visit.originalCustomerId);
		for (int customer : seen)
			if (customer > 0 && customer < static_cast<int>(routeCount.size())) ++routeCount[customer];
	}
	return static_cast<int>(std::count_if(routeCount.begin() + std::min<size_t>(1, routeCount.size()),
		routeCount.end(), [](int count) { return count >= 2; }));
}

void DynamicEliteController::onIteration(int, int nonImprovingIterations, const Individual* bestFixed)
{
	if (bestFixed == nullptr || config.trigger <= 0 || nonImprovingIterations < config.trigger ||
		nonImprovingIterations % config.trigger != 0 || !globalTimeAvailable(0.25)) return;
	++stats.dynamicTriggers;
	const double triggerDeadline = elapsedSeconds() + std::max(0.0, config.triggerBudgetSeconds);
	auto triggerTimeAvailable = [&]() {
		return globalTimeAvailable(0.05) && elapsedSeconds() < triggerDeadline;
	};

	Individual source(*bestFixed);
	source.convertFixedToDynamic(params);
	source.evaluateDynamicQuantity(params);
	const ValidationResult sourceValidation = validateSolutionFromScratch(source, params);
	if (!sourceValidation.feasible()) return;

	const std::vector<int> eligibleCustomers = getAdaptiveResplitEligibleCustomers(source, params);
	std::vector<double> singleSplitDelta(params.originalDemand.size(), std::numeric_limits<double>::infinity());
	for (int customer : eligibleCustomers) {
		if (!triggerTimeAvailable()) return;
		AdaptiveResplitResult single = generateAdaptiveResplitCandidates(source, params, 5, 3, customer, 0);
		if (single.bestSplit)
			singleSplitDelta[customer] = single.splitReport.newDistance - source.eval.distance;
	}
	const std::vector<JointCustomerPair> pairs = selectJointCustomerPairs(source, params,
		singleSplitDelta, 30, config.splitAcceptanceGap);
	if (pairs.empty() || !triggerTimeAvailable()) return;

	JointResplitConfig jointConfig;
	jointConfig.candidateRoutes = 6;
	jointConfig.maxPatternsPerCustomer = 8;
	jointConfig.maxJointPatterns = 100;
	jointConfig.maxCandidatesPerPair = 300;
	jointConfig.maxDynamicSplits = 3;
	jointConfig.insertionMode = JointInsertionMode::EXHAUSTIVE_FEASIBLE_INSERTION;
	jointConfig.topInsertionPositions = 3;
	jointConfig.patternReferenceRoutes = 6;
	jointConfig.collectSplitCandidatesUpToDelta = config.splitAcceptanceGap;
	jointConfig.enforceGlobalDeadline = true;

	for (int pairOffset = 0; pairOffset < std::max(1, config.maxPairsPerTrigger) && triggerTimeAvailable(); ++pairOffset) {
		const JointCustomerPair& pair = pairs[static_cast<size_t>(pairCursor++ % pairs.size())];
		JointResplitResult joint = generateJointResplitCandidate(source, params,
			pair.customerA, pair.customerB, jointConfig);
		stats.jointCandidates += joint.stats.generatedCandidates;
		std::sort(joint.collectedSplitCandidates.begin(), joint.collectedSplitCandidates.end(),
			[](const JointResplitResult::CandidateRecord& first, const JointResplitResult::CandidateRecord& second) {
				return first.report.candidateDistance < second.report.candidateDistance;
			});
		int tried = 0;
		for (const JointResplitResult::CandidateRecord& record : joint.collectedSplitCandidates) {
			if (tried >= std::max(1, config.maxBridgeCandidatesPerTrigger) || !triggerTimeAvailable()) break;
			++tried;
			++stats.bridgeCandidates;
			DynamicBridgeResult bridge = runBoundedUphillDynamicLS(source, record.candidate,
				std::vector<int>{ pair.customerA, pair.customerB }, params, config.bridge);
			stats.bridgeTrajectories += bridge.stats.bridgeTrajectoriesGenerated;
			if (params.deadlineReached()) return;
			if (!bridge.bestImproving) continue;
			const ValidationResult validation = validateSolutionFromScratch(*bridge.bestImproving, params);
			if (!validation.feasible()) continue;
			const int splitCustomers = countSplitCustomers(*bridge.bestImproving);
			const double before = dynamicElite ? dynamicElite->eval.distance : std::numeric_limits<double>::infinity();
			if (!dynamicElite || bridge.bestImproving->eval.distance < dynamicElite->eval.distance - MY_EPSILON) {
				dynamicElite = *bridge.bestImproving;
				++stats.dynamicEliteUpdates;
				std::cout << "[DYNAMIC-ELITE] time=" << elapsedSeconds()
					<< " fixedSource=" << objectiveValue2(source.eval.distance)
					<< " dynamicBefore=";
				if (std::isfinite(before)) std::cout << objectiveValue2(before);
				else std::cout << "NA";
				std::cout << " dynamicAfter=" << objectiveValue2(dynamicElite->eval.distance)
					<< " improvement=" << objectiveValue2(std::isfinite(before) ? before - dynamicElite->eval.distance
						: source.eval.distance - dynamicElite->eval.distance)
					<< " splitCustomers=" << splitCustomers
					<< " source=JOINT_RESPLIT_BRIDGE" << std::endl;
			}
			if (splitCustomers > 0 && (!dynamicSplitElite ||
				bridge.bestImproving->eval.distance < dynamicSplitElite->eval.distance - MY_EPSILON))
				dynamicSplitElite = *bridge.bestImproving;
		}
	}
}

int DynamicEliteController::bestDynamicSplitCustomers() const
{
	return dynamicSplitElite ? countSplitCustomers(*dynamicSplitElite) : 0;
}

bool DynamicEliteController::bestDynamicValidatorPassed() const
{
	return dynamicElite && validateSolutionFromScratch(*dynamicElite, params).feasible();
}

bool exportDynamicQuantitySolution(const Individual& individual, const std::string& fileName)
{
	std::ofstream out(fileName);
	if (!out.is_open()) return false;
	int routeNumber = 0;
	for (const auto& route : individual.routesDynamic) {
		if (route.empty()) continue;
		out << "Route #" << ++routeNumber << ':';
		for (const RouteVisit& visit : route)
			out << ' ' << visit.originalCustomerId << '(' << visit.quantity << ')';
		out << '\n';
	}
	out << "Cost " << objectiveValue2(individual.eval.distance) << '\n';
	return true;
}
