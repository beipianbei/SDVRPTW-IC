#include "AdaptiveResplitting.h"
#include "InstanceCVRPLIB.h"
#include "SolutionValidator.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>
#include <tuple>

namespace {
double routeLoad(const std::vector<RouteVisit>& route)
{
	double result = 0.0;
	for (const RouteVisit& visit : route) result += visit.quantity;
	return result;
}

std::string quantityText(double quantity)
{
	std::ostringstream out;
	if (std::fabs(quantity - std::round(quantity)) < MY_EPSILON) out << static_cast<long long>(std::llround(quantity));
	else out << std::fixed << std::setprecision(3) << quantity;
	return out.str();
}

std::string allocation(const Individual& individual, int customer)
{
	std::ostringstream out;
	bool first = true;
	for (size_t route = 0; route < individual.routesDynamic.size(); ++route)
		for (const RouteVisit& visit : individual.routesDynamic[route])
			if (visit.originalCustomerId == customer) {
				if (!first) out << " + ";
				out << quantityText(visit.quantity) << "@R" << route + 1;
				first = false;
			}
	return first ? "none" : out.str();
}

int countCustomerRoutes(const Individual& individual, int customer)
{
	int count = 0;
	for (const auto& route : individual.routesDynamic)
		if (std::any_of(route.begin(), route.end(), [customer](const RouteVisit& visit) {
			return visit.originalCustomerId == customer;
		})) ++count;
	return count;
}

double insertionDelta(const std::vector<RouteVisit>& route, int position, int customer, const Params& params)
{
	const int before = position == 0 ? 0 : route[position - 1].originalCustomerId;
	const int after = position == static_cast<int>(route.size()) ? 0 : route[position].originalCustomerId;
	return params.originalTimeCost[before][customer] + params.originalTimeCost[customer][after]
		- params.originalTimeCost[before][after];
}

bool routeTimeWindowFeasible(const std::vector<RouteVisit>& route, const Params& params)
{
	double time = 0.0;
	int previous = 0;
	for (const RouteVisit& visit : route) {
		const int customer = visit.originalCustomerId;
		time += params.originalTimeCost[previous][customer];
		time = std::max(time, params.originalEarliest[customer]);
		if (time > params.originalLatest[customer] + MY_EPSILON) return false;
		time += params.originalService[customer];
		previous = customer;
	}
	time += params.originalTimeCost[previous][0];
	return time <= params.originalLatest[0] + MY_EPSILON;
}

bool typesConflict(int first, int second)
{
	if (first < 0 || second < 0 || first >= static_cast<int>(conflict_matrix.size())) return false;
	return second < static_cast<int>(conflict_matrix[first].size()) && conflict_matrix[first][second];
}

bool routeConflictFreeFor(const std::vector<RouteVisit>& route, int customer, const Params& params)
{
	const int newType = params.originalCommodity[customer];
	for (const RouteVisit& visit : route) {
		const int oldType = params.originalCommodity[visit.originalCustomerId];
		if (typesConflict(newType, oldType) || typesConflict(oldType, newType)) return false;
	}
	return true;
}

int bestInsertionPosition(const std::vector<RouteVisit>& route, int customer, const Params& params,
	bool& hasTimeWindowFeasiblePosition)
{
	int bestAny = 0;
	int bestFeasible = -1;
	double bestAnyDelta = 1.e30;
	double bestFeasibleDelta = 1.e30;
	for (int position = 0; position <= static_cast<int>(route.size()); ++position) {
		const double candidateDelta = insertionDelta(route, position, customer, params);
		if (candidateDelta < bestAnyDelta) {
			bestAnyDelta = candidateDelta;
			bestAny = position;
		}
		std::vector<RouteVisit> trial = route;
		trial.insert(trial.begin() + position, { customer, 1.0 });
		if (routeTimeWindowFeasible(trial, params) && candidateDelta < bestFeasibleDelta) {
			bestFeasibleDelta = candidateDelta;
			bestFeasible = position;
		}
	}
	hasTimeWindowFeasiblePosition = bestFeasible >= 0;
	return hasTimeWindowFeasiblePosition ? bestFeasible : bestAny;
}

int gcdInt(int first, int second)
{
	first = std::abs(first);
	second = std::abs(second);
	while (second != 0) {
		const int next = first % second;
		first = second;
		second = next;
	}
	return first;
}

std::string patternKey(std::vector<double> quantities)
{
	std::sort(quantities.begin(), quantities.end());
	std::ostringstream out;
	out << std::fixed << std::setprecision(6);
	for (double quantity : quantities) out << quantity << ';';
	return out.str();
}

void addPattern(std::vector<std::vector<double>>& patterns, std::set<std::string>& keys,
	std::vector<double> quantities, double demand)
{
	if (quantities.empty() || quantities.size() > 3) return;
	double total = 0.0;
	for (double quantity : quantities) {
		if (quantity <= MY_EPSILON) return;
		total += quantity;
	}
	if (std::fabs(total - demand) > MY_EPSILON) return;
	std::sort(quantities.begin(), quantities.end());
	if (keys.insert(patternKey(quantities)).second) patterns.push_back(std::move(quantities));
}

std::string assignmentKey(const std::vector<int>& routes, const std::vector<double>& quantities)
{
	std::vector<std::pair<int, double>> pieces;
	for (size_t i = 0; i < routes.size(); ++i) pieces.push_back({ routes[i], quantities[i] });
	std::sort(pieces.begin(), pieces.end());
	std::ostringstream out;
	out << std::fixed << std::setprecision(6);
	for (const auto& piece : pieces) out << piece.first << ':' << piece.second << ';';
	return out.str();
}

std::string routeList(const std::vector<int>& routes)
{
	std::vector<int> ordered = routes;
	std::sort(ordered.begin(), ordered.end());
	std::ostringstream out;
	for (size_t i = 0; i < ordered.size(); ++i) {
		if (i != 0) out << ';';
		out << 'R' << ordered[i] + 1;
	}
	return out.str();
}

std::string allocationSpec(const std::vector<int>& routes, const std::vector<double>& quantities)
{
	std::vector<std::pair<int, double>> pieces;
	for (size_t i = 0; i < routes.size(); ++i) pieces.push_back({ routes[i], quantities[i] });
	std::sort(pieces.begin(), pieces.end());
	std::ostringstream out;
	for (size_t i = 0; i < pieces.size(); ++i) {
		if (i != 0) out << " + ";
		out << quantityText(pieces[i].second) << "@R" << pieces[i].first + 1;
	}
	return out.str();
}

std::string rejectionReason(const ValidationResult& validation)
{
	if (!validation.capacityFeasible) return "CAPACITY";
	if (!validation.timeWindowFeasible) return "TIME_WINDOW";
	if (!validation.commodityFeasible) return "CONFLICT";
	if (!validation.visitBlocksValid || !validation.structureValid) return "DUPLICATE";
	if (!validation.originalDemandValid) return "DEMAND";
	return validation.feasible() ? "PASS" : "NO_INSERTION_POSITION";
}

ResplitReport makeReport(const Individual& source, const Individual& candidate, int customer,
	const Params& params)
{
	ResplitReport report;
	report.customer = customer;
	report.originalDemand = params.originalDemand[customer];
	report.oldDistance = source.eval.distance;
	report.newDistance = candidate.eval.distance;
	report.oldAllocation = allocation(source, customer);
	report.newAllocation = allocation(candidate, customer);
	report.splitRoutes = countCustomerRoutes(candidate, customer);
	report.feasible = true;
	report.improving = report.newDistance < report.oldDistance - MY_EPSILON;
	report.reason = "validator PASS";
	return report;
}

void enumerateAssignments(const std::vector<int>& candidateRoutes, int count,
	const std::function<void(const std::vector<int>&)>& consume)
{
	std::vector<int> current;
	std::function<void()> visit = [&]() {
		if (static_cast<int>(current.size()) == count) {
			consume(current);
			return;
		}
		for (int route : candidateRoutes) {
			if (std::find(current.begin(), current.end(), route) != current.end()) continue;
			current.push_back(route);
			visit();
			current.pop_back();
		}
	};
	visit();
}
}

void ResplitEnumerationStats::add(const ResplitEnumerationStats& other)
{
	generatedCandidates += other.generatedCandidates;
	generatedOneWayCandidates += other.generatedOneWayCandidates;
	generatedTwoWayCandidates += other.generatedTwoWayCandidates;
	generatedThreeWayCandidates += other.generatedThreeWayCandidates;
	generatedFeasibleCandidates += other.generatedFeasibleCandidates;
	generatedFeasibleOneWayCandidates += other.generatedFeasibleOneWayCandidates;
	generatedFeasibleTwoWayCandidates += other.generatedFeasibleTwoWayCandidates;
	generatedFeasibleThreeWayCandidates += other.generatedFeasibleThreeWayCandidates;
	generatedFeasibleSplitCandidates += other.generatedFeasibleSplitCandidates;
	generatedImprovingCandidates += other.generatedImprovingCandidates;
	generatedImprovingSplitCandidates += other.generatedImprovingSplitCandidates;
}

std::vector<int> getAdaptiveResplitEligibleCustomers(const Individual& source, const Params& params)
{
	std::vector<int> eligible;
	if (source.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY) return eligible;
	std::vector<bool> present(params.originalDemand.size(), false);
	for (const auto& route : source.routesDynamic)
		for (const RouteVisit& visit : route)
			if (visit.originalCustomerId > 0 && visit.originalCustomerId < static_cast<int>(present.size()))
				present[visit.originalCustomerId] = true;
	for (int customer = 1; customer < static_cast<int>(params.originalDemand.size()); ++customer)
		if (present[customer] && params.originalDemand[customer] > MY_EPSILON) eligible.push_back(customer);
	return eligible;
}

AdaptiveResplitResult generateAdaptiveResplitCandidates(const Individual& source, Params& params,
	int candidateRouteLimit, int maxSplits, int forcedCustomer, int maxSplitDiagnostics)
{
	AdaptiveResplitResult result;
	if (source.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY) {
		result.overallReport.reason = result.splitReport.reason = "source is not dynamic";
		return result;
	}
	const ValidationResult sourceCheck = validateSolutionFromScratch(source, params);
	if (!sourceCheck.feasible()) {
		result.overallReport.reason = result.splitReport.reason = "source invalid: " + sourceCheck.message;
		return result;
	}

	const std::vector<int> eligible = getAdaptiveResplitEligibleCustomers(source, params);
	if (eligible.empty()) {
		result.overallReport.reason = result.splitReport.reason = "no eligible customer";
		return result;
	}

	int chosen = forcedCustomer;
	if (std::find(eligible.begin(), eligible.end(), chosen) == eligible.end()) {
		std::vector<std::pair<double, int>> scored;
		for (int customer : eligible) {
			int visits = 0;
			double score = 0.0;
			for (const auto& route : source.routesDynamic)
				for (const RouteVisit& visit : route)
					if (visit.originalCustomerId == customer) {
						++visits;
						score += (params.vehicleCapacity - routeLoad(route)) / params.vehicleCapacity;
					}
			score += 0.5 * std::max(0, visits - 1);
			scored.push_back({ score, customer });
		}
		std::sort(scored.begin(), scored.end(), [](const auto& first, const auto& second) {
			return first.first != second.first ? first.first > second.first : first.second < second.second;
		});
		if (scored.size() > 30) scored.resize(30);
		if (params.dynamicRan() % 100 < 30) chosen = scored[params.dynamicRan() % scored.size()].second;
		else {
			const unsigned long long totalWeight = scored.size() * (scored.size() + 1) / 2;
			unsigned long long draw = params.dynamicRan() % totalWeight;
			for (size_t rank = 0; rank < scored.size(); ++rank) {
				const unsigned long long weight = scored.size() - rank;
				if (draw < weight) { chosen = scored[rank].second; break; }
				draw -= weight;
			}
		}
	}

	Individual base = source;
	std::vector<int> currentRoutes;
	for (int route = 0; route < static_cast<int>(base.routesDynamic.size()); ++route) {
		const bool contained = std::any_of(base.routesDynamic[route].begin(), base.routesDynamic[route].end(),
			[chosen](const RouteVisit& visit) { return visit.originalCustomerId == chosen; });
		if (contained) currentRoutes.push_back(route);
		base.routesDynamic[route].erase(std::remove_if(base.routesDynamic[route].begin(), base.routesDynamic[route].end(),
			[chosen](const RouteVisit& visit) { return visit.originalCustomerId == chosen; }), base.routesDynamic[route].end());
	}

	const double demand = params.originalDemand[chosen];
	std::vector<int> candidateRoutes;
	for (int route = 0; route < static_cast<int>(base.routesDynamic.size()); ++route)
		if (!base.routesDynamic[route].empty()) candidateRoutes.push_back(route);
	std::sort(candidateRoutes.begin(), candidateRoutes.end(), [&](int first, int second) {
		const double firstResidual = params.vehicleCapacity - routeLoad(base.routesDynamic[first]);
		const double secondResidual = params.vehicleCapacity - routeLoad(base.routesDynamic[second]);
		double firstDelta = 1.e30, secondDelta = 1.e30;
		for (int position = 0; position <= static_cast<int>(base.routesDynamic[first].size()); ++position)
			firstDelta = std::min(firstDelta, insertionDelta(base.routesDynamic[first], position, chosen, params));
		for (int position = 0; position <= static_cast<int>(base.routesDynamic[second].size()); ++position)
			secondDelta = std::min(secondDelta, insertionDelta(base.routesDynamic[second], position, chosen, params));
		return std::make_tuple(firstResidual <= MY_EPSILON, std::fabs(firstResidual - demand), firstDelta, first)
			< std::make_tuple(secondResidual <= MY_EPSILON, std::fabs(secondResidual - demand), secondDelta, second);
	});
	candidateRouteLimit = std::max(1, candidateRouteLimit);
	if (static_cast<int>(candidateRoutes.size()) > candidateRouteLimit) candidateRoutes.resize(candidateRouteLimit);
	for (int current : currentRoutes) {
		if (std::find(candidateRoutes.begin(), candidateRoutes.end(), current) != candidateRoutes.end()) continue;
		if (static_cast<int>(candidateRoutes.size()) < candidateRouteLimit) candidateRoutes.push_back(current);
		else {
			auto replace = std::find_if(candidateRoutes.rbegin(), candidateRoutes.rend(), [&](int route) {
				return std::find(currentRoutes.begin(), currentRoutes.end(), route) == currentRoutes.end();
			});
			if (replace != candidateRoutes.rend()) *replace = current;
		}
	}
	for (int route = 0; route < static_cast<int>(base.routesDynamic.size()); ++route)
		if (base.routesDynamic[route].empty() &&
			std::find(candidateRoutes.begin(), candidateRoutes.end(), route) == candidateRoutes.end()) {
			candidateRoutes.push_back(route);
			break;
		}

	std::vector<std::vector<double>> patterns;
	std::set<std::string> patternKeys;
	const bool integerDemand = std::fabs(demand - std::round(demand)) < MY_EPSILON;
	const double minimumPiece = integerDemand ? 1.0 : std::max(MY_EPSILON * 10.0, demand / 100.0);
	addPattern(patterns, patternKeys, { demand }, demand);
	if (maxSplits >= 2 && demand >= 2.0 * minimumPiece - MY_EPSILON) {
		const int roundedDemand = std::max(1, static_cast<int>(std::llround(demand)));
		const int roundedCapacity = std::max(1, static_cast<int>(std::llround(params.vehicleCapacity)));
		const double quantum = std::max(1, gcdInt(roundedDemand, roundedCapacity));
		addPattern(patterns, patternKeys, { demand / 2.0, demand - demand / 2.0 }, demand);
		if (quantum < demand - MY_EPSILON) addPattern(patterns, patternKeys, { quantum, demand - quantum }, demand);
		for (int first : candidateRoutes) {
			const double firstResidual = std::max(0.0, params.vehicleCapacity - routeLoad(base.routesDynamic[first]));
			for (int second : candidateRoutes) {
				if (first == second) continue;
				const double secondResidual = std::max(0.0, params.vehicleCapacity - routeLoad(base.routesDynamic[second]));
				if (firstResidual + secondResidual + MY_EPSILON < demand) continue;
				const double firstPiece = std::min(demand - minimumPiece, std::floor(firstResidual + MY_EPSILON));
				if (firstPiece >= minimumPiece - MY_EPSILON &&
					demand - firstPiece >= minimumPiece - MY_EPSILON &&
					demand - firstPiece <= secondResidual + MY_EPSILON)
					addPattern(patterns, patternKeys, { firstPiece, demand - firstPiece }, demand);
			}
		}
	}
	if (maxSplits >= 3 && demand >= 3.0 * minimumPiece - MY_EPSILON) {
		addPattern(patterns, patternKeys, { demand / 3.0, demand / 3.0, demand - 2.0 * demand / 3.0 }, demand);
		for (int first : candidateRoutes) for (int second : candidateRoutes) {
			if (first == second) continue;
			const double firstResidual = std::max(0.0, params.vehicleCapacity - routeLoad(base.routesDynamic[first]));
			const double secondResidual = std::max(0.0, params.vehicleCapacity - routeLoad(base.routesDynamic[second]));
			const double firstPiece = std::min(std::floor(firstResidual + MY_EPSILON), demand - 2.0 * minimumPiece);
			const double remaining = demand - firstPiece;
			const double secondPiece = std::min(std::floor(secondResidual + MY_EPSILON), remaining - minimumPiece);
			if (firstPiece >= minimumPiece - MY_EPSILON && secondPiece >= minimumPiece - MY_EPSILON &&
				remaining - secondPiece >= minimumPiece - MY_EPSILON)
				addPattern(patterns, patternKeys, { firstPiece, secondPiece, remaining - secondPiece }, demand);
		}
	}

	std::set<std::string> enumeratedAllocations;
	for (const std::vector<double>& pattern : patterns) {
		if (static_cast<int>(pattern.size()) > maxSplits) continue;
		enumerateAssignments(candidateRoutes, static_cast<int>(pattern.size()), [&](const std::vector<int>& assignedRoutes) {
			const std::string key = assignmentKey(assignedRoutes, pattern);
			if (!enumeratedAllocations.insert(key).second) return;
			++result.stats.generatedCandidates;
			if (pattern.size() == 1) ++result.stats.generatedOneWayCandidates;
			else if (pattern.size() == 2) ++result.stats.generatedTwoWayCandidates;
			else ++result.stats.generatedThreeWayCandidates;

			Individual candidate = base;
			bool capacityOK = true;
			bool insertionPositionFound = true;
			bool conflictOK = true;
			for (size_t piece = 0; piece < pattern.size(); ++piece) {
				const int route = assignedRoutes[piece];
				capacityOK = capacityOK && routeLoad(candidate.routesDynamic[route]) + pattern[piece]
					<= params.vehicleCapacity + MY_EPSILON;
				conflictOK = conflictOK && routeConflictFreeFor(candidate.routesDynamic[route], chosen, params);
				bool twPosition = false;
				const int position = bestInsertionPosition(candidate.routesDynamic[route], chosen, params, twPosition);
				insertionPositionFound = insertionPositionFound && twPosition;
				candidate.routesDynamic[route].insert(candidate.routesDynamic[route].begin() + position,
					{ chosen, pattern[piece] });
			}
			candidate.quantityRepresentation = QuantityRepresentation::DYNAMIC_QUANTITY;
			candidate.evaluateDynamicQuantity(params);
			const ValidationResult validation = validateSolutionFromScratch(candidate, params);
			const bool feasible = validation.feasible();
			const bool split = pattern.size() >= 2;
			const bool improving = candidate.eval.distance < source.eval.distance - MY_EPSILON;
			if (feasible) {
				++result.stats.generatedFeasibleCandidates;
				if (pattern.size() == 1) ++result.stats.generatedFeasibleOneWayCandidates;
				else if (pattern.size() == 2) ++result.stats.generatedFeasibleTwoWayCandidates;
				else ++result.stats.generatedFeasibleThreeWayCandidates;
				if (split) ++result.stats.generatedFeasibleSplitCandidates;
				if (improving) ++result.stats.generatedImprovingCandidates;
				if (split && improving) ++result.stats.generatedImprovingSplitCandidates;
				if (!result.bestOverall || candidate.eval.distance < result.bestOverall->eval.distance - MY_EPSILON)
					result.bestOverall = candidate;
				if (split && (!result.bestSplit || candidate.eval.distance < result.bestSplit->eval.distance - MY_EPSILON))
					result.bestSplit = candidate;
			}

			if (split && static_cast<int>(result.splitDiagnostics.size()) < maxSplitDiagnostics) {
				SplitCandidateDiagnostic diagnostic;
				diagnostic.customer = chosen;
				diagnostic.demand = demand;
				diagnostic.splitRoutes = static_cast<int>(pattern.size());
				diagnostic.allocation = allocationSpec(assignedRoutes, pattern);
				diagnostic.routes = routeList(assignedRoutes);
				diagnostic.distance = candidate.eval.distance;
				diagnostic.delta = candidate.eval.distance - source.eval.distance;
				diagnostic.distanceAvailable = true;
				diagnostic.capacityOK = validation.capacityFeasible && capacityOK;
				diagnostic.twOK = validation.timeWindowFeasible && insertionPositionFound;
				diagnostic.conflictOK = validation.commodityFeasible && conflictOK;
				diagnostic.validator = feasible;
				diagnostic.reason = feasible ? "PASS" : rejectionReason(validation);
				if (!insertionPositionFound && diagnostic.reason == "PASS") diagnostic.reason = "NO_INSERTION_POSITION";
				result.splitDiagnostics.push_back(std::move(diagnostic));
			}
		});
	}

	if (result.bestOverall) result.overallReport = makeReport(source, *result.bestOverall, chosen, params);
	else {
		result.overallReport.customer = chosen;
		result.overallReport.originalDemand = demand;
		result.overallReport.oldDistance = source.eval.distance;
		result.overallReport.oldAllocation = allocation(source, chosen);
		result.overallReport.reason = "no feasible route/partition candidate";
	}
	if (result.bestSplit) result.splitReport = makeReport(source, *result.bestSplit, chosen, params);
	else {
		result.splitReport.customer = chosen;
		result.splitReport.originalDemand = demand;
		result.splitReport.oldDistance = source.eval.distance;
		result.splitReport.oldAllocation = allocation(source, chosen);
		result.splitReport.reason = "no feasible true-split candidate";
	}
	for (SplitCandidateDiagnostic& diagnostic : result.splitDiagnostics)
		diagnostic.selectedAsBestSplit = result.bestSplit && diagnostic.validator &&
			diagnostic.allocation == result.splitReport.newAllocation &&
			std::fabs(diagnostic.distance - result.splitReport.newDistance) < MY_EPSILON;
	return result;
}

std::optional<Individual> generateAdaptiveResplitCandidate(const Individual& source, Params& params,
	ResplitReport& report, int candidateRoutes, int maxDynamicSplits)
{
	AdaptiveResplitResult result = generateAdaptiveResplitCandidates(source, params, candidateRoutes,
		maxDynamicSplits, -1, 0);
	report = result.overallReport;
	return result.bestOverall;
}

bool DynamicElite::consider(const Individual& candidate, const Params& params)
{
	if (candidate.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY ||
		!validateSolutionFromScratch(candidate, params).feasible()) return false;
	if (!best || candidate.eval.distance < best->eval.distance - MY_EPSILON) {
		best = candidate;
		return true;
	}
	return false;
}
