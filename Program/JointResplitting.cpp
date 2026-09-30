#include "JointResplitting.h"
#include "InstanceCVRPLIB.h"
#include "SolutionValidator.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>

namespace {
struct AllocationProposal
{
	std::vector<double> quantities;
	std::vector<int> routes;
	double estimatedDelta = 0.0;
};

struct JointPattern
{
	AllocationProposal allocationA;
	AllocationProposal allocationB;
	double estimatedDelta = 0.0;
	JointCandidateType type = JointCandidateType::JOINT_RELOCATE;
};

double routeLoad(const std::vector<RouteVisit>& route)
{
	double load = 0.0;
	for (const RouteVisit& visit : route) load += visit.quantity;
	return load;
}

bool routeContains(const std::vector<RouteVisit>& route, int customer)
{
	return std::any_of(route.begin(), route.end(), [customer](const RouteVisit& visit) {
		return visit.originalCustomerId == customer;
	});
}

double insertionDelta(const std::vector<RouteVisit>& route, int position, int customer, const Params& params)
{
	const int before = position == 0 ? 0 : route[position - 1].originalCustomerId;
	const int after = position == static_cast<int>(route.size()) ? 0 : route[position].originalCustomerId;
	return params.originalTimeCost[before][customer] + params.originalTimeCost[customer][after]
		- params.originalTimeCost[before][after];
}

double minimumInsertionDelta(const std::vector<RouteVisit>& route, int customer, const Params& params)
{
	double best = std::numeric_limits<double>::max();
	for (int position = 0; position <= static_cast<int>(route.size()); ++position)
		best = std::min(best, insertionDelta(route, position, customer, params));
	return best;
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

bool routeConflictFree(const std::vector<RouteVisit>& route, const Params& params)
{
	for (size_t first = 0; first < route.size(); ++first)
		for (size_t second = first + 1; second < route.size(); ++second) {
			const int firstType = params.originalCommodity[route[first].originalCustomerId];
			const int secondType = params.originalCommodity[route[second].originalCustomerId];
			if (typesConflict(firstType, secondType) || typesConflict(secondType, firstType)) return false;
		}
	return true;
}

std::string quantityText(double quantity)
{
	std::ostringstream out;
	if (std::fabs(quantity - std::round(quantity)) < MY_EPSILON)
		out << static_cast<long long>(std::llround(quantity));
	else out << std::fixed << std::setprecision(3) << quantity;
	return out.str();
}

std::string patternKey(std::vector<double> quantities)
{
	std::sort(quantities.begin(), quantities.end());
	std::ostringstream out;
	out << std::fixed << std::setprecision(6);
	for (double quantity : quantities) out << quantity << ';';
	return out.str();
}

void addQuantityPattern(std::vector<std::vector<double>>& patterns, std::set<std::string>& keys,
	std::vector<double> quantities, double demand, int maxSplits)
{
	if (quantities.empty() || static_cast<int>(quantities.size()) > maxSplits) return;
	double total = 0.0;
	for (double quantity : quantities) {
		if (quantity <= MY_EPSILON) return;
		total += quantity;
	}
	if (std::fabs(total - demand) > MY_EPSILON) return;
	std::sort(quantities.begin(), quantities.end());
	if (keys.insert(patternKey(quantities)).second) patterns.push_back(std::move(quantities));
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

std::vector<double> currentQuantities(const Individual& source, int customer)
{
	std::vector<double> quantities;
	for (const auto& route : source.routesDynamic)
		for (const RouteVisit& visit : route)
			if (visit.originalCustomerId == customer) quantities.push_back(visit.quantity);
	return quantities;
}

std::vector<std::vector<double>> quantityPatterns(const Individual& source, const Individual& base,
	int customer, const std::vector<int>& routePool, const Params& params, const JointResplitConfig& config)
{
	const double demand = params.originalDemand[customer];
	const bool integerDemand = std::fabs(demand - std::round(demand)) < MY_EPSILON;
	const double minimumPiece = integerDemand ? 1.0 : std::max(MY_EPSILON * 10.0, demand / 100.0);
	std::vector<std::vector<double>> patterns;
	std::set<std::string> keys;
	addQuantityPattern(patterns, keys, { demand }, demand, config.maxDynamicSplits);
	addQuantityPattern(patterns, keys, currentQuantities(source, customer), demand, config.maxDynamicSplits);
	if (config.maxDynamicSplits >= 2 && demand >= 2.0 * minimumPiece - MY_EPSILON) {
		addQuantityPattern(patterns, keys, { demand / 2.0, demand - demand / 2.0 }, demand, config.maxDynamicSplits);
		const int roundedDemand = std::max(1, static_cast<int>(std::llround(demand)));
		const int roundedCapacity = std::max(1, static_cast<int>(std::llround(params.vehicleCapacity)));
		const double quantum = std::max(1, gcdInt(roundedDemand, roundedCapacity));
		if (quantum < demand - MY_EPSILON)
			addQuantityPattern(patterns, keys, { quantum, demand - quantum }, demand, config.maxDynamicSplits);
		for (int route : routePool) {
			const double residual = std::max(0.0, params.vehicleCapacity - routeLoad(base.routesDynamic[route]));
			const double firstPiece = std::min(demand - minimumPiece, std::floor(residual + MY_EPSILON));
			if (firstPiece >= minimumPiece - MY_EPSILON && demand - firstPiece >= minimumPiece - MY_EPSILON)
				addQuantityPattern(patterns, keys, { firstPiece, demand - firstPiece }, demand, config.maxDynamicSplits);
		}
		if (config.maxPatternsPerCustomer > 8) {
			for (double firstPiece = quantum * 2.0;
				firstPiece <= demand - minimumPiece + MY_EPSILON; firstPiece += quantum)
				addQuantityPattern(patterns, keys, { firstPiece, demand - firstPiece }, demand,
					config.maxDynamicSplits);
		}
	}
	if (config.maxDynamicSplits >= 3 && demand >= 3.0 * minimumPiece - MY_EPSILON) {
		std::vector<double> residuals;
		for (int route : routePool)
			residuals.push_back(std::max(0.0, params.vehicleCapacity - routeLoad(base.routesDynamic[route])));
		std::sort(residuals.begin(), residuals.end(), std::greater<double>());
		if (residuals.size() >= 3 && residuals[0] + residuals[1] + residuals[2] >= demand - MY_EPSILON) {
			const double first = std::min(std::floor(residuals[0] + MY_EPSILON), demand - 2.0 * minimumPiece);
			const double second = std::min(std::floor(residuals[1] + MY_EPSILON), demand - first - minimumPiece);
			addQuantityPattern(patterns, keys, { first, second, demand - first - second }, demand, config.maxDynamicSplits);
		}
	}
	if (static_cast<int>(patterns.size()) > config.maxPatternsPerCustomer)
		patterns.resize(std::max(1, config.maxPatternsPerCustomer));
	return patterns;
}

void enumerateRouteAssignments(const std::vector<int>& routePool, int count,
	const std::function<void(const std::vector<int>&)>& consume)
{
	std::vector<int> assignment;
	std::function<void()> visit = [&]() {
		if (static_cast<int>(assignment.size()) == count) {
			consume(assignment);
			return;
		}
		for (int route : routePool) {
			if (std::find(assignment.begin(), assignment.end(), route) != assignment.end()) continue;
			assignment.push_back(route);
			visit();
			assignment.pop_back();
		}
	};
	visit();
}

std::string proposalKey(const std::vector<int>& routes, const std::vector<double>& quantities)
{
	std::vector<std::pair<int, double>> pieces;
	for (size_t index = 0; index < routes.size(); ++index) pieces.push_back({ routes[index], quantities[index] });
	std::sort(pieces.begin(), pieces.end());
	std::ostringstream out;
	out << std::fixed << std::setprecision(6);
	for (const auto& piece : pieces) out << piece.first << ':' << piece.second << ';';
	return out.str();
}

std::vector<AllocationProposal> allocationProposals(const Individual& base, int customer,
	const std::vector<int>& routePool, const std::vector<std::vector<double>>& patterns,
	const Params& params, int proposalLimit)
{
	std::vector<AllocationProposal> proposals;
	std::set<std::string> keys;
	for (const std::vector<double>& quantities : patterns)
		enumerateRouteAssignments(routePool, static_cast<int>(quantities.size()), [&](const std::vector<int>& routes) {
			const std::string key = proposalKey(routes, quantities);
			if (!keys.insert(key).second) return;
			double estimate = 0.0;
			for (size_t index = 0; index < routes.size(); ++index) {
				const double residual = params.vehicleCapacity - routeLoad(base.routesDynamic[routes[index]]);
				if (quantities[index] > residual + MY_EPSILON) return;
				estimate += minimumInsertionDelta(base.routesDynamic[routes[index]], customer, params);
				estimate += 0.01 * std::fabs(residual - quantities[index]);
			}
			proposals.push_back({ quantities, routes, estimate });
		});
	std::sort(proposals.begin(), proposals.end(), [](const AllocationProposal& first, const AllocationProposal& second) {
		return first.estimatedDelta < second.estimatedDelta;
	});
	if (static_cast<int>(proposals.size()) > proposalLimit) proposals.resize(std::max(1, proposalLimit));
	return proposals;
}

JointCandidateType candidateType(const AllocationProposal& allocationA, const AllocationProposal& allocationB)
{
	const bool splitA = allocationA.quantities.size() >= 2;
	const bool splitB = allocationB.quantities.size() >= 2;
	if (splitA && splitB) return JointCandidateType::JOINT_BOTH_SPLIT;
	if (splitA || splitB) return JointCandidateType::JOINT_ONE_SPLIT;
	return JointCandidateType::JOINT_RELOCATE;
}

bool combinedCapacityFeasible(const Individual& base, const AllocationProposal& allocationA,
	const AllocationProposal& allocationB, const Params& params)
{
	std::map<int, double> added;
	for (size_t index = 0; index < allocationA.routes.size(); ++index)
		added[allocationA.routes[index]] += allocationA.quantities[index];
	for (size_t index = 0; index < allocationB.routes.size(); ++index)
		added[allocationB.routes[index]] += allocationB.quantities[index];
	for (const auto& route : added)
		if (routeLoad(base.routesDynamic[route.first]) + route.second > params.vehicleCapacity + MY_EPSILON)
			return false;
	return true;
}

bool insertAllocation(Individual& candidate, int customer, const AllocationProposal& proposal,
	const Params& params)
{
	for (size_t piece = 0; piece < proposal.routes.size(); ++piece) {
		const int route = proposal.routes[piece];
		if (routeContains(candidate.routesDynamic[route], customer)) return false;
		int bestPosition = -1;
		double bestDelta = std::numeric_limits<double>::max();
		for (int position = 0; position <= static_cast<int>(candidate.routesDynamic[route].size()); ++position) {
			std::vector<RouteVisit> trialRoute = candidate.routesDynamic[route];
			trialRoute.insert(trialRoute.begin() + position, { customer, proposal.quantities[piece] });
			const double delta = insertionDelta(candidate.routesDynamic[route], position, customer, params);
			if (routeTimeWindowFeasible(trialRoute, params) && routeConflictFree(trialRoute, params) && delta < bestDelta) {
				bestDelta = delta;
				bestPosition = position;
			}
		}
		if (bestPosition < 0) return false;
		candidate.routesDynamic[route].insert(candidate.routesDynamic[route].begin() + bestPosition,
			{ customer, proposal.quantities[piece] });
	}
	return true;
}

double routeDistance(const std::vector<RouteVisit>& route, const Params& params)
{
	double distance = 0.0;
	int previous = 0;
	for (const RouteVisit& visit : route) {
		distance += params.originalTimeCost[previous][visit.originalCustomerId];
		previous = visit.originalCustomerId;
	}
	return distance + params.originalTimeCost[previous][0];
}

bool routeStructurallyFeasible(const std::vector<RouteVisit>& route, const Params& params)
{
	double load = 0.0;
	std::set<int> customers;
	for (const RouteVisit& visit : route) {
		if (visit.quantity <= MY_EPSILON || !customers.insert(visit.originalCustomerId).second) return false;
		load += visit.quantity;
	}
	return load <= params.vehicleCapacity + MY_EPSILON && routeConflictFree(route, params) &&
		routeTimeWindowFeasible(route, params);
}

std::string routeSequenceKey(const std::vector<RouteVisit>& route)
{
	std::ostringstream out;
	for (const RouteVisit& visit : route) out << visit.originalCustomerId << ':' << quantityText(visit.quantity) << ';';
	return out.str();
}

bool optimizeInsertionsOnRoute(const std::vector<RouteVisit>& baseRoute,
	const std::vector<RouteVisit>& additions, JointInsertionMode mode, int topK,
	const Params& params, std::vector<RouteVisit>& bestRoute,
	long long& evaluated, long long& feasible)
{
	if (additions.empty()) { bestRoute = baseRoute; return true; }
	double bestDistance = std::numeric_limits<double>::max();
	std::set<std::string> seen;
	auto evaluate = [&](const std::vector<RouteVisit>& route) {
		if (!seen.insert(routeSequenceKey(route)).second) return;
		++evaluated;
		if (!routeStructurallyFeasible(route, params)) return;
		++feasible;
		const double distance = routeDistance(route, params);
		if (distance < bestDistance - MY_EPSILON) {
			bestDistance = distance;
			bestRoute = route;
		}
	};

	auto enumerateOrder = [&](std::vector<RouteVisit> orderedAdditions, bool limited) {
		std::function<void(int, const std::vector<RouteVisit>&)> insertNext;
		insertNext = [&](int index, const std::vector<RouteVisit>& route) {
			if (index == static_cast<int>(orderedAdditions.size())) { evaluate(route); return; }
			std::vector<std::pair<double, int>> positions;
			for (int position = 0; position <= static_cast<int>(route.size()); ++position)
				positions.push_back({ insertionDelta(route, position,
					orderedAdditions[index].originalCustomerId, params), position });
			std::sort(positions.begin(), positions.end());
			const int count = limited ? std::min<int>(std::max(1, topK), positions.size())
				: static_cast<int>(positions.size());
			for (int candidate = 0; candidate < count; ++candidate) {
				std::vector<RouteVisit> next = route;
				next.insert(next.begin() + positions[candidate].second, orderedAdditions[index]);
				insertNext(index + 1, next);
			}
		};
		insertNext(0, baseRoute);
	};

	if (mode == JointInsertionMode::EXHAUSTIVE_FEASIBLE_INSERTION) enumerateOrder(additions, false);
	else {
		enumerateOrder(additions, true);
		if (additions.size() == 2) {
			std::vector<RouteVisit> reversed = additions;
			std::reverse(reversed.begin(), reversed.end());
			enumerateOrder(reversed, true);
		}
	}
	return bestDistance < std::numeric_limits<double>::max();
}

bool buildJointInsertionCandidate(const Individual& base, int customerA, int customerB,
	const AllocationProposal& allocationA, const AllocationProposal& allocationB,
	JointInsertionMode mode, int topK, const Params& params, Individual& candidate,
	long long& evaluated, long long& feasible)
{
	candidate = base;
	std::map<int, std::vector<RouteVisit>> additions;
	for (size_t piece = 0; piece < allocationA.routes.size(); ++piece)
		additions[allocationA.routes[piece]].push_back({ customerA, allocationA.quantities[piece] });
	for (size_t piece = 0; piece < allocationB.routes.size(); ++piece)
		additions[allocationB.routes[piece]].push_back({ customerB, allocationB.quantities[piece] });
	for (const auto& routeAdditions : additions) {
		std::vector<RouteVisit> optimized;
		if (!optimizeInsertionsOnRoute(base.routesDynamic[routeAdditions.first], routeAdditions.second,
			mode, topK, params, optimized, evaluated, feasible)) return false;
		candidate.routesDynamic[routeAdditions.first] = std::move(optimized);
	}
	return true;
}

std::vector<int> jointRoutePool(const Individual& source, const Individual& base, int customerA,
	int customerB, int limit, const Params& params)
{
	limit = std::max(2, limit);
	std::set<int> current;
	for (int route = 0; route < static_cast<int>(source.routesDynamic.size()); ++route)
		if (routeContains(source.routesDynamic[route], customerA) || routeContains(source.routesDynamic[route], customerB))
			current.insert(route);
	std::vector<std::pair<double, int>> ranked;
	int emptyRoute = -1;
	for (int route = 0; route < static_cast<int>(base.routesDynamic.size()); ++route) {
		if (base.routesDynamic[route].empty()) {
			if (emptyRoute < 0 && !current.count(route)) emptyRoute = route;
			continue;
		}
		const double residual = params.vehicleCapacity - routeLoad(base.routesDynamic[route]);
		const double insertion = std::min(minimumInsertionDelta(base.routesDynamic[route], customerA, params),
			minimumInsertionDelta(base.routesDynamic[route], customerB, params));
		const double capacityMismatch = std::min(std::fabs(residual - params.originalDemand[customerA]),
			std::fabs(residual - params.originalDemand[customerB]));
		ranked.push_back({ insertion + 0.05 * capacityMismatch, route });
	}
	std::sort(ranked.begin(), ranked.end());
	std::vector<int> pool(current.begin(), current.end());
	const int existingTarget = std::max(static_cast<int>(pool.size()), limit - (emptyRoute >= 0 ? 1 : 0));
	for (const auto& item : ranked) {
		if (static_cast<int>(pool.size()) >= existingTarget) break;
		if (std::find(pool.begin(), pool.end(), item.second) == pool.end()) pool.push_back(item.second);
	}
	if (emptyRoute >= 0 && static_cast<int>(pool.size()) < limit) pool.push_back(emptyRoute);
	if (static_cast<int>(pool.size()) > limit) pool.resize(limit);
	return pool;
}

JointResplitReport makeReport(const Individual& source, const Individual& candidate, int customerA,
	int customerB, JointCandidateType type, const std::vector<int>& routePool, const Params& params)
{
	JointResplitReport report;
	report.customerA = customerA;
	report.customerB = customerB;
	report.demandA = params.originalDemand[customerA];
	report.demandB = params.originalDemand[customerB];
	report.oldA = jointAllocationText(source, customerA);
	report.newA = jointAllocationText(candidate, customerA);
	report.oldB = jointAllocationText(source, customerB);
	report.newB = jointAllocationText(candidate, customerB);
	report.routesBefore = jointCustomerRoutesText(source, customerA, customerB);
	report.routesAfter = jointCustomerRoutesText(candidate, customerA, customerB);
	report.routePool = routePool;
	report.type = type;
	report.sourceDistance = source.eval.distance;
	report.candidateDistance = candidate.eval.distance;
	report.feasible = true;
	report.improving = candidate.eval.distance < source.eval.distance - MY_EPSILON;
	report.reason = "validator PASS";
	return report;
}

double pairScore(const Individual& source, const Params& params, int customerA, int customerB,
	const std::vector<double>& deltas)
{
	double routeDistance = std::numeric_limits<double>::max();
	double lowLoadBonus = 0.0;
	for (const auto& routeA : source.routesDynamic) if (routeContains(routeA, customerA)) {
		lowLoadBonus += (params.vehicleCapacity - routeLoad(routeA)) / params.vehicleCapacity;
		for (const auto& routeB : source.routesDynamic) if (routeContains(routeB, customerB))
			for (const RouteVisit& visitA : routeA) for (const RouteVisit& visitB : routeB)
				routeDistance = std::min(routeDistance,
					params.originalTimeCost[visitA.originalCustomerId][visitB.originalCustomerId]);
	}
	for (const auto& routeB : source.routesDynamic) if (routeContains(routeB, customerB))
		lowLoadBonus += (params.vehicleCapacity - routeLoad(routeB)) / params.vehicleCapacity;
	if (!std::isfinite(routeDistance)) routeDistance = params.originalTimeCost[customerA][customerB];
	const double geometry = params.originalTimeCost[customerA][customerB];
	const double demandCompatibility = std::fabs(params.originalDemand[customerA] - params.originalDemand[customerB]);
	double score = geometry + 0.35 * routeDistance + 0.03 * demandCompatibility - 5.0 * lowLoadBonus;
	if (customerA < static_cast<int>(deltas.size()) && std::isfinite(deltas[customerA])) score += 0.25 * deltas[customerA];
	if (customerB < static_cast<int>(deltas.size()) && std::isfinite(deltas[customerB])) score += 0.25 * deltas[customerB];
	return score;
}
}

const char* jointCandidateTypeName(JointCandidateType type)
{
	if (type == JointCandidateType::JOINT_ONE_SPLIT) return "JOINT_ONE_SPLIT";
	if (type == JointCandidateType::JOINT_BOTH_SPLIT) return "JOINT_BOTH_SPLIT";
	return "JOINT_RELOCATE";
}

const char* jointInsertionModeName(JointInsertionMode mode)
{
	if (mode == JointInsertionMode::JOINT_INSERTION_COMBINATION) return "JOINT_INSERTION_COMBINATION";
	if (mode == JointInsertionMode::EXHAUSTIVE_FEASIBLE_INSERTION) return "EXHAUSTIVE_FEASIBLE_INSERTION";
	return "GREEDY_INSERTION";
}

std::string jointAllocationText(const Individual& individual, int customer)
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

std::string jointCustomerRoutesText(const Individual& individual, int customerA, int customerB)
{
	std::ostringstream out;
	bool first = true;
	for (size_t route = 0; route < individual.routesDynamic.size(); ++route)
		if (routeContains(individual.routesDynamic[route], customerA) ||
			routeContains(individual.routesDynamic[route], customerB)) {
			if (!first) out << ';';
			out << 'R' << route + 1;
			first = false;
		}
	return first ? "none" : out.str();
}

void JointEnumerationStats::add(const JointEnumerationStats& other)
{
	generatedCandidates += other.generatedCandidates;
	feasibleCandidates += other.feasibleCandidates;
	generatedSplitCandidates += other.generatedSplitCandidates;
	feasibleSplitCandidates += other.feasibleSplitCandidates;
	generatedImprovingCandidates += other.generatedImprovingCandidates;
	generatedImprovingSplitCandidates += other.generatedImprovingSplitCandidates;
	bothCustomersSplitCandidates += other.bothCustomersSplitCandidates;
	oneSplitImprovingCandidates += other.oneSplitImprovingCandidates;
	bothSplitImprovingCandidates += other.bothSplitImprovingCandidates;
	relocateOnlyImprovingCandidates += other.relocateOnlyImprovingCandidates;
	insertionPlacementsEvaluated += other.insertionPlacementsEvaluated;
	feasibleInsertionPlacements += other.feasibleInsertionPlacements;
	nearNeutralSplitCandidates += other.nearNeutralSplitCandidates;
	splitDeltaLE0 += other.splitDeltaLE0;
	splitDelta0To1 += other.splitDelta0To1;
	splitDelta1To3 += other.splitDelta1To3;
	splitDelta3To5 += other.splitDelta3To5;
	splitDelta5To10 += other.splitDelta5To10;
	splitDeltaAbove10 += other.splitDeltaAbove10;
}

std::vector<JointCustomerPair> selectJointCustomerPairs(const Individual& source, const Params& params,
	const std::vector<double>& singleBestSplitDelta, int maxPairs, double anchorGap)
{
	std::vector<int> eligible;
	for (int customer = 1; customer < static_cast<int>(params.originalDemand.size()); ++customer)
		if (params.originalDemand[customer] > MY_EPSILON) eligible.push_back(customer);
	std::vector<int> anchors;
	for (int customer : eligible)
		if (customer < static_cast<int>(singleBestSplitDelta.size()) &&
			singleBestSplitDelta[customer] <= anchorGap + MY_EPSILON) anchors.push_back(customer);
	for (int priority : { 21, 42 })
		if (std::find(eligible.begin(), eligible.end(), priority) != eligible.end() &&
			std::find(anchors.begin(), anchors.end(), priority) == anchors.end()) anchors.push_back(priority);
	if (anchors.empty()) {
		std::vector<std::pair<double, int>> ranked;
		for (int customer : eligible) ranked.push_back({ singleBestSplitDelta[customer], customer });
		std::sort(ranked.begin(), ranked.end());
		for (int index = 0; index < std::min<int>(5, ranked.size()); ++index) anchors.push_back(ranked[index].second);
	}

	std::vector<JointCustomerPair> all;
	std::set<std::pair<int, int>> seen;
	for (int anchor : anchors) for (int partner : eligible) {
		if (anchor == partner) continue;
		const int first = std::min(anchor, partner);
		const int second = std::max(anchor, partner);
		if (!seen.insert({ first, second }).second) continue;
		all.push_back({ first, second, pairScore(source, params, first, second, singleBestSplitDelta) });
	}
	std::sort(all.begin(), all.end(), [](const JointCustomerPair& first, const JointCustomerPair& second) {
		return first.score != second.score ? first.score < second.score
			: std::make_pair(first.customerA, first.customerB) < std::make_pair(second.customerA, second.customerB);
	});
	const int limit = std::max(0, maxPairs);
	std::vector<JointCustomerPair> selected;
	if (limit == 0) return selected;
	std::set<std::pair<int, int>> selectedKeys;
	// Give every near-neutral anchor at least one geometry/route-based partner
	// before filling the remaining bounded pool by global score.
	for (int anchor : anchors) {
		auto best = std::find_if(all.begin(), all.end(), [anchor](const JointCustomerPair& pair) {
			return pair.customerA == anchor || pair.customerB == anchor;
		});
		if (best == all.end()) continue;
		const std::pair<int, int> key = { best->customerA, best->customerB };
		if (selectedKeys.insert(key).second) selected.push_back(*best);
		if (static_cast<int>(selected.size()) >= limit) return selected;
	}
	std::vector<JointCustomerPair> fillOrder = all;
	std::sort(fillOrder.begin(), fillOrder.end(), [](const JointCustomerPair& first, const JointCustomerPair& second) {
		const bool firstPriority = first.customerA == 21 || first.customerA == 42 ||
			first.customerB == 21 || first.customerB == 42;
		const bool secondPriority = second.customerA == 21 || second.customerA == 42 ||
			second.customerB == 21 || second.customerB == 42;
		if (firstPriority != secondPriority) return firstPriority;
		return first.score != second.score ? first.score < second.score
			: std::make_pair(first.customerA, first.customerB) < std::make_pair(second.customerA, second.customerB);
	});
	for (const JointCustomerPair& pair : fillOrder) {
		const std::pair<int, int> key = { pair.customerA, pair.customerB };
		if (!selectedKeys.insert(key).second) continue;
		selected.push_back(pair);
		if (static_cast<int>(selected.size()) >= limit) break;
	}
	return selected;
}

JointResplitResult generateJointResplitCandidate(const Individual& source, Params& params,
	int customerA, int customerB, const JointResplitConfig& config)
{
	JointResplitResult result;
	if (config.enforceGlobalDeadline && params.deadlineReached()) return result;
	if (source.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY || customerA == customerB) {
		result.overallReport.reason = result.splitReport.reason = "invalid dynamic source or customer pair";
		return result;
	}
	const ValidationResult sourceValidation = validateSolutionFromScratch(source, params);
	if (!sourceValidation.feasible()) {
		result.overallReport.reason = result.splitReport.reason = "invalid source: " + sourceValidation.message;
		return result;
	}
	if (customerA <= 0 || customerB <= 0 || customerA >= static_cast<int>(params.originalDemand.size()) ||
		customerB >= static_cast<int>(params.originalDemand.size())) {
		result.overallReport.reason = result.splitReport.reason = "customer outside original-demand range";
		return result;
	}

	Individual base = source;
	for (auto& route : base.routesDynamic)
		route.erase(std::remove_if(route.begin(), route.end(), [&](const RouteVisit& visit) {
			return visit.originalCustomerId == customerA || visit.originalCustomerId == customerB;
		}), route.end());
	const std::vector<int> routePool = jointRoutePool(source, base, customerA, customerB,
		config.candidateRoutes, params);
	if (routePool.size() < 2) {
		result.overallReport.reason = result.splitReport.reason = "insufficient joint route pool";
		return result;
	}
	const std::vector<int> patternRoutePool = config.patternReferenceRoutes > 0
		? jointRoutePool(source, base, customerA, customerB, config.patternReferenceRoutes, params)
		: routePool;
	const auto patternsA = quantityPatterns(source, base, customerA, patternRoutePool, params, config);
	const auto patternsB = quantityPatterns(source, base, customerB, patternRoutePool, params, config);
	const int proposalLimit = config.maxPatternsPerCustomer > 8 ? 200 : 80;
	const auto proposalsA = allocationProposals(base, customerA, routePool, patternsA, params, proposalLimit);
	const auto proposalsB = allocationProposals(base, customerB, routePool, patternsB, params, proposalLimit);

	std::vector<JointPattern> combined;
	for (const AllocationProposal& allocationA : proposalsA)
		for (const AllocationProposal& allocationB : proposalsB) {
			if (!combinedCapacityFeasible(base, allocationA, allocationB, params)) continue;
			combined.push_back({ allocationA, allocationB,
				allocationA.estimatedDelta + allocationB.estimatedDelta,
				candidateType(allocationA, allocationB) });
		}
	std::sort(combined.begin(), combined.end(), [](const JointPattern& first, const JointPattern& second) {
		return first.estimatedDelta < second.estimatedDelta;
	});
	std::vector<JointPattern> pruned;
	std::set<std::string> jointKeys;
	const int targetPatterns = std::max(1, config.maxJointPatterns);
	auto retain = [&](bool requireSplit, int target) {
		for (const JointPattern& pattern : combined) {
			if ((pattern.type != JointCandidateType::JOINT_RELOCATE) != requireSplit) continue;
			const std::string key = proposalKey(pattern.allocationA.routes, pattern.allocationA.quantities)
				+ "|" + proposalKey(pattern.allocationB.routes, pattern.allocationB.quantities);
			if (!jointKeys.insert(key).second) continue;
			pruned.push_back(pattern);
			if (static_cast<int>(pruned.size()) >= target) break;
		}
	};
	retain(false, std::max(1, targetPatterns / 3));
	retain(true, targetPatterns);
	if (static_cast<int>(pruned.size()) < targetPatterns) {
		for (const JointPattern& pattern : combined) {
			const std::string key = proposalKey(pattern.allocationA.routes, pattern.allocationA.quantities)
				+ "|" + proposalKey(pattern.allocationB.routes, pattern.allocationB.quantities);
			if (!jointKeys.insert(key).second) continue;
			pruned.push_back(pattern);
			if (static_cast<int>(pruned.size()) >= targetPatterns) break;
		}
	}

	int candidateBudget = std::max(1, config.maxCandidatesPerPair);
	const int insertionOrders = config.insertionMode == JointInsertionMode::GREEDY_INSERTION ? 2 : 1;
	for (const JointPattern& pattern : pruned) for (int insertionOrder = 0; insertionOrder < insertionOrders; ++insertionOrder) {
		if (config.enforceGlobalDeadline && params.deadlineReached()) return result;
		if (result.stats.generatedCandidates >= candidateBudget) break;
		++result.stats.generatedCandidates;
		const bool splitCandidate = pattern.type != JointCandidateType::JOINT_RELOCATE;
		if (splitCandidate) ++result.stats.generatedSplitCandidates;
		if (pattern.type == JointCandidateType::JOINT_BOTH_SPLIT) ++result.stats.bothCustomersSplitCandidates;
		Individual candidate = base;
		bool inserted = false;
		if (config.insertionMode == JointInsertionMode::GREEDY_INSERTION) {
			++result.stats.insertionPlacementsEvaluated;
			inserted = insertionOrder == 0
				? insertAllocation(candidate, customerA, pattern.allocationA, params) &&
				  insertAllocation(candidate, customerB, pattern.allocationB, params)
				: insertAllocation(candidate, customerB, pattern.allocationB, params) &&
				  insertAllocation(candidate, customerA, pattern.allocationA, params);
			if (inserted) ++result.stats.feasibleInsertionPlacements;
		}
		else inserted = buildJointInsertionCandidate(base, customerA, customerB,
			pattern.allocationA, pattern.allocationB, config.insertionMode,
			config.topInsertionPositions, params, candidate,
			result.stats.insertionPlacementsEvaluated, result.stats.feasibleInsertionPlacements);
		if (!inserted) continue;
		candidate.quantityRepresentation = QuantityRepresentation::DYNAMIC_QUANTITY;
		candidate.evaluateDynamicQuantity(params);
		const ValidationResult validation = validateSolutionFromScratch(candidate, params);
		if (!validation.feasible()) continue;
		++result.stats.feasibleCandidates;
		if (splitCandidate) {
			++result.stats.feasibleSplitCandidates;
			const double splitDelta = candidate.eval.distance - source.eval.distance;
			if (splitDelta <= 10.0 + MY_EPSILON) ++result.stats.nearNeutralSplitCandidates;
			if (splitDelta <= MY_EPSILON) ++result.stats.splitDeltaLE0;
			else if (splitDelta <= 1.0 + MY_EPSILON) ++result.stats.splitDelta0To1;
			else if (splitDelta <= 3.0 + MY_EPSILON) ++result.stats.splitDelta1To3;
			else if (splitDelta <= 5.0 + MY_EPSILON) ++result.stats.splitDelta3To5;
			else if (splitDelta <= 10.0 + MY_EPSILON) ++result.stats.splitDelta5To10;
			else ++result.stats.splitDeltaAbove10;
		}
		const bool improving = candidate.eval.distance < source.eval.distance - MY_EPSILON;
		if (splitCandidate && config.collectSplitCandidatesUpToDelta >= 0.0 &&
			candidate.eval.distance - source.eval.distance <= config.collectSplitCandidatesUpToDelta + MY_EPSILON) {
			const JointResplitReport collectedReport = makeReport(source, candidate, customerA, customerB,
				pattern.type, routePool, params);
			result.collectedSplitCandidates.emplace_back(candidate, collectedReport);
		}
		if (improving) {
			++result.stats.generatedImprovingCandidates;
			if (splitCandidate) ++result.stats.generatedImprovingSplitCandidates;
			if (pattern.type == JointCandidateType::JOINT_ONE_SPLIT) ++result.stats.oneSplitImprovingCandidates;
			else if (pattern.type == JointCandidateType::JOINT_BOTH_SPLIT) ++result.stats.bothSplitImprovingCandidates;
			else ++result.stats.relocateOnlyImprovingCandidates;
		}
		if (!result.bestOverall || candidate.eval.distance < result.bestOverall->eval.distance - MY_EPSILON) {
			result.bestOverall = candidate;
			result.overallReport = makeReport(source, candidate, customerA, customerB, pattern.type, routePool, params);
		}
		if (splitCandidate && (!result.bestSplit || candidate.eval.distance < result.bestSplit->eval.distance - MY_EPSILON)) {
			result.bestSplit = candidate;
			result.splitReport = makeReport(source, candidate, customerA, customerB, pattern.type, routePool, params);
		}
	}
	if (!result.bestOverall) result.overallReport.reason = "no feasible joint candidate";
	if (!result.bestSplit) result.splitReport.reason = "no feasible joint split candidate";
	return result;
}
