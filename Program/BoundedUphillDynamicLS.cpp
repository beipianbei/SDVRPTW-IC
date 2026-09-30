#include "BoundedUphillDynamicLS.h"
#include "DynamicLocalImprove.h"
#include "SolutionValidator.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

namespace {
struct BridgeNode
{
	Individual state;
	std::vector<std::string> moves;
	std::vector<double> deltas;
	double peakDistance = 0.0;

	BridgeNode(const Individual& state, std::vector<std::string> moves,
		std::vector<double> deltas, double peakDistance)
		: state(state), moves(std::move(moves)), deltas(std::move(deltas)), peakDistance(peakDistance) {}
};

bool routeContains(const std::vector<RouteVisit>& route, int customer)
{
	return std::any_of(route.begin(), route.end(), [customer](const RouteVisit& visit) {
		return visit.originalCustomerId == customer;
	});
}

bool touchesAffected(int first, int second, const std::vector<int>& affected)
{
	return std::find(affected.begin(), affected.end(), first) != affected.end() ||
		std::find(affected.begin(), affected.end(), second) != affected.end();
}

std::vector<int> affectedRoutes(const Individual& source, const Individual& split,
	const std::vector<int>& customers)
{
	std::set<int> routes;
	for (int route = 0; route < static_cast<int>(split.routesDynamic.size()); ++route)
		for (int customer : customers) {
			const bool oldRoute = route < static_cast<int>(source.routesDynamic.size()) &&
				routeContains(source.routesDynamic[route], customer);
			const bool newRoute = routeContains(split.routesDynamic[route], customer);
			if (oldRoute || newRoute) { routes.insert(route); break; }
		}
	return std::vector<int>(routes.begin(), routes.end());
}

double routeProximity(const std::vector<RouteVisit>& first, const std::vector<RouteVisit>& second,
	const Params& params)
{
	double proximity = std::numeric_limits<double>::max();
	for (const RouteVisit& left : first) for (const RouteVisit& right : second)
		proximity = std::min(proximity,
			params.originalTimeCost[left.originalCustomerId][right.originalCustomerId]);
	return proximity;
}

std::vector<int> searchRoutes(const Individual& split, const std::vector<int>& affected,
	int nearbyCount, const Params& params)
{
	std::set<int> selected(affected.begin(), affected.end());
	std::vector<std::pair<double, int>> nearby;
	for (int route = 0; route < static_cast<int>(split.routesDynamic.size()); ++route) {
		if (selected.count(route) || split.routesDynamic[route].empty()) continue;
		double proximity = std::numeric_limits<double>::max();
		for (int affectedRoute : affected)
			if (!split.routesDynamic[affectedRoute].empty())
				proximity = std::min(proximity, routeProximity(split.routesDynamic[affectedRoute],
					split.routesDynamic[route], params));
		nearby.push_back({ proximity, route });
	}
	std::sort(nearby.begin(), nearby.end());
	for (int index = 0; index < std::min<int>(std::max(0, nearbyCount), nearby.size()); ++index)
		selected.insert(nearby[index].second);
	return std::vector<int>(selected.begin(), selected.end());
}

std::string solutionKey(const Individual& individual)
{
	std::ostringstream out;
	out << std::fixed << std::setprecision(6);
	for (const auto& route : individual.routesDynamic) {
		if (route.empty()) continue;
		out << '[';
		for (const RouteVisit& visit : route)
			out << visit.originalCustomerId << ':' << visit.quantity << ',';
		out << ']';
	}
	return out.str();
}

int splitCustomerCount(const Individual& individual, const std::vector<int>& customers)
{
	int splitCount = 0;
	for (int customer : customers) {
		int routes = 0;
		for (const auto& route : individual.routesDynamic)
			if (routeContains(route, customer)) ++routes;
		if (routes >= 2) ++splitCount;
	}
	return splitCount;
}

std::vector<BridgeNode> generateUphillSuccessors(const BridgeNode& parent,
	double splitStartDistance, const std::vector<int>& affected,
	const std::vector<int>& routes, Params& params,
	const BoundedUphillDynamicLSConfig& config, DynamicBridgeStats& stats)
{
	std::vector<BridgeNode> successors;
	std::set<std::string> seen;
	auto consider = [&](Individual&& trial, const std::string& description) {
		if (config.enforceGlobalDeadline && params.deadlineReached()) return;
		trial.quantityRepresentation = QuantityRepresentation::DYNAMIC_QUANTITY;
		trial.evaluateDynamicQuantity(params);
		const double stepDelta = trial.eval.distance - parent.state.eval.distance;
		if (stepDelta <= MY_EPSILON || stepDelta > config.maxSingleDelta + MY_EPSILON) return;
		if (trial.eval.distance - splitStartDistance > config.maxCumulativeDelta + MY_EPSILON) return;
		const ValidationResult validation = validateSolutionFromScratch(trial, params);
		if (!validation.feasible()) return;
		const std::string key = solutionKey(trial);
		if (!seen.insert(key).second) return;
		++stats.bridgeTrajectoriesGenerated;
		std::vector<std::string> moves = parent.moves;
		std::vector<double> deltas = parent.deltas;
		moves.push_back(description);
		deltas.push_back(stepDelta);
		successors.emplace_back(trial, std::move(moves), std::move(deltas),
			std::max(parent.peakDistance, trial.eval.distance));
	};

	// Swap first: diagnostics showed that it has the larger feasible neighborhood.
	for (size_t firstIndex = 0; firstIndex < routes.size(); ++firstIndex) {
		if (config.enforceGlobalDeadline && params.deadlineReached()) return successors;
		const int firstRoute = routes[firstIndex];
		for (size_t secondIndex = firstIndex; secondIndex < routes.size(); ++secondIndex) {
			const int secondRoute = routes[secondIndex];
			if (!touchesAffected(firstRoute, secondRoute, affected)) continue;
			for (int first = 0; first < static_cast<int>(parent.state.routesDynamic[firstRoute].size()); ++first)
				for (int second = 0; second < static_cast<int>(parent.state.routesDynamic[secondRoute].size()); ++second) {
					if (firstRoute == secondRoute && first >= second) continue;
					const int firstCustomer = parent.state.routesDynamic[firstRoute][first].originalCustomerId;
					const int secondCustomer = parent.state.routesDynamic[secondRoute][second].originalCustomerId;
					if (firstRoute != secondRoute &&
						(routeContains(parent.state.routesDynamic[firstRoute], secondCustomer) ||
						 routeContains(parent.state.routesDynamic[secondRoute], firstCustomer))) continue;
					Individual trial = parent.state;
					std::swap(trial.routesDynamic[firstRoute][first], trial.routesDynamic[secondRoute][second]);
					std::ostringstream move;
					move << "Swap(R" << firstRoute + 1 << ':' << first + 1
						<< ",R" << secondRoute + 1 << ':' << second + 1 << ')';
					consider(std::move(trial), move.str());
				}
		}
	}

	// Cross-route relocate second, followed by the two nearly-empty feasible neighborhoods.
	for (int fromRoute : routes) for (int toRoute : routes) {
		if (config.enforceGlobalDeadline && params.deadlineReached()) return successors;
		if (fromRoute == toRoute || !touchesAffected(fromRoute, toRoute, affected)) continue;
		for (int from = 0; from < static_cast<int>(parent.state.routesDynamic[fromRoute].size()); ++from) {
			const RouteVisit moved = parent.state.routesDynamic[fromRoute][from];
			if (routeContains(parent.state.routesDynamic[toRoute], moved.originalCustomerId)) continue;
			for (int to = 0; to <= static_cast<int>(parent.state.routesDynamic[toRoute].size()); ++to) {
				Individual trial = parent.state;
				trial.routesDynamic[fromRoute].erase(trial.routesDynamic[fromRoute].begin() + from);
				trial.routesDynamic[toRoute].insert(trial.routesDynamic[toRoute].begin() + to, moved);
				std::ostringstream move;
				move << "CrossRelocate(" << moved.originalCustomerId << ",R" << fromRoute + 1
					<< "->R" << toRoute + 1 << ':' << to + 1 << ')';
				consider(std::move(trial), move.str());
			}
		}
	}
	for (int route : affected) {
		if (config.enforceGlobalDeadline && params.deadlineReached()) return successors;
		const int size = static_cast<int>(parent.state.routesDynamic[route].size());
		for (int from = 0; from < size; ++from) for (int to = 0; to < size; ++to) {
			if (from == to) continue;
			Individual trial = parent.state;
			const RouteVisit moved = trial.routesDynamic[route][from];
			trial.routesDynamic[route].erase(trial.routesDynamic[route].begin() + from);
			trial.routesDynamic[route].insert(trial.routesDynamic[route].begin() + to, moved);
			std::ostringstream move;
			move << "Relocate(R" << route + 1 << ':' << from + 1 << "->" << to + 1 << ')';
			consider(std::move(trial), move.str());
		}
		for (int first = 0; first + 1 < size; ++first) for (int last = first + 1; last < size; ++last) {
			Individual trial = parent.state;
			std::reverse(trial.routesDynamic[route].begin() + first,
				trial.routesDynamic[route].begin() + last + 1);
			std::ostringstream move;
			move << "TwoOpt(R" << route + 1 << ':' << first + 1 << '-' << last + 1 << ')';
			consider(std::move(trial), move.str());
		}
	}
	std::sort(successors.begin(), successors.end(), [&](const BridgeNode& first, const BridgeNode& second) {
		const double firstStep = first.deltas.back();
		const double secondStep = second.deltas.back();
		if (std::fabs(firstStep - secondStep) > MY_EPSILON) return firstStep < secondStep;
		return first.state.eval.distance < second.state.eval.distance;
	});
	return successors;
}
}

void DynamicBridgeStats::add(const DynamicBridgeStats& other)
{
	bridgeCandidatesTried += other.bridgeCandidatesTried;
	bridgeTrajectoriesGenerated += other.bridgeTrajectoriesGenerated;
	uphillMovesAcceptedForSearch += other.uphillMovesAcceptedForSearch;
	depth1Trajectories += other.depth1Trajectories;
	depth2Trajectories += other.depth2Trajectories;
	recoveryRuns += other.recoveryRuns;
	recoveryImprovingRuns += other.recoveryImprovingRuns;
	netImprovingTrajectories += other.netImprovingTrajectories;
	if (other.hasFinalDistance && (!hasFinalDistance || other.bestFinalDistance < bestFinalDistance)) {
		hasFinalDistance = true;
		bestFinalDistance = other.bestFinalDistance;
		bestPeakExcursion = other.bestPeakExcursion;
		bestNetImprovement = other.bestNetImprovement;
	}
}

DynamicBridgeResult runBoundedUphillDynamicLS(const Individual& source, const Individual& splitStart,
	const std::vector<int>& resplitCustomers, Params& params,
	const BoundedUphillDynamicLSConfig& config)
{
	DynamicBridgeResult result;
	result.stats.bridgeCandidatesTried = 1;
	result.sourceDistance = source.eval.distance;
	result.splitStartDistance = splitStart.eval.distance;
	if (source.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY ||
		splitStart.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY ||
		!validateSolutionFromScratch(splitStart, params).feasible() ||
		splitCustomerCount(splitStart, resplitCustomers) == 0) return result;
	const std::vector<int> affected = affectedRoutes(source, splitStart, resplitCustomers);
	const std::vector<int> routes = searchRoutes(splitStart, affected, config.nearbyRoutes, params);
	BridgeNode root(splitStart, {}, {}, splitStart.eval.distance);
	std::vector<BridgeNode> depth1 = generateUphillSuccessors(root, splitStart.eval.distance,
		affected, routes, params, config, result.stats);
	if (static_cast<int>(depth1.size()) > std::max(0, config.beamWidth))
		depth1.erase(depth1.begin() + std::max(0, config.beamWidth), depth1.end());
	result.stats.depth1Trajectories = depth1.size();
	result.stats.uphillMovesAcceptedForSearch += depth1.size();

	std::vector<BridgeNode> depth2;
	if (config.maxUphillMoves >= 2) {
		std::set<std::string> depth2Keys;
		for (const BridgeNode& parent : depth1) {
			std::vector<BridgeNode> children = generateUphillSuccessors(parent, splitStart.eval.distance,
				affected, routes, params, config, result.stats);
			for (BridgeNode& child : children)
				if (depth2Keys.insert(solutionKey(child.state)).second) depth2.push_back(std::move(child));
		}
		std::sort(depth2.begin(), depth2.end(), [&](const BridgeNode& first, const BridgeNode& second) {
			const double firstExcursion = first.state.eval.distance - splitStart.eval.distance;
			const double secondExcursion = second.state.eval.distance - splitStart.eval.distance;
			return std::fabs(firstExcursion - secondExcursion) > MY_EPSILON
				? firstExcursion < secondExcursion : first.deltas.back() < second.deltas.back();
		});
		if (static_cast<int>(depth2.size()) > std::max(0, config.beamWidth))
			depth2.erase(depth2.begin() + std::max(0, config.beamWidth), depth2.end());
		result.stats.depth2Trajectories = depth2.size();
		result.stats.uphillMovesAcceptedForSearch += depth2.size();
	}

	std::vector<const BridgeNode*> recoveries;
	for (const BridgeNode& node : depth1) recoveries.push_back(&node);
	for (const BridgeNode& node : depth2) recoveries.push_back(&node);
	DynamicLocalImproveConfig recoveryConfig;
	recoveryConfig.nearbyRoutes = config.nearbyRoutes;
	recoveryConfig.maxPasses = std::max(0, config.recoveryMoves);
	recoveryConfig.enforceGlobalDeadline = config.enforceGlobalDeadline;
	for (const BridgeNode* node : recoveries) {
		if (config.enforceGlobalDeadline && params.deadlineReached()) break;
		++result.stats.recoveryRuns;
		DynamicLocalImproveResult recovery = improveDynamicQuantityLocally(source, node->state,
			resplitCustomers, params, recoveryConfig);
		if (!recovery.improved || !recovery.validatorPassed) continue;
		const double finalDistance = recovery.improved->eval.distance;
		if (finalDistance < node->state.eval.distance - MY_EPSILON) ++result.stats.recoveryImprovingRuns;
		const int finalSplits = splitCustomerCount(*recovery.improved, resplitCustomers);
		const ValidationResult validation = validateSolutionFromScratch(*recovery.improved, params);
		if (!validation.feasible() || finalSplits == 0) continue;
		const double netImprovement = source.eval.distance - finalDistance;
		if (!result.stats.hasFinalDistance || finalDistance < result.stats.bestFinalDistance) {
			result.stats.hasFinalDistance = true;
			result.stats.bestFinalDistance = finalDistance;
			result.stats.bestNetImprovement = netImprovement;
			result.stats.bestPeakExcursion = node->peakDistance - source.eval.distance;
		}
		if (netImprovement <= MY_EPSILON) continue;
		++result.stats.netImprovingTrajectories;
		if (!result.bestImproving || finalDistance < result.bestImproving->eval.distance - MY_EPSILON) {
			result.bestImproving = *recovery.improved;
			result.uphillMove1 = node->moves.empty() ? "none" : node->moves[0];
			result.uphillMove2 = node->moves.size() < 2 ? "none" : node->moves[1];
			result.delta1 = node->deltas.empty() ? 0.0 : node->deltas[0];
			result.delta2 = node->deltas.size() < 2 ? 0.0 : node->deltas[1];
			result.peakDistance = node->peakDistance;
			result.finalDistance = finalDistance;
			result.recoveryMoveCount = recovery.acceptedMoves;
			result.finalSplitCustomers = finalSplits;
			result.validatorPassed = true;
		}
	}
	return result;
}
