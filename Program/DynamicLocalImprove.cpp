#include "DynamicLocalImprove.h"
#include "SolutionValidator.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace {
bool routeContains(const std::vector<RouteVisit>& route, int customer)
{
	return std::any_of(route.begin(), route.end(), [customer](const RouteVisit& visit) {
		return visit.originalCustomerId == customer;
	});
}

double routesProximity(const std::vector<RouteVisit>& first, const std::vector<RouteVisit>& second,
	const Params& params)
{
	double proximity = std::numeric_limits<double>::max();
	for (const RouteVisit& left : first)
		for (const RouteVisit& right : second)
			proximity = std::min(proximity,
				params.originalTimeCost[left.originalCustomerId][right.originalCustomerId]);
	return proximity;
}

std::vector<int> determineAffectedRoutes(const Individual& source, const Individual& candidate,
	const std::vector<int>& customers)
{
	std::set<int> affected;
	const int count = std::max(source.routesDynamic.size(), candidate.routesDynamic.size());
	for (int route = 0; route < count; ++route) {
		for (int customer : customers) {
			const bool inSource = route < static_cast<int>(source.routesDynamic.size()) &&
				routeContains(source.routesDynamic[route], customer);
			const bool inCandidate = route < static_cast<int>(candidate.routesDynamic.size()) &&
				routeContains(candidate.routesDynamic[route], customer);
			if (inSource || inCandidate) { affected.insert(route); break; }
		}
	}
	return std::vector<int>(affected.begin(), affected.end());
}

std::vector<int> determineSearchRoutes(const Individual& candidate,
	const std::vector<int>& affected, int nearbyCount, const Params& params)
{
	std::set<int> selected(affected.begin(), affected.end());
	std::vector<std::pair<double, int>> nearby;
	for (int route = 0; route < static_cast<int>(candidate.routesDynamic.size()); ++route) {
		if (selected.count(route) || candidate.routesDynamic[route].empty()) continue;
		double proximity = std::numeric_limits<double>::max();
		for (int affectedRoute : affected) {
			if (affectedRoute < 0 || affectedRoute >= static_cast<int>(candidate.routesDynamic.size()) ||
				candidate.routesDynamic[affectedRoute].empty()) continue;
			proximity = std::min(proximity, routesProximity(candidate.routesDynamic[affectedRoute],
				candidate.routesDynamic[route], params));
		}
		nearby.push_back({ proximity, route });
	}
	std::sort(nearby.begin(), nearby.end());
	for (int index = 0; index < std::min<int>(std::max(0, nearbyCount), nearby.size()); ++index)
		selected.insert(nearby[index].second);
	return std::vector<int>(selected.begin(), selected.end());
}

bool touchesAffected(int firstRoute, int secondRoute, const std::vector<int>& affected)
{
	return std::find(affected.begin(), affected.end(), firstRoute) != affected.end() ||
		std::find(affected.begin(), affected.end(), secondRoute) != affected.end();
}
}

DynamicLocalImproveResult improveDynamicQuantityLocally(
	const Individual& source,
	const Individual& splitCandidate,
	int resplitCustomer,
	Params& params,
	const DynamicLocalImproveConfig& config)
{
	return improveDynamicQuantityLocally(source, splitCandidate,
		std::vector<int>{ resplitCustomer }, params, config);
}

DynamicLocalImproveResult improveDynamicQuantityLocally(
	const Individual& source,
	const Individual& splitCandidate,
	const std::vector<int>& resplitCustomers,
	Params& params,
	const DynamicLocalImproveConfig& config)
{
	DynamicLocalImproveResult result;
	result.initialDistance = splitCandidate.eval.distance;
	result.finalDistance = splitCandidate.eval.distance;
	if (source.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY ||
		splitCandidate.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY) {
		result.reason = "DynamicLocalImprove requires DYNAMIC_QUANTITY inputs";
		return result;
	}
	const ValidationResult inputValidation = validateSolutionFromScratch(splitCandidate, params);
	if (!inputValidation.feasible()) {
		result.reason = "invalid split candidate: " + inputValidation.message;
		return result;
	}

	result.affectedRoutes = determineAffectedRoutes(source, splitCandidate, resplitCustomers);
	result.searchRoutes = determineSearchRoutes(splitCandidate, result.affectedRoutes,
		config.nearbyRoutes, params);
	if (result.affectedRoutes.empty() || result.searchRoutes.empty()) {
		result.reason = "no affected routes";
		return result;
	}

	Individual current = splitCandidate;
	for (int pass = 0; pass < std::max(0, config.maxPasses); ++pass) {
		if (config.enforceGlobalDeadline && params.deadlineReached()) break;
		std::optional<Individual> bestMove;
		std::string bestMoveType;
		double bestDistance = current.eval.distance;
		auto consider = [&](Individual&& trial, const char* moveType) {
			if (config.enforceGlobalDeadline && params.deadlineReached()) return;
			++result.evaluatedMoves;
			if (std::string(moveType) == "INTRA_RELOCATE") ++result.generatedRelocateMoves;
			else if (std::string(moveType) == "CROSS_ROUTE_RELOCATE") ++result.generatedCrossRouteRelocateMoves;
			else if (std::string(moveType) == "SWAP") ++result.generatedSwapMoves;
			else if (std::string(moveType) == "TWO_OPT") ++result.generated2OptMoves;
			trial.quantityRepresentation = QuantityRepresentation::DYNAMIC_QUANTITY;
			trial.evaluateDynamicQuantity(params);
			const ValidationResult validation = validateSolutionFromScratch(trial, params);
			if (!validation.feasible()) return;
			if (std::string(moveType) == "INTRA_RELOCATE") ++result.feasibleRelocateMoves;
			else if (std::string(moveType) == "CROSS_ROUTE_RELOCATE") ++result.feasibleCrossRouteRelocateMoves;
			else if (std::string(moveType) == "SWAP") ++result.feasibleSwapMoves;
			else if (std::string(moveType) == "TWO_OPT") ++result.feasible2OptMoves;
			const double moveDelta = trial.eval.distance - current.eval.distance;
			if (moveDelta > MY_EPSILON) {
				if (moveDelta <= 1.0 + MY_EPSILON) ++result.feasibleUphillMoves1;
				if (moveDelta <= 3.0 + MY_EPSILON) ++result.feasibleUphillMoves3;
				if (moveDelta <= 5.0 + MY_EPSILON) ++result.feasibleUphillMoves5;
			}
			if (moveDelta < -MY_EPSILON) {
				if (std::string(moveType) == "INTRA_RELOCATE") ++result.improvingRelocateMoves;
				else if (std::string(moveType) == "CROSS_ROUTE_RELOCATE") ++result.improvingCrossRouteRelocateMoves;
				else if (std::string(moveType) == "SWAP") ++result.improvingSwapMoves;
				else if (std::string(moveType) == "TWO_OPT") ++result.improving2OptMoves;
			}
			if (trial.eval.distance >= bestDistance - MY_EPSILON) return;
			bestDistance = trial.eval.distance;
			bestMove = std::move(trial);
			bestMoveType = moveType;
		};

		// Intra-route RouteVisit relocate.
		for (int route : result.searchRoutes) {
			if (std::find(result.affectedRoutes.begin(), result.affectedRoutes.end(), route) == result.affectedRoutes.end()) continue;
			const int size = static_cast<int>(current.routesDynamic[route].size());
			for (int from = 0; from < size; ++from) for (int to = 0; to < size; ++to) {
				if (from == to) continue;
				Individual trial = current;
				const RouteVisit moved = trial.routesDynamic[route][from];
				trial.routesDynamic[route].erase(trial.routesDynamic[route].begin() + from);
				trial.routesDynamic[route].insert(trial.routesDynamic[route].begin() + to, moved);
				consider(std::move(trial), "INTRA_RELOCATE");
			}
		}

		// Intra-route 2-opt.
		for (int route : result.searchRoutes) {
			if (std::find(result.affectedRoutes.begin(), result.affectedRoutes.end(), route) == result.affectedRoutes.end()) continue;
			const int size = static_cast<int>(current.routesDynamic[route].size());
			for (int first = 0; first + 1 < size; ++first) for (int last = first + 1; last < size; ++last) {
				Individual trial = current;
				std::reverse(trial.routesDynamic[route].begin() + first,
					trial.routesDynamic[route].begin() + last + 1);
				consider(std::move(trial), "TWO_OPT");
			}
		}

		// Cross-route RouteVisit relocate. Quantities move unchanged.
		for (int fromRoute : result.searchRoutes) for (int toRoute : result.searchRoutes) {
			if (fromRoute == toRoute || !touchesAffected(fromRoute, toRoute, result.affectedRoutes)) continue;
			const int fromSize = static_cast<int>(current.routesDynamic[fromRoute].size());
			const int toSize = static_cast<int>(current.routesDynamic[toRoute].size());
			for (int from = 0; from < fromSize; ++from) {
				const RouteVisit moved = current.routesDynamic[fromRoute][from];
				if (routeContains(current.routesDynamic[toRoute], moved.originalCustomerId)) continue;
				for (int to = 0; to <= toSize; ++to) {
					Individual trial = current;
					trial.routesDynamic[fromRoute].erase(trial.routesDynamic[fromRoute].begin() + from);
					trial.routesDynamic[toRoute].insert(trial.routesDynamic[toRoute].begin() + to, moved);
					consider(std::move(trial), "CROSS_ROUTE_RELOCATE");
				}
			}
		}

		// RouteVisit swap, both within an affected route and across nearby routes.
		for (size_t firstIndex = 0; firstIndex < result.searchRoutes.size(); ++firstIndex) {
			const int firstRoute = result.searchRoutes[firstIndex];
			for (size_t secondIndex = firstIndex; secondIndex < result.searchRoutes.size(); ++secondIndex) {
				const int secondRoute = result.searchRoutes[secondIndex];
				if (!touchesAffected(firstRoute, secondRoute, result.affectedRoutes)) continue;
				for (int first = 0; first < static_cast<int>(current.routesDynamic[firstRoute].size()); ++first)
					for (int second = 0; second < static_cast<int>(current.routesDynamic[secondRoute].size()); ++second) {
						if (firstRoute == secondRoute && first >= second) continue;
						const int firstCustomer = current.routesDynamic[firstRoute][first].originalCustomerId;
						const int secondCustomer = current.routesDynamic[secondRoute][second].originalCustomerId;
						if (firstRoute != secondRoute &&
							(routeContains(current.routesDynamic[firstRoute], secondCustomer) ||
							 routeContains(current.routesDynamic[secondRoute], firstCustomer))) continue;
						Individual trial = current;
						std::swap(trial.routesDynamic[firstRoute][first], trial.routesDynamic[secondRoute][second]);
						consider(std::move(trial), "SWAP");
					}
			}
		}

		if (!bestMove) break;
		current = std::move(*bestMove);
		++result.acceptedMoves;
		if (bestMoveType == "INTRA_RELOCATE" || bestMoveType == "CROSS_ROUTE_RELOCATE") ++result.relocateMoves;
		else if (bestMoveType == "SWAP") ++result.swapMoves;
		else if (bestMoveType == "TWO_OPT") ++result.twoOptMoves;
	}

	const ValidationResult finalValidation = validateSolutionFromScratch(current, params);
	result.validatorPassed = finalValidation.feasible();
	result.finalDistance = current.eval.distance;
	result.reason = result.validatorPassed ? "validator PASS" : finalValidation.message;
	if (result.validatorPassed) result.improved = std::move(current);
	return result;
}
