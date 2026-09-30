#include "MinimalDestroyRepair.h"
#include "SolutionValidator.h"
#include "NumericFormat.h"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <unordered_set>

extern std::vector<int> demand_type;
extern std::vector<std::vector<bool>> conflict_matrix;

namespace
{
int commodityOf(int client, const Params& params)
{
	if (client <= 0 || client >= static_cast<int>(params.Corres_origin.size())) return 0;
	const int origin = params.Corres_origin[client];
	return origin >= 0 && origin < static_cast<int>(demand_type.size()) ? demand_type[origin] : 0;
}

bool commoditiesConflict(int first, int second)
{
	return first >= 0 && second >= 0 && first < static_cast<int>(conflict_matrix.size()) &&
		second < static_cast<int>(conflict_matrix[first].size()) && conflict_matrix[first][second];
}

double routeDistance(const std::vector<int>& route, const Params& params)
{
	double distance = 0.0;
	int previous = 0;
	int previousOrigin = -1;
	for (int client : route)
	{
		const int origin = params.Corres_origin[client];
		if (origin != previousOrigin) distance += params.timeCost[previous][client];
		previous = client;
		previousOrigin = origin;
	}
	if (!route.empty()) distance += params.timeCost[previous][0];
	return distance;
}

bool routeFeasible(const std::vector<int>& route, const Params& params)
{
	double load = 0.0;
	double time = 0.0;
	int previous = 0;
	int previousOrigin = -1;
	std::unordered_set<int> seenOrigins;
	std::vector<int> routeCommodities;
	for (int client : route)
	{
		if (client <= 0 || client > params.nbClients) return false;
		load += params.cli[client].demand;
		if (load > params.vehicleCapacity + MY_EPSILON) return false;
		const int origin = params.Corres_origin[client];
		if (origin != previousOrigin)
		{
			if (!seenOrigins.insert(origin).second) return false;
			time += params.timeCost[previous][client];
			const double serviceStart = std::max(time, params.cli[client].earliest_time);
			if (serviceStart > params.cli[client].latest_time + MY_EPSILON) return false;
			time = serviceStart + params.cli[client].serviceDuration;
		}
		const int commodity = commodityOf(client, params);
		for (int other : routeCommodities)
			if (commoditiesConflict(commodity, other)) return false;
		routeCommodities.push_back(commodity);
		previous = client;
		previousOrigin = origin;
	}
	if (!route.empty()) time += params.timeCost[previous][0];
	return time <= params.cli[0].latest_time + MY_EPSILON;
}

void rebuildChromosome(Individual& individual)
{
	individual.chromT.clear();
	for (const auto& route : individual.chromR)
		individual.chromT.insert(individual.chromT.end(), route.begin(), route.end());
}

std::string customerList(const std::vector<int>& customers)
{
	std::ostringstream out;
	for (size_t i = 0; i < customers.size(); ++i)
	{
		if (i) out << ';';
		out << customers[i];
	}
	return out.str();
}
}

MinimalDestroyRepair::MinimalDestroyRepair(Params& p, Population& pop, LocalSearch& ls,
	const MinimalDestroyRepairConfig& cfg)
	: params(p), population(pop), localSearch(ls), config(cfg)
{
}

bool MinimalDestroyRepair::destroyAndRegret2Repair(const Individual& parent, Individual& candidate,
	std::vector<int>& removedCustomers, int& removedRouteCount)
{
	if (parent.quantityRepresentation != QuantityRepresentation::FIXED_PSEUDO || params.deadlineReached()) return false;

	// One destroyed request is one ORIGINAL customer.  Its immutable pseudo
	// pieces are retained in route-level visit blocks, so the repair can move
	// an existing split across routes without changing any piece ID or quantity.
	struct ServiceBlock { int origin = -1; int oldRoute = -1; std::vector<int> clients; };
	struct LocatedOrigin
	{
		int origin = -1;
		int representative = -1;
		std::set<int> routes;
		std::vector<ServiceBlock> blocks;
	};
	std::vector<LocatedOrigin> origins(params.originalDemand.size());
	for (int origin = 1; origin < static_cast<int>(origins.size()); ++origin) origins[origin].origin = origin;
	for (int routeIndex = 0; routeIndex < static_cast<int>(parent.chromR.size()); ++routeIndex)
	{
		const auto& route = parent.chromR[routeIndex];
		for (size_t begin = 0; begin < route.size();)
		{
			const int origin = params.Corres_origin[route[begin]];
			size_t end = begin + 1;
			while (end < route.size() && params.Corres_origin[route[end]] == origin) ++end;
			if (origin <= 0 || origin >= static_cast<int>(origins.size())) return false;
			LocatedOrigin& item = origins[origin];
			if (item.representative < 0) item.representative = route[begin];
			item.routes.insert(routeIndex);
			item.blocks.push_back({origin, routeIndex,
				std::vector<int>(route.begin() + begin, route.begin() + end)});
			begin = end;
		}
	}
	std::vector<LocatedOrigin> located;
	for (size_t origin = 1; origin < origins.size(); ++origin)
		if (origins[origin].representative > 0) located.push_back(origins[origin]);
	if (located.size() < 2) return false;

	std::uniform_int_distribution<int> anchorDistribution(0, static_cast<int>(located.size()) - 1);
	const LocatedOrigin anchor = located[anchorDistribution(params.ran)];
	const int removalCount = std::max(1, std::min<int>(static_cast<int>(located.size()) - 1,
		static_cast<int>(std::ceil(config.removalRatio * located.size()))));
	double latestScale = 1.0;
	for (int client = 1; client <= params.nbClients; ++client)
		latestScale = std::max(latestScale, params.cli[client].latest_time);

	std::vector<LocatedOrigin> remaining;
	for (const LocatedOrigin& item : located) if (item.origin != anchor.origin) remaining.push_back(item);
	std::set<int> selectedRoutes = anchor.routes;
	std::vector<LocatedOrigin> selected{anchor};
	while (static_cast<int>(selected.size()) < removalCount)
	{
		auto best = std::min_element(remaining.begin(), remaining.end(), [&](const LocatedOrigin& a, const LocatedOrigin& b) {
			auto score = [&](const LocatedOrigin& item) {
				const double spatial = params.timeCost[anchor.representative][item.representative] / std::max(1.0, params.maxDist);
				const double timeWindow = (std::fabs(params.cli[anchor.representative].earliest_time - params.cli[item.representative].earliest_time) +
					std::fabs(params.cli[anchor.representative].latest_time - params.cli[item.representative].latest_time)) / (2.0 * latestScale);
				bool sharesAnchorRoute = false;
				bool reusesSelectedRoute = false;
				for (int route : item.routes) {
					sharesAnchorRoute = sharesAnchorRoute || anchor.routes.count(route) != 0;
					reusesSelectedRoute = reusesSelectedRoute || selectedRoutes.count(route) != 0;
				}
				const double sameAnchorRoutePenalty = sharesAnchorRoute ? 0.75 : 0.0;
				const double reusedRoutePenalty = reusesSelectedRoute ? 0.35 : 0.0;
				return spatial + 0.25 * timeWindow + sameAnchorRoutePenalty + reusedRoutePenalty;
			};
			const double scoreA = score(a), scoreB = score(b);
			return std::fabs(scoreA - scoreB) > MY_EPSILON ? scoreA < scoreB : a.origin < b.origin;
		});
		selected.push_back(*best);
		selectedRoutes.insert(best->routes.begin(), best->routes.end());
		remaining.erase(best);
	}

	candidate = parent;
	removedCustomers.clear();
	std::vector<ServiceBlock> unassigned;
	std::unordered_set<int> removedOrigins;
	for (const LocatedOrigin& item : selected) {
		removedCustomers.push_back(item.origin);
		removedOrigins.insert(item.origin);
		unassigned.insert(unassigned.end(), item.blocks.begin(), item.blocks.end());
	}
	for (auto& route : candidate.chromR)
		route.erase(std::remove_if(route.begin(), route.end(), [&](int client) {
			return removedOrigins.count(params.Corres_origin[client]) != 0;
		}), route.end());
	removedRouteCount = static_cast<int>(selectedRoutes.size());
	rebuildChromosome(candidate);

	struct Insertion { bool exists = false; double delta = 0.0; int route = -1; int position = -1; };
	while (!unassigned.empty())
	{
		if (params.deadlineReached()) return false;
		int selectedIndex = -1;
		double selectedRegret = -1.0;
		double selectedBestDelta = std::numeric_limits<double>::infinity();
		Insertion selectedInsertion;
		for (int index = 0; index < static_cast<int>(unassigned.size()); ++index)
		{
			const ServiceBlock& block = unassigned[index];
			Insertion best, second;
			best.delta = second.delta = std::numeric_limits<double>::infinity();
			for (int routeIndex = 0; routeIndex < static_cast<int>(candidate.chromR.size()); ++routeIndex)
			{
				const auto& baseRoute = candidate.chromR[routeIndex];
				bool alreadyServesOrigin = false;
				for (int client : baseRoute)
					if (params.Corres_origin[client] == block.origin) { alreadyServesOrigin = true; break; }
				// Distinct blocks of a split original must remain on distinct routes.
				if (alreadyServesOrigin) continue;
				const double baseDistance = routeDistance(baseRoute, params);
				for (int position = 0; position <= static_cast<int>(baseRoute.size()); ++position)
				{
					if (params.deadlineReached()) return false;
					if (position > 0 && position < static_cast<int>(baseRoute.size()) &&
						params.Corres_origin[baseRoute[position - 1]] == params.Corres_origin[baseRoute[position]]) continue;
					std::vector<int> trialRoute(baseRoute);
					trialRoute.insert(trialRoute.begin() + position, block.clients.begin(), block.clients.end());
					if (!routeFeasible(trialRoute, params)) continue;
					Insertion choice{true, routeDistance(trialRoute, params) - baseDistance, routeIndex, position};
					if (choice.delta < best.delta - MY_EPSILON) { second = best; best = choice; }
					else if (choice.delta < second.delta - MY_EPSILON) second = choice;
				}
			}
			if (!best.exists) return false;
			const double regret = second.exists ? second.delta - best.delta : params.maxDist;
			if (selectedIndex < 0 || regret > selectedRegret + MY_EPSILON ||
				(std::fabs(regret - selectedRegret) <= MY_EPSILON && best.delta < selectedBestDelta))
			{
				selectedIndex = index;
				selectedRegret = regret;
				selectedBestDelta = best.delta;
				selectedInsertion = best;
			}
		}
		const ServiceBlock block = unassigned[selectedIndex];
		auto& route = candidate.chromR[selectedInsertion.route];
		route.insert(route.begin() + selectedInsertion.position, block.clients.begin(), block.clients.end());
		unassigned.erase(unassigned.begin() + selectedIndex);
	}

	rebuildChromosome(candidate);
	candidate.indivsPerProximity.clear();
	candidate.evaluateCompleteCost(params);
	return validateSolutionFromScratch(candidate, params).feasible();
}

void MinimalDestroyRepair::processSurvivalChecks(int iteration)
{
	for (auto pending = pendingSurvival.begin(); pending != pendingSurvival.end();)
	{
		if (iteration < pending->dueIteration) { ++pending; continue; }
		const bool survives = population.containsRouteSolution(pending->routes);
		++stats.survivalChecks;
		if (survives) ++stats.survived;
		std::cout << "[DR-SURVIVAL] TriggerIteration=" << pending->triggerIteration
			<< " CheckIteration=" << iteration << " StillInPopulation=" << (survives ? 1 : 0) << std::endl;
		pending = pendingSurvival.erase(pending);
	}
}

bool MinimalDestroyRepair::onIteration(int iteration, int nonImprovingIterations)
{
	processSurvivalChecks(iteration);
	if (params.deadlineReached() || nonImprovingIterations < config.trigger || iteration < nextAllowedIteration) return false;
	nextAllowedIteration = iteration + config.trigger;
	const clock_t begin = clock();
	++stats.triggers;
	std::optional<Individual> selectedParent = population.selectDestroyRepairParent();
	if (!selectedParent)
	{
		++stats.skippedNoFeasibleParent;
		std::cout << "[DR-SKIP]"
			<< " reason=NO_FEASIBLE_PARENT"
			<< " time=" << params.elapsedSearchSeconds()
			<< " iteration=" << iteration << std::endl;
		stats.totalTime += static_cast<double>(clock() - begin) / CLOCKS_PER_SEC;
		return false;
	}
	++stats.successfulRuns;
	Individual parent = std::move(*selectedParent);
	const ValidationResult parentValidation = validateSolutionFromScratch(parent, params);
	const double globalBefore = population.getBestFound() ? population.getBestFound()->eval.distance : std::numeric_limits<double>::infinity();
	std::vector<int> removed;
	int removedRoutes = 0;
	Individual candidate(params);
	const bool repaired = destroyAndRegret2Repair(parent, candidate, removed, removedRoutes);
	std::cout << "[DR] TriggerIteration=" << iteration
		<< " ParentCost=" << objectiveValue2(parent.eval.distance)
		<< " ParentRoutes=" << parentValidation.routesUsed
		<< " RemovedCustomers=" << removed.size()
		<< " RemovedCustomerIds=" << customerList(removed)
		<< " RemovedRoutes=" << removedRoutes << std::endl;
	if (!repaired)
	{
		std::cout << "[DR-REPAIR] RepairedCost=NA Feasible=0 Routes=NA" << std::endl;
		stats.totalTime += static_cast<double>(clock() - begin) / CLOCKS_PER_SEC;
		return false;
	}
	++stats.feasibleRepairs;
	const ValidationResult repairValidation = validateSolutionFromScratch(candidate, params);
	const double beforeLS = candidate.eval.distance;
	std::cout << "[DR-REPAIR] RepairedCost=" << objectiveValue2(beforeLS)
		<< " Feasible=1 Routes=" << repairValidation.routesUsed << std::endl;

	localSearch.run(candidate, params.penaltyCapacity, params.penaltyDuration);
	const ValidationResult finalValidation = validateSolutionFromScratch(candidate, params);
	std::cout << "[DR-LS] BeforeLS=" << objectiveValue2(beforeLS)
		<< " AfterLS=" << objectiveValue2(candidate.eval.distance)
		<< " Improvement=" << objectiveValue2(beforeLS - candidate.eval.distance)
		<< " Validator=" << (finalValidation.feasible() ? "PASS" : "FAIL") << std::endl;
	if (!finalValidation.feasible() || params.deadlineReached())
	{
		std::cout << "[DR-POP] Inserted=0 NewGlobalBest=0 BestCost="
			<< (population.getBestFound() ? objectiveValue2(population.getBestFound()->eval.distance) : "NA") << std::endl;
		stats.totalTime += static_cast<double>(clock() - begin) / CLOCKS_PER_SEC;
		return false;
	}

	if (candidate.eval.distance < parent.eval.distance - MY_EPSILON) ++stats.improvedParent;
	candidate.indivsPerProximity.clear();
	population.setAuditIteration(iteration);
	population.addIndividual(candidate, true);
	++stats.inserted;
	const double globalAfter = population.getBestFound() ? population.getBestFound()->eval.distance : globalBefore;
	const bool newGlobalBest = globalAfter < globalBefore - MY_EPSILON;
	if (newGlobalBest) ++stats.newGlobalBest;
	pendingSurvival.push_back({iteration + config.survivalCheckDelay, iteration, candidate.chromR});
	std::cout << "[DR-POP] Inserted=1 NewGlobalBest=" << (newGlobalBest ? 1 : 0)
		<< " BestCost=" << objectiveValue2(globalAfter) << std::endl;
	stats.totalTime += static_cast<double>(clock() - begin) / CLOCKS_PER_SEC;
	return newGlobalBest;
}
