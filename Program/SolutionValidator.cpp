#include "SolutionValidator.h"
#include "InstanceCVRPLIB.h"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>

namespace
{
int typeOf(int client, const Params& params)
{
	if (client <= 0 || client >= static_cast<int>(params.Corres_origin.size())) return 0;
	const int origin = params.Corres_origin[client];
	if (origin < 0 || origin >= static_cast<int>(demand_type.size())) return 0;
	return demand_type[origin];
}

bool typesConflict(int a, int b)
{
	if (a < 0 || b < 0 || a >= static_cast<int>(conflict_matrix.size())) return false;
	if (b >= static_cast<int>(conflict_matrix[a].size())) return false;
	return conflict_matrix[a][b];
}

ValidationResult validateDynamicQuantity(const Individual& indiv, const Params& params)
{
	ValidationResult out;
	std::ostringstream errors;
	std::vector<double> delivered(params.originalDemand.size(), 0.0);
	for (size_t r = 0; r < indiv.routesDynamic.size(); ++r) {
		const auto& route = indiv.routesDynamic[r];
		if (route.empty()) continue;
		++out.routesUsed;
		double load = 0.0, time = 0.0;
		int previous = 0;
		std::unordered_set<int> seen;
		std::vector<int> routeTypes;
		for (const RouteVisit& visit : route) {
			const int origin = visit.originalCustomerId;
			if (origin <= 0 || origin >= static_cast<int>(params.originalDemand.size()) || visit.quantity <= MY_EPSILON) {
				out.structureValid = false; errors << "invalid dynamic visit; "; continue;
			}
			if (!seen.insert(origin).second) { out.visitBlocksValid = false; errors << "duplicate origin on route; "; }
			load += visit.quantity;
			delivered[origin] += visit.quantity;
			if (params.originalTimeCost.empty()) { out.structureValid = false; errors << "missing original distance matrix; "; continue; }
			time += params.originalTimeCost[previous][origin];
			const double start = std::max(time, params.originalEarliest[origin]);
			if (start > params.originalLatest[origin] + MY_EPSILON) out.timeWindowExcess += start - params.originalLatest[origin];
			time = start + params.originalService[origin];
			out.distance += params.originalTimeCost[previous][origin];
			const int type = params.originalCommodity[origin];
			for (int existing : routeTypes) if (typesConflict(type, existing)) out.commodityFeasible = false;
			routeTypes.push_back(type);
			previous = origin;
		}
		out.distance += params.originalTimeCost[previous][0];
		time += params.originalTimeCost[previous][0];
		if (time > params.originalLatest[0] + MY_EPSILON) out.timeWindowExcess += time - params.originalLatest[0];
		if (load > params.vehicleCapacity + MY_EPSILON) out.capacityExcess += load - params.vehicleCapacity;
	}
	for (size_t origin = 1; origin < delivered.size(); ++origin)
		if (std::fabs(delivered[origin] - params.originalDemand[origin]) > MY_EPSILON) {
			out.originalDemandValid = false;
			errors << "origin " << origin << " delivered " << delivered[origin] << " instead of " << params.originalDemand[origin] << "; ";
		}
	out.capacityFeasible = out.capacityExcess < MY_EPSILON;
	out.timeWindowFeasible = out.timeWindowExcess < MY_EPSILON;
	if (!out.commodityFeasible) errors << "commodity conflict; ";
	out.message = errors.str();
	return out;
}
}

ValidationResult validateSolutionFromScratch(const Individual& indiv, const Params& params)
{
	if (indiv.quantityRepresentation == QuantityRepresentation::DYNAMIC_QUANTITY) {
		ValidationResult out = validateDynamicQuantity(indiv, params);
		if (std::isfinite(indiv.eval.distance) && std::fabs(indiv.eval.distance - out.distance) > 1.e-6) {
			out.structureValid = false;
			std::ostringstream mismatch;
			mismatch << "stored=" << indiv.eval.distance << " recomputed=" << out.distance
				<< " difference=" << std::fabs(indiv.eval.distance - out.distance) << "; ";
			out.message += mismatch.str();
		}
		return out;
	}
	ValidationResult out;
	std::vector<int> occurrences(params.nbClients + 1, 0);
	std::vector<double> deliveredByOrigin(params.originalDemand.size(), 0.0);
	std::ostringstream errors;

	for (size_t routeIndex = 0; routeIndex < indiv.chromR.size(); ++routeIndex)
	{
		const auto& route = indiv.chromR[routeIndex];
		if (route.empty()) continue;
		++out.routesUsed;
		double load = 0.0;
		double time = 0.0;
		int previous = 0;
		int previousOrigin = -1;
		std::unordered_set<int> closedOrigins;
		std::vector<int> routeTypes;

		for (int client : route)
		{
			if (client <= 0 || client > params.nbClients)
			{
				out.structureValid = false;
				errors << "route " << routeIndex + 1 << " contains invalid client " << client << "; ";
				continue;
			}
			++occurrences[client];
			load += params.cli[client].demand;
			const int origin = params.Corres_origin[client];
			if (origin < 0 || origin >= static_cast<int>(deliveredByOrigin.size())) {
				out.originalDemandValid = false;
				errors << "client " << client << " has invalid origin " << origin << "; ";
			}
			else deliveredByOrigin[origin] += params.cli[client].demand;

			if (origin != previousOrigin)
			{
				if (closedOrigins.count(origin))
				{
					out.visitBlocksValid = false;
					errors << "origin " << origin << " occurs in non-contiguous blocks on route " << routeIndex + 1 << "; ";
				}
				if (previousOrigin >= 0) closedOrigins.insert(previousOrigin);
				time += params.timeCost[previous][client];
				const double start = std::max(time, params.cli[client].earliest_time);
				if (start > params.cli[client].latest_time + MY_EPSILON)
					out.timeWindowExcess += start - params.cli[client].latest_time;
				time = start + params.cli[client].serviceDuration;
				out.distance += params.timeCost[previous][client];
			}
			// Consecutive pseudo-clients of one origin form one physical visit:
			// no travel and no additional service time.
			previous = client;
			previousOrigin = origin;

			const int type = typeOf(client, params);
			for (int other : routeTypes)
				if (typesConflict(type, other)) out.commodityFeasible = false;
			routeTypes.push_back(type);
		}

		out.distance += params.timeCost[previous][0];
		time += params.timeCost[previous][0];
		if (time > params.cli[0].latest_time + MY_EPSILON)
			out.timeWindowExcess += time - params.cli[0].latest_time;
		if (load > params.vehicleCapacity + MY_EPSILON)
			out.capacityExcess += load - params.vehicleCapacity;
	}

	for (int client = 1; client <= params.nbClients; ++client)
	{
		if (occurrences[client] != 1)
		{
			out.structureValid = false;
			errors << "client " << client << " occurs " << occurrences[client] << " times; ";
		}
	}
	for (size_t origin = 1; origin < params.originalDemand.size(); ++origin)
	{
		if (std::fabs(deliveredByOrigin[origin] - params.originalDemand[origin]) > MY_EPSILON)
		{
			out.originalDemandValid = false;
			errors << "origin " << origin << " delivered " << deliveredByOrigin[origin]
				<< " instead of " << params.originalDemand[origin] << "; ";
		}
	}
	out.capacityFeasible = out.capacityExcess < MY_EPSILON;
	out.timeWindowFeasible = out.timeWindowExcess < MY_EPSILON;
	if (!out.commodityFeasible) errors << "commodity conflict; ";
	if (std::isfinite(indiv.eval.distance) && std::fabs(indiv.eval.distance - out.distance) > 1.e-6) {
		out.structureValid = false;
		errors << "stored=" << indiv.eval.distance << " recomputed=" << out.distance
			<< " difference=" << std::fabs(indiv.eval.distance - out.distance) << "; ";
	}
	out.message = errors.str();
	return out;
}

RouteDiagnostics computeRouteDiagnostics(const Individual& indiv, const Params& params)
{
	RouteDiagnostics out;
	bool hasRoute = false;
	if (indiv.quantityRepresentation == QuantityRepresentation::DYNAMIC_QUANTITY) {
		for (const auto& route : indiv.routesDynamic) {
			if (route.empty()) continue;
			double load = 0.0;
			for (const RouteVisit& visit : route) load += visit.quantity;
			out.totalDemand += load;
			++out.routesUsed;
			out.minimumRouteLoad = hasRoute ? std::min(out.minimumRouteLoad, load) : load;
			out.maximumRouteLoad = hasRoute ? std::max(out.maximumRouteLoad, load) : load;
			if (load >= params.vehicleCapacity - MY_EPSILON) ++out.fullRoutes;
			hasRoute = true;
		}
		if (out.routesUsed > 0) {
			out.averageRouteLoad = out.totalDemand / out.routesUsed;
			out.averageUtilization = out.totalDemand / (out.routesUsed * params.vehicleCapacity);
		}
		return out;
	}
	for (const auto& route : indiv.chromR)
	{
		if (route.empty()) continue;
		double load = 0.0;
		for (int client : route) load += params.cli[client].demand;
		out.totalDemand += load;
		++out.routesUsed;
		out.minimumRouteLoad = hasRoute ? std::min(out.minimumRouteLoad, load) : load;
		out.maximumRouteLoad = hasRoute ? std::max(out.maximumRouteLoad, load) : load;
		if (load >= params.vehicleCapacity - MY_EPSILON) ++out.fullRoutes;
		hasRoute = true;
	}
	if (out.routesUsed > 0) {
		out.averageRouteLoad = out.totalDemand / out.routesUsed;
		out.averageUtilization = out.totalDemand / (out.routesUsed * params.vehicleCapacity);
	}
	return out;
}

