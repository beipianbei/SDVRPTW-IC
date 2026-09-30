#include "OldSolutionReevaluation.h"

#include "Individual.h"
#include "NumericFormat.h"
#include "SolutionValidator.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <vector>

namespace
{
std::string passFail(bool value)
{
	return value ? "PASS" : "FAIL";
}

double parseLastNumber(const std::string& text)
{
	static const std::regex numberPattern(R"([-+]?[0-9]*\.?[0-9]+)");
	double value = std::numeric_limits<double>::quiet_NaN();
	for (std::sregex_iterator it(text.begin(), text.end(), numberPattern), end; it != end; ++it)
		value = std::stod((*it).str());
	return value;
}

OldSolutionLoadResult parseOldSolution(const std::string& solutionPath, Params& params)
{
	OldSolutionLoadResult parsed(params);
	parsed.ok = true;
	parsed.dynamicIndividual.quantityRepresentation = QuantityRepresentation::DYNAMIC_QUANTITY;
	parsed.dynamicIndividual.routesDynamic.assign(params.nbVehicles, {});
	parsed.dynamicIndividual.chromR.assign(params.nbVehicles, {});
	parsed.dynamicIndividual.chromT.clear();

	std::ifstream input(solutionPath);
	if (!input.is_open()) {
		parsed.ok = false;
		parsed.error = "cannot open solution file";
		return parsed;
	}

	static const std::regex routePattern(R"(^\s*Route\s*#?\s*([0-9]+)\s*:?(.*)$)",
		std::regex_constants::icase);
	static const std::regex visitPattern(R"(([0-9]+)\s*\(\s*([-+]?[0-9]*\.?[0-9]+)\s*\))");

	int routeCursor = 0;
	bool sawRoute = false;
	std::string line;
	while (std::getline(input, line)) {
		std::smatch routeMatch;
		if (std::regex_match(line, routeMatch, routePattern)) {
			sawRoute = true;
			if (routeCursor >= static_cast<int>(parsed.dynamicIndividual.routesDynamic.size()))
				parsed.dynamicIndividual.routesDynamic.resize(routeCursor + 1);

			const std::string visitsText = routeMatch[2].str();
			bool sawVisit = false;
			for (std::sregex_iterator it(visitsText.begin(), visitsText.end(), visitPattern), end; it != end; ++it) {
				sawVisit = true;
				const int rawId = std::stoi((*it)[1].str());
				const double quantity = std::stod((*it)[2].str());
				int origin = rawId;
				if (rawId <= 0) {
					parsed.ok = false;
					parsed.error = "invalid non-positive customer id in solution";
					return parsed;
				}
				if (rawId >= static_cast<int>(params.originalDemand.size())) {
					if (rawId <= params.nbClients && rawId < static_cast<int>(params.Corres_origin.size()))
						origin = params.Corres_origin[rawId];
					else {
						parsed.ok = false;
						parsed.error = "customer id cannot be mapped to an original customer";
						return parsed;
					}
				}
				if (origin <= 0 || origin >= static_cast<int>(params.originalDemand.size())) {
					parsed.ok = false;
					parsed.error = "mapped original customer id is outside the current instance";
					return parsed;
				}
				if (quantity <= MY_EPSILON) {
					parsed.ok = false;
					parsed.error = "non-positive delivery quantity in solution";
					return parsed;
				}
				parsed.dynamicIndividual.routesDynamic[routeCursor].push_back({ origin, quantity });
			}
			if (!sawVisit && visitsText.find_first_not_of(" \t\r\n") != std::string::npos) {
				parsed.ok = false;
				parsed.error = "route line contains no readable id(quantity) visits";
				return parsed;
			}
			++routeCursor;
			continue;
		}

		std::string lower = line;
		std::transform(lower.begin(), lower.end(), lower.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		if (lower.find("cost") != std::string::npos)
		{
			parsed.oldReportedCost = parseLastNumber(line);
			parsed.hasOldReportedCost = !std::isnan(parsed.oldReportedCost);
		}
	}

	if (!sawRoute) {
		parsed.ok = false;
		parsed.error = "solution file contains no Route lines";
		return parsed;
	}
	return parsed;
}

int countSplitCustomers(const Individual& individual, const Params& params)
{
	std::vector<int> routeCounts(params.originalDemand.size(), 0);
	for (const auto& route : individual.routesDynamic) {
		std::set<int> onRoute;
		for (const RouteVisit& visit : route) onRoute.insert(visit.originalCustomerId);
		for (int origin : onRoute)
			if (origin > 0 && origin < static_cast<int>(routeCounts.size())) ++routeCounts[origin];
	}
	return static_cast<int>(std::count_if(routeCounts.begin(), routeCounts.end(),
		[](int count) { return count >= 2; }));
}
}

OldSolutionLoadResult loadOldSolutionAsDynamic(const std::string& solutionPath, Params& params)
{
	OldSolutionLoadResult parsed = parseOldSolution(solutionPath, params);
	if (!parsed.ok) return parsed;
	parsed.dynamicIndividual.evaluateDynamicQuantity(params);
	parsed.validation = validateSolutionFromScratch(parsed.dynamicIndividual, params);
	return parsed;
}

std::optional<Individual> convertOldDynamicToCurrentFixed(const Individual& dynamicIndividual,
	Params& params,
	std::string& error)
{
	if (dynamicIndividual.quantityRepresentation != QuantityRepresentation::DYNAMIC_QUANTITY) {
		error = "input is not a dynamic old solution";
		return std::nullopt;
	}

	std::vector<std::vector<int>> clientsByOrigin(params.originalDemand.size());
	for (int client = 1; client <= params.nbClients; ++client) {
		const int origin = params.Corres_origin[client];
		if (origin <= 0 || origin >= static_cast<int>(clientsByOrigin.size())) {
			error = "current fixed representation has invalid origin mapping";
			return std::nullopt;
		}
		clientsByOrigin[origin].push_back(client);
	}

	std::vector<bool> used(params.nbClients + 1, false);
	Individual fixed(params);
	fixed.quantityRepresentation = QuantityRepresentation::FIXED_PSEUDO;
	for (auto& route : fixed.chromR) route.clear();
	fixed.chromT.clear();
	if (fixed.chromR.size() < dynamicIndividual.routesDynamic.size())
		fixed.chromR.resize(dynamicIndividual.routesDynamic.size());

	for (size_t r = 0; r < dynamicIndividual.routesDynamic.size(); ++r) {
		for (const RouteVisit& visit : dynamicIndividual.routesDynamic[r]) {
			if (visit.originalCustomerId <= 0 ||
				visit.originalCustomerId >= static_cast<int>(clientsByOrigin.size())) {
				error = "old solution contains an unknown original customer";
				return std::nullopt;
			}
			int matchedClient = -1;
			for (int client : clientsByOrigin[visit.originalCustomerId]) {
				if (!used[client] && std::fabs(params.cli[client].demand - visit.quantity) <= MY_EPSILON) {
					matchedClient = client;
					break;
				}
			}
			if (matchedClient < 0) {
				std::ostringstream msg;
				msg << "cannot map original " << visit.originalCustomerId
					<< " quantity " << visit.quantity << " to a current fixed pseudo-client";
				error = msg.str();
				return std::nullopt;
			}
			used[matchedClient] = true;
			fixed.chromR[r].push_back(matchedClient);
			fixed.chromT.push_back(matchedClient);
		}
	}
	for (int client = 1; client <= params.nbClients; ++client) {
		if (!used[client]) {
			error = "old solution does not cover all current fixed pseudo-clients";
			return std::nullopt;
		}
	}
	fixed.evaluateCompleteCost(params);
	const ValidationResult validation = validateSolutionFromScratch(fixed, params);
	if (!validation.feasible()) {
		error = validation.message.empty() ? "converted fixed solution failed current validator" : validation.message;
		return std::nullopt;
	}
	return fixed;
}

bool reevaluateOldSolutionFile(const std::string& instancePath,
	const std::string& solutionPath,
	Params& params,
	std::ostream& out)
{
	OldSolutionLoadResult parsed = loadOldSolutionAsDynamic(solutionPath, params);
	out << "[REEVALUATE-OLD-SOLUTION]" << '\n';
	out << "Instance=" << std::filesystem::path(instancePath).stem().string() << '\n';
	out << "SolutionFile=" << solutionPath << '\n';
	if (parsed.hasOldReportedCost)
		out << "OldReportedCost=" << objectiveValue2(parsed.oldReportedCost) << '\n';
	else
		out << "OldReportedCost=NA" << '\n';
	out << "CurrentDistanceConvention=TWO_DECIMAL_EUCLIDEAN" << '\n';

	if (!parsed.ok) {
		out << "CurrentRecomputedCost=NA" << '\n';
		out << "Routes=0" << '\n';
		out << "SplitCustomers=0" << '\n';
		out << "DemandCheck=FAIL" << '\n';
		out << "CapacityCheck=FAIL" << '\n';
		out << "TimeWindowCheck=FAIL" << '\n';
		out << "ConflictCheck=FAIL" << '\n';
		out << "DuplicateCheck=FAIL" << '\n';
		out << "Validator=FAIL" << '\n';
		out << "FailureReason=" << parsed.error << '\n';
		return false;
	}

	const ValidationResult validation = parsed.validation;
	out << "CurrentRecomputedCost=" << objectiveValue2(validation.distance) << '\n';
	out << "Routes=" << validation.routesUsed << '\n';
	out << "SplitCustomers=" << countSplitCustomers(parsed.dynamicIndividual, params) << '\n';
	out << "DemandCheck=" << passFail(validation.originalDemandValid) << '\n';
	out << "CapacityCheck=" << passFail(validation.capacityFeasible) << '\n';
	out << "TimeWindowCheck=" << passFail(validation.timeWindowFeasible) << '\n';
	out << "ConflictCheck=" << passFail(validation.commodityFeasible) << '\n';
	out << "DuplicateCheck=" << passFail(validation.visitBlocksValid) << '\n';
	out << "Validator=" << passFail(validation.feasible()) << '\n';
	if (!validation.feasible()) {
		if (!validation.structureValid) out << "FailureReason=STRUCTURE";
		else if (!validation.originalDemandValid) out << "FailureReason=DEMAND";
		else if (!validation.capacityFeasible) out << "FailureReason=CAPACITY";
		else if (!validation.timeWindowFeasible) out << "FailureReason=TIME_WINDOW";
		else if (!validation.commodityFeasible) out << "FailureReason=CONFLICT";
		else if (!validation.visitBlocksValid) out << "FailureReason=DUPLICATE";
		else out << "FailureReason=UNKNOWN";
		if (!validation.message.empty()) out << " Detail=" << validation.message;
		out << '\n';
	}
	return true;
}
