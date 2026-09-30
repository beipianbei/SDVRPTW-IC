#include "DemandSplit.h"
#include "DistanceUtils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace
{
int gcdInt(int a, int b)
{
	a = std::abs(a); b = std::abs(b);
	while (b != 0) { const int r = a % b; a = b; b = r; }
	return a;
}

int roundedInt(double value)
{
	return static_cast<int>(std::llround(value));
}

int globalGcd(const OriginalInstance& instance)
{
	int value = roundedInt(instance.vehicleCapacity);
	for (size_t i = 1; i < instance.demand.size(); ++i)
		value = gcdInt(value, roundedInt(instance.demand[i]));
	return std::max(1, value);
}

std::vector<int> currentRlhgsPieces(const OriginalInstance& instance, int customer)
{
	const int capacity = roundedInt(instance.vehicleCapacity);
	const int baseGcd = globalGcd(instance);
	const int clientCount = static_cast<int>(instance.demand.size()) - 1;
	double meanUnits = 0.0;
	for (int i = 1; i <= clientCount; ++i) meanUnits += roundedInt(instance.demand[i]) / baseGcd;
	meanUnits /= std::max(1, clientCount);
	const int sMax = static_cast<int>(std::round(std::log(std::max(1.0, meanUnits)) / std::log(2.0)));

	// This deliberately preserves Split_Node()/splitDemand() for S0: L=1,
	// p=2, flag=true and index_minus=0 make every non-depot client level one.
	std::vector<int> units;
	for (int i = 1; i <= sMax + 1; ++i)
		units.push_back(static_cast<int>(std::llround(baseGcd * std::pow(2.0, sMax - i + 1))));

	std::vector<int> pieces;
	int remaining = roundedInt(instance.demand[customer]);
	for (int unit : units)
	{
		while (unit > 0 && remaining >= unit)
		{
			pieces.push_back(unit);
			remaining -= unit;
			if (remaining <= 3) break;
		}
		if (unit == 1) break;
	}
	while (remaining > 0) { pieces.push_back(1); --remaining; }
	return pieces;
}

int localPbGcd(const OriginalInstance& instance, int customer, double eta)
{
	std::vector<std::pair<double, int>> nearest;
	for (int other = 1; other < static_cast<int>(instance.demand.size()); ++other)
	{
		if (other == customer) continue;
		nearest.emplace_back(instance.distance[customer][other], other);
	}
	std::sort(nearest.begin(), nearest.end());
	const int count = std::max(1, static_cast<int>(std::ceil(std::max(0.0, eta) * nearest.size())));
	int result = gcdInt(roundedInt(instance.vehicleCapacity), roundedInt(instance.demand[customer]));
	for (int k = 0; k < std::min<int>(count, nearest.size()); ++k)
		result = gcdInt(result, roundedInt(instance.demand[nearest[k].second]));
	return std::max(1, result);
}

std::vector<int> capacityOnlyPieces(int demand, int capacity)
{
	std::vector<int> pieces;
	while (demand > capacity) { pieces.push_back(capacity); demand -= capacity; }
	if (demand > 0) pieces.push_back(demand);
	return pieces;
}

std::vector<int> ff2sPieces(const OriginalInstance& instance, int customer)
{
	const int demand = roundedInt(instance.demand[customer]);
	const int capacity = roundedInt(instance.vehicleCapacity);
	const int minimumPiece = globalGcd(instance);
	if (demand > capacity) return capacityOnlyPieces(demand, capacity);
	// Deterministic, single balanced binary split. The stopping condition is
	// the global demand quantum, so c101 gives 20=10+10, 30=15+15, etc.
	if (demand <= minimumPiece) return { demand };
	return { demand / 2, demand - demand / 2 };
}

std::vector<int> legacyPbPieces(const OriginalInstance& instance, int customer, int gcd)
{
	const int demand = roundedInt(instance.demand[customer]);
	const int capacity = roundedInt(instance.vehicleCapacity);
	const int units = demand / gcd;
	const int maxUnits = std::max(1, capacity / gcd);
	const int count = std::max(units > 1 ? 2 : 1, (units + maxUnits - 1) / maxUnits);
	std::vector<int> pieces(count, (units / count) * gcd);
	for (int i = 0; i < units % count; ++i) pieces[i] += gcd;
	const int assigned = std::accumulate(pieces.begin(), pieces.end(), 0);
	if (assigned != demand) pieces.back() += demand - assigned;
	return pieces;
}

std::vector<int> gtsBcsReconstructionPieces(const OriginalInstance& instance, int customer, int gcd)
{
	const int demand = roundedInt(instance.demand[customer]);
	const int capacity = roundedInt(instance.vehicleCapacity);
	if (demand > capacity) return capacityOnlyPieces(demand, capacity);
	if (demand <= gcd) return { demand };
	// Engineering reconstruction only: balanced two-way FF2S, then choose the
	// closest PB-GCD-aligned integer share while conserving the exact demand.
	int leftUnits = static_cast<int>(std::llround((static_cast<double>(demand) / 2.0) / gcd));
	leftUnits = std::max(1, std::min(demand / gcd - 1, leftUnits));
	const int left = leftUnits * gcd;
	return { left, demand - left };
}

void computeDiagnostics(SplitResult& result, double capacity)
{
	auto& d = result.diagnostics;
	d.originalCustomers = 0;
	for (size_t i = 1; i < result.corresOrigin.size(); ++i)
		d.originalCustomers = std::max(d.originalCustomers, result.corresOrigin[i]);
	d.pseudoCustomers = static_cast<int>(result.pieces.size());
	std::vector<int> piecesPerOrigin(d.originalCustomers + 1, 0);
	std::vector<double> values;
	values.reserve(result.pieces.size());
	for (const auto& piece : result.pieces) {
		++piecesPerOrigin[piece.originalCustomer];
		values.push_back(piece.pieceDemand);
		if (std::fabs(piece.pieceDemand - 1.0) < 1.e-9) ++d.unitPieces;
		if (piece.pieceDemand < 10.0 - 1.e-9) ++d.piecesBelow10;
	}
	int piecesInSplitCustomers = 0;
	for (int count : piecesPerOrigin) if (count > 1) { ++d.splitOriginalCustomers; piecesInSplitCustomers += count; }
	d.averagePiecesPerSplitCustomer = d.splitOriginalCustomers == 0 ? 0.0 :
		static_cast<double>(piecesInSplitCustomers) / d.splitOriginalCustomers;
	if (!values.empty()) {
		d.minPiece = *std::min_element(values.begin(), values.end());
		d.maxPiece = *std::max_element(values.begin(), values.end());
		d.meanPiece = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
		d.uniquePieceSizes = values;
		std::sort(d.uniquePieceSizes.begin(), d.uniquePieceSizes.end());
		d.uniquePieceSizes.erase(std::unique(d.uniquePieceSizes.begin(), d.uniquePieceSizes.end()), d.uniquePieceSizes.end());
	}
	int coveredPieces = 0;
	for (size_t i = 0; i < result.pieces.size(); ++i) {
		bool covered = false;
		for (size_t j = i + 1; j < result.pieces.size(); ++j) {
			if (result.pieces[i].originalCustomer == result.pieces[j].originalCustomer) continue;
			const double gap = std::fabs(capacity - result.pieces[i].pieceDemand - result.pieces[j].pieceDemand);
			if (gap < 1.e-9) ++d.exactTwoPieceComplementPairs;
			if (gap <= 0.10 * capacity + 1.e-9) { ++d.withinTenPercentComplementPairs; covered = true; }
		}
		if (!covered) {
			for (size_t j = 0; j < i; ++j) {
				if (result.pieces[i].originalCustomer == result.pieces[j].originalCustomer) continue;
				if (std::fabs(capacity - result.pieces[i].pieceDemand - result.pieces[j].pieceDemand) <= 0.10 * capacity + 1.e-9) { covered = true; break; }
			}
		}
		if (covered) ++coveredPieces;
	}
	d.capacityComplementScore = result.pieces.empty() ? 0.0 : static_cast<double>(coveredPieces) / result.pieces.size();
}
}

const char* demandSplitStrategyName(DemandSplitStrategy strategy)
{
	switch (strategy) {
	case DemandSplitStrategy::CURRENT_RLHGS: return "CURRENT_RLHGS";
	case DemandSplitStrategy::NO_SPLIT: return "NO_SPLIT";
	case DemandSplitStrategy::FF2S: return "FF2S";
	case DemandSplitStrategy::FF2S_PB_GCD: return "FF2S_PB_GCD";
	case DemandSplitStrategy::GTS_BCS_STYLE: return "GTS_BCS_STYLE";
	}
	return "CURRENT_RLHGS";
}

DemandSplitStrategy parseDemandSplitStrategy(const std::string& value)
{
	if (value == "CURRENT_RLHGS" || value == "S0") return DemandSplitStrategy::CURRENT_RLHGS;
	if (value == "NO_SPLIT" || value == "S1") return DemandSplitStrategy::NO_SPLIT;
	if (value == "FF2S" || value == "S2") return DemandSplitStrategy::FF2S;
	if (value == "FF2S_PB_GCD" || value == "S3") return DemandSplitStrategy::FF2S_PB_GCD;
	if (value == "GTS_BCS_STYLE" || value == "S4") return DemandSplitStrategy::GTS_BCS_STYLE;
	throw std::invalid_argument("Unknown demand split strategy: " + value);
}

SplitResult buildSplitRepresentation(const OriginalInstance& instance, DemandSplitStrategy strategy, bool /* roundDistances */, double eta)
{
	if (instance.demand.empty() || instance.demand.size() != instance.x.size())
		throw std::invalid_argument("Invalid original instance supplied to buildSplitRepresentation");
	SplitResult result;
	result.strategy = strategy;
	result.x.push_back(instance.x[0]); result.y.push_back(instance.y[0]);
	result.earliest.push_back(instance.earliest[0]); result.latest.push_back(instance.latest[0]);
	result.service.push_back(instance.service[0]); result.demand.push_back(instance.demand[0]);
	result.corresOrigin.push_back(0);
	for (int customer = 1; customer < static_cast<int>(instance.demand.size()); ++customer)
	{
		const int pb = (strategy == DemandSplitStrategy::FF2S_PB_GCD || strategy == DemandSplitStrategy::GTS_BCS_STYLE)
			? localPbGcd(instance, customer, eta) : 0;
		std::vector<int> pieces;
		switch (strategy) {
		case DemandSplitStrategy::CURRENT_RLHGS: pieces = currentRlhgsPieces(instance, customer); break;
		case DemandSplitStrategy::NO_SPLIT: pieces = capacityOnlyPieces(roundedInt(instance.demand[customer]), roundedInt(instance.vehicleCapacity)); break;
		case DemandSplitStrategy::FF2S: pieces = ff2sPieces(instance, customer); break;
		case DemandSplitStrategy::FF2S_PB_GCD: pieces = legacyPbPieces(instance, customer, pb); break;
		case DemandSplitStrategy::GTS_BCS_STYLE: pieces = gtsBcsReconstructionPieces(instance, customer, pb); break;
		}
		const int assigned = std::accumulate(pieces.begin(), pieces.end(), 0);
		if (assigned != roundedInt(instance.demand[customer]) || std::any_of(pieces.begin(), pieces.end(), [](int q) { return q <= 0; }))
			throw std::logic_error("Demand conservation failed for original customer " + std::to_string(customer));
		for (int k = 0; k < static_cast<int>(pieces.size()); ++k) {
			result.x.push_back(instance.x[customer]); result.y.push_back(instance.y[customer]);
			result.earliest.push_back(instance.earliest[customer]); result.latest.push_back(instance.latest[customer]);
			result.service.push_back(instance.service[customer]); result.demand.push_back(pieces[k]);
			result.corresOrigin.push_back(customer);
			result.pieces.push_back({ customer, instance.demand[customer], instance.commodity[customer], instance.x[customer], instance.y[customer],
				instance.earliest[customer], instance.latest[customer], instance.service[customer], pb, k + 1, static_cast<double>(pieces[k]) });
		}
	}
	const size_t n = result.demand.size();
	result.distance.assign(n, std::vector<double>(n, 0.0));
	for (size_t i = 0; i < n; ++i) for (size_t j = 0; j < n; ++j) {
		result.distance[i][j] = euclideanRoutingDistance2(
			result.x[i], result.y[i], result.x[j], result.y[j]);
	}
	computeDiagnostics(result, instance.vehicleCapacity);
	return result;
}

void writeSplitPieceCsv(const SplitResult& result, const std::string& path)
{
	std::ofstream out(path);
	if (!out) throw std::runtime_error("Unable to write split CSV: " + path);
	out << "originalCustomer,originalDemand,strategy,pbGcd,pieceIndex,pieceDemand\n";
	for (const auto& piece : result.pieces)
		out << piece.originalCustomer << ',' << piece.originalDemand << ',' << demandSplitStrategyName(result.strategy) << ','
			<< piece.pbGcd << ',' << piece.pieceIndex << ',' << piece.pieceDemand << '\n';
}

void printSplitDiagnostics(const SplitResult& result)
{
	const auto& d = result.diagnostics;
	std::cout << "[SPLIT]\nstrategy=" << demandSplitStrategyName(result.strategy)
		<< "\noriginalCustomers=" << d.originalCustomers
		<< "\npseudoCustomers=" << d.pseudoCustomers
		<< "\nsplitOriginalCustomers=" << d.splitOriginalCustomers
		<< "\ntotalPieces=" << d.pseudoCustomers
		<< "\naveragePiecesPerSplitCustomer=" << d.averagePiecesPerSplitCustomer
		<< "\nminPiece=" << d.minPiece << "\nmaxPiece=" << d.maxPiece << "\nmeanPiece=" << d.meanPiece
		<< "\nunitPieces=" << d.unitPieces << "\npiecesBelow10=" << d.piecesBelow10 << "\nuniquePieceSizes=";
	for (size_t i = 0; i < d.uniquePieceSizes.size(); ++i) std::cout << (i == 0 ? "" : ";") << d.uniquePieceSizes[i];
	std::cout << "\nexactTwoPieceComplementPairs=" << d.exactTwoPieceComplementPairs
		<< "\nwithinTenPercentComplementPairs=" << d.withinTenPercentComplementPairs
		<< "\ncapacityComplementScore=" << d.capacityComplementScore << std::endl;
}
