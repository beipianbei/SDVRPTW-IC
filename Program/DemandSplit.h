#ifndef DEMAND_SPLIT_H
#define DEMAND_SPLIT_H

#include <string>
#include <vector>

enum class DemandSplitStrategy {
	CURRENT_RLHGS,
	NO_SPLIT,
	FF2S,
	FF2S_PB_GCD,
	GTS_BCS_STYLE
};

struct OriginalInstance
{
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> earliest;
	std::vector<double> latest;
	std::vector<double> service;
	std::vector<double> demand;
	std::vector<int> commodity;
	std::vector<std::vector<double>> distance;
	double vehicleCapacity = 0.0;
	double durationLimit = 0.0;
	bool isDurationConstraint = true;
};

struct SplitPieceRecord
{
	int originalCustomer = 0;
	double originalDemand = 0.0;
	int commodityType = 0;
	double coordinateX = 0.0;
	double coordinateY = 0.0;
	double earliest = 0.0;
	double latest = 0.0;
	double serviceTime = 0.0;
	int pbGcd = 0;
	int pieceIndex = 0;
	double pieceDemand = 0.0;
};

struct SplitDiagnostics
{
	int originalCustomers = 0;
	int pseudoCustomers = 0;
	int splitOriginalCustomers = 0;
	double averagePiecesPerSplitCustomer = 0.0;
	double minPiece = 0.0;
	double maxPiece = 0.0;
	double meanPiece = 0.0;
	int unitPieces = 0;
	int piecesBelow10 = 0;
	std::vector<double> uniquePieceSizes;
	int exactTwoPieceComplementPairs = 0;
	int withinTenPercentComplementPairs = 0;
	double capacityComplementScore = 0.0;
};

struct SplitResult
{
	DemandSplitStrategy strategy = DemandSplitStrategy::CURRENT_RLHGS;
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> earliest;
	std::vector<double> latest;
	std::vector<double> service;
	std::vector<double> demand;
	std::vector<int> corresOrigin;
	std::vector<std::vector<double>> distance;
	std::vector<SplitPieceRecord> pieces;
	SplitDiagnostics diagnostics;
};

const char* demandSplitStrategyName(DemandSplitStrategy strategy);
DemandSplitStrategy parseDemandSplitStrategy(const std::string& value);
SplitResult buildSplitRepresentation(const OriginalInstance& instance,
	DemandSplitStrategy strategy, bool roundDistances, double eta = 0.05);
void writeSplitPieceCsv(const SplitResult& result, const std::string& path);
void printSplitDiagnostics(const SplitResult& result);

#endif
