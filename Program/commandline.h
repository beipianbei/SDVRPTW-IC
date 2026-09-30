/*MIT License

Copyright(c) 2020 Thibaut Vidal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <iostream>
#include <string>
#include <climits>
#include "AlgorithmParameters.h"
#include "DemandSplit.h"

enum class SearchGuideMode { OFFICIAL, INTEGER_SURROGATE };

class CommandLine
{
public:
	AlgorithmParameters ap = default_algorithm_parameters();

	int nbVeh		 = INT_MAX;		// Number of vehicles. Default value: infinity
	std::string pathInstance;		// Instance path
	std::string pathSolution;		// Solution path
	bool verbose     = true;
	bool isRoundingInteger = true;
	std::string variant = "CUSTOM";
	DemandSplitStrategy splitStrategy = DemandSplitStrategy::CURRENT_RLHGS;
	std::string splitCsvPath;
	bool testDynamicQuantity = false;
	bool testAdaptiveResplit = false;
	bool testDynamicLocalImprove = false;
	bool testJointResplit = false;
	bool testJointInsertionAudit = false;
	bool testBoundedUphillDynamicLS = false;
	bool useBoundedUphillDynamicLS = false;
	bool useDynamicElite = false;
	bool testDistancePrecision = false;
	std::string reevaluateSolutionPath;
	std::string initialSolutionPath;
	SearchGuideMode searchGuide = SearchGuideMode::OFFICIAL;
	bool trajectoryAudit = false;
	// Causal warm-start audit switches. They are applied only to the A6
	// NO_SPLIT warm sub-run; the formal main search remains official.
	bool warmLegacyIntegerDecision = false;
	// If specified, use legacy decision costs only for this fraction of the
	// warm-start budget, then continue the same warm search with official costs.
	// A negative value means "not specified" and preserves the original W1 flag.
	double warmLegacyFraction = -1.0;
	bool warmInitialInfeasibleRepair = false;
	// Disabled unless -drTrigger is explicitly set to a positive value.
	int drTrigger = 0;
	double drRemovalRatio = 0.15;
	int dynamicTrigger = 200;
	bool resplitDiagnosticAllCustomers = false;
	double resplitAcceptanceGap = 5.0;
	int dynamicLSNearbyRoutes = 3;
	int dynamicLSMaxPasses = 20;
	int jointMaxPairs = 30;
	int jointCandidateRoutes = 6;
	int jointMaxPatterns = 100;
	int jointCandidatesPerPair = 300;
	double jointAcceptanceGap = 10.0;
	int topInsertionPositions = 3;
	double dynamicUphillMaxDelta = 3.0;
	double dynamicUphillMaxCumulativeDelta = 5.0;
	int dynamicUphillMaxMoves = 2;
	int dynamicRecoveryMoves = 5;
	int dynamicBeamWidth = 5;
	int resplitAttempts = 500;
	int resplitK = 5;
	int resplitMaxSplits = 3;

	// Reads the line of command and extracts possible options
	CommandLine(int argc, char* argv[])
	{
		if (argc % 2 != 1 || argc < 3)
		{
			std::cout << "----- NUMBER OF COMMANDLINE ARGUMENTS IS INCORRECT: " << argc << std::endl;
			display_help(); throw std::string("Incorrect line of command");
		}
		else
		{
			pathInstance = std::string(argv[1]);
			pathSolution = std::string(argv[2]);
			for (int i = 3; i < argc; i += 2)
			{
				if (std::string(argv[i]) == "-t")
					ap.timeLimit = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-it")
					ap.nbIter  = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-seed")
					ap.seed    = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-veh")
					nbVeh = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-round")
					isRoundingInteger = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-log")
					verbose = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbGranular")
					ap.nbGranular = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-mu")
					ap.mu = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-lambda")
					ap.lambda = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbElite")
					ap.nbElite = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbClose")
					ap.nbClose = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterPenaltyManagement")
					ap.nbIterPenaltyManagement = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterTraces")
					ap.nbIterTraces = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-targetFeasible")
					ap.targetFeasible = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyIncrease")
					ap.penaltyIncrease = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyDecrease")
					ap.penaltyDecrease = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-variant")
					applyVariant(std::string(argv[i + 1]));
				else if (std::string(argv[i]) == "-bcsSplit") ap.useBCSSplit = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-warmStart") ap.useNoSplitWarmStart = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-warmFraction") ap.warmStartFraction = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-bcsEta") ap.bcsEta = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-alns") ap.useAdaptiveLNS = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-lnsTrigger") ap.lnsTrigger = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-lnsIterations") ap.lnsIterations = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-rl") ap.useSuccessorLearning = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-splitStrategy") splitStrategy = parseDemandSplitStrategy(argv[i + 1]);
				else if (std::string(argv[i]) == "-splitCsv") splitCsvPath = argv[i + 1];
				else if (std::string(argv[i]) == "-testDynamicQuantity") testDynamicQuantity = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-testAdaptiveResplit") testAdaptiveResplit = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-testDynamicLocalImprove") {
					testDynamicLocalImprove = atoi(argv[i + 1]) != 0;
					if (testDynamicLocalImprove) testAdaptiveResplit = true;
				}
				else if (std::string(argv[i]) == "-testJointResplit") testJointResplit = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-testJointInsertionAudit") testJointInsertionAudit = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-testBoundedUphillDynamicLS") {
					testBoundedUphillDynamicLS = atoi(argv[i + 1]) != 0;
					if (testBoundedUphillDynamicLS) useBoundedUphillDynamicLS = true;
				}
				else if (std::string(argv[i]) == "-useBoundedUphillDynamicLS") useBoundedUphillDynamicLS = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-dynamicElite") useDynamicElite = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-dynamicTrigger") dynamicTrigger = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-testDistancePrecision") testDistancePrecision = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-reevaluateSolution") reevaluateSolutionPath = argv[i + 1];
				else if (std::string(argv[i]) == "-initialSolution") initialSolutionPath = argv[i + 1];
				else if (std::string(argv[i]) == "-searchGuide") {
					std::string value = argv[i + 1];
					if (value == "OFFICIAL") searchGuide = SearchGuideMode::OFFICIAL;
					else if (value == "INTEGER_SURROGATE") searchGuide = SearchGuideMode::INTEGER_SURROGATE;
					else throw std::string("Unknown searchGuide: " + value);
				}
				else if (std::string(argv[i]) == "-trajectoryAudit") trajectoryAudit = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-warmLegacyIntegerDecision") warmLegacyIntegerDecision = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-warmLegacyFraction") {
					warmLegacyFraction = atof(argv[i + 1]);
					if (warmLegacyFraction < 0.0 || warmLegacyFraction > 1.0)
						throw std::string("warmLegacyFraction must be in [0,1]");
				}
				else if (std::string(argv[i]) == "-warmInitialInfeasibleRepair") warmInitialInfeasibleRepair = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-drTrigger") {
					drTrigger = atoi(argv[i + 1]);
					if (drTrigger < 0) throw std::string("drTrigger must be non-negative");
				}
				else if (std::string(argv[i]) == "-drRemovalRatio") {
					drRemovalRatio = atof(argv[i + 1]);
					if (drRemovalRatio <= 0.0 || drRemovalRatio >= 1.0)
						throw std::string("drRemovalRatio must be in (0,1)");
				}
				else if (std::string(argv[i]) == "-resplitDiagnosticAllCustomers") resplitDiagnosticAllCustomers = atoi(argv[i + 1]) != 0;
				else if (std::string(argv[i]) == "-resplitAcceptanceGap") resplitAcceptanceGap = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-dynamicLSNearbyRoutes") dynamicLSNearbyRoutes = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-dynamicLSMaxPasses") dynamicLSMaxPasses = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-jointMaxPairs") jointMaxPairs = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-jointCandidateRoutes") jointCandidateRoutes = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-jointMaxPatterns") jointMaxPatterns = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-jointCandidatesPerPair") jointCandidatesPerPair = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-jointAcceptanceGap") jointAcceptanceGap = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-topInsertionPositions") topInsertionPositions = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-dynamicUphillMaxDelta") dynamicUphillMaxDelta = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-dynamicUphillMaxCumulativeDelta") dynamicUphillMaxCumulativeDelta = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-dynamicUphillMaxMoves") dynamicUphillMaxMoves = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-dynamicRecoveryMoves") dynamicRecoveryMoves = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-dynamicBeamWidth") dynamicBeamWidth = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-resplitAttempts") resplitAttempts = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-resplitK") resplitK = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-resplitMaxSplits") resplitMaxSplits = atoi(argv[i + 1]);
				else
				{
					std::cout << "----- ARGUMENT NOT RECOGNIZED: " << std::string(argv[i]) << std::endl;
					display_help(); throw std::string("Incorrect line of command");
				}
			}
		}
	}

	void applyVariant(const std::string& variant)
	{
		this->variant = variant;
		ap.useNoSplitWarmStart = (variant == "A2" || variant == "A3" || variant == "A6");
		ap.useBCSSplit = (variant == "A3" || variant == "A5" || variant == "A6");
		ap.useAdaptiveLNS = (variant == "A4" || variant == "A5" || variant == "A6");
		if (ap.useBCSSplit) splitStrategy = DemandSplitStrategy::FF2S_PB_GCD;
		if (variant != "A0" && variant != "A1" && variant != "A2" && variant != "A3" &&
			variant != "A4" && variant != "A5" && variant != "A6")
			throw std::string("Unknown ablation variant: " + variant);
	}

	// Printing information about how to use the code
	void display_help()
	{
		std::cout << std::endl;
		std::cout << "-------------------------------------------------- HGS-CVRP algorithm (2020) ---------------------------------------------------" << std::endl;
		std::cout << "Call with: ./hgs instancePath solPath [-it nbIter] [-t myCPUtime] [-seed mySeed] [-veh nbVehicles] [-log verbose]               " << std::endl;
		std::cout << "[-it <int>] sets a maximum number of iterations without improvement. Defaults to 20,000                                         " << std::endl;
		std::cout << "[-t <double>] sets a time limit in seconds. If this parameter is set the code will be run iteratively until the time limit      " << std::endl;
		std::cout << "[-seed <int>] sets a fixed seed. Defaults to 0                                                                                  " << std::endl;
		std::cout << "[-veh <int>] sets a prescribed fleet size. Otherwise a reasonable UB on the the fleet size is calculated                        " << std::endl;
		std::cout << "[-round <bool>] rounding the distance to the nearest integer or not. It can be 0 (not rounding) or 1 (rounding). Defaults to 1. " << std::endl;
		std::cout << "[-log <bool>] sets the verbose level of the algorithm log. It can be 0 or 1. Defaults to 1.                                     " << std::endl;
		std::cout << std::endl;
		std::cout << "Additional Arguments:                                                                                                           " << std::endl;
		std::cout << "[-nbIterTraces <int>] Number of iterations between traces display during HGS execution. Defaults to 500                         " << std::endl;
		std::cout << "[-nbGranular <int>] Granular search parameter, limits the number of moves in the RI local search. Defaults to 20                " << std::endl;
		std::cout << "[-mu <int>] Minimum population size. Defaults to 25                                                                             " << std::endl;
		std::cout << "[-lambda <int>] Number of solutions created before reaching the maximum population size (i.e., generation size). Defaults to 40 " << std::endl;
		std::cout << "[-nbElite <int>] Number of elite individuals. Defaults to 5                                                                     " << std::endl;
		std::cout << "[-nbClose <int>] Number of closest solutions/individuals considered when calculating diversity contribution. Defaults to 4      " << std::endl;
		std::cout << "[-nbIterPenaltyManagement <int>] Number of iterations between penalty updates. Defaults to 100                                  " << std::endl;
		std::cout << "[-targetFeasible <double>] target ratio of feasible individuals between penalty updates. Defaults to 0.2                        " << std::endl;
		std::cout << "[-penaltyIncrease <double>] penalty increase if insufficient feasible individuals between penalty updates. Defaults to 1.2      " << std::endl;
		std::cout << "[-penaltyDecrease <double>] penalty decrease if sufficient feasible individuals between penalty updates. Defaults to 0.85       " << std::endl;
		std::cout << "[-variant <A0..A6>] selects a reproducible ablation configuration                                                    " << std::endl;
		std::cout << "[-splitStrategy CURRENT_RLHGS|NO_SPLIT|FF2S|FF2S_PB_GCD|GTS_BCS_STYLE] [-splitCsv path]" << std::endl;
		std::cout << "[-bcsSplit 0|1] [-bcsEta ratio] [-warmStart 0|1] [-warmFraction ratio] [-alns 0|1] [-lnsTrigger n] [-lnsIterations n]" << std::endl;
		std::cout << "[-testAdaptiveResplit 0|1] [-resplitAttempts n] [-resplitK n] [-resplitMaxSplits 1..3] [-resplitDiagnosticAllCustomers 0|1]" << std::endl;
		std::cout << "[-testDynamicLocalImprove 0|1] [-resplitAcceptanceGap gap] [-dynamicLSNearbyRoutes n] [-dynamicLSMaxPasses n]" << std::endl;
		std::cout << "[-testJointResplit 0|1] [-jointMaxPairs n] [-jointCandidateRoutes n] [-jointMaxPatterns n] [-jointCandidatesPerPair n] [-jointAcceptanceGap gap]" << std::endl;
		std::cout << "[-testJointInsertionAudit 0|1] [-topInsertionPositions n]" << std::endl;
		std::cout << "[-testBoundedUphillDynamicLS 0|1] [-useBoundedUphillDynamicLS 0|1] [-dynamicUphillMaxDelta d] [-dynamicUphillMaxCumulativeDelta d] [-dynamicUphillMaxMoves n] [-dynamicRecoveryMoves n] [-dynamicBeamWidth n]" << std::endl;
		std::cout << "[-dynamicElite 0|1] [-dynamicTrigger n] enables the isolated R3_STAGE1_DYNAMIC_ELITE side path" << std::endl;
		std::cout << "[-testDistancePrecision 0|1] prints ten two-decimal matrix consistency checks" << std::endl;
		std::cout << "[-reevaluateSolution path] read an old .sol, recompute with current evaluator/validator, and exit without search" << std::endl;
		std::cout << "[-initialSolution path] inject a validated old .sol into the current HGS initial feasible population" << std::endl;
		std::cout << "[-searchGuide OFFICIAL|INTEGER_SURROGATE] uses an optional route-cost search surrogate while reporting official cost" << std::endl;
		std::cout << "[-trajectoryAudit 0|1] emits read-only initial-population, restart, and new-best trajectory records" << std::endl;
		std::cout << "[-warmLegacyIntegerDecision 0|1] [-warmLegacyFraction 0..1] [-warmInitialInfeasibleRepair 0|1] warm-start decision guidance" << std::endl;
		std::cout << "[-drTrigger iterations] [-drRemovalRatio ratio] enables main-stage related destroy + regret-2 repair" << std::endl;
		std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
		std::cout << std::endl;
	};
};
#endif
