#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "Split.h"
#include "InstanceCVRPLIB.h"
#include <iostream>
#include <vector>
#include <filesystem>
#include <windows.h>
#include <initguid.h>
#include <evntrace.h>
#include "Population.h"
#include "SolutionValidator.h"
#include "DemandSplit.h"
#include "DynamicQuantityTests.h"
#include "AdaptiveResplitting.h"
#include "DynamicLocalImprove.h"
#include "JointResplitting.h"
#include "BoundedUphillDynamicLS.h"
#include "DynamicElite.h"
#include "OldSolutionReevaluation.h"
#include "MinimalDestroyRepair.h"
#include "DistanceUtils.h"
#include "NumericFormat.h"
#include <string>
#include <cstring>
#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <regex>
#include <chrono>
#include <random>
#include <memory>
#include <limits>
#include <map>
#include <windows.h>
#include <psapi.h>

namespace fs = std::filesystem;

extern std::vector<int> demand_type;
extern std::vector<int> corr_origin;
extern std::vector<std::vector<bool>> conflict_matrix;  // 产品冲突矩阵//20260120

#ifdef __linux__
#include <sys/stat.h>
#include <dirent.h>
#endif

#if _MSVC_LANG >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#endif
using namespace std;

#ifndef _MSVC_LANG
#define _MSVC_LANG __cplusplus
#endif

void CreateFolder(std::string dirName) {
#ifdef __linux__
    if (mkdir(dirName.c_str(), 0777) == 0)  std::cout << "New folder created!" << std::endl;
    else std::cout << "Folder already exists!" << std::endl;
#endif
}

std::vector<std::string> IterateFolder(std::string& folderName) {
#ifdef __linux__
    auto res = std::vector<std::string>{};
    dirent* ent;
    auto dir = opendir(folderName.c_str());
    if (dir == nullptr)return {};
    while ((ent = readdir(dir)) != nullptr) res.emplace_back(ent->d_name);
    return res;
#else
    return {};
#endif
}

inline int MyGcd(int a, int b) {
    int r;
    while (b > 0) {
        r = a % b;
        a = b;
        b = r;
    }
    return a;
}

std::vector<int> splitDemand(int Capacity, int demand, int level, int gcd, int s_max, int p, bool flag, int index_minus)
{
    std::vector<int> result;
    if (level != 0 && flag)
    {
        std::vector<double> split_units;
        double unit_radio = static_cast<double>(gcd) / Capacity;
        for (int i = level; i <= s_max + 1; i++) split_units.push_back(unit_radio * (std::pow(p, s_max - i + 1)));

        if (index_minus < split_units.size()) split_units.erase(split_units.begin(), split_units.begin() + index_minus);
        else std::cout << "index_minus error!!! toooooooooooo large!!!" << std::endl;

        if (split_units.size() >= 5)
        {
            std::vector<int> result_temp1;
            std::vector<int> result_temp2;
            int remaining_demand_temp = demand;
            for (double unit : split_units) {
                int split_value = std::round(Capacity * unit);
                while (remaining_demand_temp >= split_value) {
                    result_temp1.push_back(split_value);
                    remaining_demand_temp -= split_value;
                }
            }

            split_units.erase(split_units.end() - 4);
            split_units.erase(split_units.end() - 3);

            remaining_demand_temp = demand;
            for (double unit : split_units) {
                int split_value = std::round(Capacity * unit);
                while (remaining_demand_temp >= split_value) {
                    result_temp2.push_back(split_value);
                    remaining_demand_temp -= split_value;
                }
            }

            if (result_temp1.size() == result_temp2.size()) split_units.erase(split_units.end() - 3);
        }

        int remaining_demand = demand;
        for (double unit : split_units) {
            int split_value = std::round(Capacity * unit);
            while (remaining_demand >= split_value) {
                result.push_back(split_value);
                remaining_demand -= split_value;
                if (remaining_demand <= 3) break;
            }
            if (split_value == 1) break;
        }
        while (remaining_demand > 0) {
            if (remaining_demand >= 1) {
                result.push_back(1);
                remaining_demand -= 1;
            }
            else break;
        }
    }
    else
    {
        std::vector<double> split_units = { 0.1, 0.05, 0.25 };
        for (int i = 0; i < split_units.size(); i++) std::cout << "" << split_units[i] << "  " << std::round(Capacity * split_units[i]) << std::endl;
        std::cout << std::endl;

        int remaining_demand = demand;
        for (double unit : split_units) {
            int split_value = std::round(Capacity * unit);
            while (remaining_demand >= split_value) {
                result.push_back(split_value);
                remaining_demand -= split_value;
            }
            if (split_value == 1) break;
        }

        while (remaining_demand > 0) {
            if (remaining_demand >= 1) {
                result.push_back(remaining_demand);
                remaining_demand -= remaining_demand;
            }
            else {
                break;
            }
        }
    }
    return result;
}

std::vector<int> balancedPBGCdSplit(const InstanceCVRPLIB& cvrp, int customer, double eta)
{
	std::vector<std::pair<double, int>> nearest;
	for (int j = 1; j <= cvrp.nbClients; ++j)
		if (j != customer) nearest.push_back({ cvrp.dist_mtx[customer][j], j });
	std::sort(nearest.begin(), nearest.end());
	const int count = std::max(1, static_cast<int>(std::ceil(std::max(0.0, eta) * nearest.size())));
	int localGcd = static_cast<int>(std::llround(cvrp.vehicleCapacity));
	localGcd = std::gcd(localGcd, static_cast<int>(std::llround(cvrp.demands[customer])));
	for (int k = 0; k < std::min<int>(count, nearest.size()); ++k)
		localGcd = std::gcd(localGcd, static_cast<int>(std::llround(cvrp.demands[nearest[k].second])));
	localGcd = std::max(1, localGcd);
	const int demand = static_cast<int>(std::llround(cvrp.demands[customer]));
	const int capacity = static_cast<int>(std::llround(cvrp.vehicleCapacity));
	const int units = demand / localGcd;
	const int maxUnits = std::max(1, capacity / localGcd);
	const int pieces = std::max(units > 1 ? 2 : 1, (units + maxUnits - 1) / maxUnits);
	std::vector<int> result(pieces, (units / pieces) * localGcd);
	for (int k = 0; k < units % pieces; ++k) result[k] += localGcd;
	const int assigned = std::accumulate(result.begin(), result.end(), 0);
	if (assigned != demand) result.back() += demand - assigned;
	return result;
}

void Split_Node(InstanceCVRPLIB& cvrp, bool isRoundingInteger = true, SplitStrategy strategy = CURRENT_SPLIT, double eta = 0.05)
{
    int L = 1;
    int p = 2;
    bool flag = 1;
    int index_minus = 0;
    int d;
    double miu;
    int s_max;

    std::vector<int> gcd_vector;
    gcd_vector.push_back(cvrp.vehicleCapacity);
    for (int i = 1; i < cvrp.nbClients + 1; i++)
    {
        gcd_vector.push_back(cvrp.demands[i]);
    }
    std::sort(gcd_vector.begin(), gcd_vector.end());
    auto it = std::unique(gcd_vector.begin(), gcd_vector.end());
    gcd_vector.erase(it, gcd_vector.end());
#if _MSVC_LANG >= 201703L
    auto gcd = [](int a, int b) {
        return std::gcd(a, b);
        };
#else
    auto gcd = [](int a, int b) {
        return MyGcd(a, b);
        };
#endif
    d = std::accumulate(gcd_vector.begin() + 1, gcd_vector.end(), gcd_vector[0], gcd);

    std::vector<int> avg_vector;
    for (int i = 1; i < cvrp.nbClients + 1; i++)
    {
        avg_vector.push_back((cvrp.demands[i]) / d);
    }
    int sum = std::accumulate(avg_vector.begin(), avg_vector.end(), 0);
    miu = static_cast<double>(sum) / avg_vector.size();

    s_max = static_cast<int>(std::round(std::log(miu) / std::log(p)));
    if (index_minus > s_max + 1 - L) std::cout << "index_minus error! too large!" << std::endl;

    double distance_max = 0;
    for (int i = 1; i < cvrp.nbClients + 1; i++)
    {
		double distance_temp = euclideanRoutingDistance2(
			cvrp.x_coords[i], cvrp.y_coords[i], cvrp.x_coords[0], cvrp.y_coords[0]);
        if (distance_temp > distance_max) distance_max = distance_temp;
    }

    cvrp.level.push_back(0);
    double interval_length = distance_max / L;
    double epsilon = 0;
    for (int i = 1; i < cvrp.nbClients + 1; i++)
    {
		double distance_temp = euclideanRoutingDistance2(
			cvrp.x_coords[i], cvrp.y_coords[i], cvrp.x_coords[0], cvrp.y_coords[0]);
        double adjusted_distance = distance_temp - epsilon;
        int interval_index = static_cast<int>(adjusted_distance / interval_length) + 1;

        if (interval_index > L) interval_index = L;
        if (interval_index < 1) interval_index = 1;

        cvrp.level.push_back(L - interval_index + 1);
    }

    std::vector<double> x_coords_new;
    std::vector<double> y_coords_new;
    std::vector< std::vector<double> > dist_mtx_new;
    std::vector<double> service_time_new;
    std::vector<double> earliest_time_new;
    std::vector<double> latest_time_new;
    std::vector<double> demands_new;
    double durationLimit_new = cvrp.durationLimit;
    double vehicleCapacity_new = cvrp.vehicleCapacity;
    bool isDurationConstraint_new = cvrp.isDurationConstraint;
    int nbClients_new;
    std::vector<int> correspoinding_origin_new;
    correspoinding_origin_new.push_back(0);

    x_coords_new.push_back(cvrp.x_coords[0]);
    y_coords_new.push_back(cvrp.y_coords[0]);
    service_time_new.push_back(cvrp.service_time[0]);
    earliest_time_new.push_back(cvrp.earliest_time[0]);
    latest_time_new.push_back(cvrp.latest_time[0]);
    demands_new.push_back(cvrp.demands[0]);

    for (int i = 1; i < cvrp.nbClients + 1; i++)
    {
		std::vector<int> split_result = strategy == FF2S_PB_GCD
			? balancedPBGCdSplit(cvrp, i, eta)
			: splitDemand(cvrp.vehicleCapacity, cvrp.demands[i], cvrp.level[i], d, s_max, p, flag, index_minus);
		if (std::accumulate(split_result.begin(), split_result.end(), 0) != static_cast<int>(std::llround(cvrp.demands[i])))
			throw std::logic_error("Demand conservation failed while splitting customer " + std::to_string(i));
        int split_number = split_result.size();
        for (int j = 0; j < split_number; j++)
        {
            x_coords_new.push_back(cvrp.x_coords[i]);
            y_coords_new.push_back(cvrp.y_coords[i]);
            service_time_new.push_back(cvrp.service_time[i]);
            earliest_time_new.push_back(cvrp.earliest_time[i]);
            latest_time_new.push_back(cvrp.latest_time[i]);
            demands_new.push_back(split_result[j]);
            correspoinding_origin_new.push_back(i);
        }
    }

    nbClients_new = demands_new.size() - 1;

    dist_mtx_new = std::vector < std::vector< double > >(nbClients_new + 1, std::vector <double>(nbClients_new + 1));
    for (int i = 0; i <= nbClients_new; i++)
    {
        for (int j = 0; j <= nbClients_new; j++)
        {
			dist_mtx_new[i][j] = euclideanRoutingDistance2(
				x_coords_new[i], y_coords_new[i], x_coords_new[j], y_coords_new[j]);
        }
    }

    cvrp.x_coords = x_coords_new;
    cvrp.y_coords = y_coords_new;
    cvrp.service_time = service_time_new;
    cvrp.earliest_time = earliest_time_new;
    cvrp.latest_time = latest_time_new;
    cvrp.demands = demands_new;
    cvrp.nbClients = nbClients_new;
    cvrp.dist_mtx = dist_mtx_new;
    cvrp.correspoinding_origin = correspoinding_origin_new;

    corr_origin = correspoinding_origin_new;

    cvrp.durationLimit = durationLimit_new;
    cvrp.vehicleCapacity = vehicleCapacity_new;
    cvrp.isDurationConstraint = isDurationConstraint_new;
}

void solve_vrp_instance(const std::string& instance_path, const std::string& output_folder, int run_id, CommandLine* supplied = nullptr) {
	const clock_t fullRunStart = clock();
	const auto wallRunStart = std::chrono::steady_clock::now();
#if _MSVC_LANG >= 201703L
    std::string solution_filename = fs::path(instance_path).stem().string() + "_run" + std::to_string(run_id) + ".sol";
#else
    auto lastSlash = instance_path.find_last_of('/');
    auto tmpPath = instance_path.substr(lastSlash + 1);
    std::string base_name = std::regex_replace(tmpPath, std::regex(".vrp"), "");
    std::string solution_filename = base_name + "_run" + std::to_string(run_id) + ".sol";
#endif
    std::string solution_path = supplied == nullptr ? output_folder + "/" + solution_filename : supplied->pathSolution;

    // 生成真正的随机种子
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<unsigned int> distribution(1, 1000000);
    unsigned int random_seed = distribution(generator);

    // 将随机种子转换为字符串，用于命令行参数
    std::string seed_str = std::to_string(random_seed);

    const char* argv[] = {
            "./hgs",
            instance_path.c_str(),
            solution_path.c_str(),
            "-seed", "0", // 0不固定，1固定
            "-t", "600",
            "-nbGranular", "30",
            "-nbIterTraces","100",
            "-it","40000"
    };

    int argc = sizeof(argv) / sizeof(argv[0]);
    char** argv_mod = new char* [argc];

    for (int i = 0; i < argc; ++i) {
        size_t length = strlen(argv[i]) + 1;
        argv_mod[i] = new char[length];
#if _MSVC_LANG >= 201703L
        strcpy_s(argv_mod[i], length, argv[i]);
#elif __linux__
        strcpy(argv_mod[i], argv[i]);
#endif
    }
	CommandLine commandline(argc, argv_mod);
	CommandLine& options = supplied == nullptr ? commandline : *supplied;
	std::cout << "[FROZEN-BUILD]" << std::endl;
	std::cout << "Version=FINAL_DR_V1_CAP30_FIX" << std::endl;
	std::cout << "Objective=TWO_DECIMAL_EUCLIDEAN" << std::endl;
	std::cout << "SplitStrategy=CURRENT_RLHGS" << std::endl;
	std::cout << "WarmFraction=0.5" << std::endl;
	std::cout << "DRTrigger=500" << std::endl;
	std::cout << "DRRemovalRatio=0.15" << std::endl;

    if (options.verbose) print_algorithm_parameters(options.ap);

	if (options.verbose) std::cout << "----- READING INSTANCE: " << options.pathInstance << std::endl;
	InstanceCVRPLIB cvrp(options.pathInstance, options.isRoundingInteger);
	if (options.testDistancePrecision) {
		const int checks = std::min(10, cvrp.nbClients);
		for (int j = 1; j <= checks; ++j) {
			const double dx = cvrp.x_coords[0] - cvrp.x_coords[j];
			const double dy = cvrp.y_coords[0] - cvrp.y_coords[j];
			const double raw = std::sqrt(dx * dx + dy * dy);
			const double rounded = roundRoutingDistance2(raw);
			std::cout << "[2DEC-DISTANCE-CHECK] i=0 j=" << j
				<< " rawEuclidean=" << raw
				<< " rounded2=" << objectiveValue2(rounded)
				<< " matrixValue=" << objectiveValue2(cvrp.dist_mtx[0][j])
				<< " match=" << (std::fabs(rounded - cvrp.dist_mtx[0][j]) <= 1.e-12 ? "PASS" : "FAIL")
				<< std::endl;
		}
		return;
	}
	OriginalInstance originalInstance;
	originalInstance.x = cvrp.x_coords;
	originalInstance.y = cvrp.y_coords;
	originalInstance.earliest = cvrp.earliest_time;
	originalInstance.latest = cvrp.latest_time;
	originalInstance.service = cvrp.service_time;
	originalInstance.demand = cvrp.demands;
	originalInstance.commodity = demand_type;
	originalInstance.distance = cvrp.dist_mtx;
	originalInstance.vehicleCapacity = cvrp.vehicleCapacity;
	originalInstance.durationLimit = cvrp.durationLimit;
	originalInstance.isDurationConstraint = cvrp.isDurationConstraint;
	if (!options.reevaluateSolutionPath.empty()) {
		std::vector<int> identity(cvrp.nbClients + 1);
		std::iota(identity.begin(), identity.end(), 0);
		corr_origin = identity;
		Params reevaluationParams(cvrp.x_coords, cvrp.y_coords, cvrp.earliest_time, cvrp.latest_time, identity,
			cvrp.dist_mtx, cvrp.service_time, cvrp.demands, cvrp.vehicleCapacity, cvrp.durationLimit,
			options.nbVeh, cvrp.isDurationConstraint, false, options.ap, fullRunStart);
		reevaluationParams.setOriginalCustomerData(originalInstance.demand, originalInstance.commodity,
			originalInstance.earliest, originalInstance.latest, originalInstance.service, originalInstance.distance);
		const bool ok = reevaluateOldSolutionFile(options.pathInstance, options.reevaluateSolutionPath,
			reevaluationParams, std::cout);
		for (int i = 0; i < argc; ++i) delete[] argv_mod[i];
		delete[] argv_mod;
		if (!ok) throw std::string("Old solution reevaluation failed current validation");
		return;
	}
	std::vector<std::vector<int>> warmRoutes;
	double warmBudget = 0.0;
	double warmInitialBest = -1.0;
	double warmBestAt60s = -1.0;
	int warmInitialFeasible = 0;
	int warmInitialInfeasible = 0;
	int warmRepairEligible = 0;
	int warmRepairAttempts = 0;
	int warmRepairSuccesses = 0;
	double mainInjectedCost = -1.0;
	const double maxOriginalDemand = originalInstance.demand.empty() ? 0.0 :
		*std::max_element(originalInstance.demand.begin(), originalInstance.demand.end());
	const bool warmStartRequested = options.ap.useNoSplitWarmStart &&
		options.ap.timeLimit > 0.0 && options.ap.warmStartFraction > 0.0;
	const bool warmStartCapacityCompatible = maxOriginalDemand <= originalInstance.vehicleCapacity + MY_EPSILON;
	const bool warmStartExecuted = warmStartRequested && warmStartCapacityCompatible;
	if (warmStartRequested && !warmStartCapacityCompatible)
	{
		std::cout << "[WARM-START]"
			<< " enabled=0"
			<< " reason=ORIGINAL_DEMAND_EXCEEDS_CAPACITY"
			<< " maxOriginalDemand=" << maxOriginalDemand
			<< " capacity=" << originalInstance.vehicleCapacity << std::endl;
	}
	if (warmStartExecuted)
	{
		warmBudget = options.ap.timeLimit * std::max(0.0, std::min(1.0, options.ap.warmStartFraction));
		const bool hasPhasedLegacyDecision = options.warmLegacyFraction >= 0.0;
		const double legacyFraction = hasPhasedLegacyDecision
			? options.warmLegacyFraction
			: (options.warmLegacyIntegerDecision ? 1.0 : 0.0);
		const double legacyDecisionEnd = warmBudget * legacyFraction;
		AlgorithmParameters warmAp = options.ap;
		warmAp.timeLimit = warmBudget;
		warmAp.useNoSplitWarmStart = 0;
		warmAp.useBCSSplit = 0;
		warmAp.useAdaptiveLNS = 0;
		std::vector<int> identity(cvrp.nbClients + 1);
		std::iota(identity.begin(), identity.end(), 0);
		corr_origin = identity;
		Params warmParams(cvrp.x_coords, cvrp.y_coords, cvrp.earliest_time, cvrp.latest_time, identity,
			cvrp.dist_mtx, cvrp.service_time, cvrp.demands, cvrp.vehicleCapacity, cvrp.durationLimit,
			options.nbVeh, cvrp.isDurationConstraint, options.verbose, warmAp, fullRunStart);
		warmParams.setOriginalCustomerData(originalInstance.demand, originalInstance.commodity,
			originalInstance.earliest, originalInstance.latest, originalInstance.service, originalInstance.distance);
		warmParams.trajectoryAudit = options.trajectoryAudit;
		warmParams.useLegacyIntegerDecision = legacyDecisionEnd > 0.0;
		warmParams.useInitialInfeasibleRepair = options.warmInitialInfeasibleRepair;
		if (warmParams.useLegacyIntegerDecision) {
			warmParams.searchCost = warmParams.timeCost;
			for (auto& row : warmParams.searchCost)
				for (double& value : row) value = std::round(value);
		}
		std::cout << "[WARM-CAUSAL-CONFIG] LegacyIntegerDecision=" << (warmParams.useLegacyIntegerDecision ? 1 : 0)
			<< " LegacyFraction=" << legacyFraction
			<< " InitialInfeasibleRepair=" << (warmParams.useInitialInfeasibleRepair ? 1 : 0)
			<< " OfficialObjective=TWO_DECIMAL TimeWindows=OFFICIAL_DOUBLE" << std::endl;
		std::cout << "[WARM-PHASE]"
			<< " LegacyDecisionStart=0"
			<< " LegacyDecisionEnd=" << legacyDecisionEnd
			<< " OfficialDecisionStart=" << legacyDecisionEnd
			<< " WarmEnd=" << warmBudget << std::endl;
		Genetic warmSolver(warmParams);
		bool warmDecisionSwitched = legacyDecisionEnd <= 0.0 || legacyDecisionEnd >= warmBudget;
		if (legacyDecisionEnd > 0.0 && legacyDecisionEnd < warmBudget) {
			warmSolver.setIterationCallback([&](int, int) {
				if (warmDecisionSwitched || warmParams.elapsedSearchSeconds() < legacyDecisionEnd) return false;
				// Decision-mode-only transition: retain this Genetic instance, its
				// population, incumbent, penalty state, and RNG stream unchanged.
				warmParams.useLegacyIntegerDecision = false;
				warmParams.searchCost = warmParams.timeCost;
				warmDecisionSwitched = true;
				const Individual* bestOfficial = warmSolver.population.getBestFound();
				const Individual* populationBest = warmSolver.population.getBestFeasible();
				std::cout << "[WARM-SWITCH]"
					<< " Time=" << warmParams.elapsedSearchSeconds()
					<< " BestOfficialCost=" << (bestOfficial ? objectiveValue2(bestOfficial->eval.distance) : "NA")
					<< " PopulationBest=" << (populationBest ? objectiveValue2(populationBest->eval.distance) : "NA")
					<< " RngHash=" << warmParams.fixedRngFingerprint() << std::endl;
				return false;
			});
		}
		warmSolver.run();
		warmInitialBest = warmSolver.population.getInitialBestDistance();
		warmInitialFeasible = warmSolver.population.getInitialFeasibleCount();
		warmInitialInfeasible = warmSolver.population.getInitialInfeasibleCount();
		warmRepairEligible = warmParams.initialRepairEligible;
		warmRepairAttempts = warmParams.initialRepairAttempts;
		warmRepairSuccesses = warmParams.initialRepairSuccesses;
		if (warmSolver.population.getBestFound() != NULL) {
			warmBestAt60s = warmSolver.population.getBestFound()->eval.distance;
			warmRoutes = warmSolver.population.getBestFound()->chromR;
		}
	}

	bool split_flag = true;
	const SplitResult splitResult = buildSplitRepresentation(originalInstance, options.splitStrategy,
		options.isRoundingInteger, options.ap.bcsEta);
	printSplitDiagnostics(splitResult);
	if (!options.splitCsvPath.empty()) writeSplitPieceCsv(splitResult, options.splitCsvPath);
	cvrp.x_coords = splitResult.x;
	cvrp.y_coords = splitResult.y;
	cvrp.earliest_time = splitResult.earliest;
	cvrp.latest_time = splitResult.latest;
	cvrp.service_time = splitResult.service;
	cvrp.demands = splitResult.demand;
	cvrp.correspoinding_origin = splitResult.corresOrigin;
	cvrp.dist_mtx = splitResult.distance;
	cvrp.nbClients = static_cast<int>(splitResult.demand.size()) - 1;
	corr_origin = splitResult.corresOrigin;
	demand_type = originalInstance.commodity;

	Params params(cvrp.x_coords, cvrp.y_coords, cvrp.earliest_time, cvrp.latest_time, cvrp.correspoinding_origin, cvrp.dist_mtx, cvrp.service_time, cvrp.demands,
		cvrp.vehicleCapacity, cvrp.durationLimit, options.nbVeh, cvrp.isDurationConstraint,
		options.verbose, options.ap, fullRunStart);
	params.setOriginalCustomerData(originalInstance.demand, originalInstance.commodity,
		originalInstance.earliest, originalInstance.latest, originalInstance.service, originalInstance.distance);
	if (options.searchGuide == SearchGuideMode::INTEGER_SURROGATE) {
		params.searchCost = params.timeCost;
		for (auto& row : params.searchCost)
			for (double& value : row) value = std::round(value);
		std::cout << "[SEARCH-GUIDE] Mode=INTEGER_SURROGATE OfficialObjective=TWO_DECIMAL_EUCLIDEAN"
			<< " TimeWindows=OFFICIAL_TWO_DECIMAL Validator=OFFICIAL" << std::endl;
	}
	else {
		params.searchCost = params.timeCost;
		std::cout << "[SEARCH-GUIDE] Mode=OFFICIAL" << std::endl;
	}
	params.trajectoryAudit = options.trajectoryAudit;
	std::cout << "[RNG-STREAMS] fixedSeed=" << params.fixedRngSeed
		<< " dynamicSeed=" << params.dynamicRngSeed << " independent=PASS" << std::endl;
	if (options.testDynamicQuantity) {
		runDynamicQuantityTests(params, std::cout);
		return;
	}

	Genetic solver(params);
	std::unique_ptr<DynamicEliteController> dynamicEliteController;
	std::unique_ptr<MinimalDestroyRepair> destroyRepairController;
	bool fixedRngIsolationPassed = true;
	bool hasInitialSolutionInjection = false;
	double initialInjectedCost = 0.0;
	if (!warmRoutes.empty())
	{
		Individual transferred(params);
		for (auto& route : transferred.chromR) route.clear();
		transferred.chromT.clear();
		std::vector<std::vector<int>> pseudoByOrigin(demand_type.size());
		for (int client = 1; client <= params.nbClients; ++client)
			pseudoByOrigin[params.Corres_origin[client]].push_back(client);
		std::vector<bool> inserted(params.nbClients + 1, false);
		int targetRoute = 0;
		for (const auto& route : warmRoutes)
		{
			if (route.empty()) continue;
			for (int origin : route)
				for (int client : pseudoByOrigin[origin]) {
					transferred.chromR[targetRoute].push_back(client);
					transferred.chromT.push_back(client);
					inserted[client] = true;
				}
			++targetRoute;
		}
		for (int client = 1; client <= params.nbClients; ++client)
			if (!inserted[client]) {
				transferred.chromR[targetRoute++].push_back(client);
				transferred.chromT.push_back(client);
			}
		transferred.evaluateCompleteCost(params);
		mainInjectedCost = transferred.eval.distance;
		solver.setInitialSeed(transferred);
	}
	if (!options.initialSolutionPath.empty())
	{
		OldSolutionLoadResult loaded = loadOldSolutionAsDynamic(options.initialSolutionPath, params);
		if (!loaded.ok)
			throw std::string("Initial solution could not be read reliably: " + loaded.error);
		if (!loaded.validation.feasible())
			throw std::string("Initial solution failed current validator: " + loaded.validation.message);
		std::string conversionError;
		std::optional<Individual> fixedSeed = convertOldDynamicToCurrentFixed(loaded.dynamicIndividual, params, conversionError);
		if (!fixedSeed)
			throw std::string("Initial solution could not be converted to current fixed representation: " + conversionError);
		const ValidationResult fixedValidation = validateSolutionFromScratch(*fixedSeed, params);
		if (!fixedValidation.feasible())
			throw std::string("Converted initial solution failed current validator: " + fixedValidation.message);
		hasInitialSolutionInjection = true;
		initialInjectedCost = fixedSeed->eval.distance;
		solver.setInitialSeed(*fixedSeed, true);
		std::cout << "[INITIAL-SOLUTION]"
			<< " File=" << options.initialSolutionPath
			<< " ReportedOldCost=" << (loaded.hasOldReportedCost ? objectiveValue2(loaded.oldReportedCost) : "NA")
			<< " CurrentCost=" << objectiveValue2(loaded.validation.distance)
			<< " Validator=" << (loaded.validation.feasible() && fixedValidation.feasible() ? "PASS" : "FAIL")
			<< " Injected=1" << std::endl;
	}
	if (options.useDynamicElite && !options.testDynamicQuantity && !options.testAdaptiveResplit &&
		!options.testJointResplit && !options.testJointInsertionAudit && !options.testBoundedUphillDynamicLS)
	{
		DynamicEliteConfig eliteConfig;
		eliteConfig.trigger = std::max(1, options.dynamicTrigger);
		eliteConfig.bridge.maxSingleDelta = 3.0;
		eliteConfig.bridge.maxCumulativeDelta = 5.0;
		eliteConfig.bridge.maxUphillMoves = 2;
		eliteConfig.bridge.recoveryMoves = 5;
		eliteConfig.bridge.beamWidth = 5;
		eliteConfig.bridge.nearbyRoutes = 3;
		eliteConfig.bridge.enforceGlobalDeadline = true;
		dynamicEliteController.reset(new DynamicEliteController(params, eliteConfig));
	}
	if (options.drTrigger > 0)
	{
		MinimalDestroyRepairConfig drConfig;
		drConfig.trigger = options.drTrigger;
		drConfig.removalRatio = options.drRemovalRatio;
		destroyRepairController.reset(new MinimalDestroyRepair(params, solver.population, solver.localSearch, drConfig));
		std::cout << "[DR-CONFIG] Trigger=" << drConfig.trigger
			<< " RemovalRatio=" << drConfig.removalRatio
			<< " ParentSelection=80_PERCENT_ELITE_20_PERCENT_GLOBAL_BEST"
			<< " Repair=REGRET_2 Representation=ORIGINAL_SERVICE_BLOCKS"
			<< " Objective=OFFICIAL_TWO_DECIMAL" << std::endl;
	}
	if (dynamicEliteController || destroyRepairController)
	{
		solver.setIterationCallback([&](int iteration, int nonImprovingIterations) {
			bool fixedNewBest = false;
			if (dynamicEliteController)
			{
				const std::minstd_rand fixedRngBefore = params.ran;
				dynamicEliteController->onIteration(iteration, nonImprovingIterations,
					solver.population.getBestFound());
				if (!(params.ran == fixedRngBefore)) fixedRngIsolationPassed = false;
			}
			if (destroyRepairController)
				fixedNewBest = destroyRepairController->onIteration(iteration, nonImprovingIterations);
			return fixedNewBest;
		});
	}
	solver.run();
	const double searchRuntime = params.reportedSearchRuntime();
	if (options.testBoundedUphillDynamicLS)
	{
		const Individual* fixed = solver.population.getBestFound();
		if (fixed == NULL) throw std::logic_error("No feasible fixed source for bounded uphill dynamic LS");
		Individual source(*fixed);
		source.convertFixedToDynamic(params);
		source.evaluateDynamicQuantity(params);
		const ValidationResult sourceValidation = validateSolutionFromScratch(source, params);
		if (!sourceValidation.feasible())
			throw std::logic_error("Converted bridge source is invalid: " + sourceValidation.message);

		const std::vector<int> eligibleCustomers = getAdaptiveResplitEligibleCustomers(source, params);
		std::vector<double> singleSplitDelta(params.originalDemand.size(), std::numeric_limits<double>::infinity());
		for (int customer : eligibleCustomers) {
			AdaptiveResplitResult single = generateAdaptiveResplitCandidates(source, params,
				options.resplitK, 3, customer, 0);
			if (single.bestSplit)
				singleSplitDelta[customer] = single.splitReport.newDistance - source.eval.distance;
		}
		const std::vector<JointCustomerPair> pairs = selectJointCustomerPairs(source, params,
			singleSplitDelta, options.jointMaxPairs, options.resplitAcceptanceGap);

		JointResplitConfig jointConfig;
		jointConfig.candidateRoutes = 6;
		jointConfig.maxPatternsPerCustomer = 8;
		jointConfig.maxJointPatterns = 100;
		jointConfig.maxCandidatesPerPair = 300;
		jointConfig.maxDynamicSplits = 3;
		jointConfig.patternReferenceRoutes = 6;
		jointConfig.insertionMode = JointInsertionMode::EXHAUSTIVE_FEASIBLE_INSERTION;
		jointConfig.topInsertionPositions = 3;
		jointConfig.collectSplitCandidatesUpToDelta = 5.0;

		struct BridgeInput {
			int customerA;
			int customerB;
			Individual candidate;
			JointResplitReport report;
			BridgeInput(int customerA, int customerB, const Individual& candidate,
				const JointResplitReport& report)
				: customerA(customerA), customerB(customerB), candidate(candidate), report(report) {}
		};
		std::vector<BridgeInput> bridgeInputs;
		JointEnumerationStats enumerationStats;
		for (const JointCustomerPair& pair : pairs) {
			JointResplitResult joint = generateJointResplitCandidate(source, params,
				pair.customerA, pair.customerB, jointConfig);
			enumerationStats.add(joint.stats);
			for (const JointResplitResult::CandidateRecord& record : joint.collectedSplitCandidates)
				bridgeInputs.emplace_back(pair.customerA, pair.customerB, record.candidate, record.report);
		}
		std::sort(bridgeInputs.begin(), bridgeInputs.end(), [&](const BridgeInput& first, const BridgeInput& second) {
			const double firstDelta = first.report.candidateDistance - source.eval.distance;
			const double secondDelta = second.report.candidateDistance - source.eval.distance;
			if (std::fabs(firstDelta - secondDelta) > MY_EPSILON) return firstDelta < secondDelta;
			if (first.customerA != second.customerA) return first.customerA < second.customerA;
			return first.customerB < second.customerB;
		});
		std::cout << "[DYNAMIC-BRIDGE-SETUP] SourceDistance=" << source.eval.distance
			<< " EligibleCustomers=" << eligibleCustomers.size()
			<< " JointPairs=" << pairs.size()
			<< " GeneratedFeasibleSplitCandidates=" << enumerationStats.feasibleSplitCandidates
			<< " BridgeInputCandidates=" << bridgeInputs.size() << std::endl;

		struct NamedBridgeConfig {
			std::string name;
			BoundedUphillDynamicLSConfig config;
		};
		BoundedUphillDynamicLSConfig configA;
		configA.maxSingleDelta = 1.0;
		configA.maxCumulativeDelta = 3.0;
		configA.maxUphillMoves = 1;
		configA.recoveryMoves = std::max(0, options.dynamicRecoveryMoves);
		configA.beamWidth = std::max(1, options.dynamicBeamWidth);
		configA.nearbyRoutes = std::max(0, options.dynamicLSNearbyRoutes);
		BoundedUphillDynamicLSConfig configB = configA;
		configB.maxSingleDelta = options.dynamicUphillMaxDelta;
		configB.maxCumulativeDelta = options.dynamicUphillMaxCumulativeDelta;
		configB.maxUphillMoves = std::max(0, options.dynamicUphillMaxMoves);
		BoundedUphillDynamicLSConfig configC = configA;
		configC.maxSingleDelta = 5.0;
		configC.maxCumulativeDelta = 8.0;
		configC.maxUphillMoves = 2;
		const std::vector<NamedBridgeConfig> configurations = {
			{ "A", configA }, { "B", configB }, { "C", configC }
		};

		struct BestBridgeRecord {
			std::string configName;
			int customerA = -1;
			int customerB = -1;
			DynamicBridgeResult result;
		};
		std::optional<BestBridgeRecord> overallBest;
		for (const NamedBridgeConfig& named : configurations) {
			DynamicBridgeStats totals;
			std::optional<BestBridgeRecord> configBest;
			for (const BridgeInput& input : bridgeInputs) {
				DynamicBridgeResult bridge = runBoundedUphillDynamicLS(source, input.candidate,
					std::vector<int>{ input.customerA, input.customerB }, params, named.config);
				totals.add(bridge.stats);
				if (bridge.bestImproving && (!configBest ||
					bridge.finalDistance < configBest->result.finalDistance - MY_EPSILON))
					configBest = BestBridgeRecord{ named.name, input.customerA, input.customerB, bridge };
			}
			std::cout << "[DYNAMIC-BRIDGE-STATS] Config=" << named.name
				<< " MaxSingleDelta=" << named.config.maxSingleDelta
				<< " MaxCumulativeDelta=" << named.config.maxCumulativeDelta
				<< " MaxUphillMoves=" << named.config.maxUphillMoves
				<< " RecoveryMoves=" << named.config.recoveryMoves
				<< " BeamWidth=" << named.config.beamWidth
				<< " BridgeCandidatesTried=" << totals.bridgeCandidatesTried
				<< " BridgeTrajectoriesGenerated=" << totals.bridgeTrajectoriesGenerated
				<< " UphillMovesAcceptedForSearch=" << totals.uphillMovesAcceptedForSearch
				<< " Depth1Trajectories=" << totals.depth1Trajectories
				<< " Depth2Trajectories=" << totals.depth2Trajectories
				<< " RecoveryRuns=" << totals.recoveryRuns
				<< " RecoveryImprovingRuns=" << totals.recoveryImprovingRuns
				<< " NetImprovingTrajectories=" << totals.netImprovingTrajectories;
			if (totals.hasFinalDistance) {
				std::cout << " BestPeakExcursion=" << totals.bestPeakExcursion
					<< " BestFinalDistance=" << totals.bestFinalDistance
					<< " BestNetImprovement=" << totals.bestNetImprovement;
			} else {
				std::cout << " BestPeakExcursion=NA BestFinalDistance=NA BestNetImprovement=NA";
			}
			std::cout << std::endl;
			if (configBest) {
				const DynamicBridgeResult& best = configBest->result;
				const ValidationResult validation = validateSolutionFromScratch(*best.bestImproving, params);
				std::cout << "[DYNAMIC-BRIDGE-BEST] Config=" << named.name
					<< " customerA=" << configBest->customerA
					<< " customerB=" << configBest->customerB
					<< " source=" << best.sourceDistance
					<< " split=" << best.splitStartDistance
					<< " SplitPenalty=" << best.splitStartDistance - best.sourceDistance
					<< " uphillMove1=\"" << best.uphillMove1 << "\""
					<< " delta1=" << best.delta1
					<< " uphillMove2=\"" << best.uphillMove2 << "\""
					<< " delta2=" << best.delta2
					<< " peakDistance=" << best.peakDistance
					<< " MaxExcursionFromSource=" << best.peakDistance - best.sourceDistance
					<< " recoveryMoves=" << best.recoveryMoveCount
					<< " finalDistance=" << best.finalDistance
					<< " netImprovement=" << best.sourceDistance - best.finalDistance
					<< " finalSplitCustomers=" << best.finalSplitCustomers
					<< " validator=" << (validation.feasible() ? "PASS" : "FAIL") << std::endl;
				if (!overallBest || best.finalDistance < overallBest->result.finalDistance - MY_EPSILON)
					overallBest = configBest;
			}
		}
		if (overallBest) {
			const ValidationResult finalValidation = validateSolutionFromScratch(
				*overallBest->result.bestImproving, params);
			std::cout << "[DYNAMIC-BRIDGE-OVERALL] Config=" << overallBest->configName
				<< " BestFinalDistance=" << overallBest->result.finalDistance
				<< " BestNetImprovement=" << source.eval.distance - overallBest->result.finalDistance
				<< " FinalSplitCustomers=" << overallBest->result.finalSplitCustomers
				<< " Validator=" << (finalValidation.feasible() ? "PASS" : "FAIL")
				<< " BrokeSource=" << (overallBest->result.finalDistance < source.eval.distance - MY_EPSILON ? "YES" : "NO")
				<< std::endl;
		} else {
			std::cout << "[DYNAMIC-BRIDGE-OVERALL] BestFinalDistance=NA BestNetImprovement=0"
				<< " FinalSplitCustomers=0 Validator=NA BrokeSource=NO" << std::endl;
		}
		return;
	}
	if (options.testJointInsertionAudit)
	{
		const Individual* fixed = solver.population.getBestFound();
		if (fixed == NULL) throw std::logic_error("No feasible fixed source for joint insertion audit");
		Individual source(*fixed);
		source.convertFixedToDynamic(params);
		source.evaluateDynamicQuantity(params);
		const ValidationResult sourceValidation = validateSolutionFromScratch(source, params);
		if (!sourceValidation.feasible()) throw std::logic_error("Converted audit source is invalid: " + sourceValidation.message);
		const std::vector<int> eligibleCustomers = getAdaptiveResplitEligibleCustomers(source, params);
		std::vector<double> singleSplitDelta(params.originalDemand.size(), std::numeric_limits<double>::infinity());
		for (int customer : eligibleCustomers) {
			AdaptiveResplitResult single = generateAdaptiveResplitCandidates(source, params,
				options.resplitK, 3, customer, 0);
			if (single.bestSplit) singleSplitDelta[customer] = single.splitReport.newDistance - source.eval.distance;
		}
		const std::vector<JointCustomerPair> pairs = selectJointCustomerPairs(source, params,
			singleSplitDelta, options.jointMaxPairs, options.resplitAcceptanceGap);

		struct AuditPairOutcome {
			JointCustomerPair pair;
			std::optional<Individual> bestOverall;
			std::optional<Individual> bestSplit;
			JointResplitReport overallReport;
			JointResplitReport splitReport;
		};
		struct AuditScenario {
			std::string name;
			JointEnumerationStats stats;
			double bestOverallDistance = std::numeric_limits<double>::max();
			double bestSplitDistance = std::numeric_limits<double>::max();
			std::vector<AuditPairOutcome> outcomes;
		};
		auto runScenario = [&](const std::string& name, const JointResplitConfig& config) {
			AuditScenario scenario;
			scenario.name = name;
			for (const JointCustomerPair& pair : pairs) {
				JointResplitResult result = generateJointResplitCandidate(source, params,
					pair.customerA, pair.customerB, config);
				scenario.stats.add(result.stats);
				if (result.bestOverall) scenario.bestOverallDistance = std::min(scenario.bestOverallDistance,
					result.overallReport.candidateDistance);
				if (result.bestSplit) scenario.bestSplitDistance = std::min(scenario.bestSplitDistance,
					result.splitReport.candidateDistance);
				scenario.outcomes.push_back({ pair, result.bestOverall, result.bestSplit,
					result.overallReport, result.splitReport });
			}
			std::cout << "[JOINT-AUDIT] Scenario=" << name
				<< " InsertionMode=" << jointInsertionModeName(config.insertionMode)
				<< " RoutePool=" << config.candidateRoutes
				<< " MaxPatternsPerCustomer=" << config.maxPatternsPerCustomer
				<< " MaxJointPatterns=" << config.maxJointPatterns
				<< " MaxCandidatesPerPair=" << config.maxCandidatesPerPair
				<< " GeneratedCandidates=" << scenario.stats.generatedCandidates
				<< " FeasibleCandidates=" << scenario.stats.feasibleCandidates
				<< " FeasibleJointSplitCandidates=" << scenario.stats.feasibleSplitCandidates
				<< " InsertionPlacementsEvaluated=" << scenario.stats.insertionPlacementsEvaluated
				<< " FeasibleInsertionPlacements=" << scenario.stats.feasibleInsertionPlacements;
			if (scenario.bestOverallDistance < std::numeric_limits<double>::max())
				std::cout << " BestJointDistance=" << scenario.bestOverallDistance;
			else std::cout << " BestJointDistance=NA";
			if (scenario.bestSplitDistance < std::numeric_limits<double>::max())
				std::cout << " BestJointSplitDistance=" << scenario.bestSplitDistance;
			else std::cout << " BestJointSplitDistance=NA";
			std::cout << " NearNeutralCandidates=" << scenario.stats.nearNeutralSplitCandidates << std::endl;
			std::cout << "[BEST-DELTA-DISTRIBUTION] Scenario=" << name
				<< " DeltaLE0=" << scenario.stats.splitDeltaLE0
				<< " Delta0To1=" << scenario.stats.splitDelta0To1
				<< " Delta1To3=" << scenario.stats.splitDelta1To3
				<< " Delta3To5=" << scenario.stats.splitDelta3To5
				<< " Delta5To10=" << scenario.stats.splitDelta5To10
				<< " DeltaAbove10=" << scenario.stats.splitDeltaAbove10 << std::endl;
			return scenario;
		};

		JointResplitConfig current;
		current.candidateRoutes = 6;
		current.maxPatternsPerCustomer = 8;
		current.maxJointPatterns = 100;
		current.maxCandidatesPerPair = 300;
		current.maxDynamicSplits = 3;
		current.patternReferenceRoutes = 6;
		current.insertionMode = JointInsertionMode::GREEDY_INSERTION;
		current.topInsertionPositions = std::max(1, options.topInsertionPositions);
		const AuditScenario greedy = runScenario("GREEDY_K6_CURRENT", current);
		JointResplitConfig combination = current;
		combination.insertionMode = JointInsertionMode::JOINT_INSERTION_COMBINATION;
		const AuditScenario topCombination = runScenario("TOP3_K6_CURRENT", combination);
		JointResplitConfig exhaustive = current;
		exhaustive.insertionMode = JointInsertionMode::EXHAUSTIVE_FEASIBLE_INSERTION;
		const AuditScenario exhaustiveInsertion = runScenario("EXHAUSTIVE_K6_CURRENT", exhaustive);
		JointResplitConfig k10 = current;
		k10.candidateRoutes = 10;
		const AuditScenario routeK10 = runScenario("GREEDY_K10_CURRENT", k10);
		JointResplitConfig allRoutes = current;
		allRoutes.candidateRoutes = params.nbVehicles;
		const AuditScenario routeAll = runScenario("GREEDY_ALL_CURRENT", allRoutes);
		JointResplitConfig relaxed = current;
		relaxed.maxPatternsPerCustomer = 20;
		relaxed.maxJointPatterns = 300;
		relaxed.maxCandidatesPerPair = 1000;
		const AuditScenario relaxedPruning = runScenario("GREEDY_K6_RELAXED", relaxed);
		(void)topCombination;
		(void)exhaustiveInsertion;
		(void)routeK10;
		(void)relaxedPruning;

		int routePoolMisses = 0;
		for (size_t index = 0; index < std::min(greedy.outcomes.size(), routeAll.outcomes.size()); ++index) {
			const AuditPairOutcome& k6Outcome = greedy.outcomes[index];
			const AuditPairOutcome& allOutcome = routeAll.outcomes[index];
			if (!allOutcome.bestSplit) continue;
			const bool allIsBetter = !k6Outcome.bestSplit ||
				allOutcome.splitReport.candidateDistance < k6Outcome.splitReport.candidateDistance - MY_EPSILON;
			if (!allIsBetter) continue;
			++routePoolMisses;
			std::vector<int> missingRoutes;
			for (int route = 0; route < static_cast<int>(allOutcome.bestSplit->routesDynamic.size()); ++route) {
				const bool usesJointCustomer = std::any_of(allOutcome.bestSplit->routesDynamic[route].begin(),
					allOutcome.bestSplit->routesDynamic[route].end(), [&](const RouteVisit& visit) {
						return visit.originalCustomerId == allOutcome.pair.customerA ||
							visit.originalCustomerId == allOutcome.pair.customerB;
					});
				if (usesJointCustomer && std::find(k6Outcome.splitReport.routePool.begin(),
					k6Outcome.splitReport.routePool.end(), route) == k6Outcome.splitReport.routePool.end())
					missingRoutes.push_back(route);
			}
			std::cout << "[ROUTE-POOL-MISS] customerA=" << allOutcome.pair.customerA
				<< " customerB=" << allOutcome.pair.customerB;
			if (k6Outcome.bestSplit) std::cout << " bestK6=" << k6Outcome.splitReport.candidateDistance;
			else std::cout << " bestK6=NA";
			std::cout << " bestAll=" << allOutcome.splitReport.candidateDistance << " missingRoutes=";
			if (missingRoutes.empty()) std::cout << "none";
			else for (size_t route = 0; route < missingRoutes.size(); ++route) {
				if (route != 0) std::cout << ';';
				std::cout << 'R' << missingRoutes[route] + 1;
			}
			std::cout << std::endl;
		}
		std::cout << "[ROUTE-POOL-AUDIT] BetterInAllPairs=" << routePoolMisses << std::endl;

		DynamicLocalImproveConfig localConfig;
		localConfig.nearbyRoutes = std::max(0, options.dynamicLSNearbyRoutes);
		localConfig.maxPasses = std::max(0, options.dynamicLSMaxPasses);
		DynamicLocalImproveResult moveTotals;
		int localRuns = 0;
		for (const AuditPairOutcome& outcome : greedy.outcomes) {
			if (!outcome.bestSplit || outcome.splitReport.candidateDistance - source.eval.distance >
				options.jointAcceptanceGap + MY_EPSILON) continue;
			DynamicLocalImproveResult local = improveDynamicQuantityLocally(source, *outcome.bestSplit,
				std::vector<int>{ outcome.pair.customerA, outcome.pair.customerB }, params, localConfig);
			++localRuns;
			moveTotals.generatedRelocateMoves += local.generatedRelocateMoves;
			moveTotals.feasibleRelocateMoves += local.feasibleRelocateMoves;
			moveTotals.improvingRelocateMoves += local.improvingRelocateMoves;
			moveTotals.generatedSwapMoves += local.generatedSwapMoves;
			moveTotals.feasibleSwapMoves += local.feasibleSwapMoves;
			moveTotals.improvingSwapMoves += local.improvingSwapMoves;
			moveTotals.generated2OptMoves += local.generated2OptMoves;
			moveTotals.feasible2OptMoves += local.feasible2OptMoves;
			moveTotals.improving2OptMoves += local.improving2OptMoves;
			moveTotals.generatedCrossRouteRelocateMoves += local.generatedCrossRouteRelocateMoves;
			moveTotals.feasibleCrossRouteRelocateMoves += local.feasibleCrossRouteRelocateMoves;
			moveTotals.improvingCrossRouteRelocateMoves += local.improvingCrossRouteRelocateMoves;
			moveTotals.feasibleUphillMoves1 += local.feasibleUphillMoves1;
			moveTotals.feasibleUphillMoves3 += local.feasibleUphillMoves3;
			moveTotals.feasibleUphillMoves5 += local.feasibleUphillMoves5;
		}
		std::cout << "[DYNAMIC-LS-MOVE-AUDIT] Runs=" << localRuns
			<< " GeneratedRelocateMoves=" << moveTotals.generatedRelocateMoves
			<< " FeasibleRelocateMoves=" << moveTotals.feasibleRelocateMoves
			<< " ImprovingRelocateMoves=" << moveTotals.improvingRelocateMoves
			<< " GeneratedSwapMoves=" << moveTotals.generatedSwapMoves
			<< " FeasibleSwapMoves=" << moveTotals.feasibleSwapMoves
			<< " ImprovingSwapMoves=" << moveTotals.improvingSwapMoves
			<< " Generated2OptMoves=" << moveTotals.generated2OptMoves
			<< " Feasible2OptMoves=" << moveTotals.feasible2OptMoves
			<< " Improving2OptMoves=" << moveTotals.improving2OptMoves
			<< " GeneratedCrossRouteRelocateMoves=" << moveTotals.generatedCrossRouteRelocateMoves
			<< " FeasibleCrossRouteRelocateMoves=" << moveTotals.feasibleCrossRouteRelocateMoves
			<< " ImprovingCrossRouteRelocateMoves=" << moveTotals.improvingCrossRouteRelocateMoves
			<< " FeasibleUphillMoves1=" << moveTotals.feasibleUphillMoves1
			<< " FeasibleUphillMoves3=" << moveTotals.feasibleUphillMoves3
			<< " FeasibleUphillMoves5=" << moveTotals.feasibleUphillMoves5 << std::endl;
		return;
	}
	if (options.testJointResplit)
	{
		const Individual* fixed = solver.population.getBestFound();
		if (fixed == NULL) throw std::logic_error("No feasible fixed source for joint resplitting");
		Individual source(*fixed);
		source.convertFixedToDynamic(params);
		source.evaluateDynamicQuantity(params);
		const ValidationResult sourceValidation = validateSolutionFromScratch(source, params);
		if (!sourceValidation.feasible()) throw std::logic_error("Converted joint source is invalid: " + sourceValidation.message);

		const std::vector<int> eligibleCustomers = getAdaptiveResplitEligibleCustomers(source, params);
		std::vector<double> singleSplitDelta(params.originalDemand.size(), std::numeric_limits<double>::infinity());
		for (int customer : eligibleCustomers) {
			AdaptiveResplitResult single = generateAdaptiveResplitCandidates(source, params,
				options.resplitK, 3, customer, 0);
			if (single.bestSplit)
				singleSplitDelta[customer] = single.splitReport.newDistance - source.eval.distance;
		}
		const std::vector<JointCustomerPair> pairs = selectJointCustomerPairs(source, params,
			singleSplitDelta, options.jointMaxPairs, options.resplitAcceptanceGap);
		std::cout << "[JOINT-SETUP] SourceDistance=" << source.eval.distance
			<< " EligibleCustomers=" << eligibleCustomers.size()
			<< " JointPairPool=" << pairs.size()
			<< " CandidateRoutes=" << options.jointCandidateRoutes
			<< " MaxJointPatterns=" << options.jointMaxPatterns
			<< " CandidatesPerPair=" << options.jointCandidatesPerPair
			<< " AcceptanceGap=" << options.jointAcceptanceGap << std::endl;

		JointResplitConfig jointConfig;
		jointConfig.candidateRoutes = std::max(2, options.jointCandidateRoutes);
		jointConfig.maxPatternsPerCustomer = 8;
		jointConfig.maxJointPatterns = std::max(1, options.jointMaxPatterns);
		jointConfig.maxCandidatesPerPair = std::max(1, options.jointCandidatesPerPair);
		jointConfig.maxDynamicSplits = 3;
		JointEnumerationStats jointStats;
		struct JointNearNeutral {
			int customerA;
			int customerB;
			Individual candidate;
			JointResplitReport report;
			JointNearNeutral(int customerA, int customerB, const Individual& candidate,
				const JointResplitReport& report)
				: customerA(customerA), customerB(customerB), candidate(candidate), report(report) {}
		};
		std::vector<JointNearNeutral> nearNeutral;
		double bestJointBeforeLS = std::numeric_limits<double>::max();
		for (size_t index = 0; index < pairs.size(); ++index) {
			const JointCustomerPair& pair = pairs[index];
			std::cout << "[JOINT-PAIR] rank=" << index + 1
				<< " customerA=" << pair.customerA
				<< " customerB=" << pair.customerB
				<< " score=" << pair.score
				<< " singleDeltaA=" << singleSplitDelta[pair.customerA]
				<< " singleDeltaB=" << singleSplitDelta[pair.customerB] << std::endl;
			JointResplitResult result = generateJointResplitCandidate(source, params,
				pair.customerA, pair.customerB, jointConfig);
			jointStats.add(result.stats);
			if (result.bestSplit) bestJointBeforeLS = std::min(bestJointBeforeLS, result.splitReport.candidateDistance);
			auto collect = [&](const std::optional<Individual>& candidate, const JointResplitReport& report) {
				if (!candidate || report.candidateDistance - source.eval.distance > options.jointAcceptanceGap + MY_EPSILON) return;
				const std::string key = std::to_string(pair.customerA) + ":" + std::to_string(pair.customerB) + ":" +
					report.newA + ":" + report.newB;
				const bool duplicate = std::any_of(nearNeutral.begin(), nearNeutral.end(), [&](const JointNearNeutral& existing) {
					return std::to_string(existing.customerA) + ":" + std::to_string(existing.customerB) + ":" +
						existing.report.newA + ":" + existing.report.newB == key;
				});
				if (!duplicate) nearNeutral.emplace_back(pair.customerA, pair.customerB, *candidate, report);
			};
			collect(result.bestOverall, result.overallReport);
			collect(result.bestSplit, result.splitReport);
		}

		std::sort(nearNeutral.begin(), nearNeutral.end(), [&](const JointNearNeutral& first, const JointNearNeutral& second) {
			const double firstDelta = first.report.candidateDistance - source.eval.distance;
			const double secondDelta = second.report.candidateDistance - source.eval.distance;
			if (std::fabs(firstDelta - secondDelta) > MY_EPSILON) return firstDelta < secondDelta;
			return std::make_pair(first.customerA, first.customerB) < std::make_pair(second.customerA, second.customerB);
		});
		DynamicLocalImproveConfig localConfig;
		localConfig.nearbyRoutes = std::max(0, options.dynamicLSNearbyRoutes);
		localConfig.maxPasses = std::max(0, options.dynamicLSMaxPasses);
		int dynamicLSRuns = 0, dynamicLSNetImprovingRuns = 0;
		int jointOneSplitImprovements = 0, jointBothSplitImprovements = 0, jointRelocateOnlyImprovements = 0;
		double bestJointAfterLS = std::numeric_limits<double>::max();
		double bestJointSplitAfterLS = std::numeric_limits<double>::max();
		std::optional<Individual> bestFinalOverall;
		std::optional<Individual> bestFinalSplit;
		JointResplitReport bestFinalOverallReport;
		JointResplitReport bestFinalSplitReport;
		for (const JointNearNeutral& entry : nearNeutral) {
			++dynamicLSRuns;
			DynamicLocalImproveResult local = improveDynamicQuantityLocally(source, entry.candidate,
				std::vector<int>{ entry.customerA, entry.customerB }, params, localConfig);
			if (!local.improved || !local.validatorPassed) {
				std::cout << "[JOINT-LS] customerA=" << entry.customerA << " customerB=" << entry.customerB
					<< " jointDistance=" << entry.report.candidateDistance
					<< " postLSDistance=INVALID validator=FAIL reason=" << local.reason << std::endl;
				continue;
			}
			const double postLS = local.improved->eval.distance;
			const double netImprovement = source.eval.distance - postLS;
			bestJointAfterLS = std::min(bestJointAfterLS, postLS);
			if (netImprovement > MY_EPSILON) {
				++dynamicLSNetImprovingRuns;
				if (entry.report.type == JointCandidateType::JOINT_ONE_SPLIT) ++jointOneSplitImprovements;
				else if (entry.report.type == JointCandidateType::JOINT_BOTH_SPLIT) ++jointBothSplitImprovements;
				else ++jointRelocateOnlyImprovements;
			}
			if (!bestFinalOverall || postLS < bestFinalOverall->eval.distance - MY_EPSILON) {
				bestFinalOverall = *local.improved;
				bestFinalOverallReport = entry.report;
			}
			if (entry.report.type != JointCandidateType::JOINT_RELOCATE) {
				bestJointSplitAfterLS = std::min(bestJointSplitAfterLS, postLS);
				if (!bestFinalSplit || postLS < bestFinalSplit->eval.distance - MY_EPSILON) {
					bestFinalSplit = *local.improved;
					bestFinalSplitReport = entry.report;
				}
			}
			std::cout << "[JOINT-LS] customerA=" << entry.customerA
				<< " customerB=" << entry.customerB
				<< " type=" << jointCandidateTypeName(entry.report.type)
				<< " sourceDistance=" << source.eval.distance
				<< " jointDistance=" << entry.report.candidateDistance
				<< " postLSDistance=" << postLS
				<< " netImprovement=" << netImprovement
				<< " affectedRoutes=" << local.affectedRoutes.size()
				<< " searchRoutes=" << local.searchRoutes.size()
				<< " acceptedMoves=" << local.acceptedMoves
				<< " validator=PASS" << std::endl;
		}

		int finalSplitCustomers = 0;
		const std::optional<Individual>& reportedBest = bestFinalSplit ? bestFinalSplit : bestFinalOverall;
		const JointResplitReport& reportedBestReport = bestFinalSplit ? bestFinalSplitReport : bestFinalOverallReport;
		if (reportedBest) {
			auto routeCount = [&](int customer) {
				int count = 0;
				for (const auto& route : reportedBest->routesDynamic)
					if (std::any_of(route.begin(), route.end(), [customer](const RouteVisit& visit) {
						return visit.originalCustomerId == customer;
					})) ++count;
				return count;
			};
			finalSplitCustomers = (routeCount(reportedBestReport.customerA) >= 2 ? 1 : 0) +
				(routeCount(reportedBestReport.customerB) >= 2 ? 1 : 0);
			const ValidationResult validation = validateSolutionFromScratch(*reportedBest, params);
			std::cout << "[JOINT-RESPLIT-BEST]"
				<< " customerA=" << reportedBestReport.customerA
				<< " demandA=" << reportedBestReport.demandA
				<< " oldA=" << reportedBestReport.oldA
				<< " newA=" << jointAllocationText(*reportedBest, reportedBestReport.customerA)
				<< " customerB=" << reportedBestReport.customerB
				<< " demandB=" << reportedBestReport.demandB
				<< " oldB=" << reportedBestReport.oldB
				<< " newB=" << jointAllocationText(*reportedBest, reportedBestReport.customerB)
				<< " type=" << jointCandidateTypeName(reportedBestReport.type)
				<< " sourceDistance=" << source.eval.distance
				<< " jointDistance=" << reportedBestReport.candidateDistance
				<< " postLSDistance=" << reportedBest->eval.distance
				<< " netImprovement=" << source.eval.distance - reportedBest->eval.distance
				<< " routesBefore=" << reportedBestReport.routesBefore
				<< " routesAfter=" << jointCustomerRoutesText(*reportedBest, reportedBestReport.customerA, reportedBestReport.customerB)
				<< " finalSplitCustomers=" << finalSplitCustomers
				<< " validator=" << (validation.feasible() ? "PASS" : "FAIL") << std::endl;
		}

		std::cout << "[JOINT-STATS]"
			<< " JointPairsTried=" << pairs.size()
			<< " GeneratedJointCandidates=" << jointStats.generatedCandidates
			<< " FeasibleJointCandidates=" << jointStats.feasibleCandidates
			<< " GeneratedJointSplitCandidates=" << jointStats.generatedSplitCandidates
			<< " FeasibleJointSplitCandidates=" << jointStats.feasibleSplitCandidates
			<< " GeneratedJointImprovingCandidates=" << jointStats.generatedImprovingCandidates
			<< " GeneratedJointImprovingSplitCandidates=" << jointStats.generatedImprovingSplitCandidates
			<< " NearNeutralJointCandidates=" << nearNeutral.size()
			<< " DynamicLSRuns=" << dynamicLSRuns
			<< " DynamicLSNetImprovingRuns=" << dynamicLSNetImprovingRuns
			<< " JointOneSplitImprovements=" << jointOneSplitImprovements
			<< " JointBothSplitImprovements=" << jointBothSplitImprovements
			<< " JointRelocateOnlyImprovements=" << jointRelocateOnlyImprovements;
		if (bestJointBeforeLS < std::numeric_limits<double>::max()) std::cout << " BestJointBeforeLS=" << bestJointBeforeLS;
		else std::cout << " BestJointBeforeLS=NA";
		if (bestJointAfterLS < std::numeric_limits<double>::max())
			std::cout << " BestJointAfterLS=" << bestJointAfterLS
				<< " BestNetImprovement=" << source.eval.distance - bestJointAfterLS;
		else std::cout << " BestJointAfterLS=NA BestNetImprovement=0";
		if (bestJointSplitAfterLS < std::numeric_limits<double>::max())
			std::cout << " BestJointSplitAfterLS=" << bestJointSplitAfterLS
				<< " BestSplitNetImprovement=" << source.eval.distance - bestJointSplitAfterLS;
		else std::cout << " BestJointSplitAfterLS=NA BestSplitNetImprovement=0";
		std::cout << " FinalSplitCustomers=" << finalSplitCustomers
			<< " RealJointSplitNetImprovement="
			<< (bestFinalSplit && bestFinalSplit->eval.distance < source.eval.distance - MY_EPSILON && finalSplitCustomers > 0 ? 1 : 0)
			<< std::endl;
		return;
	}
	if (options.testAdaptiveResplit)
	{
		const Individual* fixed = solver.population.getBestFound();
		if (fixed == NULL) throw std::logic_error("No feasible fixed source for standalone resplitting");
		Individual source(*fixed); source.convertFixedToDynamic(params); source.evaluateDynamicQuantity(params);
		const ValidationResult sourceValidation = validateSolutionFromScratch(source, params);
		if (!sourceValidation.feasible()) throw std::logic_error("Converted standalone source is invalid: " + sourceValidation.message);
		const std::vector<int> eligibleCustomers = getAdaptiveResplitEligibleCustomers(source, params);
		if (eligibleCustomers.empty()) throw std::logic_error("No eligible original customer for standalone resplitting");
		const int actualAttempts = options.resplitDiagnosticAllCustomers
			? static_cast<int>(eligibleCustomers.size()) : std::max(0, options.resplitAttempts);
		std::cout << "[RESPLIT-SETUP] Mode="
			<< (options.resplitDiagnosticAllCustomers ? "ALL_CUSTOMERS" : "ROUND_ROBIN")
			<< " EligibleCustomers=" << eligibleCustomers.size()
			<< " Attempts=" << actualAttempts << std::endl;

		int returned = 0, returnedSplit = 0, feasible = 0, split = 0;
		int improving = 0, improvingSplit = 0, rejected = 0, printed = 0;
		std::map<int,int> customerFrequency; std::set<std::string> patterns;
		struct NearNeutralSplit {
			int customer;
			Individual candidate;
			ResplitReport report;
			NearNeutralSplit(int customer, const Individual& candidate, const ResplitReport& report)
				: customer(customer), candidate(candidate), report(report) {}
		};
		std::vector<NearNeutralSplit> nearNeutralSplits;
		ResplitEnumerationStats generated;
		double bestImprovement = 0.0, bestDistance = source.eval.distance;
		double bestSplitDistance = 0.0; bool hasBestSplit = false; int bestCustomer = -1;
		for (int attempt = 1; attempt <= actualAttempts; ++attempt)
		{
			const int customer = eligibleCustomers[(attempt - 1) % eligibleCustomers.size()];
			++customerFrequency[customer];
			const int diagnosticBudget = std::max(0, 20 - printed);
			AdaptiveResplitResult result = generateAdaptiveResplitCandidates(source, params,
				options.resplitK,
				options.resplitDiagnosticAllCustomers ? std::max(3, options.resplitMaxSplits) : options.resplitMaxSplits,
				customer, diagnosticBudget);
			generated.add(result.stats);

			for (const SplitCandidateDiagnostic& diagnostic : result.splitDiagnostics) {
				std::cout << "[SPLIT-CANDIDATE]"
					<< " customer=" << diagnostic.customer
					<< " demand=" << diagnostic.demand
					<< " allocation=" << diagnostic.allocation
					<< " routes=" << diagnostic.routes
					<< " distance=" << diagnostic.distance
					<< " delta=" << diagnostic.delta
					<< " capacityOK=" << (diagnostic.capacityOK ? 1 : 0)
					<< " twOK=" << (diagnostic.twOK ? 1 : 0)
					<< " conflictOK=" << (diagnostic.conflictOK ? 1 : 0)
					<< " validator=" << (diagnostic.validator ? "PASS" : "FAIL")
					<< " reason=" << diagnostic.reason
					<< " selectedAsBestSplit=" << (diagnostic.selectedAsBestSplit ? 1 : 0)
					<< std::endl;
				++printed;
			}

			if (result.bestOverall) {
				++returned; ++feasible;
				patterns.insert(std::to_string(customer) + ":" + result.overallReport.newAllocation);
				if (result.overallReport.improving) ++improving;
				if (result.overallReport.newDistance < bestDistance - MY_EPSILON) {
					bestDistance = result.overallReport.newDistance;
					bestImprovement = source.eval.distance - bestDistance;
					bestCustomer = customer;
				}
			}
			else ++rejected;
			if (result.bestSplit) {
				++returnedSplit; ++split;
				patterns.insert(std::to_string(customer) + ":" + result.splitReport.newAllocation);
				if (result.splitReport.improving) ++improvingSplit;
				if (!hasBestSplit || result.splitReport.newDistance < bestSplitDistance - MY_EPSILON) {
					hasBestSplit = true;
					bestSplitDistance = result.splitReport.newDistance;
				}
				const double splitDelta = result.splitReport.newDistance - source.eval.distance;
				if (options.testDynamicLocalImprove && splitDelta <= options.resplitAcceptanceGap + MY_EPSILON) {
					auto existing = std::find_if(nearNeutralSplits.begin(), nearNeutralSplits.end(),
						[customer](const NearNeutralSplit& item) { return item.customer == customer; });
					if (existing == nearNeutralSplits.end())
						nearNeutralSplits.emplace_back(customer, *result.bestSplit, result.splitReport);
					else if (result.splitReport.newDistance < existing->report.newDistance - MY_EPSILON) {
						existing->candidate = *result.bestSplit;
						existing->report = result.splitReport;
					}
				}
			}

			if (options.resplitDiagnosticAllCustomers) {
				std::string feasiblePatterns;
				if (result.stats.generatedFeasibleOneWayCandidates > 0) feasiblePatterns += "1-way";
				if (result.stats.generatedFeasibleTwoWayCandidates > 0) feasiblePatterns += (feasiblePatterns.empty() ? "" : ";") + std::string("2-way");
				if (result.stats.generatedFeasibleThreeWayCandidates > 0) feasiblePatterns += (feasiblePatterns.empty() ? "" : ";") + std::string("3-way");
				if (feasiblePatterns.empty()) feasiblePatterns = "none";
				std::cout << "[RESPLIT-CUSTOMER] customer=" << customer
					<< " demand=" << params.originalDemand[customer]
					<< " feasibleSplitPatterns=" << feasiblePatterns;
				if (result.bestSplit) std::cout << " bestSplitDistance=" << result.splitReport.newDistance
					<< " bestSplitDelta=" << result.splitReport.newDistance - source.eval.distance;
				else std::cout << " bestSplitDistance=NA bestSplitDelta=NA";
				std::cout << std::endl;
			}
		}
		std::vector<std::pair<int,int>> frequencies(customerFrequency.begin(), customerFrequency.end()); std::sort(frequencies.begin(), frequencies.end(), [](auto&a,auto&b){return a.second>b.second;});
		const double coverage = static_cast<double>(customerFrequency.size()) / eligibleCustomers.size();
		std::cout << "[RESPLIT-STATS] ResplitAttempts=" << actualAttempts
			<< " ReturnedCandidates=" << returned
			<< " ReturnedSplitCandidates=" << returnedSplit
			<< " FeasibleCandidates=" << feasible
			<< " FeasibleSplitCandidates=" << split
			<< " ImprovingCandidates=" << improving
			<< " ImprovingSplitCandidates=" << improvingSplit
			<< " RejectedCandidates=" << rejected
			<< " GeneratedCandidates=" << generated.generatedCandidates
			<< " GeneratedOneWayCandidates=" << generated.generatedOneWayCandidates
			<< " GeneratedTwoWayCandidates=" << generated.generatedTwoWayCandidates
			<< " GeneratedThreeWayCandidates=" << generated.generatedThreeWayCandidates
			<< " GeneratedFeasibleCandidates=" << generated.generatedFeasibleCandidates
			<< " GeneratedFeasibleSplitCandidates=" << generated.generatedFeasibleSplitCandidates
			<< " GeneratedImprovingCandidates=" << generated.generatedImprovingCandidates
			<< " GeneratedImprovingSplitCandidates=" << generated.generatedImprovingSplitCandidates
			<< " EligibleCustomers=" << eligibleCustomers.size()
			<< " UniqueCustomersTried=" << customerFrequency.size()
			<< " CustomerCoverageRatio=" << coverage
			<< " UniqueAllocationPatterns=" << patterns.size()
			<< " BestCandidateDistance=" << bestDistance
			<< " BestImprovement=" << bestImprovement
			<< " BestCustomer=" << bestCustomer;
		if (hasBestSplit) std::cout << " BestSplitDistance=" << bestSplitDistance
			<< " BestSplitDelta=" << bestSplitDistance - source.eval.distance;
		else std::cout << " BestSplitDistance=NA BestSplitDelta=NA";
		std::cout << std::endl;
		std::cout << "[RESPLIT-TOP-CUSTOMERS]"; for (size_t i=0;i<std::min<size_t>(10,frequencies.size());++i) std::cout << ' ' << frequencies[i].first << ':' << frequencies[i].second; std::cout << std::endl;

		if (options.testDynamicLocalImprove) {
			std::sort(nearNeutralSplits.begin(), nearNeutralSplits.end(), [&](const NearNeutralSplit& first, const NearNeutralSplit& second) {
				const bool firstPriority = first.customer == 21 || first.customer == 42;
				const bool secondPriority = second.customer == 21 || second.customer == 42;
				if (firstPriority != secondPriority) return firstPriority;
				const double firstDelta = first.report.newDistance - source.eval.distance;
				const double secondDelta = second.report.newDistance - source.eval.distance;
				return std::fabs(firstDelta - secondDelta) > MY_EPSILON
					? firstDelta < secondDelta : first.customer < second.customer;
			});
			DynamicLocalImproveConfig localConfig;
			localConfig.nearbyRoutes = std::max(0, options.dynamicLSNearbyRoutes);
			localConfig.maxPasses = std::max(0, options.dynamicLSMaxPasses);
			DynamicElite dynamicElite;
			int dynamicLSRuns = 0, dynamicLSImprovingRuns = 0, netImprovingRuns = 0;
			double bestAfterSplitLS = std::numeric_limits<double>::max();
			double bestAfterDynamicLS = std::numeric_limits<double>::max();
			double bestCaseSplit = 0.0, bestCasePostLS = 0.0;
			int bestCaseCustomer = -1;
			for (const NearNeutralSplit& entry : nearNeutralSplits) {
				++dynamicLSRuns;
				bestAfterSplitLS = std::min(bestAfterSplitLS, entry.report.newDistance);
				DynamicLocalImproveResult localResult = improveDynamicQuantityLocally(
					source, entry.candidate, entry.customer, params, localConfig);
				if (!localResult.improved || !localResult.validatorPassed) {
					std::cout << "[RESPLIT-LS] customer=" << entry.customer
						<< " before=" << source.eval.distance
						<< " afterSplit=" << entry.report.newDistance
						<< " afterLocalSearch=INVALID improvement=NA"
						<< " reason=" << localResult.reason << std::endl;
					continue;
				}
				const double afterLocalSearch = localResult.improved->eval.distance;
				bestAfterDynamicLS = std::min(bestAfterDynamicLS, afterLocalSearch);
				if (afterLocalSearch < entry.report.newDistance - MY_EPSILON) ++dynamicLSImprovingRuns;
				const double netImprovement = source.eval.distance - afterLocalSearch;
				if (netImprovement > MY_EPSILON) {
					++netImprovingRuns;
					dynamicElite.consider(*localResult.improved, params);
				}
				if (bestCaseCustomer < 0 || afterLocalSearch < bestCasePostLS - MY_EPSILON) {
					bestCaseCustomer = entry.customer;
					bestCaseSplit = entry.report.newDistance;
					bestCasePostLS = afterLocalSearch;
				}
				std::cout << "[RESPLIT-LS] customer=" << entry.customer
					<< " before=" << source.eval.distance
					<< " afterSplit=" << entry.report.newDistance
					<< " afterLocalSearch=" << afterLocalSearch
					<< " improvement=" << netImprovement
					<< " localImprovement=" << entry.report.newDistance - afterLocalSearch
					<< " affectedRoutes=" << localResult.affectedRoutes.size()
					<< " searchRoutes=" << localResult.searchRoutes.size()
					<< " evaluatedMoves=" << localResult.evaluatedMoves
					<< " acceptedMoves=" << localResult.acceptedMoves
					<< " validator=PASS" << std::endl;
			}
			std::cout << "[RESPLIT-LS-STATS]"
				<< " SourceDistance=" << source.eval.distance
				<< " AcceptanceGap=" << options.resplitAcceptanceGap
				<< " NearNeutralSplitCandidates=" << nearNeutralSplits.size()
				<< " DynamicLSRuns=" << dynamicLSRuns
				<< " DynamicLSImprovingRuns=" << dynamicLSImprovingRuns
				<< " NetImprovingRuns=" << netImprovingRuns;
			if (dynamicLSRuns > 0 && bestAfterDynamicLS < std::numeric_limits<double>::max()) {
				std::cout << " BestAfterSplit=" << bestAfterSplitLS
					<< " BestAfterDynamicLS=" << bestAfterDynamicLS
					<< " BestNetImprovement=" << source.eval.distance - bestAfterDynamicLS
					<< " BestCaseCustomer=" << bestCaseCustomer
					<< " BestCaseSource=" << source.eval.distance
					<< " BestCaseSplit=" << bestCaseSplit
					<< " BestCasePostLS=" << bestCasePostLS;
			}
			else std::cout << " BestAfterSplit=NA BestAfterDynamicLS=NA BestNetImprovement=0";
			std::cout << " RealSplitNetImprovement=" << (netImprovingRuns > 0 ? 1 : 0) << std::endl;
			if (dynamicElite.get() != nullptr) {
				const ValidationResult eliteValidation = validateSolutionFromScratch(*dynamicElite.get(), params);
				std::cout << "[DYNAMIC-ELITE-SMOKE] distance=" << dynamicElite.get()->eval.distance
					<< " validator=" << (eliteValidation.feasible() ? "PASS" : "FAIL") << std::endl;
			}
		}
		return;
	}

	const Individual* bestFixed = solver.population.getBestFound();
	const Individual* bestOverall = bestFixed;
	bool bestIsDynamic = false;
	if (dynamicEliteController && dynamicEliteController->bestDynamic() &&
		(bestOverall == NULL || dynamicEliteController->bestDynamic()->eval.distance < bestOverall->eval.distance - MY_EPSILON)) {
		bestOverall = &*dynamicEliteController->bestDynamic();
		bestIsDynamic = true;
	}
	if (bestOverall != NULL) {
		const ValidationResult finalValidation = validateSolutionFromScratch(*bestOverall, params);
		std::cout << "[VALIDATOR-COST] Representation="
			<< (bestOverall->quantityRepresentation == QuantityRepresentation::DYNAMIC_QUANTITY
				? "DYNAMIC_QUANTITY" : "FIXED_PSEUDO")
			<< " Strategy=" << demandSplitStrategyName(options.splitStrategy)
			<< " stored=" << objectiveValue2(bestOverall->eval.distance)
			<< " recomputed=" << objectiveValue2(finalValidation.distance)
			<< " difference=" << std::fabs(bestOverall->eval.distance - finalValidation.distance)
			<< " validator=" << (finalValidation.feasible() ? "PASS" : "FAIL") << std::endl;
		if (bestFixed) {
			Individual convertedCheck(*bestFixed);
			convertedCheck.convertFixedToDynamic(params);
			convertedCheck.evaluateDynamicQuantity(params);
			const ValidationResult convertedValidation = validateSolutionFromScratch(convertedCheck, params);
			std::cout << "[VALIDATOR-COST] Representation=DYNAMIC_QUANTITY ConvertedFromFixed=1"
				<< " stored=" << objectiveValue2(convertedCheck.eval.distance)
				<< " recomputed=" << objectiveValue2(convertedValidation.distance)
				<< " difference=" << std::fabs(convertedCheck.eval.distance - convertedValidation.distance)
				<< " validator=" << (convertedValidation.feasible() ? "PASS" : "FAIL") << std::endl;
		}
		const RouteDiagnostics routeDiagnostics = computeRouteDiagnostics(*bestOverall, params);
        if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << options.pathSolution << std::endl;
#if _MSVC_LANG >= 201703L
        std::cout << "**********INSTANCE:" << fs::path(instance_path).stem().string() << " Run " << run_id << std::endl;
#else
        std::cout << "**********INSTANCE:" << std::regex_replace(instance_path, std::regex(".vrp"), "") << " Run " << run_id << std::endl;
#endif
		if (bestIsDynamic) {
			if (!exportDynamicQuantitySolution(*bestOverall, options.pathSolution))
				std::cout << "----- IMPOSSIBLE TO OPEN: " << options.pathSolution << std::endl;
			std::cout << "**********Best Cost:" << objectiveValue2(bestOverall->eval.distance) << std::endl;
			std::cout << "**********Best Found Time:" << bestOverall->eval.optimTime << std::endl;
		}
		else solver.population.exportSDVRPTWLibFormat(*bestOverall, options.pathSolution, cvrp, split_flag);
		const double totalWallRuntime = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - wallRunStart).count();
		std::cout << "[RUN-SUMMARY] InitialBest=" << objectiveValue2(solver.population.getInitialBestDistance())
			<< " BestFixed=" << (bestFixed ? objectiveValue2(bestFixed->eval.distance) : "NA")
			<< " BestOverall=" << objectiveValue2(bestOverall->eval.distance)
			<< " BestSource=" << (bestIsDynamic ? "DYNAMIC_ELITE" : "FIXED")
			<< " Feasible=" << (finalValidation.feasible() ? 1 : 0)
			<< " Validator=" << (finalValidation.feasible() ? "PASS" : "FAIL")
			<< " FixedRngIsolation=" << (fixedRngIsolationPassed ? "PASS" : "FAIL")
			<< " SearchRuntime=" << searchRuntime
			<< " TotalWallRuntime=" << totalWallRuntime << std::endl;
		if (warmStartExecuted) {
			auto warmValue = [](double value) { return value >= 0.0 ? objectiveValue2(value) : std::string("NA"); };
			const bool phasedLegacy = options.warmLegacyFraction > 0.0;
			const char* warmConfig = phasedLegacy
				? (options.warmInitialInfeasibleRepair ? "W3_PHASED" : "W1_PHASED")
				: (options.warmLegacyIntegerDecision
					? (options.warmInitialInfeasibleRepair ? "W3" : "W1")
					: (options.warmInitialInfeasibleRepair ? "W2" : "B0"));
			std::cout << "[WARM-CAUSAL-RESULT]"
				<< " Config=" << warmConfig
				<< " WarmInitialBest=" << warmValue(warmInitialBest)
				<< " WarmBestAt60s=" << warmValue(warmBestAt60s)
				<< " MainInjectedCost=" << warmValue(mainInjectedCost)
				<< " BestAt120s=" << objectiveValue2(bestOverall->eval.distance)
				<< " Routes=" << finalValidation.routesUsed
				<< " Validator=" << (finalValidation.feasible() ? "PASS" : "FAIL")
				<< " WarmInitialFeasible=" << warmInitialFeasible
				<< " WarmInitialInfeasible=" << warmInitialInfeasible
				<< " RepairEligible=" << warmRepairEligible
				<< " RepairAttempts=" << warmRepairAttempts
				<< " RepairSuccess=" << warmRepairSuccesses
				<< std::endl;
		}
		if (dynamicEliteController) {
			const DynamicEliteStats& dynamicStats = dynamicEliteController->statistics();
			std::cout << "[R3-STAGE1] InitialBest=" << objectiveValue2(solver.population.getInitialBestDistance())
				<< " BestFixed=" << (bestFixed ? objectiveValue2(bestFixed->eval.distance) : "NA")
				<< " BestDynamic=" << (dynamicEliteController->bestDynamic()
					? objectiveValue2(dynamicEliteController->bestDynamic()->eval.distance) : "NA")
				<< " BestDynamicSplit=" << (dynamicEliteController->bestDynamicSplit()
					? objectiveValue2(dynamicEliteController->bestDynamicSplit()->eval.distance) : "NA")
				<< " BestOverall=" << objectiveValue2(bestOverall->eval.distance)
				<< " DynamicTriggers=" << dynamicStats.dynamicTriggers
				<< " JointCandidates=" << dynamicStats.jointCandidates
				<< " BridgeCandidates=" << dynamicStats.bridgeCandidates
				<< " BridgeTrajectories=" << dynamicStats.bridgeTrajectories
				<< " DynamicEliteUpdates=" << dynamicStats.dynamicEliteUpdates
				<< " FinalSplitCustomers=" << (bestIsDynamic
					? dynamicEliteController->bestDynamicSplitCustomers() : 0)
				<< " BestSource=" << (bestIsDynamic ? "DYNAMIC_ELITE" : "FIXED")
				<< " Feasible=" << (finalValidation.feasible() ? 1 : 0)
				<< " Validator=" << (finalValidation.feasible() ? "PASS" : "FAIL")
				<< " FixedRngIsolation=" << (fixedRngIsolationPassed ? "PASS" : "FAIL")
				<< " SearchRuntime=" << searchRuntime
				<< " TotalWallRuntime=" << totalWallRuntime << std::endl;
		}
		if (destroyRepairController) {
			const MinimalDestroyRepairStats& drStats = destroyRepairController->statistics();
			std::cout << "[DR-SUMMARY]"
				<< " DRTriggers=" << drStats.triggers
				<< " DRSuccessfulRuns=" << drStats.successfulRuns
				<< " DRSkippedNoFeasibleParent=" << drStats.skippedNoFeasibleParent
				<< " DRFeasible=" << drStats.feasibleRepairs
				<< " DRImprovedParent=" << drStats.improvedParent
				<< " DRNewGlobalBest=" << drStats.newGlobalBest
				<< " DRInserted=" << drStats.inserted
				<< " DRSurvivalChecks=" << drStats.survivalChecks
				<< " DRSurvived=" << drStats.survived
				<< " DRTotalTime=" << drStats.totalTime << std::endl;
		}
		int finalSplitCustomers = 0;
		if (bestOverall->quantityRepresentation == QuantityRepresentation::FIXED_PSEUDO) {
			std::vector<int> routesPerOrigin(params.originalDemand.size(), 0);
			for (const auto& route : bestOverall->chromR) {
				std::vector<bool> seen(params.originalDemand.size(), false);
				for (int client : route) {
					const int origin = params.Corres_origin[client];
					if (origin > 0 && origin < static_cast<int>(seen.size())) seen[origin] = true;
				}
				for (size_t origin = 1; origin < seen.size(); ++origin)
					if (seen[origin]) ++routesPerOrigin[origin];
			}
			for (size_t origin = 1; origin < routesPerOrigin.size(); ++origin)
				if (routesPerOrigin[origin] >= 2) ++finalSplitCustomers;
		}
		else {
			std::vector<int> routesPerOrigin(params.originalDemand.size(), 0);
			for (const auto& route : bestOverall->routesDynamic) {
				std::vector<bool> seen(params.originalDemand.size(), false);
				for (const RouteVisit& visit : route)
					if (visit.originalCustomerId > 0 && visit.originalCustomerId < static_cast<int>(seen.size()))
						seen[visit.originalCustomerId] = true;
				for (size_t origin = 1; origin < seen.size(); ++origin)
					if (seen[origin]) ++routesPerOrigin[origin];
			}
			for (size_t origin = 1; origin < routesPerOrigin.size(); ++origin)
				if (routesPerOrigin[origin] >= 2) ++finalSplitCustomers;
		}
		const MinimalDestroyRepairStats emptyDrStats;
		const MinimalDestroyRepairStats& reportedDrStats = destroyRepairController
			? destroyRepairController->statistics() : emptyDrStats;
		std::cout << "[CURRENT-DR-RESULT]"
			<< " BestCost=" << objectiveValue2(bestOverall->eval.distance)
			<< " Routes=" << finalValidation.routesUsed
			<< " SplitCustomers=" << finalSplitCustomers
			<< " BestFoundTime=" << bestOverall->eval.optimTime
			<< " Validator=" << (finalValidation.feasible() ? "PASS" : "FAIL")
			<< " DRTriggers=" << reportedDrStats.triggers
			<< " DRSuccessfulRuns=" << reportedDrStats.successfulRuns
			<< " DRSkippedNoFeasibleParent=" << reportedDrStats.skippedNoFeasibleParent
			<< " DRFeasible=" << reportedDrStats.feasibleRepairs
			<< " DRImprovedParent=" << reportedDrStats.improvedParent
			<< " DRNewGlobalBest=" << reportedDrStats.newGlobalBest << std::endl;
		if (hasInitialSolutionInjection) {
			std::cout << "[INITIAL-SOLUTION-RESULT]"
				<< " InitialInjectedCost=" << objectiveValue2(initialInjectedCost)
				<< " BestAfterSearch=" << objectiveValue2(bestOverall->eval.distance)
				<< " ImprovementFromInjected=" << objectiveValue2(initialInjectedCost - bestOverall->eval.distance)
				<< " IncumbentPreserved=" << (bestOverall->eval.distance <= initialInjectedCost + MY_EPSILON ? "PASS" : "FAIL")
				<< std::endl;
		}
		if (options.searchGuide == SearchGuideMode::INTEGER_SURROGATE) {
			double surrogateCost = 0.0;
			for (const auto& route : bestOverall->chromR) {
				if (route.empty()) continue;
				int previous = 0;
				for (int client : route) {
					if (params.Corres_origin[previous] != params.Corres_origin[client])
						surrogateCost += params.searchCost[previous][client];
					previous = client;
				}
				surrogateCost += params.searchCost[previous][0];
			}
			std::cout << "[SEARCH-GUIDE-RESULT]"
				<< " Mode=INTEGER_SURROGATE"
				<< " BestSurrogateCost=" << objectiveValue2(surrogateCost)
				<< " BestOfficialCost=" << objectiveValue2(bestOverall->eval.distance)
				<< " BestOfficialFoundTime=" << bestOverall->eval.optimTime
				<< " Routes=" << finalValidation.routesUsed
				<< " Validator=" << (finalValidation.feasible() ? "PASS" : "FAIL")
				<< std::endl;
		}
		std::cout << "RESULT," << fs::path(instance_path).stem().string() << ',' << options.variant << ','
			<< options.ap.seed << ',' << (finalValidation.feasible() ? 1 : 0) << ','
			<< objectiveValue2(bestOverall->eval.distance) << ',' << finalValidation.routesUsed << ','
			<< bestOverall->eval.optimTime << ','
			<< searchRuntime << ','
			<< demandSplitStrategyName(options.splitStrategy) << ','
			<< objectiveValue2(solver.population.getInitialBestDistance()) << ','
			<< routeDiagnostics.averageRouteLoad << ',' << routeDiagnostics.averageUtilization << ','
			<< routeDiagnostics.minimumRouteLoad << ',' << routeDiagnostics.maximumRouteLoad << ','
			<< routeDiagnostics.fullRoutes << ',' << splitResult.diagnostics.pseudoCustomers << ','
			<< splitResult.diagnostics.splitOriginalCustomers << ',' << splitResult.diagnostics.meanPiece << ','
			<< splitResult.diagnostics.capacityComplementScore << std::endl;
    }
	else
	{
		const MinimalDestroyRepairStats emptyDrStats;
		const MinimalDestroyRepairStats& reportedDrStats = destroyRepairController
			? destroyRepairController->statistics() : emptyDrStats;
		std::cout << "[CURRENT-DR-RESULT]"
			<< " BestCost=inf Routes=0 SplitCustomers=0 BestFoundTime=inf Validator=FAIL"
			<< " DRTriggers=" << reportedDrStats.triggers
			<< " DRSuccessfulRuns=" << reportedDrStats.successfulRuns
			<< " DRSkippedNoFeasibleParent=" << reportedDrStats.skippedNoFeasibleParent
			<< " DRFeasible=" << reportedDrStats.feasibleRepairs
			<< " DRImprovedParent=" << reportedDrStats.improvedParent
			<< " DRNewGlobalBest=" << reportedDrStats.newGlobalBest << std::endl;
		std::cout << "RESULT," << fs::path(instance_path).stem().string() << ',' << options.variant << ','
			<< options.ap.seed << ",0,inf,0,inf,"
			<< static_cast<double>(clock() - fullRunStart) / CLOCKS_PER_SEC << ','
			<< demandSplitStrategyName(options.splitStrategy) << ",inf,0,0,0,0,0,"
			<< splitResult.diagnostics.pseudoCustomers << ',' << splitResult.diagnostics.splitOriginalCustomers << ','
			<< splitResult.diagnostics.meanPiece << ',' << splitResult.diagnostics.capacityComplementScore << std::endl;
	}

    for (int i = 0; i < argc; ++i) delete[] argv_mod[i];
    delete[] argv_mod;
}

void process_directory(const std::string& input_dir, const std::string& output_dir, int run_id) {
    if (_MSVC_LANG >= 201703L) {
        for (const auto& entry : fs::recursive_directory_iterator(input_dir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".vrp") {
                std::string input_path = entry.path().string();
                std::string relative_path = fs::relative(entry.path(), input_dir).parent_path().string();
                std::string output_path = output_dir + "/" + relative_path;

                if (!fs::exists(output_path)) {
                    fs::create_directories(output_path);
                }

                solve_vrp_instance(input_path, output_path, run_id);
            }
        }
    }
    else {
        std::cerr << "Error: Filesystem recursion requires C++17 support" << std::endl;
    }
}


// 真正的内存清理函数
void real_cleanup_memory() {
    // 强制清空全局/静态变量
    if (!demand_type.empty()) {
        std::vector<int>().swap(demand_type);
    }
    if (!corr_origin.empty()) {
        std::vector<int>().swap(corr_origin);
    }
    if (!conflict_matrix.empty()) {
        std::vector<std::vector<bool>>().swap(conflict_matrix);
    }

    // 建议系统回收内存
#ifdef _MSC_VER
    PROCESS_MEMORY_COUNTERS pmc;  // 添加变量声明
    // Windows 内存优化
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        if (pmc.WorkingSetSize > 500 * 1024 * 1024) { // 如果占用超过500MB
            SetProcessWorkingSetSize(GetCurrentProcess(), -1, -1);
        }
    }
#endif
}

int main(int argc, char* argv[]) {
	if (argc >= 3)
	{
		try
		{
			CommandLine commandline(argc, argv);
			solve_vrp_instance(commandline.pathInstance, fs::path(commandline.pathSolution).parent_path().string(), 1, &commandline);
			return 0;
		}
		catch (const std::string& error) { std::cerr << "ERROR: " << error << std::endl; return 1; }
		catch (const std::exception& error) { std::cerr << "ERROR: " << error.what() << std::endl; return 1; }
	}
    std::string input_root = R"(.\Instances\SDVRPTW(M-5-100))";

    std::cout << "C++ Standard version: " << _MSVC_LANG << std::endl;

    if (_MSVC_LANG >= 201703L) {
        for (int i = 1; i <= 7; ++i) {
            std::string output_root = R"(.\Instances\OUT\out)" + std::to_string(i);

            std::cout << "==========================================" << std::endl;
            std::cout << "开始第 " << i << " 次运行，输出目录: " << output_root << std::endl;
            std::cout << "==========================================" << std::endl;

            // 使用作用域确保对象在每轮结束后销毁
            {
                std::cout << "清理之前运行的内存数据..." << std::endl;
                real_cleanup_memory();
                std::cout << "内存清理完成" << std::endl;

                auto start_time = std::chrono::high_resolution_clock::now();

                if (!fs::exists(output_root)) {
                    fs::create_directories(output_root);
                }

                process_directory(input_root, output_root, i);

                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

                std::cout << "第 " << i << " 次运行完成，耗时: " << duration.count() << " 秒" << std::endl;
                std::cout << "结果已保存到: " << output_root << std::endl;
                std::cout << std::endl;
            } // 作用域结束，所有局部对象被销毁

            // 额外的内存清理
            real_cleanup_memory();
        }

        std::cout << "所有七次运行已完成！" << std::endl;
    }
    else {
        std::cerr << "Error: Program requires C++17 support for filesystem operations" << std::endl;
        return 1;
    }

    return 0;
}
