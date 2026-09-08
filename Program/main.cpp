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
#include <string>
#include <cstring>
#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <regex>
#include <chrono>
#include <random>

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

void Split_Node(InstanceCVRPLIB& cvrp, bool isRoundingInteger = true)
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
        double distance_temp = 0;
        distance_temp = std::sqrt((cvrp.x_coords[i] - cvrp.x_coords[0]) * (cvrp.x_coords[i] - cvrp.x_coords[0])
            + (cvrp.y_coords[i] - cvrp.y_coords[0]) * (cvrp.y_coords[i] - cvrp.y_coords[0]));
        if (isRoundingInteger) distance_temp = round(distance_temp);
        if (distance_temp > distance_max) distance_max = distance_temp;
    }

    cvrp.level.push_back(0);
    double interval_length = distance_max / L;
    double epsilon = 0;
    for (int i = 1; i < cvrp.nbClients + 1; i++)
    {
        double distance_temp = 0;
        distance_temp = std::sqrt((cvrp.x_coords[i] - cvrp.x_coords[0]) * (cvrp.x_coords[i] - cvrp.x_coords[0])
            + (cvrp.y_coords[i] - cvrp.y_coords[0]) * (cvrp.y_coords[i] - cvrp.y_coords[0]));
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
        std::vector<int> split_result = splitDemand(cvrp.vehicleCapacity, cvrp.demands[i], cvrp.level[i], d, s_max, p, flag, index_minus);
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
            dist_mtx_new[i][j] = std::sqrt(
                (x_coords_new[i] - x_coords_new[j]) * (x_coords_new[i] - x_coords_new[j])
                + (y_coords_new[i] - y_coords_new[j]) * (y_coords_new[i] - y_coords_new[j])
            );

            if (isRoundingInteger) dist_mtx_new[i][j] = round(dist_mtx_new[i][j]);
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

void solve_vrp_instance(const std::string& instance_path, const std::string& output_path) {
    // 生成真正的随机种子
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<unsigned int> distribution(1, 1000000);
    unsigned int random_seed = distribution(generator);

    // 确保输出目录存在
    fs::path output_dir = fs::path(output_path).parent_path();
    if (!output_dir.empty() && !fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    const char* argv[] = {
            "hgs_vrp_solver",  // 程序名
            instance_path.c_str(),
            output_path.c_str(),
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

    if (commandline.verbose) print_algorithm_parameters(commandline.ap);

    if (commandline.verbose) std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
    InstanceCVRPLIB cvrp(commandline.pathInstance, commandline.isRoundingInteger);

    bool split_flag = true;
    Split_Node(cvrp, commandline.isRoundingInteger);

    Params params(cvrp.x_coords, cvrp.y_coords, cvrp.earliest_time, cvrp.latest_time, cvrp.correspoinding_origin, cvrp.dist_mtx, cvrp.service_time, cvrp.demands,
        cvrp.vehicleCapacity, cvrp.durationLimit, commandline.nbVeh, cvrp.isDurationConstraint,
        commandline.verbose, commandline.ap);

    Genetic solver(params);
    solver.run();

    if (solver.population.getBestFound() != NULL) {
        if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << commandline.pathSolution << std::endl;
        std::cout << "**********INSTANCE: " << fs::path(instance_path).stem().string() << std::endl;
        solver.population.exportSDVRPTWLibFormat(*solver.population.getBestFound(), commandline.pathSolution, cvrp, split_flag);
        std::cout << "Solution written to: " << output_path << std::endl;

        // 新增：在控制台打印所有个体的适应度值
    std::cout << std::endl << "**********FINAL POPULATION FITNESS VALUES**********" << std::endl;
    solver.population.printAllFitnessValues(-1);
    }
    else {
        std::cerr << "No solution found for instance: " << instance_path << std::endl;
    }

    for (int i = 0; i < argc; ++i) delete[] argv_mod[i];
    delete[] argv_mod;
}

// 内存清理函数
void cleanup_memory() {
    // 强制垃圾回收（C++风格）
    {
        std::vector<std::string> temp;
        temp.reserve(1000);
        temp.clear();
    }

#ifdef _MSC_VER
    _CrtDumpMemoryLeaks(); // 仅用于调试，检测内存泄漏
#endif
}

int main(int argc, char* argv[]) {
    // 检查命令行参数
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_instance_path> <output_solution_path>" << std::endl;
        std::cerr << "Example: " << argv[0] << " instance.vrp solution.sol" << std::endl;
        return 1;
    }

    std::string instance_path = argv[1];
    std::string output_path = argv[2];

    // 检查输入文件是否存在
    if (!fs::exists(instance_path)) {
        std::cerr << "Error: Input file '" << instance_path << "' does not exist!" << std::endl;
        return 1;
    }

    std::cout << "==========================================" << std::endl;
    std::cout << "VRP Solver Started" << std::endl;
    std::cout << "Input instance: " << instance_path << std::endl;
    std::cout << "Output solution: " << output_path << std::endl;
    std::cout << "==========================================" << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();


    try {
        solve_vrp_instance(instance_path, output_path);
    }
    catch (const std::exception& e) {
        std::cerr << "Error solving VRP instance: " << e.what() << std::endl;
        return 1;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    std::cout << "==========================================" << std::endl;
    std::cout << "VRP Solver Finished" << std::endl;
    std::cout << "Time elapsed: " << duration.count() << " seconds" << std::endl;
    std::cout << "==========================================" << std::endl;

    return 0;
}