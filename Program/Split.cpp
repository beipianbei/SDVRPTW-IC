#include "Split.h" 
#include "params.h" 
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <cstdlib> 
#include <iomanip>

// 只返回 earliest_time 最小的 n 个客户的 id
std::vector<int> getNEarliestClients(const Params& params, int n, Individual& indiv) {
	// 1. 临时存 <id, earliest_time> 用于排序
	std::vector<std::pair<int, int>> temp;
	temp.reserve(indiv.chromT.size());

	for (int i = 0; i < indiv.chromT.size(); ++i) {
		temp.emplace_back(i, params.cli[indiv.chromT[i]].earliest_time);
	}

	// 2. 按 earliest_time 升序排序
	std::sort(temp.begin(), temp.end(),
		[](const auto& a, const auto& b) {
			return a.second < b.second;
		});

	// 3. 只要 id，且最多取 n 个
	std::vector<int> ids;
	int prev_cli = 0;
	ids.reserve(std::min(static_cast<size_t>(n), temp.size()));
	for (size_t k = 0; k < temp.size(); ++k) {
		if (params.Corres_origin[indiv.chromT[temp[k].first]] == params.Corres_origin[prev_cli]) continue;
		ids.push_back(indiv.chromT[temp[k].first]);
		if (ids.size() == n)break;
		prev_cli = indiv.chromT[temp[k].first];
	}
	if (ids.size() < n) {
		for (size_t k = 0; k < n && k < temp.size(); ++k) {
			if (std::find(ids.begin(), ids.end(), indiv.chromT[temp[k].first]) == ids.end()) {
				ids.push_back(indiv.chromT[temp[k].first]);
				if (ids.size() == n) break;
			}
		}
	}
	return ids;
}

bool CanInsert(int node, std::vector <int> route_R)
{
	//20250929 快速合规：把 node 插到目标路线是否安全
	int type = demand_type[corr_origin[node]];
	for (int i = 0; i < route_R.size(); i++)
	{
		if (conflict_matrix[type][demand_type[corr_origin[route_R[i]]]]) return false; // 冲突
	}
	return true;
}

// 返回 true：全部合规；false：至少一条违规
bool validateRoutes(const std::vector<std::vector<int>>& chromr_ab, const Params& params)
{
	bool allOk = true;

	//std::cout << "========== 所有路径明细 ==========\n";
	for (size_t r = 0; r < chromr_ab.size(); ++r)
	{
		const auto& route = chromr_ab[r];
		if (route.empty()) continue;

		int routeDemand = 0;
		int demandtype = demand_type[corr_origin[route[0]]];

		/* ---------- 1. 类型检查 ---------- */
		for (int cust : route)
		{
			const auto& c = params.cli[cust];
			routeDemand += c.demand;
			if (conflict_matrix[demandtype][demand_type[corr_origin[cust]]]) {
				std::cout << "路径货物类型不一致" << std::endl;
			}
		}

		/* ---------- 2. 容量检查 ---------- */
		if (routeDemand > params.vehicleCapacity) allOk = false;

		/* ---------- 3. 时间窗检查 ---------- */
		int t = params.timeCost[0][route[0]]; // 到达首客户
		if (t > params.cli[route[0]].latest_time) {
			allOk = false;
		}
		t += params.cli[route[0]].serviceDuration; // 本客户服务结束

		for (size_t i = 1; i < route.size(); ++i)
		{
			int cust = route[i];
			t += params.timeCost[route[i - 1]][cust];

			// 同一 origin 连续客户 → 跳过服务时间
			if (params.Corres_origin[cust] == params.Corres_origin[route[i - 1]])
				continue;
			else {
				int start = std::max<int>(t, params.cli[cust].earliest_time);
				if (start > params.cli[cust].latest_time)
				{
					std::cout << "❌ Route " << r + 1 << " 客户 " << cust
						<< " 时间窗违反：开始 " << start
						<< " > 最晚 " << params.cli[cust].latest_time << '\n';
					allOk = false;
				}
				t = start + params.cli[cust].serviceDuration; // 本客户服务结束
			}
		}
		// 返回 depot
		t += params.timeCost[route.back()][0];
		if (t > params.cli[0].latest_time)
		{
			std::cout << "❌ Route " << r + 1 << " 返回 depot 超时："
				<< t << " > " << params.cli[0].latest_time << '\n';
			allOk = false;
		}
		//std::cout << "----------------------------------\n";
	}
	//std::cout << "========== 校验结束 ==========\n";
	return allOk;
}

bool SvalidateRoutes(const std::vector<std::vector<int>>& chromr_ab, const Params& params)
{
	bool allOk = true;

	std::cout << "========== 所有路径明细 ==========\n";
	for (size_t r = 0; r < chromr_ab.size(); ++r)
	{
		const auto& route = chromr_ab[r];
		if (route.empty()) continue;

		std::cout << "Route " << r + 1 << " : ";
		for (int cust : route) std::cout << cust << ' ';
		std::cout << "\n---- 客户详情 ----\n";

		int routeDemand = 0;
		for (int cust : route)
		{
			const auto& c = params.cli[cust];
			std::cout << "  客户 " << std::setw(3) << cust
				<< "  demand= " << std::setw(4) << c.demand
				<< "  earliest= " << std::setw(4) << c.earliest_time
				<< "  latest= " << std::setw(4) << c.latest_time
				<< "  origin= " << std::setw(2) << params.Corres_origin[cust]
				<< '\n';
			routeDemand += c.demand;
		}
		std::cout << "  本路径总需求 = " << routeDemand
			<< " / 车辆容量 = " << params.vehicleCapacity << "\n\n";

		/* ---------- 1. 容量检查 ---------- */
		if (routeDemand > params.vehicleCapacity)
		{
			std::cout << "❌ Route " << r + 1 << " 超载！\n";
			allOk = false;
		}

		/* ---------- 2. 时间窗检查 ---------- */
		int t = params.timeCost[0][route[0]]; // 到达首客户
		if (t > params.cli[route[0]].latest_time) {
			std::cout << "❌ Route " << r + 1 << " 客户 " << route[0]
				<< " 时间窗违反：开始 " << t
				<< " > 最晚 " << params.cli[route[0]].latest_time << '\n';
			allOk = false;
		}
		t += params.cli[route[0]].serviceDuration; // 本客户服务结束

		for (size_t i = 1; i < route.size(); ++i)
		{
			int cust = route[i];
			t += params.timeCost[route[i - 1]][cust];

			// 同一 origin 连续客户 → 跳过服务时间
			if (params.Corres_origin[cust] == params.Corres_origin[route[i - 1]])
				continue;
			else {
				int start = std::max<int>(t, params.cli[cust].earliest_time);
				if (start > params.cli[cust].latest_time)
				{
					std::cout << "❌ Route " << r + 1 << " 客户 " << cust
						<< " 时间窗违反：开始 " << start
						<< " > 最晚 " << params.cli[cust].latest_time << '\n';
					allOk = false;
				}
				t = start + params.cli[cust].serviceDuration; // 本客户服务结束
			}
		}
		// 返回 depot
		t += params.timeCost[route.back()][0];
		if (t > params.cli[0].latest_time)
		{
			std::cout << "❌ Route " << r + 1 << " 返回 depot 超时："
				<< t << " > " << params.cli[0].latest_time << '\n';
			allOk = false;
		}
		std::cout << "----------------------------------\n";
	}
	std::cout << "========== 校验结束 ==========\n";
	return allOk;
}

// 计算路径 `route` 的总时间窗违反量,或者路径距离总成本
int calcCost(std::vector<int>& route, Params& params) {
	int cost = params.timeCost[0][route[0]];// 从 depot(0) 到第一个客户

	int exptime = 0; //时间
	if (cost > params.cli[route[0]].latest_time) return INT_MAX;
	else exptime = std::max<int>(cost, params.cli[route[0]].earliest_time) + params.cli[route[0]].serviceDuration;

	for (size_t i = 0; i + 1 < route.size(); ++i) {// 客户间行驶

		if (params.Corres_origin[route[i]] == params.Corres_origin[route[i + 1]]) continue;
		else {
			cost += params.timeCost[route[i]][route[i + 1]];
			exptime += params.timeCost[route[i]][route[i + 1]];
			if (exptime > params.cli[route[i + 1]].latest_time) return INT_MAX;
			else exptime = std::max<int>(exptime, params.cli[route[i + 1]].earliest_time) + params.cli[route[i + 1]].serviceDuration;
		}
	}

	cost += params.timeCost[route.back()][0];// 从最后一个客户返回 depot(0)
	return cost;
}

InsertPos SelectInsertionPosition(Individual& indiv, Params& params, int cli_i) {
	InsertPos best{ -1, -1, INT_MAX };

	for (int r = 0; r < indiv.chromR.size(); ++r) {          // 遍历已有路径
		if (indiv.chromR[r].empty() || params.chromdemand[r]
			+ params.cli[cli_i].demand > params.vehicleCapacity) continue;//顺便检查是否超载
		auto& route = indiv.chromR[r];

		// --------------- 检测路径中是否有不兼容客户
		if (!CanInsert(cli_i, indiv.chromR[r])) continue;

		//计算原有成本
		int	Cost = calcCost(route, params);

		for (int p = 0; p <= static_cast<int>(route.size()); ++p) {

			route.insert(route.begin() + p, cli_i); // 尝试插入到位置 p
			int newCost = calcCost(route, params);

			if (newCost == INT_MAX) {
				int delta = newCost - Cost;             //计算成本变化值
				route.erase(route.begin() + p);         // 恢复原路径
				continue;
			}

			int delta = newCost - Cost;             //计算成本变化值
			route.erase(route.begin() + p);         // 恢复原路径

			if (delta < best.delta) {
				best = { r, p, delta };
			}
		}
	}
	return best;
}

void Split::generalSplitEnhanced(Individual& indiv, Params& params)
{
	// Do not apply Split with fewer vehicles than the trivial (LP) bin packing bound
	int TotalDemand = 0;
	for (int clii : indiv.chromT) TotalDemand += params.cli[clii].demand;
	int maxvehicles = std::max<int>(0, std::ceil(TotalDemand / params.vehicleCapacity));
	int Vehicles = maxvehicles - 1;

	auto earlistTimeCli = getNEarliestClients(params, maxvehicles, indiv);

	// 使用assign函数重置：清除所有现有元素，然后分配nbVehicles个0。
	params.chromdemand.assign(params.nbVehicles, 0);

	// 1. 给每条路添加起始节点（后续也可能改变）
	for (int i = 0; i < earlistTimeCli.size(); i++) {
		indiv.chromR[i].push_back(earlistTimeCli[i]);
		params.chromdemand[i] += params.cli[earlistTimeCli[i]].demand;
	}
	// 2. 标记哪些客户已经被写入
	std::vector<bool> used(params.nbClients + 1, false); // 1-based
	for (int id : earlistTimeCli) used[id] = true;

	// 3. 把剩余客户按原始编号顺序逐一插到 chromR 后面
	for (int j = 0; j < indiv.chromT.size(); ++j) {
		int i = indiv.chromT[j];
		if (used[i]) continue;                             // 写过的客户
		
		InsertPos best = SelectInsertionPosition(indiv, params, i);
		
		if (best.routeIdx == -1) {                 // 无可行位，新建路径

			Vehicles++;
			indiv.chromR[Vehicles].push_back(i);
			params.chromdemand[Vehicles] += params.cli[i].demand;
		}
		else {                                   // 插入最佳位置
			auto& route = indiv.chromR[best.routeIdx];
			route.insert(route.begin() + best.pos, i);
			params.chromdemand[best.routeIdx] += params.cli[i].demand;
		}

		if (!validateRoutes(indiv.chromR, params)) {
			std::cout << "存在违规路径！\n";
			SvalidateRoutes(indiv.chromR, params);
		}
	}

	if (!validateRoutes(indiv.chromR, params)) {
		std::cout << "存在违规路径！\n";
	}
}

void Split::generalSplit(Individual & indiv, int nbMaxVehicles, Params & params)
{
	// Do not apply Split with fewer vehicles than the trivial (LP) bin packing bound
	//maxVehicles = std::max<int>(nbMaxVehicles, std::ceil(params.totalDemand/params.vehicleCapacity));
	maxVehicles = std::max<int>(0, std::ceil(params.totalDemand / params.vehicleCapacity));
	
	// ---------- Filling the chromR structure ----------
	for (int k = 0; k < indiv.chromR.size(); k++) indiv.chromR[k].clear();

	// 初始化 cliSplit 与累加数组
	cliSplit.clear();
	cliSplit.resize(indiv.chromT.size() + 1);
	sumDistance.resize(indiv.chromT.size() + 1);
	sumLoad.resize(indiv.chromT.size() + 1);
	sumService.resize(indiv.chromT.size() + 1);
	indiv.chromTAorB = indiv.chromT;
	for (size_t i = 1; i <= indiv.chromT.size(); ++i)
	{
		int client = indiv.chromTAorB[i - 1];
		cliSplit[i].demand = params.cli[client].demand;
		cliSplit[i].serviceTime = params.cli[client].serviceDuration;
		cliSplit[i].d0_x = params.timeCost[0][client];
		cliSplit[i].dx_0 = params.timeCost[client][0];
		if (i < indiv.chromT.size())
			cliSplit[i].dnext = params.timeCost[client][indiv.chromTAorB[i]];
		else
			cliSplit[i].dnext = -1.e30;

		sumLoad[i] = sumLoad[i - 1] + cliSplit[i].demand;
		sumService[i] = sumService[i - 1] + cliSplit[i].serviceTime;
		sumDistance[i] = sumDistance[i - 1] + cliSplit[i - 1].dnext;
	}
	generalSplitEnhanced(indiv, params);

	// Build up the rest of the Individual structure
	indiv.evaluateCompleteCost(params);
}

// 检查单个客户是否满足时间要求20241130
bool checkCustomerTime(Params& params, int customerIndex, int currentTime) {
	if (currentTime < params.cli[customerIndex].earliest_time || currentTime > params.cli[customerIndex].latest_time) {
		return false; // 时间不满足要求
	}
	return true; // 时间满足要求
}

Split::Split(Params & params): params(params)
{
	// Structures of the linear Split
	cliSplit = std::vector <ClientSplit>(params.nbClients + 1);
	sumDistance = std::vector <double>(params.nbClients + 1,0.);
	sumLoad = std::vector <double>(params.nbClients + 1,0.);
	sumService = std::vector <double>(params.nbClients + 1, 0.);
	potential = std::vector < std::vector <double> >(params.nbVehicles + 1, std::vector <double>(params.nbClients + 1,1.e30));
	pred = std::vector < std::vector <int> >(params.nbVehicles + 1, std::vector <int>(params.nbClients + 1,0));
}
