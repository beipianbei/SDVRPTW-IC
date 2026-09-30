#include "Individual.h" 

#include "SolutionValidator.h"
#include <map>
#include <limits>

void Individual::evaluateCompleteCost(const Params & params)
{
	bool TW_Flag = true;

	eval = EvalIndiv();

	if (TW_Flag)
	{
		for (int r = 0; r < params.nbVehicles; r++)
		{
			if (!chromR[r].empty())
			{
				double TimeWindowExcess = 0.0;//此项仅考虑到达时间超过最晚时间窗的超过内容，不考虑提前到达，因为提前到达不构成不可行解
				//首先考虑第一个客户节点
				double time = params.timeCost[0][chromR[r][0]];
				if (time >= params.cli[chromR[r][0]].earliest_time && time <= params.cli[chromR[r][0]].latest_time)
				{
					time += params.cli[chromR[r][0]].serviceDuration;
				}
				else if (time < params.cli[chromR[r][0]].earliest_time)
				{
					time = params.cli[chromR[r][0]].earliest_time + params.cli[chromR[r][0]].serviceDuration;
				}
				else//到达时间超过服务允许时间窗
				{
					TimeWindowExcess += time - params.cli[chromR[r][0]].latest_time;
					time += params.cli[chromR[r][0]].serviceDuration;
				}

				double distance = params.timeCost[0][chromR[r][0]];			//距离
				double load = params.cli[chromR[r][0]].demand;				//需求量
				predecessors[chromR[r][0]] = 0;

				for (int i = 1; i < (int)chromR[r].size(); i++)
				{

					if (params.Corres_origin[chromR[r][i - 1]] == params.Corres_origin[chromR[r][i]]){
						//time += params.cli[chromR[r][i]].serviceDuration;//20241201
						load += params.cli[chromR[r][i]].demand;
					}
					else{
						time += params.timeCost[chromR[r][i - 1]][chromR[r][i]];
						
						if (time >= params.cli[chromR[r][i]].earliest_time && time <= params.cli[chromR[r][i]].latest_time)
						{
							time += params.cli[chromR[r][i]].serviceDuration;
						}
						else if (time < params.cli[chromR[r][i]].earliest_time)
						{
							time = params.cli[chromR[r][i]].earliest_time;
							time += params.cli[chromR[r][i]].serviceDuration;
						}
						else//到达时间超过服务允许时间窗
						{
							TimeWindowExcess += time - params.cli[chromR[r][i]].latest_time;
							time += params.cli[chromR[r][i]].serviceDuration;
						}

						distance += params.timeCost[chromR[r][i - 1]][chromR[r][i]];
						load += params.cli[chromR[r][i]].demand;
						predecessors[chromR[r][i]] = chromR[r][i - 1];
						successors[chromR[r][i - 1]] = chromR[r][i];
					}
				}
				//加上是否逾越depot的最晚时间窗
				time += params.timeCost[chromR[r][chromR[r].size() - 1]][0];

				if (time > params.cli[0].latest_time) TimeWindowExcess += time - params.cli[0].latest_time;

				successors[chromR[r][chromR[r].size() - 1]] = 0;
				distance += params.timeCost[chromR[r][chromR[r].size() - 1]][0];
				eval.distance += distance;
				eval.nbRoutes++;

				if (load > params.vehicleCapacity) eval.capacityExcess += load - params.vehicleCapacity;
				//if (distance + service > params.durationLimit) eval.durationExcess += distance + service - params.durationLimit;
				if (TimeWindowExcess > MY_EPSILON) eval.durationExcess += TimeWindowExcess;
			}
		}

		eval.penalizedCost = eval.distance + eval.capacityExcess * params.penaltyCapacity + eval.durationExcess * params.penaltyDuration;
		eval.isFeasible = (eval.capacityExcess < MY_EPSILON && eval.durationExcess < MY_EPSILON);// && TotalTimeWindowExcess < MY_EPSILON
		
		eval.optimTime = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;

	}
	else
	{
		for (int r = 0; r < params.nbVehicles; r++)
		{
			if (!chromR[r].empty())
			{
				double distance = params.timeCost[0][chromR[r][0]];
				double load = params.cli[chromR[r][0]].demand;
				double service = params.cli[chromR[r][0]].serviceDuration;
				predecessors[chromR[r][0]] = 0;
				for (int i = 1; i < (int)chromR[r].size(); i++)
				{
					distance += params.timeCost[chromR[r][i - 1]][chromR[r][i]];
					load += params.cli[chromR[r][i]].demand;
					service += params.cli[chromR[r][i]].serviceDuration;
					predecessors[chromR[r][i]] = chromR[r][i - 1];
					successors[chromR[r][i - 1]] = chromR[r][i];
				}
				successors[chromR[r][chromR[r].size() - 1]] = 0;
				distance += params.timeCost[chromR[r][chromR[r].size() - 1]][0];
				eval.distance += distance;
				eval.nbRoutes++;
				if (load > params.vehicleCapacity) eval.capacityExcess += load - params.vehicleCapacity;
				if (distance + service > params.durationLimit) eval.durationExcess += distance + service - params.durationLimit;
			}
		}

		eval.penalizedCost = eval.distance + eval.capacityExcess * params.penaltyCapacity + eval.durationExcess * params.penaltyDuration;
		eval.isFeasible = (eval.capacityExcess < MY_EPSILON && eval.durationExcess < MY_EPSILON);

		eval.optimTime = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;
	}

}

Individual::Individual(Params & params)
{
	//20250916
	chromR_ab = std::vector < std::vector <int> >(params.nbVehicles);

	// These arrays are indexed by pseudo-client IDs, not original-customer IDs.
	successors = std::vector <int>(params.nbClients + 1);
	predecessors = std::vector <int>(params.nbClients + 1);
	chromR = std::vector < std::vector <int> >(params.nbVehicles);
	// Shuffle original-customer blocks, not independent pseudo-clients.
	std::vector<std::vector<int>> groups(params.nbClients + 1);
	for (int client = 1; client <= params.nbClients; ++client)
		groups[params.Corres_origin[client]].push_back(client);
	std::vector<int> origins;
	for (int origin = 1; origin < static_cast<int>(groups.size()); ++origin)
		if (!groups[origin].empty()) origins.push_back(origin);
	std::shuffle(origins.begin(), origins.end(), params.ran);
	chromT.clear();
	for (int origin : origins) chromT.insert(chromT.end(), groups[origin].begin(), groups[origin].end());
	eval.penalizedCost = 1.e30;	
}

Individual::Individual(Params & params, std::string fileName) : Individual(params)
{
	double readCost;
	chromT.clear();
	std::ifstream inputFile(fileName);
	if (inputFile.is_open())
	{
		std::string inputString;
		inputFile >> inputString;
		// Loops in the input file as long as the first line keyword is "Route"
		for (int r = 0; inputString == "Route"; r++)
		{
			inputFile >> inputString;
			getline(inputFile, inputString);
			std::stringstream ss(inputString);
			int inputCustomer;
			while (ss >> inputCustomer) // Loops as long as there is an integer to read in this route
			{
				chromT.push_back(inputCustomer);
				chromR[r].push_back(inputCustomer);
			}
			inputFile >> inputString;
		}
		if (inputString == "Cost") inputFile >> readCost;
		else throw std::string("Unexpected token in input solution");

		// Some safety checks and printouts
		evaluateCompleteCost(params);
		if ((int)chromT.size() != params.nbClients) throw std::string("Input solution does not contain the correct number of clients");
		if (!eval.isFeasible) throw std::string("Input solution is infeasible");
		if (eval.penalizedCost != readCost)throw std::string("Input solution has a different cost than announced in the file");
		if (params.verbose) std::cout << "----- INPUT SOLUTION HAS BEEN SUCCESSFULLY READ WITH COST " << eval.penalizedCost << std::endl;
	}
	else 
		throw std::string("Impossible to open solution file provided in input in : " + fileName);
}

void Individual::convertFixedToDynamic(const Params& params)
{
	routesDynamic.assign(chromR.size(), {});
	for (size_t r = 0; r < chromR.size(); ++r) {
		std::map<int, size_t> firstVisit;
		for (int client : chromR[r]) {
			const int origin = params.Corres_origin[client];
			auto it = firstVisit.find(origin);
			if (it == firstVisit.end()) {
				firstVisit[origin] = routesDynamic[r].size();
				routesDynamic[r].push_back({ origin, params.cli[client].demand });
			}
			else routesDynamic[r][it->second].quantity += params.cli[client].demand;
		}
	}
	quantityRepresentation = QuantityRepresentation::DYNAMIC_QUANTITY;
}

void Individual::evaluateDynamicQuantity(const Params& params)
{
	// Mark the stored objective unavailable while recomputing it. The validator
	// compares stored and recomputed costs for completed individuals.
	eval.distance = std::numeric_limits<double>::quiet_NaN();
	const ValidationResult checked = validateSolutionFromScratch(*this, params);
	eval = EvalIndiv();
	eval.distance = checked.distance;
	eval.capacityExcess = checked.capacityExcess;
	eval.durationExcess = checked.timeWindowExcess;
	eval.nbRoutes = checked.routesUsed;
	eval.penalizedCost = eval.distance + eval.capacityExcess * params.penaltyCapacity + eval.durationExcess * params.penaltyDuration;
	eval.isFeasible = checked.feasible();
	eval.optimTime = static_cast<double>(clock() - params.startTime) / CLOCKS_PER_SEC;
}

