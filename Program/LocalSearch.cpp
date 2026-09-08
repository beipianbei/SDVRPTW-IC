#include "LocalSearch.h" 
#include "algorithm"
#include <vector>
#include <iostream>

extern std::vector<std::vector<bool>> conflict_matrix;

void LocalSearch::run(Individual& indiv, double penaltyCapacityLS, double penaltyDurationLS)
{
	this->penaltyCapacityLS = penaltyCapacityLS;
	this->penaltyDurationLS = penaltyDurationLS;

	loadIndividual(indiv);

	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderNodes.begin(), orderNodes.end(), params.ran);
	std::shuffle(orderRoutes.begin(), orderRoutes.end(), params.ran);
	
	for (int i = 1; i <= params.nbClients; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVertices[i].begin(), params.correlatedVertices[i].end(), params.ran);

	searchCompleted = false;
	for (loopID = 0; !searchCompleted; loopID++)
	{
		if (loopID > 1) // Allows at least two loops since some moves involving empty routes are not checked at the first loop
			searchCompleted = true;

		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
	 	for (int posU = 0; posU < params.nbClients; posU++)
		{
			nodeU = &clients[orderNodes[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;

			for (int posV = 0; posV < (int)params.correlatedVertices[nodeU->cour].size(); posV++)
			{
				nodeV = &clients[params.correlatedVertices[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (move1()) continue; // RELOCATE
					if (move2()) continue; // RELOCATE
					if (move3()) continue;// RELOCATE
					if (nodeUIndex <= nodeVIndex && move4()) continue;// RELOCATE
					if (move5()) continue; // SWAP
					if (nodeUIndex <= nodeVIndex && move6()) continue; // SWAP
					if (intraRouteMove && move7()) continue; // 2-OPT
					if (!intraRouteMove && move8()) continue; // 2-OPT*
					if (!intraRouteMove && move9()) continue; // 2-OPT*

					// Trying moves that insert nodeU directly after the depot
					if (nodeV->prev->isDepot)
					{
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						if (move1()) continue; // RELOCATE
						if (move2())continue; // RELOCATE
						if (move3()) continue;// RELOCATE
						if (!intraRouteMove && move8()) continue; // 2-OPT*
						if (!intraRouteMove && move9()) continue; // 2-OPT*
					}
				}
			}

			/* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO AVOID INCREASING TOO MUCH THE FLEET SIZE */
			if (loopID > 0 && !emptyRoutes.empty())
			{
				nodeV = routes[*emptyRoutes.begin()].depot;
				setLocalVariablesRouteU();
				setLocalVariablesRouteV();
				if (move1()) continue; // RELOCATE
				if (move2()) continue; // RELOCATE
				if (move3()) continue; // RELOCATE
				if (move9()) continue; // 2-OPT*
			}
		}

		if (params.ap.useSwapStar == 1 && params.areCoordinatesProvided)
		{
			/* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
			for (int rU = 0; rU < params.nbVehicles; rU++)
			{
				routeU = &routes[orderRoutes[rU]];
				int lastTestSWAPStarRouteU = routeU->whenLastTestedSWAPStar;
				routeU->whenLastTestedSWAPStar = nbMoves;
				for (int rV = 0; rV < params.nbVehicles; rV++)
				{
					routeV = &routes[orderRoutes[rV]];
					if (routeU->nbCustomers > 0 && routeV->nbCustomers > 0 && routeU->cour < routeV->cour
						&& (loopID == 0 || std::max<int>(routeU->whenLastModified, routeV->whenLastModified)
					> lastTestSWAPStarRouteU))
						if (CircleSector::overlap(routeU->sector, routeV->sector))
							swapStar();
				}
			}
		}
	}
	// Register the solution produced by the LS in the individual
	exportIndividual(indiv);
}

void LocalSearch::setLocalVariablesRouteU()
{
	routeU = nodeU->route;
	nodeX = nodeU->next;
	nodeXNextIndex = nodeX->next->cour;
	nodeUIndex = nodeU->cour;
	nodeUPrevIndex = nodeU->prev->cour;
	nodeXIndex = nodeX->cour;
	loadU = params.cli[nodeUIndex].demand;
	serviceU = params.cli[nodeUIndex].serviceDuration;
	loadX = params.cli[nodeXIndex].demand;
	serviceX = params.cli[nodeXIndex].serviceDuration;

	// 找出路由中的客户顺序，并找出nodeU的索引
	RouteCliSeqU.clear();
	Node* nodeNew = routeU->depot;
	int index = 0;
	while (true) {
		RouteCliSeqU.push_back(nodeNew->cour);

		nodeNew = nodeNew->next;
		index++;
		if (nodeNew->cour == nodeUIndex) nodeUN = index;
		if (nodeNew->cour == 0.) {
			RouteCliSeqU.push_back(0);
			break;
		}
	}
}

void LocalSearch::setLocalVariablesRouteV()
{
	routeV = nodeV->route;
	nodeY = nodeV->next;
	nodeYNextIndex = nodeY->next->cour;
	nodeVIndex = nodeV->cour;
	nodeVPrevIndex = nodeV->prev->cour;
	nodeYIndex = nodeY->cour;
	loadV = params.cli[nodeVIndex].demand;
	serviceV = params.cli[nodeVIndex].serviceDuration;
	loadY = params.cli[nodeYIndex].demand;
	serviceY = params.cli[nodeYIndex].serviceDuration;
	intraRouteMove = (routeU == routeV);

	// 找出路由中的客户顺序，并找出nodeV的索引
	RouteCliSeqV.clear();
	Node* nodeNew = routeV->depot;
	int index = 0;
	while (true) {
		RouteCliSeqV.push_back(nodeNew->cour);
		nodeNew = nodeNew->next;
		index++;
		if (nodeNew->cour == nodeVIndex) nodeVN = index;
		if (nodeNew->cour == 0) {
			RouteCliSeqV.push_back(0);
			break;
		}
	}
}

// 判断移动节点后,当前节点时间窗是否被满足20241201
int LocalSearch::TimeWindowIsMetCVRP(std::vector<int>& RouteCliSeq) {

	int expressTime = 0;
	double cost = 0;
	int bb = 0;
	int nodecli;

	for (int i = 1; i < RouteCliSeq.size(); i++) {
		nodecli = RouteCliSeq[i];
		cost += params.timeCost[RouteCliSeq[i - 1]][nodecli];

		if (params.Corres_origin[nodecli] != params.Corres_origin[RouteCliSeq[i - 1]]){
			if (cost > params.cli[nodecli].latest_time) {
				expressTime += cost - params.cli[nodecli].latest_time;
				cost += params.cli[nodecli].serviceDuration;
			}
			else {
				if (cost >= params.cli[nodecli].earliest_time) cost += params.cli[nodecli].serviceDuration;
				else cost = params.cli[nodecli].earliest_time + params.cli[nodecli].serviceDuration;
			}
		}
		
	}
	return expressTime;
}
// 判断移动节点后,当前节点时间窗是否被满足20241201
int LocalSearch::TimeWindowIsMetswapStar(Node* U, Node* V) {
	if ( U->cour == 0 || V->cour == 0) return 1;//返回值大于1 相当于改进结果变差
	int costTime = 0;
	double cost = 0;
	// 找出路由中的客户顺序，并找出nodeV的索引
	std::vector<int> RouteCliSeqX;
	std::vector<int> RouteCliSeqY;
	int node_X = U->cour;
	Route* routeX = U->route;
	Route* routeY = V->route;
 
	Node* nodeNew = routeX->depot;
	int index = 0;
	int nodeX_index = 0;
	int nodeY_index = 0;
	while (true) {
		RouteCliSeqX.push_back(nodeNew->cour);
		nodeNew = nodeNew->next;
		if (nodeNew->cour == U->cour) nodeX_index = index;
		if (nodeNew->cour == 0) {
			RouteCliSeqX.push_back(0);
			break;
		}
		index++;
	}
	nodeNew = routeY->depot;
	index = 0;
	while (true) {
		RouteCliSeqY.push_back(nodeNew->cour);
		nodeNew = nodeNew->next;
		if (nodeNew->cour == V->cour) nodeY_index = index;
		if (nodeNew->cour == 0) {
			RouteCliSeqY.push_back(0);
			break;
		}
		index++;
	}

	double prev_time = 0;
	if (RouteCliSeqX == RouteCliSeqY){
		prev_time = TimeWindowIsMetCVRP(RouteCliSeqX);
		RouteCliSeqX.erase(RouteCliSeqX.begin() + nodeX_index); // 从其当前位置移除节点 U
		if(nodeY_index > nodeX_index){
			RouteCliSeqX.insert(RouteCliSeqX.begin() + nodeY_index, node_X); // 将节点 U 插入到节点 V 的位置之后
		}
		else{
			RouteCliSeqX.insert(RouteCliSeqX.begin() + nodeY_index + 1, node_X); // 将节点 U 插入到节点 V 的位置之后
		}
		double costTimeV = TimeWindowIsMetCVRP(RouteCliSeqX);

		//如果超期时间增加则直接返回
		if (costTimeV - prev_time > -MY_EPSILON) return 1;

	}
	else {
		//计算原路线超期服务时间
		prev_time = TimeWindowIsMetCVRP(RouteCliSeqX) + TimeWindowIsMetCVRP(RouteCliSeqY);

		RouteCliSeqX.erase(RouteCliSeqX.begin() + nodeX_index); // 从其当前位置移除节点 U
		double costTimeU = TimeWindowIsMetCVRP(RouteCliSeqU);
		RouteCliSeqY.insert(RouteCliSeqY.begin() + nodeY_index + 1, node_X); // 将节点 U 插入到节点 V 的位置之后
		double costTimeV = TimeWindowIsMetCVRP(RouteCliSeqY);
		
		if (costTimeU + costTimeV - prev_time > -MY_EPSILON) return 1;
	}
	
	return costTime;
}

bool LocalSearch::move1()  
{
	//𝑴𝟏：将客户点𝒖从原有位置移除，插入𝒗之后
	//20250928 如果U为奇数，只要routeV中有不同的奇数，则跳过，否则再详细判断
	if (!intraRouteMove) if (!canInsert(nodeUIndex, routeV)) return false;   // 存在冲突，放弃 M1

	// 加时间窗计算参数20241201
	if (nodeUIndex == 0 || nodeVIndex == 0) return false;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;
	double costTimeU = 0;
	double costTimeV = 0;
	double prevTime = 0;
	if (RouteCliSequ == RouteCliSeqv) {//同一条路线
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ);
		if (nodeVN <= 0) return false;
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN); // 从其当前位置移除节点 U
		if (nodeUN >= nodeVN) {
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN + 1, nodeUIndex); // 将节点 U 插入到节点 V 的位置之后
		}
		else {
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN, nodeUIndex); // 将节点 U 插入到节点 V 的位置之后
		}
		costTimeV = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeU = costTimeV;
	}
	else {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);
		if (nodeVN + 1 <= 0) return false;
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN); // 从其当前位置移除节点 U
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		RouteCliSeqv.insert(RouteCliSeqv.begin() + nodeVN + 1, nodeUIndex); // 将节点 U 插入到节点 V 的位置之后
		costTimeV = TimeWindowIsMetCVRP(RouteCliSeqv);
	}

	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex];
	double costSuppV = params.timeCost[nodeVIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)//来自两条路径
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costTimeU + costTimeV > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;

		if (routeV->load + loadU > params.vehicleCapacity) return false;
	}
	else//来自一条路径
	{
		if (costTimeU > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeUIndex == nodeYIndex) return false;

	insertNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);

	return true;
}

bool LocalSearch::move2()
{
	//𝑴𝟐：将连续的两个客户点𝒖、𝒙从原有位置并移除插入𝒗之后
	
	// 货物冲突检查
	if (!intraRouteMove) //两条不同的路径
	{
		bool ok = canInsert(nodeUIndex, routeV) && canInsert(nodeXIndex, routeV);
		if (!ok) return false;   // 存在冲突，放弃 M2
	}

	if (nodeUIndex == 0 || nodeVIndex == 0 || nodeXIndex == 0) return false;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;
	double costTimeU = 0;
	double costTimeV = 0;
	double prevTime = 0;
	// 加时间窗计算参数20241201
	if (RouteCliSequ == RouteCliSeqv) {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ);
		if (nodeVN - 1 <= 0) return false;
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN, RouteCliSequ.begin() + nodeUN + 2);// 从其当前位置移除节点 U 和 X
		if (nodeUN >= nodeVN) {
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN + 1, { nodeUIndex, nodeXIndex }); // 将节点 U 插入到节点 V 的位置之后
		}
		else {
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN - 1, { nodeUIndex, nodeXIndex }); // 将节点 U 插入到节点 V 的位置之后
		}
		costTimeV = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeU = costTimeV;
	}
	else {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);
		if (nodeVN + 1 <= 0) return false;
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN, RouteCliSequ.begin() + nodeUN + 2);// 从其当前位置移除节点 U 和 X
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		RouteCliSeqv.insert(RouteCliSeqv.begin() + nodeVN + 1, { nodeUIndex, nodeXIndex });// 将节点 U 和 X 插入到节点 V 的位置之后
		costTimeV = TimeWindowIsMetCVRP(RouteCliSeqv);
	}

	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	double different = 0.0;
	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costTimeU + costTimeV > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;
		if (routeV->load + loadU + loadX > params.vehicleCapacity) return false;
	}
	else
	{
		if (costTimeU > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot) return false;

	insertNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);

	return true;
}

bool LocalSearch::move3()
{
	//𝑴𝟑：将连续的两个客户点𝒖、𝒙从原有位置移除并以𝒙、𝒖的顺序插入𝒗之后
	
	//货物类型检查
	if (!intraRouteMove) //两条不同的路径
	{
		bool ok = canInsert(nodeUIndex, routeV) && canInsert(nodeXIndex, routeV);
		if (!ok) return false;   // 存在冲突，放弃 M3
	}

	//时间窗检查
	if (nodeUIndex == 0 || nodeVIndex == 0 || nodeXIndex == 0) return false;
	// 加时间窗计算参数20241201
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;
	double costTimeU = 0;
	double costTimeV = 0;
	double prevTime = 0;
	// 加时间窗计算参数20241201
	if (RouteCliSequ == RouteCliSeqv) {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ);
		if (nodeVN - 1 <= 0) return false;
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN, RouteCliSequ.begin() + nodeUN + 2);// 从其当前位置移除节点 U 和 X
		if (nodeUN >= nodeVN)
		{
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN + 1, nodeXIndex);
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN + 2, nodeUIndex);
		}
		else
		{
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN - 1, nodeXIndex);
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN, nodeUIndex);
		}
		//// 将节点 X 和 U 插入到节点 V 的位置之后（按照 XU 的顺序）
		costTimeV = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeU = costTimeV;
	}
	else {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);
		if (nodeVN + 1 <= 0) return false;
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN, RouteCliSequ.begin() + nodeUN + 2);// 从其当前位置移除节点 U 和 X
		// 将节点 X 和 U 插入到节点 V 的位置之后（按照 XU 的顺序）
		RouteCliSeqv.insert(RouteCliSeqv.begin() + nodeVN + 1, nodeXIndex);
		RouteCliSeqv.insert(RouteCliSeqv.begin() + nodeVN + 2, nodeUIndex);
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeV = TimeWindowIsMetCVRP(RouteCliSeqv);
	}

	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVIndex][nodeXIndex] + params.timeCost[nodeXIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	double different = 0.0;
	if (!intraRouteMove)
	{
		if (costTimeU + costTimeV > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;
		if (routeV->load + loadU + loadX > params.vehicleCapacity) return false;
	}
	else
	{
		if (costTimeU > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot) return false;

	insertNode(nodeX, nodeV);
	insertNode(nodeU, nodeX);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);

	return true;
}

bool LocalSearch::move4()
{
	//𝑴𝟒：交换客户点𝒖和𝒗的位置
	
	// 货物类型检查
	if (!intraRouteMove) //两条不同的路径
	{
		bool ok = canInsert(nodeUIndex, routeV) && canInsert(nodeVIndex, routeU);
		if (!ok) return false;   // 存在冲突，放弃 M4
	}
	// 加时间窗计算参数20241201
	if (nodeUIndex == 0 || nodeVIndex == 0) return false;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;
	double costTimeU = 0;
	double costTimeV = 0;
	double prevTime = 0;
	// 加时间窗计算参数20241201
	if (RouteCliSequ == RouteCliSeqv) {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ);
		std::swap(RouteCliSequ[nodeUN], RouteCliSequ[nodeVN]);// 交换节点 U 和 X 的值
		costTimeV = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeU = costTimeV;

	}
	else {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);
		RouteCliSequ[nodeUN] = nodeVIndex;
		RouteCliSeqv[nodeVN] = nodeUIndex;
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeV = TimeWindowIsMetCVRP(RouteCliSeqv);
	}
	
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costTimeU + costTimeV > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;
		if (routeU->load + loadV - loadU > params.vehicleCapacity || routeV->load + loadU - loadV > params.vehicleCapacity) return false;
	}
	else
	{
		if (costTimeU > prevTime || costSuppU + costSuppV > -MY_EPSILON) return false;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex) return false;

	swapNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);

	return true;
}

bool LocalSearch::move5()
{
	//𝑴𝟓：将连续的两个客户点（𝒖, 𝒙）与客户点𝒗交换位置
	//货物类型检查
	if (!intraRouteMove) //两条不同的路径
	{
		bool ok = canInsert(nodeUIndex, routeV) && canInsert(nodeXIndex, routeV) && canInsert(nodeVIndex, routeU);
		if (!ok) return false;   // 存在冲突，放弃 M6
	}
	
	// 加时间窗计算参数20241201
	if (nodeUIndex == 0 || nodeVIndex == 0 || nodeXIndex == 0) return false;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;
	double costTimeU = 0;
	double costTimeV = 0;
	double prevTime = 0;
	// 加时间窗计算参数20241201
	if (RouteCliSequ == RouteCliSeqv) {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ);
		if (nodeVN <= 0) return false;
		std::swap(RouteCliSequ[nodeUN], RouteCliSequ[nodeVN]);//先交换U和V
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN + 1);// 从其当前位置移除X
		if(nodeVN > nodeUN){
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN, nodeXIndex);//再V后边插入X
		}
		else{
			RouteCliSequ.insert(RouteCliSequ.begin() + nodeVN+1, nodeXIndex);
		}
		costTimeV = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeU = costTimeV;
	}
	else {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);
		std::swap(RouteCliSequ[nodeUN], RouteCliSeqv[nodeVN]);//先交换U和V
		RouteCliSequ.erase(RouteCliSequ.begin() + nodeUN + 1);// 从其当前位置移除X
		RouteCliSeqv.insert(RouteCliSeqv.begin() + nodeVN + 1, nodeXIndex);//再U后边插入X	
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeV = TimeWindowIsMetCVRP(RouteCliSeqv);
	}

	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	double different = 0.;
	if (!intraRouteMove)
	{
		if(costSuppU + costSuppV > -MY_EPSILON || costTimeU + costTimeV > prevTime) return false;
		if (routeU->load + loadV - loadU - loadX > params.vehicleCapacity || routeV->load + loadU + loadX - loadV > params.vehicleCapacity) return false;
	}
	else
	{
		if (costSuppU + costSuppV > -MY_EPSILON || costTimeU > prevTime) return false;
	}

	if (costSuppU + costSuppV + different > -MY_EPSILON) return false;
	if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY || nodeX->isDepot) return false;

	swapNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);

	return true;
}

bool LocalSearch::move6()
{
	//𝑴𝟔：交换两组客户点(𝒖,𝒙)和（𝒗,𝒚）的位置
	//货物类型检查20251015
	if (!intraRouteMove) // 不同路径间的交换
	{
		bool ok = canInsert(nodeUIndex, routeV) && canInsert(nodeXIndex, routeV) && canInsert(nodeVIndex, routeU) && canInsert(nodeYIndex, routeU);
		if (!ok) return false;   // 存在冲突，放弃 M6
	}
	
	// 加时间窗计算参数20241201
	if (nodeUIndex == 0 || nodeVIndex == 0 || nodeXIndex == 0 || nodeYIndex == 0) return false;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;
	
	double prevTime = 0;
	double costTimeU = 0;
	double costTimeV = 0;
	// 加时间窗计算参数20241201
	if (RouteCliSequ == RouteCliSeqv) {

		//计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ);

		// 交换 UX 和 VY 子序列的位置
		RouteCliSequ[nodeUN] = nodeVIndex;
		RouteCliSequ[nodeUN + 1] = nodeYIndex;
		RouteCliSequ[nodeVN] = nodeUIndex;
		RouteCliSequ[nodeVN + 1] = nodeXIndex;
		costTimeV = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeU = costTimeV;
	}
	else {
		// 计算原路径服务超期时间
		prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);
		// 交换 UX 和 VY 子序列的位置
		RouteCliSequ[nodeUN] = nodeVIndex;
		RouteCliSequ[nodeUN + 1] = nodeYIndex;
		RouteCliSeqv[nodeVN] = nodeUIndex;
		RouteCliSeqv[nodeVN + 1] = nodeXIndex;
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeV = TimeWindowIsMetCVRP(RouteCliSeqv);
	}

	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeYIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYNextIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeYIndex][nodeYNextIndex];

	if (!intraRouteMove)
	{
		if (costSuppU + costSuppV > -MY_EPSILON || costTimeU + costTimeV > prevTime) return false;
		if (routeU->load + loadV + loadY - loadU - loadX > params.vehicleCapacity || routeV->load + loadU + loadX - loadV - loadY > params.vehicleCapacity) return false;
		
	}
	else
	{
		if (costSuppU + costSuppV > -MY_EPSILON || costTimeU > prevTime) return false;
	}

	
	if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;

	swapNode(nodeU, nodeV);
	swapNode(nodeX, nodeY);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);

	return true;
}

bool LocalSearch::move7()
{
	//𝑴𝟕：若𝒖和𝒗位于同一路径中，令（𝒖,𝒗）和(𝒙,𝒚)分别取代（𝒖,𝒙）和（𝒗,𝒚）
	if (nodeU->position > nodeV->position) return false;
	if (nodeVIndex == nodeV->cour) return false;
	
	// 加时间窗计算参数20241201
	if (nodeUIndex == 0 || nodeVIndex == 0 || nodeXIndex == 0 || nodeYIndex == 0) return false;
	double costTime = 0;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	//计算原路径服务超期时间
	double prevTime = TimeWindowIsMetCVRP(RouteCliSequ);

	// 交换 UX 和 VY 子序列的位置
	RouteCliSequ[nodeUN + 1] = nodeVIndex;
	RouteCliSequ[nodeVN] = nodeXIndex;
	costTime = TimeWindowIsMetCVRP(RouteCliSequ);

	double cost = 0;
	if (nodeVIndex != nodeXNextIndex) {
		cost = params.timeCost[nodeUIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXNextIndex] + params.timeCost[nodeXIndex][nodeYIndex] + params.timeCost[nodeVPrevIndex][nodeXIndex]
			- params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeXIndex][nodeXNextIndex] - params.timeCost[nodeVIndex][nodeYIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex];
	}
	else {
		cost = params.timeCost[nodeUIndex][nodeVIndex] + params.timeCost[nodeXIndex][nodeYIndex]
			- params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex];
	}

	if (cost > -MY_EPSILON || costTime > prevTime) return false;
	double costUV = penaltyExcessDuration(costTime)
		+ penaltyExcessLoad(routeV->load)
		- routeU->penalty;
	if (costUV > -MY_EPSILON) return false;

	cost += costUV;
	if (cost > -MY_EPSILON) return false;
	if (nodeU->next == nodeV) return false;

	if (nodeU->next->next == nodeV) {
		Node* nodeNum = nodeX->next;
		nodeX->prev = nodeNum;
		nodeX->next = nodeY;

		while (nodeNum != nodeV)
		{
			Node* temp = nodeNum->next;
			nodeNum->next = nodeNum->prev;
			nodeNum->prev = temp;
			nodeNum = temp;
		}

		nodeV->next = nodeV->prev;
		nodeV->prev = nodeU;
		nodeU->next = nodeV;
		nodeY->prev = nodeX;
	}
	else {
		if (nodeX->isDepot)                // x 恰好是 depotU
		{
			Node* nodeUnext = nodeU->next;        // routeU 的起点 depot
			Node* nodeVprev = nodeV->prev;        // routeU 的起点 depot
			nodeU->next = nodeV;               // u 指向 v
			nodeV->prev = nodeU;               // v 指回 u
			nodeV->next = nodeUnext;               // v 指回 u
			nodeVprev->next = nodeY;
			nodeY->prev = nodeVprev;               // y 指回 x
		}
		else if (nodeV->isDepot)           // v 恰好是 depotV
		{
			Node* nodeXnext = nodeX->next;        // routeU 的起点 depot
			Node* nodeYprev = nodeV->prev;        // routeU 的起点 depot
			nodeU->next = nodeXnext;               // u 指向 v
			nodeXnext->prev = nodeU;

			nodeX->prev = nodeV;               // v 指回 u
			nodeV->next = nodeX;
			nodeX->next = nodeY;               // v 指回 u
			nodeY->prev = nodeX;               // y 指回 x
		}
		else                               // 普通情况：depot 不参与翻转
		{
			Node* nodeXnext = nodeX->next;
			Node* nodeVprev = nodeV->prev;
			// 1. routeU的操作
			nodeU->next = nodeV;
			nodeV->next = nodeXnext;
			nodeX->next->prev = nodeV;
			nodeV->prev = nodeU;
			// 20 routeV的操作
			nodeX->next = nodeY;
			nodeY->prev = nodeX;
			nodeX->next = nodeY;
			nodeY->prev = nodeX;
			// 3. 现在再改 routeV 的指针，不会影响前面已拼好的 routeU
			nodeVprev->next = nodeX;         // 原 routeV 中 v 的前驱现在指向 x
			nodeX->prev = nodeVprev;         // x 的 prev 指回该前驱（双向链表闭合）
		}
	}

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);

	return true;
}

bool LocalSearch::move8()
{
	//𝑴𝟖：若𝒖和𝒗位于不同路径中，令（𝒖,𝒗）和(𝒙,𝒚)分别取代（𝒖,𝒙）和（𝒗,𝒚）
	// 货物类型条件检查
	if (nodeX->isDepot || nodeV->isDepot) return false;
	/* 1. x → routeV ：先临时删掉 v，再判断能否插 x */
	if (!canInsert(nodeXIndex, routeV)) return false;
	/* 2. v → routeU ：先临时删掉 x，再判断能否插 v */
	if (!canInsert(nodeVIndex, routeU)) return false;
	
	// 加时间窗计算参数20241201
	if (nodeUIndex == 0 || nodeVIndex == 0 || nodeXIndex == 0 || nodeYIndex == 0) return false;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;
	//计算原路径服务超期时间
	double prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);

	double costTimeU = 0;
	double costTimeV= 0;
	if (RouteCliSequ != RouteCliSeqv) {
		RouteCliSequ[nodeUN + 1] = nodeVIndex;
		RouteCliSeqv[nodeVN] = nodeXIndex;
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeV+= TimeWindowIsMetCVRP(RouteCliSeqv);
	}
	else{
		return false;
	}

	double costSuppU = params.timeCost[nodeUIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXNextIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeXIndex][nodeYIndex] + params.timeCost[nodeVPrevIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeVPrevIndex] - params.timeCost[nodeVIndex][nodeYIndex];
	double cost = costSuppU + costSuppV;

	// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
	if (cost >= -MY_EPSILON || costTimeU + costTimeV > prevTime) return false;
	if (routeU->load + loadV - loadX > params.vehicleCapacity || routeV->load + loadX - loadV > params.vehicleCapacity) return false;
	
	/* 4. 处理 depot 指针的 3 种边界情况（保证环形链表不断） */
	if (nodeX->isDepot)                // x 恰好是 depotU
	{
		//std::cout << "M8-1" << std::endl;

		Node* nodeUnext = nodeU->next;        // routeU 的起点 depot
		Node* nodeVprev = nodeV->prev;        // routeU 的起点 depot
		nodeU->next = nodeV;               // u 指向 v
		nodeV->prev = nodeU;               // v 指回 u
		nodeV->next = nodeUnext;               // v 指回 u
		nodeVprev->next = nodeY;
		nodeY->prev = nodeVprev;               // y 指回 x
	}
	else if (nodeV->isDepot)           // v 恰好是 depotV
	{
		Node* nodeXnext = nodeX->next;        // routeU 的起点 depot
		Node* nodeYprev = nodeV->prev;        // routeU 的起点 depot
		nodeU->next = nodeXnext;               // u 指向 v
		nodeXnext->prev = nodeU;

		nodeX->prev = nodeV;               // v 指回 u
		nodeV->next = nodeX;
		nodeX->next = nodeY;               // v 指回 u
		nodeY->prev = nodeX;               // y 指回 x
	}
	else                               // 普通情况：depot 不参与翻转
	{
		Node* nodeXnext = nodeX->next;
		Node* nodeVprev = nodeV->prev;
		// 1. routeU的操作
		nodeU->next = nodeV;
		nodeV->next = nodeXnext;
		nodeX->next->prev = nodeV;
		nodeV->prev = nodeU;
		// 20 routeV的操作
		nodeX->next = nodeY;
		nodeY->prev = nodeX;
		nodeX->next = nodeY;
		nodeY->prev = nodeX;
		// 3. 现在再改 routeV 的指针，不会影响前面已拼好的 routeU
		nodeVprev->next = nodeX;         // 原 routeV 中 v 的前驱现在指向 x
		nodeX->prev = nodeVprev;         // x 的 prev 指回该前驱（双向链表闭合）
	}
	nodeX->route = routeV;
	nodeV->route = routeU;

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);

	return true;
}

bool LocalSearch::move9()
{
	//𝑴𝟗：若𝒖和𝒗位于不同路径中，令（𝒖,𝒚）和(𝒙,𝒗)分别取代（𝒖,𝒙）和（𝒗,𝒚）

	//货物类型条件验证
	if (!canInsert(nodeXIndex, routeV)) return false; // x 要进 routeV
	if (!canInsert(nodeYIndex, routeU)) return false; // y 要进 routeU
	if (nodeX->isDepot || nodeY->isDepot || nodeV->isDepot) return false;
	
	// 加时间窗计算参数20241201
	if (nodeUIndex == 0 || nodeVIndex == 0 || nodeXIndex == 0 || nodeYIndex == 0) return false;
	std::vector < int> RouteCliSequ = RouteCliSeqU;
	std::vector < int> RouteCliSeqv = RouteCliSeqV;

	//计算原路径服务超期时间
	double prevTime = TimeWindowIsMetCVRP(RouteCliSequ) + TimeWindowIsMetCVRP(RouteCliSeqv);

	double costTimeU = 0;
	double costTimeV = 0;
	if (RouteCliSequ != RouteCliSeqv) {
		RouteCliSequ[nodeUN + 1] = nodeYIndex;
		RouteCliSeqv[nodeVN] = nodeXIndex;
		RouteCliSeqv[nodeVN + 1] = nodeVIndex;
		costTimeU = TimeWindowIsMetCVRP(RouteCliSequ);
		costTimeV += TimeWindowIsMetCVRP(RouteCliSeqv);
	}
	else{
		return false;
	}

	double costSuppU = params.timeCost[nodeUIndex][nodeYIndex] + params.timeCost[nodeYIndex][nodeXNextIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeXIndex][nodeVIndex] + params.timeCost[nodeVPrevIndex][nodeXIndex] + params.timeCost[nodeVIndex][nodeYNextIndex]
		- params.timeCost[nodeVIndex][nodeVPrevIndex] - params.timeCost[nodeVIndex][nodeYIndex] - params.timeCost[nodeYIndex][nodeYNextIndex];
	double cost = costSuppU + costSuppV;
	// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
	if (cost >= -MY_EPSILON || costTimeU + costTimeV > prevTime) return false;
	if (routeU->load + loadY - loadX > params.vehicleCapacity || routeV->load + loadX - loadY > params.vehicleCapacity) return false;

	double Penality_OriginSeqU = routeU->penalty;
	double Penality_OriginSeqV = routeV->penalty;

	double Penality_CurrentSeqU = penaltyExcessDuration(costTimeU) + penaltyExcessLoad(routeU->load - loadX + loadV);
	double Penality_CurrentSeqV = penaltyExcessDuration(costTimeV) + penaltyExcessLoad(routeU->load - loadV + loadX);

	if (Penality_CurrentSeqV - Penality_OriginSeqV > -MY_EPSILON || Penality_CurrentSeqU - Penality_OriginSeqU > -MY_EPSILON) return false;

	/* 4. 处理 depot 指针的 3 种边界情况（保证环形链表不断） */
	if (nodeX->isDepot)                // x 恰好是 depotU
	{
		Node* nodeXnext = nodeX->next;
		Node* nodeYnext = nodeY->next;
		// 1. routeU的操作
		nodeU->next = nodeY;
		nodeY->next = nodeXnext;
		nodeX->next->prev = nodeY;
		nodeY->prev = nodeU;
		// 2. routeV的操作
		nodeV->next = nodeYnext;
		nodeYnext->prev = nodeV;
	}
	else                               // 普通情况：depot 不参与翻转
	{
		Node* nodeXnext = nodeX->next;
		Node* nodeVprev = nodeV->prev;
		Node* nodeYnext = nodeY->next;
		// 1. routeU的操作
		nodeU->next = nodeY;
		nodeY->next = nodeXnext;
		nodeX->next->prev = nodeY;
		nodeY->prev = nodeU;
		// 2. routeV的操作
		nodeVprev->next = nodeX;
		nodeX->next = nodeV;
		nodeV->next = nodeYnext;
		nodeYnext->prev = nodeV;
		nodeV->prev = nodeX;
		nodeX->prev = nodeVprev;
	}

	nodeX->route = routeV;
	nodeY->route = routeU;

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);

	return true;
}

bool LocalSearch::swapStar()
{
	SwapStarElement myBestSwapStar;

	// Preprocessing insertion costs
	preprocessInsertions(routeU, routeV);
	preprocessInsertions(routeV, routeU);

	// Evaluating the moves
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		if (!canInsert(nodeU->cour, routeV)) continue; //20250929 U 要进 routeV

		for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
		{
			if (!canInsert(nodeV->cour, routeU)) continue; //20250929 V 要进 routeU

			double deltaPenRouteU = routeU->load + params.cli[nodeV->cour].demand - params.cli[nodeU->cour].demand;
			double deltaPenRouteV = routeV->load + params.cli[nodeU->cour].demand - params.cli[nodeV->cour].demand;

			// Quick filter: possibly early elimination of many SWAP* due to the capacity constraints/penalties and bounds on insertion costs
			if (deltaPenRouteU + nodeU->deltaRemoval + deltaPenRouteV + nodeV->deltaRemoval <= 0)
			{
				SwapStarElement mySwapStar;
				mySwapStar.U = nodeU;
				mySwapStar.V = nodeV;

				// Evaluate best reinsertion cost of U in the route of V where V has been removed
				double extraV = getCheapestInsertSimultRemoval(nodeU, nodeV, mySwapStar.bestPositionU);

				// Evaluate best reinsertion cost of V in the route of U where U has been removed
				double extraU = getCheapestInsertSimultRemoval(nodeV, nodeU, mySwapStar.bestPositionV);

				// Evaluating final cost
				mySwapStar.moveCost = deltaPenRouteU + nodeU->deltaRemoval + extraU + deltaPenRouteV + nodeV->deltaRemoval + extraV
					+ penaltyExcessDuration(routeU->duration + nodeU->deltaRemoval + extraU + params.cli[nodeV->cour].serviceDuration - params.cli[nodeU->cour].serviceDuration)
					+ penaltyExcessDuration(routeV->duration + nodeV->deltaRemoval + extraV - params.cli[nodeV->cour].serviceDuration + params.cli[nodeU->cour].serviceDuration);

				if (mySwapStar.moveCost < myBestSwapStar.moveCost)
					myBestSwapStar = mySwapStar;
			}
		}
	}

	// Including RELOCATE from nodeU towards routeV (costs nothing to include in the evaluation at this step since we already have the best insertion location)
	// Moreover, since the granularity criterion is different, this can lead to different improving moves
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		//20250928
		if (!canInsert(nodeU->cour, routeV)) continue;   // 存在不同奇数，跳过

		SwapStarElement mySwapStar;
		mySwapStar.U = nodeU;
		mySwapStar.bestPositionU = bestInsertClient[routeV->cour][nodeU->cour].bestLocation[0];
		double deltaDistRouteU = params.timeCost[nodeU->prev->cour][nodeU->next->cour] - params.timeCost[nodeU->prev->cour][nodeU->cour] - params.timeCost[nodeU->cour][nodeU->next->cour];
		double deltaDistRouteV = bestInsertClient[routeV->cour][nodeU->cour].bestCost[0];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load - params.cli[nodeU->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load + params.cli[nodeU->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU - params.cli[nodeU->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV + params.cli[nodeU->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}

	// Including RELOCATE from nodeV towards routeU
	for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
	{
		//20250928
		if (!canInsert(nodeV->cour, routeU)) continue;// 存在不同奇数，跳过
		
		SwapStarElement mySwapStar;
		mySwapStar.V = nodeV;
		mySwapStar.bestPositionV = bestInsertClient[routeU->cour][nodeV->cour].bestLocation[0];
		double deltaDistRouteU = bestInsertClient[routeU->cour][nodeV->cour].bestCost[0];
		double deltaDistRouteV = params.timeCost[nodeV->prev->cour][nodeV->next->cour] - params.timeCost[nodeV->prev->cour][nodeV->cour] - params.timeCost[nodeV->cour][nodeV->next->cour];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load + params.cli[nodeV->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load - params.cli[nodeV->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU + params.cli[nodeV->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV - params.cli[nodeV->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}

	if (myBestSwapStar.moveCost > -MY_EPSILON) return false;

	if (myBestSwapStar.bestPositionU != NULL) {
		double costTime = TimeWindowIsMetswapStar(myBestSwapStar.U, myBestSwapStar.bestPositionU);
		if (costTime > -MY_EPSILON) return false;
		insertNode(myBestSwapStar.U, myBestSwapStar.bestPositionU);
	}
	if (myBestSwapStar.bestPositionV != NULL) {
		double costTime = TimeWindowIsMetswapStar(myBestSwapStar.V, myBestSwapStar.bestPositionV);
		if (costTime > -MY_EPSILON) return false;
		insertNode(myBestSwapStar.V, myBestSwapStar.bestPositionV);
	}
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;

	updateRouteData(routeU);
	updateRouteData(routeV);

	return true;
}

double LocalSearch::getCheapestInsertSimultRemoval(Node* U, Node* V, Node*& bestPosition)
{
	ThreeBestInsert* myBestInsert = &bestInsertClient[V->route->cour][U->cour];
	bool found = false;

	// Find best insertion in the route such that V is not next or pred (can only belong to the top three locations)
	bestPosition = myBestInsert->bestLocation[0];
	double bestCost = myBestInsert->bestCost[0];
	found = (bestPosition != V && bestPosition->next != V);
	if (!found && myBestInsert->bestLocation[1] != NULL)
	{
		bestPosition = myBestInsert->bestLocation[1];
		bestCost = myBestInsert->bestCost[1];
		found = (bestPosition != V && bestPosition->next != V);
		if (!found && myBestInsert->bestLocation[2] != NULL)
		{
			bestPosition = myBestInsert->bestLocation[2];
			bestCost = myBestInsert->bestCost[2];
			found = true;
		}
	}

	// Compute insertion in the place of V
	double deltaCost = params.timeCost[V->prev->cour][U->cour] + params.timeCost[U->cour][V->next->cour] - params.timeCost[V->prev->cour][V->next->cour];
	if (!found || deltaCost < bestCost)
	{
		bestPosition = V->prev;
		bestCost = deltaCost;
	}

	return bestCost;
}

void LocalSearch::preprocessInsertions(Route* R1, Route* R2)
{
	for (Node* U = R1->depot->next; !U->isDepot; U = U->next)
	{
		// Performs the preprocessing
		U->deltaRemoval = params.timeCost[U->prev->cour][U->next->cour] - params.timeCost[U->prev->cour][U->cour] - params.timeCost[U->cour][U->next->cour];
		if (R2->whenLastModified > bestInsertClient[R2->cour][U->cour].whenLastCalculated)
		{
			bestInsertClient[R2->cour][U->cour].reset();
			bestInsertClient[R2->cour][U->cour].whenLastCalculated = nbMoves;
			bestInsertClient[R2->cour][U->cour].bestCost[0] = params.timeCost[0][U->cour] + params.timeCost[U->cour][R2->depot->next->cour] - params.timeCost[0][R2->depot->next->cour];
			bestInsertClient[R2->cour][U->cour].bestLocation[0] = R2->depot;
			for (Node* V = R2->depot->next; !V->isDepot; V = V->next)
			{
				double deltaCost = params.timeCost[V->cour][U->cour] + params.timeCost[U->cour][V->next->cour] - params.timeCost[V->cour][V->next->cour];
				bestInsertClient[R2->cour][U->cour].compareAndAdd(deltaCost, V);
			}
		}
	}
}

void LocalSearch::insertNode(Node* U, Node* V)
{
	U->prev->next = U->next;
	U->next->prev = U->prev;
	V->next->prev = U;
	U->prev = V;
	U->next = V->next;
	V->next = U;
	U->route = V->route;
}

void LocalSearch::swapNode(Node* U, Node* V)
{
	Node* myVPred = V->prev;
	Node* myVSuiv = V->next;
	Node* myUPred = U->prev;
	Node* myUSuiv = U->next;
	Route* myRouteU = U->route;
	Route* myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->prev = V;
	myVPred->next = U;
	myVSuiv->prev = U;

	U->prev = myVPred;
	U->next = myVSuiv;
	V->prev = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}

void LocalSearch::updateRouteData(Route* myRoute)
{
	int myplace = 0;
	double myload = 0.;
	double time = 0.0;
	double mytime = 0.;//违反时间窗累计时长
	double myReversalDistance = 0.;
	double cumulatedX = 0.;
	double cumulatedY = 0.;

	Node* mynode = myRoute->depot;
	mynode->position = 0;
	mynode->cumulatedLoad = 0.;
	mynode->cumulatedTime = 0.;
	mynode->cumulatedReversalDistance = 0.;

	bool firstIt = true;
	while (!mynode->isDepot || firstIt)
	{
		mynode = mynode->next;
		myplace++;
		mynode->position = myplace;
		myload += params.cli[mynode->cour].demand;
		//mytime += params.timeCost[mynode->prev->cour][mynode->cour] + params.cli[mynode->cour].serviceDuration;//原始程序

		//20241201加时间窗
		if (params.Corres_origin[mynode->cour] == params.Corres_origin[mynode->prev->cour]){
			time += params.cli[mynode->cour].serviceDuration;//20241201
		}
		else{
			time += params.timeCost[mynode->prev->cour][mynode->cour];

			//计算超期服务时间
			if (time > params.cli[mynode->cour].latest_time) mytime += time - params.cli[mynode->cour].latest_time;
			
			//计算车辆行驶和服务累计时间
			if (params.cli[mynode->cour].earliest_time > time) time = params.cli[mynode->cour].earliest_time + params.cli[mynode->cour].serviceDuration;//20241201
			else time += params.cli[mynode->cour].serviceDuration;//20241201
		}
		myReversalDistance += params.timeCost[mynode->cour][mynode->prev->cour] - params.timeCost[mynode->prev->cour][mynode->cour];
		mynode->cumulatedLoad = myload;
		mynode->cumulatedTime = mytime; //累计时间如果考虑时间窗，则表示到达节点的时间加上服务时间
		mynode->cumulatedReversalDistance = myReversalDistance;
		if (!mynode->isDepot)
		{
			cumulatedX += params.cli[mynode->cour].coordX;
			cumulatedY += params.cli[mynode->cour].coordY;
			if (firstIt) myRoute->sector.initialize(params.cli[mynode->cour].polarAngle);
			else myRoute->sector.extend(params.cli[mynode->cour].polarAngle);
		}
		firstIt = false;
	}

	myRoute->duration = mytime;
	myRoute->load = myload;
	myRoute->penalty = penaltyExcessDuration(mytime) + penaltyExcessLoad(myload);
	myRoute->nbCustomers = myplace - 1;
	myRoute->reversalDistance = myReversalDistance;
	// Remember "when" this route has been last modified (will be used to filter unnecessary move evaluations)
	myRoute->whenLastModified = nbMoves;

	if (myRoute->nbCustomers == 0)
	{
		myRoute->polarAngleBarycenter = 1.e30;
		emptyRoutes.insert(myRoute->cour);
	}
	else
	{
		myRoute->polarAngleBarycenter = atan2(cumulatedY / (double)myRoute->nbCustomers - params.cli[0].coordY, cumulatedX / (double)myRoute->nbCustomers - params.cli[0].coordX);
		emptyRoutes.erase(myRoute->cour);
	}
}

void LocalSearch::loadIndividual(const Individual& indiv)
{
	emptyRoutes.clear();
	nbMoves = 0;
	for (int r = 0; r < params.nbVehicles; r++)
	{
		Node* myDepot = &depots[r];
		Node* myDepotFin = &depotsEnd[r];
		Route* myRoute = &routes[r];
		myDepot->prev = myDepotFin;
		myDepotFin->next = myDepot;
		if (!indiv.chromR[r].empty())
		{
			Node* myClient = &clients[indiv.chromR[r][0]];
			myClient->route = myRoute;
			myClient->prev = myDepot;
			myDepot->next = myClient;
			for (int i = 1; i < (int)indiv.chromR[r].size(); i++)
			{
				Node* myClientPred = myClient;
				myClient = &clients[indiv.chromR[r][i]];
				myClient->prev = myClientPred;
				myClientPred->next = myClient;
				myClient->route = myRoute;
			}
			myClient->next = myDepotFin;
			myDepotFin->prev = myClient;
		}
		else
		{
			myDepot->next = myDepotFin;
			myDepotFin->prev = myDepot;
		}
		updateRouteData(&routes[r]);
		routes[r].whenLastTestedSWAPStar = -1;

		std::vector<std::vector<ThreeBestInsert>> newBestInsertClient(params.nbVehicles, std::vector<ThreeBestInsert>(params.nbClients + 1));

		if (bestInsertClient[0].size() < params.nbClients + 1) {
			// 复制现有数据
			for (int v = 0; v < params.nbVehicles; ++v) {
				for (int c = 0; c < bestInsertClient[0].size(); ++c) {
					newBestInsertClient[v][c] = bestInsertClient[v][c];
				}
			}
			// 填充新客户的默认数据
			// 这里需要根据 ThreeBestInsert 的具体实现来决定如何初始化
			for (int v = 0; v < params.nbVehicles; ++v) {
				for (int c = bestInsertClient[0].size(); c < params.nbClients; ++c) {
					// 假设 ThreeBestInsert 有一个默认构造函数
					newBestInsertClient[v][c] = ThreeBestInsert(); // 使用默认构造函数
				}
			}
			bestInsertClient = newBestInsertClient;  // 更新 bestInsertClient 和其他相关参数
		}
		if (bestInsertClient[0].size() > params.nbClients + 1) {
			// 复制现有数据
			for (int v = 0; v < params.nbVehicles; ++v) {
				for (int c = 0; c < params.nbClients + 1; ++c) {
					newBestInsertClient[v][c] = bestInsertClient[v][c];
				}
			}
			bestInsertClient = newBestInsertClient; // 更新 bestInsertClient 和其他相关参数
		}
		for (int i = 1; i <= params.nbClients; i++) // Initializing memory structures
			bestInsertClient[r][i].whenLastCalculated = -1;
	}

	for (int i = 1; i <= params.nbClients; i++) // Initializing memory structures
		clients[i].whenLastTestedRI = -1;
}

void LocalSearch::exportIndividual(Individual& indiv)
{
	std::vector < std::pair <double, int> > routePolarAngles;
	for (int r = 0; r < params.nbVehicles; r++)
		routePolarAngles.push_back(std::pair <double, int>(routes[r].polarAngleBarycenter, r));
	std::sort(routePolarAngles.begin(), routePolarAngles.end()); // empty routes have a polar angle of 1.e30, and therefore will always appear at the end

	int pos = 0;
	for (int r = 0; r < params.nbVehicles; r++)
	{
		indiv.chromR[r].clear();
		Node* node = depots[routePolarAngles[r].second].next;
		while (!node->isDepot)
		{
			indiv.chromR[r].push_back(node->cour);
			node = node->next;
			pos++;
		}
	}
	indiv.evaluateCompleteCost(params);
}

bool LocalSearch::canInsert(int node, Route* tgtRoute)
{
	//20250929 快速合规：把 node 插到目标路线是否安全
	int type = demand_type[corr_origin[node]];
	for (Node* nd = tgtRoute->depot->next; !nd->isDepot; nd = nd->next)
	{
		if (conflict_matrix[type][demand_type[corr_origin[nd->cour]]]) return false; // 不同奇数冲突
	}
	return true;
}

bool LocalSearch::checkroute(Route* tgtRouteU, Route* tgtRouteV)
// 检查routeU的双向链表完整性
{
	std::vector<Node*> forward, backward;
	Node* nd;

	// 正序遍历收集节点
	for (Node* nd = tgtRouteU->depot->next; !nd->isDepot; nd = nd->next) {
		forward.push_back(nd);
	}
	// 逆序遍历收集节点
	nd = tgtRouteU->depot->prev;
	while (nd->isDepot) {
		nd = nd->prev;
	}
	for (nd; !nd->isDepot; nd = nd->prev) {
		backward.push_back(nd);
	}
	// 反转逆序序列以便比较
	std::reverse(backward.begin(), backward.end());
	// 检查是否一致
	if (forward != backward) {
		std::cerr << "错误：RouteU双向链表不一致！\n";
		std::cerr << "正序: ";
		for (Node* nd : forward) std::cerr << nd->cour << " ";
		std::cerr << "\n逆序: ";
		for (Node* nd : backward) std::cerr << nd->cour << " ";
		std::cerr << "\n";
		exit(EXIT_FAILURE);
	}
	// 检查routeV的双向链表完整性
	forward.clear();
	backward.clear();

	// 正序遍历收集节点
	for (Node* nd = tgtRouteV->depot->next; !nd->isDepot; nd = nd->next) {
		forward.push_back(nd);
	}

	// 逆序遍历收集节点
	nd = tgtRouteV->depot->prev;
	while (nd->isDepot) {
		nd = nd->prev;
	}
	for (nd; !nd->isDepot; nd = nd->prev) {
		backward.push_back(nd);
	}

	// 反转逆序序列以便比较
	std::reverse(backward.begin(), backward.end());

	// 检查是否一致
	if (forward != backward) {
		std::cerr << "错误：RouteV双向链表不一致！\n";
		std::cerr << "正序: ";
		for (Node* nd : forward) std::cerr << nd->cour << " ";
		std::cerr << "\n逆序: ";
		for (Node* nd : backward) std::cerr << nd->cour << " ";
		std::cerr << "\n";
		exit(EXIT_FAILURE);
	}
}

LocalSearch::LocalSearch(Params& params) : params(params)
{
	clients = std::vector < Node >(params.nbClients + 1);
	routes = std::vector < Route >(params.nbVehicles);
	depots = std::vector < Node >(params.nbVehicles);
	depotsEnd = std::vector < Node >(params.nbVehicles);
	bestInsertClient = std::vector < std::vector <ThreeBestInsert> >(params.nbVehicles, std::vector <ThreeBestInsert>(params.nbClients + 1));

	for (int i = 0; i <= params.nbClients; i++)
	{
		clients[i].cour = i;
		clients[i].isDepot = false;
	}
	for (int i = 0; i < params.nbVehicles; i++)
	{
		routes[i].cour = i;
		routes[i].depot = &depots[i];
		depots[i].cour = 0;
		depots[i].isDepot = true;
		depots[i].route = &routes[i];
		depotsEnd[i].cour = 0;
		depotsEnd[i].isDepot = true;
		depotsEnd[i].route = &routes[i];
	}
	for (int i = 1; i <= params.nbClients; i++) orderNodes.push_back(i);
	for (int r = 0; r < params.nbVehicles; r++) orderRoutes.push_back(r);
}