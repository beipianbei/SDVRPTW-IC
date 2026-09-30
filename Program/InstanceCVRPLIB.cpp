//
// Created by chkwon on 3/22/22.
//

#include <fstream>
#include <cmath>
#include "InstanceCVRPLIB.h"
#include <iostream>
#include <sstream>
#include "DistanceUtils.h"

// 添加全局变量的外部声明
extern std::vector<int> demand_type;
std::vector<std::vector<bool>> conflict_matrix;

InstanceCVRPLIB::InstanceCVRPLIB(std::string pathToInstance, bool /* isRoundingInteger */ = true)
{
	std::string content, content2, content3;
	double serviceTimeData = 0.;

	// Read INPUT dataset
	std::ifstream inputFile(pathToInstance);
	if (inputFile.is_open())
	{
		getline(inputFile, content);
		getline(inputFile, content);
		getline(inputFile, content);
		for (inputFile >> content ; content != "NODE_COORD_SECTION" ; inputFile >> content)
		{
			if (content == "DIMENSION") { inputFile >> content2 >> nbClients; nbClients--; } // Need to substract the depot from the number of nodes
			else if (content == "EDGE_WEIGHT_TYPE")	inputFile >> content2 >> content3;
			else if (content == "CAPACITY")	inputFile >> content2 >> vehicleCapacity;
			else if (content == "DISTANCE") { inputFile >> content2 >> durationLimit; isDurationConstraint = true; }
			else if (content == "SERVICE_TIME")	inputFile >> content2 >> serviceTimeData;
			else throw std::string("Unexpected data in input file: " + content);
		}
		if (nbClients <= 0) throw std::string("Number of nodes is undefined");
		if (vehicleCapacity == 1.e30) throw std::string("Vehicle capacity is undefined");

		x_coords = std::vector<double>(nbClients + 1);
		y_coords = std::vector<double>(nbClients + 1);
		earliest_time = std::vector<double>(nbClients + 1); 
		latest_time = std::vector<double>(nbClients + 1); 
		demands = std::vector<double>(nbClients + 1);
		service_time = std::vector<double>(nbClients + 1);

		demand_type = std::vector<int>(nbClients + 1);//20250928

		//20260120
		// 假设有6种产品类型
		const int NUM_PRODUCT_TYPES = 6;
		// 初始化冲突矩阵为6x6，初始值为false
		conflict_matrix.resize(NUM_PRODUCT_TYPES + 1,
			std::vector<bool>(NUM_PRODUCT_TYPES + 1, false));

		int node_number;
		for (int i = 0; i <= nbClients; i++)
		{
			inputFile >> node_number >> x_coords[i] >> y_coords[i];
			if (node_number != i + 1) throw std::string("The node numbering is not in order.");
		}

		// Reading demand information
		inputFile >> content;
		if (content != "DEMAND_SECTION") throw std::string("Unexpected data in input file: " + content);
		for (int i = 0; i <= nbClients; i++)
		{
			inputFile >> content >> demands[i] >> demand_type[i];
		}

		// 读取TYPE_CONFLICT部分
		inputFile >> content;
		if (content == "TYPE_CONFLICT") {
			// This section has no mandatory numeric sentinel. Parse complete lines
			// until the next section marker instead of seeking backwards in a
			// formatted stream (the old code desynchronised the parser).
			std::string line;
			std::getline(inputFile, line);
			while (std::getline(inputFile, line)) {
				if (line == "TIME_WINDOW_SECTION") { content = line; break; }
				std::istringstream pair(line);
				int type1 = 0, type2 = 0;
				if (!(pair >> type1 >> type2)) continue;
				if (type1 >= 1 && type1 <= NUM_PRODUCT_TYPES && type2 >= 1 && type2 <= NUM_PRODUCT_TYPES) {
					conflict_matrix[type1][type2] = true;
					conflict_matrix[type2][type1] = true;
				}
				else throw std::string("Product type outside supported range");
			}
		}
		else if (content != "TIME_WINDOW_SECTION") std::cerr << "期望TYPE_CONFLICT或TIME_WINDOW_SECTION, 实际: " << content << std::endl;// 如果既不是TYPE_CONFLICT也不是TIME_WINDOW_SECTION，报错

		// 如果当前内容不是TIME_WINDOW_SECTION，则读取下一个
		if (content != "TIME_WINDOW_SECTION") inputFile >> content;

		// Reading time window information
		//inputFile >> content;
		if (content != "TIME_WINDOW_SECTION") throw std::string("Unexpected data in input file: " + content);
		for (int i = 0; i <= nbClients; i++) {
			inputFile >> content >> earliest_time[i] >> latest_time[i];
		}

		// Reading service time information
		inputFile >> content;
		if (content != "SERVICE_TIME_SECTION") throw std::string("Unexpected data in input file: " + content);
		for (int i = 0; i <= nbClients; i++) {
			inputFile >> content >> service_time[i];
		}

		// Calculating 2D Euclidean Distance
		dist_mtx = std::vector < std::vector< double > >(nbClients + 1, std::vector <double>(nbClients + 1));
		for (int i = 0; i <= nbClients; i++)
		{
			for (int j = 0; j <= nbClients; j++)
			{
				dist_mtx[i][j] = euclideanRoutingDistance2(
					x_coords[i], y_coords[i], x_coords[j], y_coords[j]);
			}
		}

		// Reading depot information (in all current instances the depot is represented as node 1, the program will return an error otherwise)
		inputFile >> content >> content2 >> content3 >> content3;
		if (content != "DEPOT_SECTION") throw std::string("Unexpected data in input file: " + content);
		if (content2 != "1") throw std::string("Expected depot index 1 instead of " + content2);
		if (content3 != "EOF") throw std::string("Unexpected data in input file: " + content3);
	}
	else
		throw std::string("Impossible to open instance file: " + pathToInstance);
}

