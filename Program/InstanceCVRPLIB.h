//
// Created by chkwon on 3/22/22.
//

#ifndef INSTANCECVRPLIB_H
#define INSTANCECVRPLIB_H
#include<string>
#include<vector>

// 添加全局变量的外部声明
extern std::vector<int> demand_type;
extern std::vector<std::vector<bool>> conflict_matrix;

class InstanceCVRPLIB
{
public:
	std::vector<double> x_coords;
	std::vector<double> y_coords;
	std::vector< std::vector<double> > dist_mtx;
	std::vector<double> service_time;
	std::vector<double> demands;
	std::vector<double> earliest_time; //20241129��
	std::vector<double> latest_time; //20241129��
	double durationLimit = 0;								// Route duration limit
	double vehicleCapacity = 1.e30;							// Capacity limit
	bool isDurationConstraint = true;						// Indicates if the problem includes duration constraints
	int nbClients ;											// Number of clients (excluding the depot)

	//2024/11/20
	std::vector<int> correspoinding_origin;
	std::vector<int> level;	//根据距离聚类后处于的层级

	InstanceCVRPLIB(std::string pathToInstance, bool isRoundingInteger);
};


#endif //INSTANCECVRPLIB_H
