//
// Created by chkwon on 3/23/22.
//

#include "C_Interface.h"
#include "DistanceUtils.h"
#include "Population.h"
#include "Params.h"
#include "Genetic.h"
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

Solution *prepare_solution(Population &population, Params &params)
{
	// Preparing the best solution
	Solution *sol = new Solution;
	sol->time = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;

	if (population.getBestFound() != nullptr) {
		// Best individual
		auto best = population.getBestFound();

		// setting the cost
		sol->cost = best->eval.penalizedCost;

		// finding out the number of routes in the best individual
		int n_routes = 0;
		for (int k = 0; k < params.nbVehicles; k++)
			if (!best->chromR[k].empty()) ++n_routes;

		// filling out the route information
		sol->n_routes = n_routes;
		sol->routes = new SolutionRoute[n_routes];
		int outputRoute = 0;
		for (int k = 0; k < params.nbVehicles; k++) {
			if (best->chromR[k].empty()) continue;
			sol->routes[outputRoute].length = (int)best->chromR[k].size();
			sol->routes[outputRoute].path = new int[sol->routes[outputRoute].length];
			std::copy(best->chromR[k].begin(), best->chromR[k].end(), sol->routes[outputRoute].path);
			++outputRoute;
		}
	}
	else {
		sol->cost = 0.0;
		sol->n_routes = 0;
		sol->routes = nullptr;
	}
	return sol;
}


extern "C" Solution *solve_cvrp(
	int n, double *x, double *y, double *serv_time, double *dem,
	double vehicleCapacity, double durationLimit, char isRoundingInteger, char isDurationConstraint,
	int max_nbVeh, const AlgorithmParameters *ap, char verbose)
{
	Solution *result = nullptr;

	try {
		std::vector<double> x_coords(x, x + n);
		std::vector<double> y_coords(y, y + n);
		std::vector<double> service_time(serv_time, serv_time + n);
		std::vector<double> demands(dem, dem + n);

		std::vector<std::vector<double> > distance_matrix(n, std::vector<double>(n));
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				distance_matrix[i][j] = euclideanRoutingDistance2(
					x_coords[i], y_coords[i], x_coords[j], y_coords[j]);
			}
		}

		//加时间窗就多家了第三四两个参数20241201
		std::vector<int> correspondingOrigin(n + 1);
		for (int i = 0; i <= n; ++i) correspondingOrigin[i] = i;
		Params params(x_coords,y_coords, x_coords, y_coords, correspondingOrigin, distance_matrix,service_time,demands,vehicleCapacity,durationLimit,max_nbVeh,isDurationConstraint,verbose,*ap);

		// Running HGS and returning the result
		Genetic solver(params);
		solver.run();
		result = prepare_solution(solver.population, params);
	}
	catch (const std::string &e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception &e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }

	return result;
}

extern "C" Solution *solve_cvrp_dist_mtx(
	int n, double *x, double *y, double *dist_mtx, double *serv_time, double *dem,
	double vehicleCapacity, double durationLimit, char isDurationConstraint,
	int max_nbVeh, const AlgorithmParameters *ap, char verbose)
{
	Solution *result = nullptr;
	std::vector<double> x_coords;
	std::vector<double> y_coords;

	try {
		if (x != nullptr && y != nullptr) {
			x_coords = {x, x + n};
			y_coords = {y, y + n};
		}

		std::vector<double> service_time(serv_time, serv_time + n);
		std::vector<double> demands(dem, dem + n);

		std::vector<std::vector<double> > distance_matrix(n, std::vector<double>(n));
		for (int i = 0; i < n; i++) { // row
			for (int j = 0; j < n; j++) { // column
				distance_matrix[i][j] = roundRoutingDistance2(dist_mtx[n * i + j]);
			}
		}

		//加时间窗就多家了第三四两个参数20241201
		std::vector<int> correspondingOrigin(n + 1);
		for (int i = 0; i <= n; ++i) correspondingOrigin[i] = i;
		Params params(x_coords, y_coords, x_coords, y_coords, correspondingOrigin, distance_matrix,service_time,demands,vehicleCapacity,durationLimit,max_nbVeh,isDurationConstraint,verbose,*ap);
		
		// Running HGS and returning the result
		Genetic solver(params);
		solver.run();
		result = prepare_solution(solver.population, params);
	}
	catch (const std::string &e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception &e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }

	return result;
}

extern "C" void delete_solution(Solution *sol)
{
	for (int i = 0; i < sol->n_routes; ++i)
		delete[] sol->routes[i].path;

	delete[] sol->routes;
	delete sol;
}
