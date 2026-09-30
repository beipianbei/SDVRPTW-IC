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

#ifndef GENETIC_H
#define GENETIC_H

#include "Population.h"
#include "Individual.h"
#include "AdaptiveLNS.h"
#include <functional>

class Genetic
{
public:

	Params & params;				// Problem parameters
	Split split;					// Split algorithm
	LocalSearch localSearch;		// Local Search structure
	AdaptiveLNS adaptiveLNS;		// Stagnation-only adaptive ruin-and-recreate
	Population population;			// Population (public for now to give access to the solutions, but should be be improved later on)
	Individual offspring;			// First individual to be used as input for the crossover
	std::vector<std::vector<double>> successorProb;
	std::function<bool(int, int)> iterationCallback;

	// OX Crossover
	void crossoverOX(Individual & result, const Individual & parent1, const Individual & parent2);
	void reinforceSuccessors(const Individual& indiv);

    // Running the genetic algorithm until maxIterNonProd consecutive iterations or a time limit
	void run() ;
	void setInitialSeed(const Individual& seed) { population.setInitialSeed(seed); }
	void setInitialSeed(const Individual& seed, bool preserveRaw) { population.setInitialSeed(seed, preserveRaw); }
	void setIterationCallback(std::function<bool(int, int)> callback) { iterationCallback = std::move(callback); }

	// 自适应选择策略：决定是否使用聚类选择
	bool shouldUseClusterSelection(int nbIter, int maxIter, double improvementRate,
		double clusterSuccessRate, int nbIterNonProd);

	// Constructor
	Genetic(Params & params);
};

#endif
