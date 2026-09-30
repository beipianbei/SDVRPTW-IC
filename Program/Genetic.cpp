#include "Genetic.h"
#include <algorithm>
#include <limits>
#include <vector>
#include <map>
#include <cmath>
#include <unordered_map>

void Genetic::run()
{	
	/* INITIAL POPULATION */
	population.setAuditIteration(-1);
	population.generatePopulation();
	int nbIter;
	int nbIterNonProd = 1;
	if (params.verbose) std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
	for (nbIter = 0 ; nbIterNonProd <= params.ap.nbIter && !params.deadlineReached(); nbIter++)
	{
		population.setAuditIteration(nbIter);
		/* SELECTION AND CROSSOVER */
		crossoverOX(offspring, population.getBinaryTournament(),population.getBinaryTournament());

		/* LOCAL SEARCH */
		localSearch.run(offspring, params.penaltyCapacity, params.penaltyDuration);
		if (params.deadlineReached()) break;

		bool isNewBest = population.addIndividual(offspring,true);
		if (!offspring.eval.isFeasible && params.ran()%2 == 0) // Repair half of the solutions in case of infeasibility
		{
			localSearch.run(offspring, params.penaltyCapacity*10., params.penaltyDuration*10.);
			if (params.deadlineReached()) break;
			if (offspring.eval.isFeasible) isNewBest = (population.addIndividual(offspring,false) || isNewBest);
		}
		/* TRACKING THE NUMBER OF ITERATIONS SINCE LAST SOLUTION IMPROVEMENT */
		if (isNewBest) {
			if (params.ap.useSuccessorLearning) reinforceSuccessors(offspring);
			nbIterNonProd = 1;
		}
		else nbIterNonProd ++ ;

		if (params.ap.useAdaptiveLNS && params.ap.lnsTrigger > 0 &&
			nbIterNonProd >= params.ap.lnsTrigger && nbIterNonProd % params.ap.lnsTrigger == 0)
		{
			if (params.deadlineReached()) break;
			const Individual* elite = population.getBestFeasible();
			if (elite != NULL)
			{
				Individual kicked(params);
				const Individual* global = population.getBestFound();
				const double globalCost = global == NULL ? elite->eval.distance : global->eval.distance;
				adaptiveLNS.run(*elite, kicked, globalCost);
				if (params.deadlineReached()) break;
				localSearch.run(kicked, params.penaltyCapacity, params.penaltyDuration);
				if (params.deadlineReached()) break;
				if (population.addIndividual(kicked, true)) nbIterNonProd = 1;
			}
		}

		/* DIVERSIFICATION, PENALTY MANAGEMENT AND TRACES */
		if (nbIter % params.ap.nbIterPenaltyManagement == 0) population.managePenalties();
		if (nbIter % params.ap.nbIterTraces == 0) population.printState(nbIter, nbIterNonProd);
		if (iterationCallback && iterationCallback(nbIter, nbIterNonProd)) nbIterNonProd = 1;
		/* FOR TESTS INVOLVING SUCCESSIVE RUNS UNTIL A TIME LIMIT: WE RESET THE ALGORITHM/POPULATION EACH TIME maxIterNonProd IS ATTAINED */
		if (params.ap.timeLimit != 0 && nbIterNonProd == params.ap.nbIter)
		{
			population.restart();
			nbIterNonProd = 1;
		}
	}
	if (params.verbose) std::cout << "----- GENETIC ALGORITHM FINISHED AFTER " << nbIter << " ITERATIONS. TIME SPENT: " << (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC << std::endl;
}

void Genetic::crossoverOX(Individual& result, const Individual& parent1, const Individual& parent2)
{
	// Frequency table to track the customers which have been already inserted
	std::vector <bool> freqClient = std::vector <bool>(params.nbClients + 1, false);

	// Picking the beginning and end of the crossover zone
	std::uniform_int_distribution<> distr(0, params.nbClients - 1);
	int start = distr(params.ran);
	int end = distr(params.ran);

	// Avoid that start and end coincide by accident
	while (end == start) end = distr(params.ran);

	// Copy from start to end
	int j = start;
	while (j % params.nbClients != (end + 1) % params.nbClients)
	{
		result.chromT[j % params.nbClients] = parent1.chromT[j % params.nbClients];
		freqClient[result.chromT[j % params.nbClients]] = true;
		j++;
	}

	std::vector<int> completionOrder;
	completionOrder.reserve(params.nbClients);
	for (int i = 1; i <= params.nbClients; ++i)
		completionOrder.push_back(parent2.chromT[(end + i) % params.nbClients]);
	if (params.ap.useSuccessorLearning)
	{
		completionOrder.clear();
		std::vector<bool> selected = freqClient;
		int current = result.chromT[(j - 1 + params.nbClients) % params.nbClients];
		std::uniform_real_distribution<double> unit(0.0, 1.0);
		while (std::count(selected.begin(), selected.end(), true) < params.nbClients)
		{
			std::vector<int> available;
			for (int client = 1; client <= params.nbClients; ++client) if (!selected[client]) available.push_back(client);
			if (available.empty()) break;
			int next = available.front();
			if (unit(params.ran) < 0.10)
			{
				std::uniform_int_distribution<int> randomClient(0, static_cast<int>(available.size()) - 1);
				next = available[randomClient(params.ran)];
			}
			else
				next = *std::max_element(available.begin(), available.end(), [&](int a, int b) {
					return successorProb[current][a] < successorProb[current][b];
				});
			completionOrder.push_back(next);
			selected[next] = true;
			current = next;
		}
	}

	// Fill the remaining elements in parent-2 or learned-successor order.
	for (int temp : completionOrder)
	{
		if (freqClient[temp] == false)
		{
			result.chromT[j % params.nbClients] = temp;
			j++;
		}
	}
	// OX works on pseudo-clients; condense each original customer's pieces back
	// into one consecutive visit block before route splitting.
	std::vector<int> blockOrder;
	std::vector<bool> seenOrigin(params.nbClients + 1, false);
	for (int client : result.chromT) {
		const int origin = params.Corres_origin[client];
		if (!seenOrigin[origin]) { seenOrigin[origin] = true; blockOrder.push_back(origin); }
	}
	std::vector<int> condensed;
	condensed.reserve(result.chromT.size());
	for (int origin : blockOrder)
		for (int client : result.chromT)
			if (params.Corres_origin[client] == origin) condensed.push_back(client);
	result.chromT.swap(condensed);

	// Complete the individual with the Split algorithm
	split.generalSplit(result, parent1.eval.nbRoutes, params);

}

void Genetic::reinforceSuccessors(const Individual& indiv)
{
	const double reward = 0.30;
	for (const auto& route : indiv.chromR)
	{
		for (size_t k = 1; k < route.size(); ++k)
		{
			const int from = route[k - 1], to = route[k];
			for (int client = 1; client <= params.nbClients; ++client)
				successorProb[from][client] *= (1.0 - reward);
			successorProb[from][to] += reward;
			double sum = 0.0;
			for (int client = 1; client <= params.nbClients; ++client) if (client != from) sum += successorProb[from][client];
			if (sum > 0.0) for (int client = 1; client <= params.nbClients; ++client)
				successorProb[from][client] = client == from ? 0.0 : successorProb[from][client] / sum;
		}
	}
}

Genetic::Genetic(Params & params) : 
	params(params), 
	split(params),
	localSearch(params),
	adaptiveLNS(params),
	population(params,this->split,this->localSearch),
	offspring(params),
	successorProb(params.nbClients + 1, std::vector<double>(params.nbClients + 1,
		params.nbClients > 1 ? 1.0 / (params.nbClients - 1) : 1.0))
{
	for (int i = 1; i <= params.nbClients; ++i) successorProb[i][i] = 0.0;
}


