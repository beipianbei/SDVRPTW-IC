#ifndef OLD_SOLUTION_REEVALUATION_H
#define OLD_SOLUTION_REEVALUATION_H

#include "Individual.h"
#include "Params.h"
#include "SolutionValidator.h"
#include <iosfwd>
#include <optional>
#include <string>

struct OldSolutionLoadResult
{
	bool ok = false;
	std::string error;
	double oldReportedCost = 0.0;
	bool hasOldReportedCost = false;
	Individual dynamicIndividual;
	ValidationResult validation;

	explicit OldSolutionLoadResult(Params& params) : dynamicIndividual(params) {}
};

OldSolutionLoadResult loadOldSolutionAsDynamic(const std::string& solutionPath, Params& params);

std::optional<Individual> convertOldDynamicToCurrentFixed(const Individual& dynamicIndividual,
	Params& params,
	std::string& error);

bool reevaluateOldSolutionFile(const std::string& instancePath,
	const std::string& solutionPath,
	Params& params,
	std::ostream& out);

#endif
