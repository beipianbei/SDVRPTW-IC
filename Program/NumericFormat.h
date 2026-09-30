#ifndef NUMERIC_FORMAT_H
#define NUMERIC_FORMAT_H

#include <iomanip>
#include <sstream>
#include <string>

inline std::string objectiveValue2(double value)
{
	std::ostringstream out;
	out << std::fixed << std::setprecision(2) << value;
	return out.str();
}

#endif
