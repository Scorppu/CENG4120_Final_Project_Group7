#ifndef OUTPUT_H
#define OUTPUT_H

#include "Interface/Reader.hpp"
#include <string>

// Output functions
bool writeRoutingResult(const std::string& outputPath);
void printRoutingStats(int routedNets, bool congested, int wirelength, double runtime);

#endif // OUTPUT_H 