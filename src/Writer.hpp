#ifndef WRITER_HPP
#define WRITER_HPP

#include <string>
#include <vector>
#include "Datastructure.hpp"

class Writer {
private:
    std::string outputPath;
    const std::vector<NetRoute>* routingResults;
    const std::vector<Net>* originalNets;

public:
    // Constructor
    Writer(std::string outputPath);
    
    // Write routing results to output file
    void writeOutput();
    
    // Set the routing results to write
    void setRoutingResults(const std::vector<NetRoute>* results);
    
    // Set the original nets for name lookup
    void setOriginalNets(const std::vector<Net>* nets);
};

#endif // WRITER_HPP 