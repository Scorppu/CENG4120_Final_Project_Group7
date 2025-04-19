#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "../Datastructure.hpp"

class Writer {
    private:
        std::string outputPath;

    public:
        virtual void writeOutput() {
            // Implement output writing logic here
        }

        Writer(std::string outputPath) {
            this->outputPath = outputPath;
        }
};

