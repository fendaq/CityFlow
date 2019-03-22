#include "boost/test/minimal.hpp"
#include "boost/filesystem.hpp"
#include <iostream>
#include <fstream>
#include "roadnet/roadnet.h"

int test_main(int argc, char *argv[])
{
    CitySimulator::RoadNet roadnet;
    roadnet.loadFromJson("../src/roadnet/tools/roadnet.json");
    std::cout << "loaded" << std::endl;
    std::ofstream output;
    output.open("test/output.json");
    output << roadnet.convertToJson();
    output.close();
    //std::cout << 1;
    return 0;
}