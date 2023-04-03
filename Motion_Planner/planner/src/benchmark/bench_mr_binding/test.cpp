#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <base/PlannerSettings.h>
#include "BenchmarkAdapter.h"

int main(int argc, char **argv) {
    std::ifstream stream(argv[1]);
    const nlohmann::json settings = nlohmann::json::parse(stream);
    global::settings.load(settings);
//    Log::instantiateRun();
    std::cout << "Loaded the following settings from " << argv[1] << ":"
              << std::endl
              << settings << std::endl << global::settings << std::endl;
    BenchmarkAdapter benchmark(global::settings);
    std::cout << benchmark.collides(20, 20) << std::endl;
}