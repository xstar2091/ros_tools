#include "ros_graph/worker/dependby/dependby_worker.h"

#include <algorithm>
#include <queue>
#include <boost/algorithm/string.hpp>
#include <fmt/format.h>

DependByWorker::~DependByWorker()
{}

bool DependByWorker::init(int& argc, char**& argv)
{
    fmt::print("{}:{}\n", __FILE__, __LINE__);
    return false;
}

bool DependByWorker::check()
{
    return false;
}

bool DependByWorker::run()
{
    return false;
}

void DependByWorker::findPackages(const std::string& package_name,
                                  std::unordered_set<std::string>& table,
                                  std::vector<std::string>& package_list)
{}

void DependByWorker::sortPackages(std::vector<std::string>& package_list)
{}

void DependByWorker::printPackages(std::vector<std::string>& package_list)
{}
