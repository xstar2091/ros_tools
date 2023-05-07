#include "ros_graph/worker/dependtreeby/dependtreeby_worker.h"

#include <algorithm>
#include <queue>
#include <boost/algorithm/string.hpp>
#include <fmt/format.h>

DependTreeByWorker::~DependTreeByWorker()
{}

bool DependTreeByWorker::init(int& argc, char**& argv)
{
    if (!command_line_param_.init(argc, argv)) return false;
    if (!param_.reset(command_line_param_)) return false;
    if (!depend_graph_.init(param_.workspace_dir)) return false;

    if (!param_.package.empty())
    {
        boost::split(package_list_, param_.package, boost::is_any_of(";"));
    }
    else
    {
        for (const auto& pair : depend_graph_.depend_map())
        {
            package_list_.emplace_back(pair.first);
        }
    }
    package_list_.erase(std::remove(package_list_.begin(), package_list_.end(), std::string()), package_list_.end());
    return true;
}

bool DependTreeByWorker::check()
{
    for (const auto& package : package_list_)
    {
        if (!depend_graph_.depend_graph().isNodeExist(package))
        {
            fmt::print(stderr, "unknown package: {}\n", package);
            return false;
        }
    }
    return true;
}

bool DependTreeByWorker::run()
{
    for (const auto& package : package_list_)
    {
        findPackageDependTree(package);
    }
    
    return true;
}

void DependTreeByWorker::findPackageDependTree(const std::string& package_name)
{
    GraphNode<std::string>* package = depend_graph_.depend_graph().nodes()[package_name];
    int indent = 0;
    findPackageDependTree(indent, package);
}

void DependTreeByWorker::findPackageDependTree(int indent, GraphNode<std::string>* package)
{
    printPackage(indent, package->value());
    for (auto pair : package->prev_nodes())
    {
        findPackageDependTree(indent + param_.indent, pair.second);
    }
}

void DependTreeByWorker::printPackage(int indent, const std::string& package_name)
{
    fmt::print(param_.separator_format_string, param_.separator, indent);
    fmt::print("{}\n", package_name);
}
