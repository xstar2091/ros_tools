#include "ros_graph/worker/dependtree/dependtree_worker.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <queue>
#include <boost/algorithm/string.hpp>

DependTreeWorker::~DependTreeWorker()
{}

bool DependTreeWorker::init(int& argc, char**& argv)
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

bool DependTreeWorker::check()
{
    for (const auto& package : package_list_)
    {
        if (!depend_graph_.depend_graph().isNodeExist(package))
        {
            std::cerr<<"unknown package: "<<package<<std::endl;
            return false;
        }
    }
    return true;
}

bool DependTreeWorker::run()
{
    for (const auto& package : package_list_)
    {
        findPackageDependTree(package);
    }
    
    return true;
}

void DependTreeWorker::findPackageDependTree(const std::string& package_name)
{
    GraphNode<std::string>* package = depend_graph_.depend_graph().nodes()[package_name];
    int indent = 0;
    int level = 0;
    findPackageDependTree(indent, level, package);
}

void DependTreeWorker::findPackageDependTree(int indent, int level, GraphNode<std::string>* package)
{
    if (level >= param_.level) return;
    printPackage(indent, package->value());
    for (auto pair : package->next_nodes())
    {
        findPackageDependTree(indent + param_.indent, level + 1, pair.second);
    }
}

void DependTreeWorker::printPackage(int indent, const std::string& package_name)
{
    if (indent == 0)
    {
        std::cout<<package_name<<std::endl;
    }
    else
    {
        std::cout<<std::setw(indent)<<std::setfill(param_.separator[0])<<param_.separator<<package_name<<std::endl;
    }
}
